/**
 * Sensor server for various temperature sensors.
 * 
 * Original use was for calibration of NTC sensors, serving as reference,
 * 
 * <h>Supported sensors:</h2>
 * 	<li> KY-001 DS18B20 Temperature: ±0.5°C over the range ?10 to +85°C.
 *        Pinout (left-to-right): GND - Data - Vcc 3.3V
 *        OneWire: 4k7 PullUp-Widerstand between Data and Vcc
 * <li> BMP180: i2c, +-2°C accuracy
 *        Pinout: (left-to-right): Vcc 3.3V, GND, SCL, SDA
 * <li> BME280: i2c, +-1°C accuracy
 *        Pinout: (left-to-right): Vcc 3.3V, GND, SCL, SDA
 * 
 * Pins used on Wemos D1 Mini:
 * * D1 = GPIO5 = SCL
 * * D2 = GPIO4 = SDA
 * * OneWire (DS18B20) on any pin you like, configured is GPIO16 (D0)
 * 
 * Note: <ul>
 * <li>multiple devices at I2C bus: bus topology, no star!
 * <li>if lines get longer, use 4k7 pull-up resistors on I2C lines (at master)
 * </ul>
 */
/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>

    Copyright 2021 makerMcl
 */

#include <Arduino.h>
#include <Streaming.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <WEMOS_SHT3X.h>

#include <SPI.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <DallasTemperature.h>

#define LOGBUF_LENGTH 15000 //20000 // log buffer size; need at least 25k for ESP async server!
// note: `#define COPY_TO_SERIAL` must be ommitted in universalSettings.h!
#define UNIVERSALUI_WIFI_MAX_CONNECT_TRIES 20
#define UNIVERSALUI_WIFI_RECONNECT_WAIT 1000

#define PIN_DS18B20 D0           // GPIO2 // do not use GPIO0=D3!!
#define TEMPERATURE_PRECISION 10 // resolution for DS18B20

#define COLUMN_SEPARATOR ("; ")

#include "universalUi.h"
#include "webUiGenericPlaceHolder.h"
#include "appendBuffer.h"

// global ui instance
UniversalUI ui = UniversalUI("calibrationServer");
WiFiUDP ntpUDP;
NTPClient *timeClient = new NTPClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);
SHT3X *sht = new SHT3X(0x44);
OneWire oneWire(PIN_DS18B20);
DallasTemperature *sensorDS18B20 = new DallasTemperature(&oneWire);
DeviceAddress ds18b20Address;

BME280I2C *bme = new BME280I2C(); // Default : forced mode, standby time = 1000 ms; Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off
BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_Pa);

float temperatureDs18b20(NAN);
float temperatureBmp180(NAN), humidityBmp180(NAN);
float temperatureBme280(NAN), humidityBme280(NAN), pressureBme280(NAN);

AppendBuffer buf = AppendBuffer(2000);
AsyncWebServer *webUiServer = new AsyncWebServer(80);
RefreshState *refreshState = new RefreshState(2);

String placeholderProcessor(const String &var)
{
  if (0 == strcmp_P(var.c_str(), PSTR("DS18B20_TEMPERATURE")))
    return buf.format(PSTR("%lu"), temperatureDs18b20);
  if (0 == strcmp_P(var.c_str(), PSTR("BMP180_TEMPERATURE")))
    return buf.format(PSTR("%.1f"), temperatureBmp180);
  if (0 == strcmp_P(var.c_str(), PSTR("BMP180_HUMIDITY")))
    return buf.format(PSTR("%.1f"), humidityBmp180);
  if (0 == strcmp_P(var.c_str(), PSTR("BME280_TEMPERATURE")))
    return buf.format(PSTR("%.1f"), temperatureBme280);
  if (0 == strcmp_P(var.c_str(), PSTR("BME280_HUMIDITY")))
    return buf.format(PSTR("%.1f"), humidityBme280);
  if (0 == strcmp_P(var.c_str(), PSTR("BME280_PRESSURE")))
    return buf.format(PSTR("%.1f"), pressureBme280);

  if (0 == strcmp_P(var.c_str(), PSTR("REFRESHINDEXTAG")))
    return refreshState->getRefreshTag(buf, "/index.html");
  if (0 == strcmp_P(var.c_str(), PSTR("REFRESHLOGTAG")))
    return refreshState->getRefreshTag(buf, "/log.html");
  if (0 == strcmp_P(var.c_str(), PSTR("REFRESHINDEXLINK")))
    return refreshState->getRefreshLink(buf, "/index.html");
  if (0 == strcmp_P(var.c_str(), PSTR("REFRESHLOGLINK")))
    return refreshState->getRefreshLink(buf, "/log.html");
  return universalUiPlaceholderProcessor(var, buf);
}

void serverSetup()
{
  // Initialize SPIFFS
  // note: it must be SPIFFS, LittleFS is not supported by ESP-FlashTool!
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  if (!SPIFFS.begin())
  {
    ui.logError() << "An Error has occurred while mounting SPIFFS\n";
    return;
  }
  // webUI control page
  webUiServer->on("/index.html", HTTP_ANY, [](AsyncWebServerRequest *request) {
    refreshState->evaluateRefreshParameters(request);
    AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/index.html", "text/html", false, placeholderProcessor);
    response->addHeader(F("Cache-Control"), F("no-cache, must-revalidate"));
    response->addHeader(F("Pragma"), F("no-cache"));
    request->send(response);
  });
  webUiServer->on("/log.html", HTTP_ANY, [](AsyncWebServerRequest *request) {
    refreshState->evaluateRefreshParameters(request);
    // note: placeholder for log buffer content is $LOG$
    AsyncWebServerResponse *response = //request->beginResponse(SPIFFS, "/log.html", "text/html", false, placeholderProcessor);
        request->beginStatefulResponse("text/html", 0, new FileWithLogBufferResponseDataSource(SPIFFS, "/log.html"), placeholderProcessor);
    response->addHeader(F("Cache-Control"), F("no-cache, must-revalidate"));
    response->addHeader(F("Pragma"), F("no-cache"));
    request->send(response);
  });
  webUiServer->on("/download", HTTP_ANY, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/csv");
    response->addHeader(F("Content-Disposition"), F("attachment; filename=\"sensordata.csv\""));
    response->addHeader(F("Cache-Control"), F("no-cache, must-revalidate"));
    response->addHeader(F("Pragma"), F("no-cache"));
    response->print(ui.getHtmlLog(0));
    response->print(ui.getHtmlLog(1));
    request->send(response);
  });

  webUiServer->on("/log/memory", HTTP_ANY, [](AsyncWebServerRequest *request) {
    ui.logInfo() << F("free heap = ") << ESP.getFreeHeap() << endl;
    request->redirect(F("/log.html"));
  });
  webUiServer->serveStatic("/favicon.ico", SPIFFS, "/calibrationServer.ico").setCacheControl("max-age=86400"); // 1 day = 24*60*60 [sec]
  webUiServer->serveStatic("/style.css", SPIFFS, "/style.css").setCacheControl("max-age=86400");
  webUiServer->on("/", HTTP_ANY, [](AsyncWebServerRequest *request) {
    request->redirect(F("/index.html"));
  });
  webUiServer->onNotFound([](AsyncWebServerRequest *request) {
    String body = (request->hasParam("body", true)) ? request->getParam("body", true)->value() : String();
    Serial << " not found! " << request->url();
    ui.logInfo() << F("unknown uri=") << request->url() << ", method=" << request->method() << ", body=" << body << endl;
  });
#pragma GCC diagnostic pop
  webUiServer->begin();
}

Print &logToSerial()
{
  if (ui.isNtpTimeValid())
  {
    Serial << ui.getFormattedTime();
  }
  else
  {
    Serial << _WIDTH(millis(), 8);
  }
  Serial << F(" ERROR ");
  return Serial;
}

void setup()
{
  ui.setNtpClient(timeClient);
  ui.init(LED_BUILTIN, true, F(__FILE__), F(__TIMESTAMP__));
  ui.setBlink(100, 4900);

  serverSetup();

  sensorDS18B20->begin();
  // Search the wire for address
  if (sensorDS18B20->getAddress(ds18b20Address, 0))
  {
    ui.logInfo() << "Found DS18B20 device, ";
    sensorDS18B20->setResolution(ds18b20Address, TEMPERATURE_PRECISION);
    ui.logInfo() << "Resolution set to: " << _DEC(sensorDS18B20->getResolution(ds18b20Address)) << endl;
  }
  else
  {
    ui.logWarn() << "Found no DS18B20 device\n";
    delete sensorDS18B20;
    sensorDS18B20 = nullptr;
  }

  Wire.begin();
  if (bme->begin())
  {
    switch (bme->chipModel())
    {
    case BME280::ChipModel_BME280:
      ui.logInfo() << "Found BME280 sensor! Success.\n";
      break;
    case BME280::ChipModel_BMP280:
      ui.logInfo() << "Found BMP280 sensor! No Humidity available.\n";
      break;
    default:
      ui.logWarn() << "Found UNKNOWN sensor!\n";
    }
  }
  else
  {
    bme = nullptr;
    ui.logWarn() << "Found no BME280 device\n";
  }

  ui.logInfo() << "STARTED\n\n\n\n";
  ui.logInfo() << COLUMN_SEPARATOR << "DS18B20-Temp [" << ((char)176) << "C]" << COLUMN_SEPARATOR << "DS18B20-tofs [ms]" << COLUMN_SEPARATOR
               << COLUMN_SEPARATOR << "BMP180-Temp [" << ((char)176) << "C]" << COLUMN_SEPARATOR << " BMP180-Hum. [%]" << COLUMN_SEPARATOR << "BMP180-tofs [ms]" << COLUMN_SEPARATOR
               << COLUMN_SEPARATOR << "BME280-Temp [" << ((char)176) << "C]" << COLUMN_SEPARATOR << " BME280-Hum. [%]" << COLUMN_SEPARATOR << "BME280-Press. [Pa]" << COLUMN_SEPARATOR << "BME280-tofs [ms]" << COLUMN_SEPARATOR << endl;
}

void loop()
{
  if (ui.handle() && (millis() % 2000) == 1500)
  {
    Print &out = ui.logInfo();

    // DS18B20 sensor
    unsigned long now = millis();
    if (sensorDS18B20 && (DEVICE_DISCONNECTED_C != (temperatureDs18b20 = sensorDS18B20->getTempC(ds18b20Address))))
    {
      out << COLUMN_SEPARATOR << _FLOAT(temperatureDs18b20, 1) << COLUMN_SEPARATOR;
      out << (millis() < now) << COLUMN_SEPARATOR;
    }
    else
    {
      out << COLUMN_SEPARATOR << COLUMN_SEPARATOR;
    }

    // BMP-180 sensor
    now = millis();
    if (0 == sht->get())
    {
      temperatureBmp180 = sht->cTemp;
      humidityBmp180 = sht->humidity;
      out << COLUMN_SEPARATOR << _FLOAT(temperatureBmp180, 1) << COLUMN_SEPARATOR;
      out << humidityBmp180 << COLUMN_SEPARATOR;
      out << (millis() < now) << COLUMN_SEPARATOR;
    }
    else
    {
      out << COLUMN_SEPARATOR << COLUMN_SEPARATOR << COLUMN_SEPARATOR << COLUMN_SEPARATOR;
      temperatureBmp180 = NAN;
      humidityBmp180 = NAN;
      //logToSerial() << "BMP180 status: " << sht2xStatus << endl;
    }

    // BME-280 sensor
    now = millis();
    if (nullptr != bme)
    {
      bme->read(pressureBme280, temperatureBme280, humidityBme280, tempUnit, presUnit);
    }
    if (isnan(pressureBme280) && isnan(temperatureBme280) && isnan(humidityBme280))
    {
      out << COLUMN_SEPARATOR << COLUMN_SEPARATOR << COLUMN_SEPARATOR << COLUMN_SEPARATOR << COLUMN_SEPARATOR;
    }
    else
    {
      out << COLUMN_SEPARATOR << _FLOAT(temperatureBme280, 1) << COLUMN_SEPARATOR;
      out << _FLOAT(humidityBme280, 1) << COLUMN_SEPARATOR;
      out << _FLOAT(pressureBme280, 1) << COLUMN_SEPARATOR;
      out << (millis() < now) << COLUMN_SEPARATOR;
    }
    // end of spreadsheet output
    out << endl;
    logToSerial() << "[MAIN] Free heap: " << ESP.getFreeHeap() << " bytes\n";
    delay(1); // enforce next millisecond
  }
}