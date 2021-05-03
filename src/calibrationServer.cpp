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
/**
 * Sensor server for various temperature sensors.
 * 
 * Original use was for calibration of NTC sensors, serving as reference,
 * 
 * <h>Supported sensors:</h2>
 * 	<li> KY-001 DS18B20 Temperature: ±0.5°C over the range -10 to +85°C.
 *        Pinout (left-to-right): GND - Data - Vcc 3.3V
 *        OneWire: 4k7 PullUp-Widerstand between Data and Vcc
 * <li> BMP180: i2c, +-2°C accuracy
 *        Pinout: (left-to-right): Vcc 3.3V, GND, SCL, SDA
 * <li> BME280: i2c, +-1°C accuracy
 *        Pinout: (left-to-right): Vcc 3.3V, GND, SCL, SDA
 * 
 * Circuitry:
 * * OneWire: 4k7 Pull-UP against Vcc=3.3V
 * * I2C: 2x 4k7k against Vcc=3.3V   ( to have some safety against serial resistancies)
 * * Vcc: 100..220nF directly an Vcc-Pins; + 1000µF for buffering WLAN-transmission
 * 
 * Pins used on Wemos D1 Mini:
 * * D1 = GPIO5 = SCL (I2C)
 * * D2 = GPIO4 = SDA (I2C)
 * * OneWire (DS18B20) on a free pin, configured is GPIO14 (D5) // D0 does not work
 * 
 * * D8 = GPIO15 = HC12-Rx (swapped UART0 Tx)   -> 10kOhm-pull-down required to boot (I~55µA)
 * * D7 = GPIO13 = HC12-Tx (swapped UART0 Rx)
 * * D6 = GPIO12 = HC12-Set
 * 
 * 
 * Note: <ul>
 * <li>multiple devices at I2C bus: bus topology, no star!
 * <li>if lines get longer, use 4k7 pull-up resistors on I2C lines (at master)
 * <li>confiuration uses hardwareserial UART0 for the HC-12 device, for hw-buffering (less data loss).
 * Debug output to USB is thereof realized with SoftwareSerial to RX/TX pin.
 * </ul>
 */

#include <Arduino.h>
#include <Streaming.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>

#include <SPI.h> // BME280.h needs that, to compile
#include <Wire.h>
#include <DallasTemperature.h>
#include <SFE_BMP180.h>
#include <BME280I2C.h>
#include <SoftwareSerial.h>

#define LOGBUF_LENGTH 20000 // log buffer size
//#define VERBOSE_DEBUG_LOGBUFFER

// note: `#define COPY_TO_SERIAL` must be ommitted in universalSettings.h!
#define UNIVERSALUI_WIFI_MAX_CONNECT_TRIES 20
#define UNIVERSALUI_WIFI_RECONNECT_WAIT 1000

#define PIN_DS18B20 D5           // GPIO14 // do not use GPIO0=D3! does not work with D0/GPIO16
#define PIN_HC12SET D6           // GPIO12
#define PIN_HC12TX D7            // GPIO13, Tx-pin of HC-12, from ESP's view it is Rx
#define PIN_HC12RX D8            // GPIO15, Rx-pin of HC-12, from ESP's view it is Tx
#define TEMPERATURE_PRECISION 10 // resolution for DS18B20

#define COLUMN_SEPARATOR "; "

#include "universalUi.h"
#include "webUiGenericPlaceHolder.h"
#include "appendBuffer.h"
#define VERBOSE_DEBUG_HC12TOOL
#include "hc12tool.h"
#define HEXNR(X) _WIDTHZ(_HEX(X), 2)
#define ONEWIREADR(X) HEXNR(X[1]) << HEXNR(X[2]) << HEXNR(X[3]) << HEXNR(X[4]) << HEXNR(X[5]) << HEXNR(X[6])

// global ui instance
UniversalUI ui = UniversalUI("calibrationServer");
WiFiUDP ntpUDP;
NTPClient *timeClient = new NTPClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);

OneWire oneWire(PIN_DS18B20);
DallasTemperature *sensorDS18B20 = new DallasTemperature(&oneWire);
DeviceAddress ds18b20Address;

SFE_BMP180 *sensorBmp180 = new SFE_BMP180();

BME280I2C *bme = new BME280I2C(); // Default, address=0x76, forced mode, standby time = 1000 ms; Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off
BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_Pa);

float temperatureDs18b20(NAN);
double temperatureBmp180(NAN), pressureBmp180(NAN);
float temperatureBme280(NAN), humidityBme280(NAN), pressureBme280(NAN);
char *hc12ConfigInfo;

AppendBuffer buf = AppendBuffer(2000);
AsyncWebServer *webUiServer = new AsyncWebServer(80);
RefreshState *refreshState = new RefreshState(5);

SoftwareSerial hc12serial(PIN_HC12TX, PIN_HC12RX); // args: Rx, Tx
Stream &usbSerial = Serial;
// TODO if need to use larger buffer (default: 64byte) then need to parameterize Softwareserial.begin() in hc12tool, via callback?
Hc12Tool<SoftwareSerial> hc12tool(PIN_HC12SET, hc12serial);
// variant: hc12 at hardware serial, but Serial.swap() dit not work. SoftwareSerial uses interrupts so we won't loose data
// HardwareSerial hc12serial = Serial;
//SoftwareSerial usbSerial(RX, TX); //(RX, TX);
// Hc12Tool<HardwareSerial> hc12tool(PIN_HC12SET, hc12serial);
LogBuffer *hc12Content = new LogBuffer(10000, true);

String placeholderProcessor(const String &var)
{
  if (0 == strcmp_P(var.c_str(), PSTR("DS18B20_TEMPERATURE")))
    return buf.format(PSTR("%lu"), temperatureDs18b20);
  if (0 == strcmp_P(var.c_str(), PSTR("BMP180_TEMPERATURE")))
    return buf.format(PSTR("%.1f"), temperatureBmp180);
  if (0 == strcmp_P(var.c_str(), PSTR("BMP180_PRESSURE")))
    return buf.format(PSTR("%.1f"), pressureBmp180);
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

Print &logToSerial()
{
  if (ui.isNtpTimeValid())
  {
    usbSerial << ui.getFormattedTime();
  }
  else
  {
    usbSerial << _WIDTH(millis(), 8);
  }
  return usbSerial;
}

void serverSetup();
void scanOneWire();
void scanI2C();

void setup()
{
  ui.setNtpClient(timeClient);
  ui.init(LED_BUILTIN, true, F(__FILE__), F(__TIMESTAMP__));
  // variant: use hardware serial for HC-12
  //hc12serial.updateBaudRate(9600); // ui.init() calls Serial.begin() so we must initialize swapped config after
  //hc12serial.swap();               // map UART0 to pins 13/15
  //usbSerial.begin(74800);          // default baudrate of ESP8266's bootloader
  ui.setBlink(100, 4900);
  serverSetup();

  hc12serial.begin(9600);
  hc12tool.setVerbosity(true, true, usbSerial); // Softwareserial.begin() is called by hc12tool
  //hc12tool.setParameters(BPS57600, DBM8, 3);
  hc12tool.setBaudrate(BPS57600);
  hc12ConfigInfo = hc12tool.getConfigurationInfo();
  usbSerial.print(hc12ConfigInfo);
  ui.logInfo() << F("HC-12 info:\n") << hc12ConfigInfo;

  Wire.begin();
  scanI2C();
  scanOneWire();

  /////////////  init sensors  ////////////////////
  sensorDS18B20->begin();
  // Search the wire for address
  if (sensorDS18B20->getAddress(ds18b20Address, 0))
  {
    // at index 0 is fixed code for DS18B20, at index 7 is crc
    ui.logInfo() << F("Found DS18B20 device at address 0x") << ONEWIREADR(ds18b20Address) << ", ";
    sensorDS18B20->setResolution(ds18b20Address, TEMPERATURE_PRECISION);
    ui.logInfo() << "Resolution set to: " << _DEC(sensorDS18B20->getResolution(ds18b20Address)) << endl;
    ui.logInfo() << F("Resolution set to: ") << _DEC(sensorDS18B20->getResolution(ds18b20Address)) << endl;
  }
  else
  {
    ui.logWarn() << F("Found no DS18B20 device\n");
    delete sensorDS18B20;
    sensorDS18B20 = nullptr;
  }

  if (sensorBmp180->begin())
    ui.logDebug() << F("BMP180 init success\n");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    ui.logError() << F("BMP180 init fail\n");
    delete sensorBmp180;
    sensorBmp180 = nullptr;
  }

  if (bme->begin())
  {
    switch (bme->chipModel())
    {
    case BME280::ChipModel_BME280:
      ui.logInfo() << F("Found BME280 sensor\n");
      break;
    case BME280::ChipModel_BMP280:
      ui.logInfo() << F("Found BMP280 sensor - No Humidity available\n");
      break;
    default:
      ui.logWarn() << F("Found UNKNOWN sensor instead of BME/BMP280!\n");
    }
  }
  else
  {
    bme = nullptr;
    ui.logWarn() << F("Found no BME280 device\n");
  }

  ui.logInfo() << F("STARTED\n\n\n\n");
  ui.logInfo() << COLUMN_SEPARATOR << F("DS18B20-Temp [") << ((char)176) << F("C]") << COLUMN_SEPARATOR << F("DS18B20-tofs [ms]") << COLUMN_SEPARATOR
               << COLUMN_SEPARATOR << F("BMP180-Temp [") << ((char)176) << F("C]") << COLUMN_SEPARATOR << F(" BMP180-Press. [%]") << COLUMN_SEPARATOR << F("BMP180-tofs [ms]") << COLUMN_SEPARATOR
               << COLUMN_SEPARATOR << F("BME280-Temp [") << ((char)176) << F("C]") << COLUMN_SEPARATOR << F("BME280-Press. [Pa]") << COLUMN_SEPARATOR << F(" BME280-Hum. [%]") << COLUMN_SEPARATOR << F("BME280-tofs [ms]") << COLUMN_SEPARATOR << endl;
}

// at 115200 Baud with 8N1 = 12.8kByte/sec ~ 13Byte/msec -> default 64Byte buffer can cover 5 milliseconds
void handleHC12data()
{
  const int num = hc12serial.available();
  if (num)
    usbSerial << " <" << num << "> ";
  while (hc12serial.available())
  {
    const int data = hc12serial.read();
    hc12Content->write(data);
    usbSerial.write(data);
  }
}

void loop()
{
  handleHC12data();
  if (ui.handle() && (millis() % 5000) == 4500)
  {
    Print &out = ui.logInfo();

    // DEBUG test loop for bug of missing 2 characters in part 0 of clipped log buf content
    if (false)
    {
      logToSerial() << F("[MAIN] Free heap: ") << ESP.getFreeHeap() << F(" bytes\n");
      out << F("     01234567890123456789012345678901234567890123456789") << endl; // 70chars per line
      delay(1);                                                                    // enforce next millisecond
      return;
    }

    // DS18B20 sensor
    unsigned long now = millis();
    if (sensorDS18B20 && sensorDS18B20->requestTemperaturesByAddress(ds18b20Address) && (DEVICE_DISCONNECTED_C != (temperatureDs18b20 = sensorDS18B20->getTempC(ds18b20Address))))
    {
      out << COLUMN_SEPARATOR << _FLOAT(temperatureDs18b20, 1) << COLUMN_SEPARATOR;
      out << (millis() < now) << COLUMN_SEPARATOR;
    }
    else
    {
      out << COLUMN_SEPARATOR << COLUMN_SEPARATOR;
    }
    handleHC12data();

    // BMP-180 sensor
    now = millis();
    char statusBmp180 = (nullptr == sensorBmp180) ? 0 : sensorBmp180->startTemperature();
    if (statusBmp180 != 0)
    {
      delay(statusBmp180);
      statusBmp180 = sensorBmp180->getTemperature(temperatureBmp180);
      if (statusBmp180 != 0)
      {
        out << COLUMN_SEPARATOR << _FLOAT(temperatureBmp180, 1) << COLUMN_SEPARATOR;
        statusBmp180 = sensorBmp180->startPressure(3);
        if (statusBmp180 != 0)
        {
          delay(statusBmp180);
          statusBmp180 = sensorBmp180->getPressure(pressureBmp180, temperatureBmp180);
          if (statusBmp180 != 0)
          {
            out << pressureBmp180 << COLUMN_SEPARATOR;
          }
          else
            //BMP180: error retrieving pressure measurement
            out << COLUMN_SEPARATOR;
        }
        //else BMP180: error starting pressure measurement
        out << (millis() < now) << COLUMN_SEPARATOR;
      }
      else
        // BMP180: error retrieving temperature measurement
        out << COLUMN_SEPARATOR << COLUMN_SEPARATOR << COLUMN_SEPARATOR << COLUMN_SEPARATOR;
    }
    else
    {
      // BMP180: error starting temperature measurement
      out << COLUMN_SEPARATOR << COLUMN_SEPARATOR << COLUMN_SEPARATOR << COLUMN_SEPARATOR;
    }
    handleHC12data();

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
      out << _FLOAT(pressureBme280, 1) << COLUMN_SEPARATOR;
      out << _FLOAT(humidityBme280, 1) << COLUMN_SEPARATOR;
      out << (millis() < now) << COLUMN_SEPARATOR;
    }
    // end of spreadsheet output
    out << endl;
    handleHC12data();
    logToSerial() << F("[MAIN] Free heap: ") << ESP.getFreeHeap() << F(" bytes\n");
    delay(1); // enforce next millisecond
  }
}

void scanI2C()
{
  int nDevices = 0;
  for (byte address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0)
    {
      usbSerial << F("  * I2C device found at address 0x") << HEXNR(address) << endl;
      ui.logInfo() << F("  * I2C device found at address 0x") << HEXNR(address) << endl;
      nDevices++;
    }
    else if (error == 4)
    {
      usbSerial << F("  * Unknown error at address 0x") << HEXNR(address) << endl;
      ui.logError() << F("  * Unknown error at address 0x") << HEXNR(address) << endl;
    }
  }
  if (nDevices == 0)
  {
    usbSerial << F("No I2C devices found\n");
    ui.logWarn() << F("No I2C devices found\n");
  }
  else
  {
    usbSerial << F("I2C-devices found: ") << nDevices << endl;
    ui.logInfo() << F("I2C-devices found: ") << nDevices << endl;
  }
}

void scanOneWire()
{
  byte addr[8];
  byte numFound = 0;
  usbSerial.print(F("Looking for 1-Wire devices: "));
  while (oneWire.search(addr))
  {
    ++numFound;
    usbSerial << F("\n  * Found '1-Wire' device with address 0x") << ONEWIREADR(addr);
    Print &log = ui.logInfo() << F("\n  *Found '1-Wire' device with address 0x") << ONEWIREADR(addr);
    if (OneWire::crc8(addr, 7) != addr[7])
    {
      log << F(", CRC is not valid!\n");
      usbSerial << F(", CRC is not valid!\n");
    }
    else
    {
      log << endl;
      usbSerial << endl;
    }
  }
  if (numFound > 0)
  {
    usbSerial << F("Found ") << numFound << F(" devices.\n");
    ui.logInfo() << F("found ") << numFound << F(" devices\n");
  }
  else
  {
    usbSerial << F("No 1-wire devices found.\n");
    ui.logInfo() << F("No 1-wire devices found.\n");
  }
  oneWire.reset_search();
}

class LogBufferResponseDataSource : public AwsResponseDataSource
{
private:
  LogBuffer *_logBuffer;
  size_t bufferSourceIndex = 0; // next index from log buffer
  size_t bufferRotationPoint;

public:
  LogBufferResponseDataSource(LogBuffer *logBuffer) : _logBuffer(logBuffer) {}

  // read data from log buffer as long as more data available and space left in buf
  virtual size_t fillBuffer(uint8_t *buf, size_t maxLen, size_t index)
  {
    size_t filledLen = 0;
    uint8_t *targetBuf = buf;
    size_t bufferReadLen;
    do
    {
      bufferReadLen = _logBuffer->getLog(targetBuf, maxLen, bufferSourceIndex, bufferRotationPoint);
      if (RESPONSE_TRY_AGAIN == bufferReadLen)
        return filledLen > 0 ? filledLen : RESPONSE_TRY_AGAIN;
      bufferSourceIndex += bufferReadLen;
      filledLen += bufferReadLen;
      targetBuf += bufferReadLen;
      maxLen -= bufferReadLen;
    } while (bufferReadLen > 0 && maxLen > 0);
    return filledLen;
  }

  virtual ~LogBufferResponseDataSource() {}
};

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
    AsyncWebServerResponse *response = request->beginStatefulResponse("text/html", 0, new FileWithLogBufferResponseDataSource(SPIFFS, "/log.html"), placeholderProcessor);
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
  // serve hc12content buffer directly as response object, to provide refresh via ajax
  webUiServer->on("/hc12received", HTTP_ANY, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginStatefulResponse("text/html", 0, new LogBufferResponseDataSource(hc12Content));
    response->addHeader(F("Cache-Control"), F("no-cache, must-revalidate"));
    response->addHeader(F("Pragma"), F("no-cache"));
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
    usbSerial << " not found! " << request->url();
    ui.logInfo() << F("unknown uri=") << request->url() << ", method=" << request->method() << ", body=" << body << endl;
  });
#pragma GCC diagnostic pop
  webUiServer->begin();
}
