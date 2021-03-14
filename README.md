# calibrationServer

Small web server for logging several sensors. 

Primarily triggered by wanting a comparison to adjust NTC temperature sensors (don't call it calibration).

Built on top of makerMcl's universalUi module.

## Functionality
* show all available data from sensors attached via I2C and OneWire
* provide long-term-logging via console log, exportable and/or download to your favorite spreadsheet tool

* also serves as demo for building simple webUI using universalUi


## Build preparations

* check out project

* copy `universalUIsettings.h_sample` into `universalUIsettings.h` and provide WLAN and OTA settings
    * for MD5 generation you can use any (online) tool
    * configuration of WIFI credentials
    * define OTA authentication data (port and password to be accepted for OTA requests)

* copy and customize local instance of `platformio_local.ini`, use `platformio_local.ini_sample` as template
    * configure OTA authentication (to be used by OTA tool)

* copy static resources for webserver to SPIFFS:
    * replace empty favicon.ico - repo contains only empty file so you are free to choose
    * upload in PIO-Console using: `pio run --target uploadfs`




## Licensing & Contact
Licensed under GPL v3.

Email: makerMcl (at) clauss.pro

Please only email me if it is more appropriate than creating an Issue / PR. I will not respond to requests for adding support for particular boards, unless of course you are the creator of the board and would like to cooperate on the project. I will also ignore any emails asking me to tell you how to implement your ideas. However, if you have a private inquiry that you would only apply to you and you would prefer it to be via email, by all means.

## Copyright

Copyright 2021 Matthias Clauß

Note: To avoid unmonitored commercial use of this work while giving back to the community, I choose the GPL licence.

## TODOs
* improve styling of HTML
* REST-API, use AJAX/jQuery to update measurements on index.html
* known bug: 2 missing chars, when clipped logbuf in part 0 at index=105 and index=119
    ** problem is deterministic, always 2 missing characters at same index position
    ** in logbuffer the string is complete
    ** to speed-up reproduction: reduce loop time (i.e. 500ms), reduce logbuffer to 2500, print
    ** in webUiGenericPlaceHolder.h the complete buf without missing chars is returned to caller
        -> look for the bug in calling code of webUiGenericPlaceHolder.h
    ** not relevant, but: heavy-load-test with 500ms loop-time and 3 webclients refreshing every second in parallel showed no problems

