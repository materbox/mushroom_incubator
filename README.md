# MbMushroom-incubator Sending telemetry to thingsboard server

## Devices
| Supported Devices |
|-------------------|
|  ESP8266          |

## Libraries needed
[WiFiManager.h](https://github.com/tzapu/WiFiManager)
[LittleFS.h](https://github.com/esp8266/Arduino/tree/master/libraries/LittleFS)
[Thingsboard](https://github.com/thingsboard/thingsboard-client-sdk)
[ESP_DoubleResetDetector.h](https://github.com/khoih-prog/ESP_DoubleResetDetector)
[BH1750.h](https://github.com/claws/BH1750)
[BME280](https://github.com/adafruit/Adafruit_BME280_Library)
[MQUnifiedsensor.h](https://github.com/miguel5612/MQSensorsLib)
[Wire.h]
[time.h]
[stdio.h]
[FS.h]

## Feature
Send telemetry from a BH1750 sensor to thingsboard server. Values sent are
(co, co2, alcohol, toluen, nh4, aceton, temperature, humidity, pressure, lux)
WifiManager manages wifi connection
DoubleReset manages reconfig in case we need to change device data configuration.
