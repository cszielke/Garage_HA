# PlatformIO
Konfigurationsoptionen findet man auf der PlatformIO Seite für den [Expressif 8266 Chip](https://docs.platformio.org/en/latest/platforms/espressif8266.html){target=_blank} und im speziellen auf der Seite vom  [NodeMCU 1.0](https://docs.platformio.org/en/latest/boards/espressif8266/nodemcuv2.html#){target=_blank}.

Das Update kann entweder über die USB Schnittstelle erfolgen, oder über WLan (OTA-Update)

## platformio.ini
```ini
; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:nodemcuv2]
platform = espressif8266
board = nodemcuv2
framework = arduino

lib_deps =
  # Using a library name
  #WifiManager
  #ArduinoJson@5.13.5
  LiquidCrystal_I2C

upload_protocol = espota
# Test device
upload_port = ESP-6575F4
# Real RainIBC (6649332)
#upload_port = ESP-6598D7
upload_flags =
  --auth=rovema

monitor_port = COM7
monitor_speed = 115200

```
