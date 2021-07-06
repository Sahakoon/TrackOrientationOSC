# TrackOrientationOSC for arduino IDE
Features
1. M5Stack can track the orientation (pitch, roll, yaw).
2. M5Stack send the data by WiFi OSC (Open Sound Control) protocol.

Hardware
1. M5Stack ESP32 GRAY Development Kit with 9-Axis Sensor
Link: https://shop.m5stack.com/collections/m5-core/products/grey-development-core
2. Battery Module for ESP32 Core Development Kit
Link: https://shop.m5stack.com/collections/m5-module/products/battery-module
  
Install Libraries:
1. OSC Protocol
https://github.com/CNMAT/OSC
2. arduino-esp32 --> This project have to use WiFiUDPClient.
https://github.com/espressif/arduino-esp32

Reference Code:
1. omegatao: M5StackGrey_MPU6886_BMM150_AHRS_sample
This is a M5Stack grey (MPU6886 + BMM150 version) sample to calculate AHRS status.
Link: https://github.com/omegatao/M5StackGrey_MPU6886_BMM150_AHRS_sample
2. 9PEO0xNd: M5Gray_MPU6886_BMM150
Link: https://github.com/9PEO0xNd/M5Gray_MPU6886_BMM150

*M5Stack Sensor Issue: https://github.com/m5stack/M5Stack/issues/191
