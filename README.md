# Arduino-Alarm-Clock
Arduino Alarm Clock with Arduino Giga R1 Wifi and Arduino Giga Display Shield

A feature-rich alarm clock built with Arduino Giga R1 WiFi, featuring environmental monitoring, gesture control, and WiFi time synchronization.

## Features

- **WiFi Time Synchronization**: Automatic time sync via NTP with RTC backup
- **Environmental Monitoring**: Temperature and humidity using BME680 sensor
- **Gesture Control**: Snooze function using BMI270 gyroscope (tilt detection include on Giga R1 Wifi)
- **Touch Interface**: Capacitive touch sensors for alarm control
- **Configurable Alarms**: Set multiple alarms with individual enable/disable
- **Smart Alerts**: Buzzer with Snooze 8 min.

## Hardware Required

- Arduino Giga R1 WiFi
- Arduino Giga Display Shield
- BME680 environmental sensor
- Buzzer

## Installation

1. Install required libraries via Arduino Library Manager:
  -  #include <WiFi.h>
  -  #include <WiFiUdp.h>
  -  #include <NTPClient.h>
  -  #include <time.h>
  -  #include "seeed_bme680.h"
  -  #include "Arduino_BMI270_BMM150.h"
  -  #include <Arduino_GigaDisplay_GFX.h>
  -  #include <Arduino_GigaDisplayTouch.h>
  -  #include "mbed.h"
  -  #include "config.h"
   
2. Update WiFi credentials in the code
3. Upload to your Arduino Giga R1 WiFi

## Usage

- Touch sensors to navigate and set alarms
- Tilt the device to snooze (gyroscope detection)
- Display automatically cycles through time, date, and environmental data

---

*Developed with assistance from Claude Code*
