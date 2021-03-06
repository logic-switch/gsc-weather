# gsc-weather

[Glenmore Sailing Club](https://www.glenmoresailingclub.com/) Weather Monitor

Providing general weather for sailors, high wind cutoff for GSC co-op boats and
live wind conditions for race committee.

# Overview

```mermaid
graph LR;
    Anemometer-->Arduino;
    AK09918-->Arduino;
    BMP180-->Arduino;
    ICM20600-->Arduino;
    Arduino-->RFM95W_LoRa_barge;
    RFM95W_LoRa_barge-->RFM95W_LoRa_basestation;
    RFM95W_LoRa_basestation-->Raspberry_pi;
    Raspberry_pi-->WeeWx;
```

# Arduino

Add to File->Preferences->Additional Boards Manager URLs:
https://adafruit.github.io/arduino-board-index/package_adafruit_index.json

## Wiring

Atmega32u4 - Adafruit Feather 32u4

* Anemometer - 6P4C (RJ11 phone) connector
    1. black - GND
    2. red - 3V
    3. green - PCINT5 (A9)
    4. yellow - unused
* I2C sensors - VCC, GND, SDA (2), SCL(3)

## Libraries

Libraries that need to be downloaded using Library Manager (Tools -> Manage Libraries)
* CRC - https://www.arduino.cc/reference/en/libraries/crc/
* LoRa - https://www.arduino.cc/reference/en/libraries/lora/
* EWMA - https://www.arduino.cc/reference/en/libraries/ewma/
* BMP180
   * https://github.com/adafruit/Adafruit_Sensor
   * https://github.com/adafruit/Adafruit_BMP183_Unified_Library
* Grove - IMU 9DOF - https://github.com/Seeed-Studio/Seeed_ICM20600_AK0991
* PCINT library - https://github.com/paulo-raca/YetAnotherArduinoPcIntLibrary

# Raspberry Pi

## Wiring

## Ramdisk setup
```
pi@raspberrypi:~ $ sudo mkdir /var/ramdisk
pi@raspberrypi:~ $ echo "tmpfs /var/ramdisk tmpfs nodev,nosuid,size=256k 0 0" | sudo tee -a /etc/fstab
```

## SPI Setup
```
pi@raspberrypi:~ $ sudo raspi-config
        -> Interfacing Options -> SPI -> yes -> Finish
```

## Python

### Install Python Libraries
```
python3 -m pip install crc
python3 -m pip install adafruit-circuitpython-rfm9x
```

### Running unit tests
```
~/gsc-weather $ cd basestation
~/gsc-weather/basestation $ python3 -m unittest *
```

# Credit

[Adafruit](https://www.adafruit.com) for great products and tutorials

WeeXx [pi weather station](https://github.com/weewx/weewx/wiki/Raspberry-Pi-weather-station-with-i2C-sensors), [sensors and scripts](https://github.com/weewx/weewx/wiki/i2C-sensor-and-other-python-scripts)
