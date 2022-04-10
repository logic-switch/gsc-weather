# gsc-weather

[Glenmore Sailing Club](https://www.glenmoresailingclub.com/) Weather Monitor

Providing general weather for sailors, high wind cutoff for GSC coop boats and
live wind conditions for race committee.

# Overview

```mermaid
graph Overview;
    Anemometer-->Arduino;
    MPL3115A2-->Arduino;
    Arduino-->RFM95W_LoRa_barge;
    RFM95W_LoRa_barge-->RFM95W_LoRa_basestation;
    RFM95W_LoRa_basestation -> raspberry_pi;
    Raspberry_pi -> WeeWx;
```

# Setup

## Arduino

### Wiring

### Libraries

Libraries that need to be downloaded using Library Manager (Tools -> Manage Libraries)
* CRC - https://www.arduino.cc/reference/en/libraries/crc/
* LoRa - https://www.arduino.cc/reference/en/libraries/lora/
* MPL3115A2 - https://www.arduino.cc/reference/en/libraries/adafruit-mpl3115a2-library/

## Raspberry Pi

### Wiring

### Ramdisk setup
```
pi@raspberrypi:~ $ sudo mkdir /var/ramdisk
pi@raspberrypi:~ $ echo "tmpfs /var/ramdisk tmpfs nodev,nosuid,size=128k 0 0" | sudo tee -a /etc/fstab
```

### Python
```
pip install crc
```

# Credit

[Adafruit](https://www.adafruit.com) for great products and tutorials

WeeXx [examples](https://github.com/weewx/weewx/wiki/i2C-sensor-and-other-python-scripts)