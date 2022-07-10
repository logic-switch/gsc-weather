# SPDX-FileCopyrightText: 2018 Brent Rubell for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
Wiring Check, Pi Radio w/RFM9x

Learn Guide: https://learn.adafruit.com/lora-and-lorawan-for-raspberry-pi
Author: Brent Rubell for Adafruit Industries
"""
import time
import busio
from digitalio import DigitalInOut, Direction, Pull
import board
# Import the RFM9x radio module.
import adafruit_rfm9x

# Configure RFM9x LoRa Radio
#CS = DigitalInOut(board.D25)  # Pin 22
#RESET = DigitalInOut(board.D17)  # Pin 11
CS = DigitalInOut(board.CE1) # Pin 26
RESET = DigitalInOut(board.D25) # Pin 22
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Attempt to set up the RFM9x Module
try:
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0)
    print('RFM9x: Detected')
except RuntimeError as error:
    # Thrown on version mismatch
    print('RFM9x Error: ', error)
