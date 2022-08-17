# GscLoRa driver
# Copyright 2022 David Laing, all rights reserved
#
"""
WeeWx driver to collect data from the GSC Barge
weather station using LoRa RFM9x

https://github.com/logic-switch/gsc-weather
"""

from __future__ import with_statement
from __future__ import absolute_import
from __future__ import print_function
import time
import logging

import weewx.drivers
import weeutil.weeutil

from crc import CrcCalculator, Crc8
import busio
from digitalio import DigitalInOut
import board
import adafruit_rfm9x

log = logging.getLogger(__name__)

DRIVER_NAME = 'GscLoRa'
DRIVER_VERSION = "0.1"

def loader(config_dict, engine):
    return GscLoRa(**config_dict[DRIVER_NAME])

class GscLoRa(weewx.drivers.AbstractDevice):
    """GSC LoRa remote sensor driver"""
    
    def __init__(self, **stn_dict):
        """Initialize the receiver"""
        log.info('driver version is %s' % DRIVER_VERSION)

    @property
    def hardware_name(self):
        return "GSC LoRa v1"

    def genLoopPackets(self):
        sensor = LoRaData()
        while True:
            packet = {'dateTime': int(time.time() + 0.0),
                    'usUnits': weewx.METRIC}
            readings = sensor.get_readings()
            packet.update(readings)
            yield packet
    
    @property
    def hardware_name(self):
        return "GscLoRa"

def confeditor_loader():
    return GscLoRaConfEditor()


class GscLoRaConfEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[GscLoRa]
    # This section is for GSC LoRa weather station

    # The driver to use:
    driver = user.gsclora
"""

class GSC_Data:
    # Packed data format
    # By using slighty unusual units it is possible to pack a reasonable range
    # of values into a 16 bit field
    # uint8 length
    # uint8[3] identification string == GSC (Glenmore Sailing Club)
    # int16 temperature  (centi-degrees celsius,  10^-2 °C)
    # uint16 pressure + 80000 (Pa)
    # uint16 wind_speed  (rotations per interval)
    # uint16 wind_gust_speed (rotations per interval)
    # int16 barge_orientation x
    # int16 barge_orientation y
    # uint16 barge_acceleration z
    # int16 barge battery ADC reading
    # uint8 crc8 (ccitt)

    # Returns
    #   Temperature in °C
    #   Pressure in kPa

    def __init__(self, packet):
        if len(packet) < 4:
            raise ValueError('Packet too short')
        if str(packet[1:4], "utf-8") != 'GSC':
            raise ValueError('Packet not for GSC')
        length = packet[0]
        if packet[0] != len(packet):
            raise ValueError(
                'Packet length does not agree {} != {}'.format(packet[0],
                                                               len(packet)))

        packet = packet[4:]  # Strip the header off of the packet

        self.temperature = 0
        if len(packet) >= 2:
            self.temperature = self._convert_value(packet.pop(0),
                                                   packet.pop(0)) / 100

        self.pressure = 0
        if len(packet) >= 2:
            self.pressure = (self._convert_value_unsigned(
                packet.pop(0),
                packet.pop(0)) + 80000) / 100

        self.wind = 0
        if len(packet) >= 2:
            self.wind = (self._convert_value_unsigned(
                packet.pop(0),
                packet.pop(0))) / 100

        self.gust = 0
        if len(packet) >= 2:
            self.gust = (self._convert_value_unsigned(
                packet.pop(0),
                packet.pop(0))) / 100

        # Compass data
        if len(packet) >= 2:
            self.x = (self._convert_value(packet.pop(0), packet.pop(0)))
        if len(packet) >= 2:
            self.y = (self._convert_value(packet.pop(0), packet.pop(0)))

        # Acceleration data
        if len(packet) >= 2:
            self.acc_z = (self._convert_value_unsigned(packet.pop(0), packet.pop(0)))

        # Battery data
        if len(packet) >= 2:
            self.battery = (self._convert_value(packet.pop(0), packet.pop(0)))

    def __repr__(self):
        return str(self.__dict__)

    def _convert_value(self, high, low):
        # Convert to 16-bit signed value.
        value = ((high & 0xFF) << 8) | (low & 0xFF)
        # Check for sign bit and turn into a negative value if set.
        if value & 0x8000 != 0:
            value -= 1 << 16
        return float(value)

    def _convert_value_unsigned(self, high, low):
        # Convert to 16-bit unsigned value.
        value = ((high & 0xFF) << 8) | (low & 0xFF)
        return float(value)

class LoRaData():
    def __init__(self):
        # Configure LoRa Radio
        cs = DigitalInOut(board.CE1) # Pin 26
        reset = DigitalInOut(board.D25) # Pin 22 - previously used pin 11
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        radio_freq_mhz = 906.2 # 902-928 MHz (centre frequency 915 MHz)
        self.rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, radio_freq_mhz,
                                    preamble_length=8, baudrate=1000000, crc=False)
        self.rfm9x.spreading_factor = 12

        # Section 4.1.1.6 - Set low datarate automatically based on BW and SF
        symbolDuration = 1000 / ( self.rfm9x.signal_bandwidth / (1 << self.rfm9x.spreading_factor) )
        if symbolDuration > 16:
            self.rfm9x.low_datarate_optimize = 1
            log.debug("low datarate on")
        else:
            self.rfm9x.low_datarate_optimize = 0
            log.debug("low datarate off")

        use_table = True
        self.crc_calculator = CrcCalculator(Crc8.CCITT, use_table)

    def verify_checksum(self, packet):
        expected_checksum = packet[-1]
        payload = packet[:-1]
        if self.crc_calculator.verify_checksum(payload, expected_checksum):
            return True
        else:
            checksum = self.crc_calculator.calculate_checksum(packet[:-1])
            print(f'Checksum error: {checksum} != found: {packet[-1]}')
            return False

    @staticmethod
    def convert_wind(rotations):
        WIND_SPEED_MULTIPLIER = 1.50 # Turn rotations per period into km/h
        wind_speed = WIND_SPEED_MULTIPLIER * rotations
        return wind_speed

    @staticmethod
    def calculate_chop(z):
        magnitude = z - 1000 # sqrt( squares summed )
        return magnitude

    @staticmethod
    def calculate_voltage(adc_reading):
        voltage = 17.6 * adc_reading / 1024
        return voltage

    def get_readings(self):
        packet = None
        data = dict()

        # check for packet rx
        packet = self.rfm9x.receive(timeout=60, with_header=True)

        if packet is None:
            # Ignore invalid packets
            return data

        # last_rssi -20..-140 -> 
        rxCheckPercent = min(100, max(0, (self.rfm9x.last_rssi + 140) * 1.2))
        data['outTemp'] = self.rfm9x.last_rssi
        data['rxCheckPercent'] = rxCheckPercent
        data['noise'] = self.rfm9x.last_snr

        if not self.verify_checksum(packet):
            # Ignore invalid packets
            log.debug(f'crc: {packet}, {self.rfm9x.last_rssi}, {self.rfm9x.last_snr}')
            return data

        try:
            gsc_data = GSC_Data(packet[:-1])
        except ValueError as ve:
            log.debug(f'{ve}: {packet}, {self.rfm9x.last_rssi}, {self.rfm9x.last_snr}')
            print(f'{ve}: {packet}, {self.rfm9x.last_rssi}, {self.rfm9x.last_snr}')
            return data

        data['outTemp'] = gsc_data.temperature
        data['pressure'] = gsc_data.pressure

        data['windSpeed'] = self.convert_wind(gsc_data.wind)
        data['windGust']  = self.convert_wind(gsc_data.gust)
        if data['windGust'] < 1.1 * data['windSpeed'] or data['windGust'] < 1 + data['windSpeed']:
            # Gusts must be 10% greater than average speed to count
            # and at least 1 km/h greater
            data['windGust'] = data['windSpeed']

        # Wind direction calculation
        # data['windDir'] = ??

        # Windy driver doesn't like something with km_per_hour2
        # data['rms'] = self.calculate_chop(gsc_data.acc_z)

        data['supplyVoltage'] = self.calculate_voltage(gsc_data.battery)
        # print(f'{gsc_data}, {self.rfm9x.last_rssi}, {self.rfm9x.last_snr}')

        return data

if __name__ == "__main__":
    sensor = LoRaData()
    while True:
        print(weeutil.weeutil.timestamp_to_string(time.time()),
                sensor.get_readings())
