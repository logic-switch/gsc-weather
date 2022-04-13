#!/usr/bin/python3

# Import Python System Libraries
from time import strftime, localtime, sleep
# Import Blinka Libraries
import busio
from digitalio import DigitalInOut, Direction, Pull
import board
# Import RFM9x
import adafruit_rfm9x

from crc import CrcCalculator, Crc8

import gsc_data


def verify_checksum(crc_calculator, packet):
    expected_checksum = packet[-1]
    payload = packet[:-1]
    if crc_calculator.verify_checksum(payload, expected_checksum):
        return True
    else:
        print('Checksum error')
        checksum = crc_calculator.calculate_checksum(packet[:-1])
        print('{} != {}'.format(checksum, packet[-1]))
        return False


def write_data_to_file(data):

    datetime = strftime("%Y-%m-%d %H:%M:%S", localtime())

    data_list = []
    data_list.append(datetime)
    data_list.append(data.temperature)
    data_list.append(data.pressure)

    with open('/var/ramdisk/wxdata.csv', 'a') as file:
        file.write(','.join(map(str,data_list)))
        file.write('\n')

if __name__ == '__main__':
    # Configure LoRa Radio
    CS = DigitalInOut(board.D25)  # Pin 22
    RESET = DigitalInOut(board.D17)  # Pin 11
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0)
    rfm9x.tx_power = 23
    prev_packet = None
    print('- Waiting for PKT -')

    use_table = True
    crc_calculator = CrcCalculator(Crc8.CCITT, use_table)

    while True:
        packet = None

        # check for packet rx
        packet = rfm9x.receive(with_header=True)

        if packet is None:
            # Ignore invalid packets
            continue

        if not verify_checksum(crc_calculator, packet):
            # Ignore invalid packets
            print(packet)
            continue

        try:
            data = gsc_data.GSC_Data(packet[:-1])
        except ValueError as ve:
            print(ve)
            data = None

        if data is not None:
            print(packet)
            print(data)
            write_data_to_file(data)

        sleep(1)
