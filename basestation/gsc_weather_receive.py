#!/usr/bin/python3

# Import Python System Libraries
import time
# Import Blinka Libraries
import busio
from digitalio import DigitalInOut, Direction, Pull
import board
# Import RFM9x
import adafruit_rfm9x

from crc import CrcCalculator, Crc8

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
        pass
    else:
        # Display the packet text and rssi
        prev_packet = packet
        expected_checksum = packet[-1]
        packet_text = str(packet[1:4], "utf-8")
        print(packet_text)

        if crc_calculator.verify_checksum(packet[:-1], expected_checksum):
            if str(packet[1:4], "utf-8") == 'GSC':
                print('Packet received - len {}'.format(int(packet[0])))
                packet_text = str(packet[1:-1], "utf-8")
                print(packet_text)
            else:
                print('Packet not for GSC')
        else:
            print('Checksum error')
            checksum = crc_calculator.calculate_checksum(packet[:-1])
            print('{} != {}'.format(checksum, packet[-1]))

        time.sleep(1)

    time.sleep(0.1)
