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
        checksum = crc_calculator.calculate_checksum(packet[:-1])
        print(f'Checksum error: {checksum} != found: {packet[-1]}')
        return False


def write_packet_to_file(error, packet, rfm9x):

    datetime = strftime("%Y-%m-%d %H:%M:%S", localtime())
    data_list = []
    data_list.append(datetime)
    data_list.append(error)
    data_list.append(packet)
    data_list.append(rfm9x.last_rssi)
    data_list.append(rfm9x.last_snr)

    with open('/var/ramdisk/wxdata.csv', 'a') as file:
        file.write(','.join(map(str,data_list)))
        file.write('\n')

def write_data_to_file(data, rfm9x):

    datetime = strftime("%Y-%m-%d %H:%M:%S", localtime())

    data_list = []
    data_list.append(datetime)
    data_list.append(data.temperature)
    data_list.append(data.pressure)
    data_list.append(data.wind)
    data_list.append(data.gust)
    data_list.append(data.x)
    data_list.append(data.y)
    data_list.append(data.z)
    data_list.append(data.acc_x)
    data_list.append(data.acc_y)
    data_list.append(data.acc_z)

    data_list.append(rfm9x.last_rssi)
    data_list.append(rfm9x.last_snr)

    # Open file
    # If file > 250KB
    #   remove every other line from the file
    # Continue with writing new data
    with open('/var/ramdisk/wxdata.csv', 'a') as file:
        file.write(','.join(map(str,data_list)))
        file.write('\n')

if __name__ == '__main__':
    # Configure LoRa Radio
    CS = DigitalInOut(board.CE1) # Pin 26 - previously used pin 22
    RESET = DigitalInOut(board.D25) # Pin 22 - previously used pin 11
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    RADIO_FREQ_MHZ = 906.2 # 902-928 MHz (centre frequency 915 MHz)

    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ,
                                 preamble_length=8, baudrate=1000000, crc=False)
    rfm9x.spreading_factor = 12

    # Section 4.1.1.6 - Set low datarate automatically based on BW and SF
    symbolDuration = 1000 / ( rfm9x.signal_bandwidth / (1 << rfm9x.spreading_factor) )
    if symbolDuration > 16:
        rfm9x.low_datarate_optimize = 1
        print("low datarate on")
    else:
        rfm9x.low_datarate_optimize = 0
        print("low datarate off")

    print('- Waiting for PKT -')

    use_table = True
    crc_calculator = CrcCalculator(Crc8.CCITT, use_table)

    while True:
        packet = None

        # check for packet rx
        packet = rfm9x.receive(timeout=10, with_header=True)

        if packet is None:
            # Ignore invalid packets
            continue

        if not verify_checksum(crc_calculator, packet):
            # Ignore invalid packets
            print(f'crc: {packet}, {rfm9x.last_rssi}, {rfm9x.last_snr}')
            write_packet_to_file('crc', packet, rfm9x)
            continue

        try:
            data = gsc_data.GSC_Data(packet[:-1])
        except ValueError as ve:
            write_packet_to_file(ve, packet, rfm9x)
            print(f'{ve}: {packet}, {rfm9x.last_rssi}, {rfm9x.last_snr}')
            data = None

        if data is not None:
            print(f'{packet}, {data}, {rfm9x.last_rssi}, {rfm9x.last_snr}')
            write_data_to_file(data, rfm9x)

        sleep(1)
