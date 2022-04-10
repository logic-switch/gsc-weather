#!/usr/bin/python3

from time import strftime, localtime, sleep
import argparse
import random


def generate_record():
    datetime = strftime("%Y-%m-%d %H:%M:%S", localtime())
    temperature = random.uniform(18, 25)
    humidity = random.randint(35, 50)
    wind_speed = random.triangular(0, 40, 5)
    wind_gust_speed = random.triangular(5, 80, 25)
    wind_direction = random.triangular(0, 360, 180)
    pressure = random.uniform(850, 950)
    pressure2 = random.uniform(950, 1050)

    data = "%s,%0.2f,%d,%0.2f,%0.2f,%0.0f,%0.2f,%0.2f \n" % (
        datetime,
        temperature,
        humidity,
        wind_speed,
        wind_gust_speed,
        wind_direction,
        pressure,
        pressure2
        )
    return data


def write_record(filename):
    with open(filename, 'w') as f:
        f.write(generate_record())


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--filename', type=str,
                        default='/var/ramdisk/wxdata.csv')
    parser.add_argument('-r', '--records', type=int, default=1,
                        help="number of records to generate")
    args = parser.parse_args()

    sleep_time = 60

    if args.records is None:
        while True:
            write_record(args.filename)
            sleep(sleep_time)
    else:
        for x in range(0, args.records):
            write_record(args.filename)
            if x == args.records - 1:
                # Skip the last sleep
                break
            sleep(sleep_time)
