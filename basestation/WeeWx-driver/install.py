# installer for the GscLoRa driver
# Copyright 2022 David Laing, all rights reserved
#
# https://github.com/logic-switch/gsc-weather

from setup import ExtensionInstaller

def loader():
    return GscLoRaInstaller()

class GscLoRaInstaller(ExtensionInstaller):
    def __init__(self):
        super(GscLoRaInstaller, self).__init__(
            version="0.0.1",
            name='GscLoRa',
            description='Capture weather data from GSC Barge sensor',
            author="David Laing",
            author_email="david.laing.sailing@gmail.com",
            config={
                'StdArchive': {
                    'record_generation': 'software'},
                'StdConvert': {
                    'target_unit': 'METRIC'},
            },

            files=[('bin/user', ['bin/user/gsclora.py'])]
        )
