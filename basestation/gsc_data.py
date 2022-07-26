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
    # int16 barge_orientation z
    # uint16 barge_acceleration x
    # uint16 barge_acceleration y
    # uint16 barge_acceleration z
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
                packet.pop(0)) + 80000) / 1000

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
        if len(packet) >= 2:
            self.z = (self._convert_value(packet.pop(0), packet.pop(0)))

        # Acceleration data
        if len(packet) >= 2:
            self.acc_x = (self._convert_value_unsigned(packet.pop(0), packet.pop(0)))
        if len(packet) >= 2:
            self.acc_y = (self._convert_value_unsigned(packet.pop(0), packet.pop(0)))
        if len(packet) >= 2:
            self.acc_z = (self._convert_value_unsigned(packet.pop(0), packet.pop(0)))

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
