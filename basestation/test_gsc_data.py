import unittest
import gsc_data

class Test(unittest.TestCase):
    def test_too_short(self):
        """
        packet must be at least 4 bytes
        """
        packet = bytearray(b'012')

        with self.assertRaises(ValueError) as ve:
            data = gsc_data.GSC_Data(packet)

        self.assertEqual('Packet too short', str(ve.exception))

    def test_not_gsc(self):
        """
        'GSC' must be in the GSC data
        """
        packet = bytearray(b'0123')

        with self.assertRaises(ValueError) as ve:
            data = gsc_data.GSC_Data(packet)

        self.assertEqual('Packet not for GSC', str(ve.exception))

    def test_invalid_length(self):
        """
        First element must be the length of the array
        """
        packet = bytearray(b'0GSC')
        packet[0] = 1
        print(packet)

        with self.assertRaises(ValueError) as ve:
            data = gsc_data.GSC_Data(packet)

        self.assertEqual('Packet length does not agree 1 != 4',
                         str(ve.exception))

    def test_temp_from_hex(self):
        """
        Get temperature setting packet manually
        """
        packet = bytearray(b'0GSC00')
        packet[0] = 6
        packet[4] = 0x08
        packet[5] = 0x7F
        print(packet)

        data = gsc_data.GSC_Data(packet)

        self.assertEqual(data.temperature, 21.75)

    def test_temp_from_int(self):
        """
        Get temperature from positive int
        """
        packet = bytearray(b'0GSC00')
        packet[0] = 6
        temp = 2175
        temp_bytes = temp.to_bytes(2, 'big')
        packet[4] = temp_bytes[0]
        packet[5] = temp_bytes[1]
        print(packet)

        data = gsc_data.GSC_Data(packet)

        self.assertEqual(data.temperature, 21.75)

    def test_temp_negative_val(self):
        """
        Get temperature from negative int
        """
        packet = bytearray(b'0GSC00')
        packet[0] = 6
        temp = -4589
        temp_bytes = temp.to_bytes(2, 'big', signed=True)
        packet[4] = temp_bytes[0]
        packet[5] = temp_bytes[1]
        print(packet)

        data = gsc_data.GSC_Data(packet)

        self.assertEqual(data.temperature, -45.89)

    def test_pressure_under_16_bits(self):
        """
        Get pressure from int
        """
        packet = bytearray(b'0GSC0000')
        packet[0] = 8
        pressure = 20100
        pressure_bytes = pressure.to_bytes(2, 'big')
        packet[6] = pressure_bytes[0]
        packet[7] = pressure_bytes[1]

        data = gsc_data.GSC_Data(packet)

        self.assertEqual(data.pressure, 100.1)

    def test_pressure_16_bits(self):
        """
        Get pressure from int
        """
        packet = bytearray(b'0GSC0000')
        packet[0] = 8
        pressure = 60120
        pressure_bytes = pressure.to_bytes(2, 'big')
        packet[6] = pressure_bytes[0]
        packet[7] = pressure_bytes[1]

        data = gsc_data.GSC_Data(packet)

        self.assertEqual(data.pressure, 140.12)

if __name__ == '__main__':
    unittest.main()
