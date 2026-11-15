import unittest
from postal_robot.bus import packet, decode_reply, BusError


class ProtocolTests(unittest.TestCase):
    def test_read_packet(self):
        self.assertEqual(packet(1, 2, [56, 2]), bytes.fromhex('ffff0104023802be'))

    def test_zero_is_valid_when_received(self):
        self.assertEqual(decode_reply(bytes.fromhex('ffff0104000000fa'), 1, 2), b'\x00\x00')

    def test_invalid_reply(self):
        for raw in (b'', bytes.fromhex('ffff010400000000'), bytes.fromhex('ffff0204000000f9')):
            with self.assertRaises(BusError):
                decode_reply(raw, 1, 2)

    def test_device_error(self):
        with self.assertRaises(BusError):
            decode_reply(bytes.fromhex('ffff0104010000f9'), 1, 2)
