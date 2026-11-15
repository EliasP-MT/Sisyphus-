import unittest

from postal_robot import st3215
from postal_robot.bus import Bus, BusError
from postal_robot.st3215 import ProtocolError, decode_reply, packet


class FakeSerial:
    def __init__(self, replies=()):
        self.replies, self.written = list(replies), []

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.written.append(bytes(data))

    def read(self, size):
        return self.replies.pop(0) if self.replies else b''

    def close(self):
        pass


def reply(motor_id, payload, status=0):
    body = bytes([motor_id, len(payload) + 2, status]) + bytes(payload)
    return b'\xff\xff' + body + bytes([st3215.checksum(body)])


class ProtocolTests(unittest.TestCase):
    def test_read_packet(self):
        self.assertEqual(packet(1, 2, [56, 2]), bytes.fromhex('ffff0104023802be'))

    def test_zero_is_valid_when_received(self):
        self.assertEqual(decode_reply(bytes.fromhex('ffff0104000000fa'), 1, 2), (b'\x00\x00', 0))

    def test_invalid_reply(self):
        for raw in (b'', bytes.fromhex('ffff010400000000'), bytes.fromhex('ffff0204000000f9'),
                    bytes.fromhex('ffff0105000000f9')):
            with self.assertRaises(ProtocolError):
                decode_reply(raw, 1, 2)

    def test_packet_error_is_reported_not_hidden(self):
        payload, status = decode_reply(bytes.fromhex('ffff0104010000f9'), 1, 2)
        self.assertEqual((payload, status), (b'\x00\x00', 1))

    def test_dispenser_speed_encoding_matches_teleop(self):
        self.assertEqual(st3215.encode_speed(-1000), 1000 | 0x8000)
        self.assertEqual(st3215.encode_speed(1000), 1000)
        self.assertEqual(st3215.decode_sign_magnitude(0x83E8, 15), -1000)

    def test_block_decoding_uses_0x41_for_errors(self):
        block = bytearray(st3215.BLOCK_SIZE)
        o = -st3215.BLOCK_START
        block[o + 0x28] = 1
        block[o + 0x38:o + 0x3A] = (2048).to_bytes(2, 'little')
        block[o + 0x3A:o + 0x3C] = (0x8000 | 25).to_bytes(2, 'little')
        block[o + 0x40] = 1   # asynchronous-write flag: NOT an error
        block[o + 0x41] = 0
        d = st3215.decode_block(bytes(block))
        self.assertEqual((d['position'], d['speed'], d['servo_status'], d['torque_enabled']),
                         (2048, -25, 0, True))
        block[o + 0x41] = 0x20
        self.assertEqual(st3215.decode_block(bytes(block))['servo_status'], 0x20)
        block[o + 0x38:o + 0x3A] = (5000).to_bytes(2, 'little')
        with self.assertRaises(ProtocolError):
            st3215.decode_block(bytes(block))


class BusTests(unittest.TestCase):
    def test_read_returns_payload_status_and_time(self):
        bus = Bus('fake', [1], transport=FakeSerial([reply(1, b'\x00\x08')]))
        payload, status, received = bus.read(1, 0x38, 2)
        self.assertEqual((payload, status), (b'\x00\x08', 0))
        self.assertGreater(received, 0)

    def test_corrupt_read_raises(self):
        bus = Bus('fake', [1], transport=FakeSerial([b'\xff\xff\x01']))
        with self.assertRaises(BusError):
            bus.read(1, 0x38, 2)

    def test_write_restrictions(self):
        serial = FakeSerial()
        bus = Bus('fake', [1, 9], transport=serial)
        with self.assertRaises(BusError):
            bus.write(2, st3215.GOAL_POSITION, 100)          # not a writable ID
        with self.assertRaises(BusError):
            bus.write(1, st3215.MODE, 1)                     # EEPROM
        with self.assertRaises(BusError):
            bus.write(1, 0x05, 3)                            # ID register
        with self.assertRaises(BusError):
            bus.write(1, st3215.TORQUE_ENABLE, 256)
        self.assertEqual(serial.written, [])
        with self.assertRaises(ValueError):
            Bus('fake', [st3215.BROADCAST_ID], transport=serial)

    def test_write_frame_and_reply(self):
        serial = FakeSerial([reply(9, b'')])
        bus = Bus('fake', [9], transport=serial)
        self.assertEqual(bus.write(9, st3215.GOAL_SPEED, st3215.encode_speed(-1000)), 0)
        self.assertEqual(serial.written[0], packet(9, st3215.INST_WRITE, [0x2E, 0xE8, 0x83]))

    def test_write_without_reply_level(self):
        serial = FakeSerial()
        bus = Bus('fake', [9], transport=serial)
        bus.reply_to_writes[9] = False
        self.assertEqual(bus.write(9, st3215.TORQUE_ENABLE, 0), 0)
        self.assertEqual(len(serial.written), 1)


if __name__ == '__main__':
    unittest.main()
