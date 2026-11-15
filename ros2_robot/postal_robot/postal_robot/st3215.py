"""ST3215/STS3215 register map and pure encoding helpers (no I/O).

Addresses follow the Feetech STS memory table used by the existing tools:
0x40 is the asynchronous-write flag, 0x41 the servo error status and 0x42 the
moving flag. Only RAM registers listed in WRITABLE may ever be written: EEPROM
(addresses < 0x28: ID, baud rate, limits, mode, offsets) is never touched.
"""

INST_READ = 0x02
INST_WRITE = 0x03
BROADCAST_ID = 0xFE

# EEPROM (read-only for this software)
MODEL = 0x03
MIN_ANGLE_LIMIT = 0x09
MAX_ANGLE_LIMIT = 0x0B
MODE = 0x21             # 0 position, 1 wheel (velocity)
# RAM
TORQUE_ENABLE = 0x28
ACCELERATION = 0x29
GOAL_POSITION = 0x2A
GOAL_SPEED = 0x2E
LOCK = 0x37
PRESENT_POSITION = 0x38

MODE_POSITION = 0
MODE_WHEEL = 1

# Writable registers and their exact sizes.
WRITABLE = {TORQUE_ENABLE: 1, ACCELERATION: 1, GOAL_POSITION: 2, GOAL_SPEED: 2}

# One read covers torque (0x28) .. present current (0x46): 31 bytes.
BLOCK_START = TORQUE_ENABLE
BLOCK_SIZE = 0x47 - BLOCK_START

MAX_SPEED_MAGNITUDE = 0x7FFF
ENCODER_MAX = 4095


class ProtocolError(RuntimeError):
    pass


def checksum(body):
    return (~sum(body)) & 0xFF


def packet(motor_id, instruction, parameters):
    if not 0 <= motor_id <= 0xFE:
        raise ValueError('Invalid servo ID')
    body = bytes([motor_id, len(parameters) + 2, instruction]) + bytes(parameters)
    return b'\xff\xff' + body + bytes([checksum(body)])


def decode_reply(raw, motor_id, size):
    """Validate header, ID, length and checksum. Return (payload, packet_status).

    A non-zero packet status is returned, not hidden: the caller reports it as
    a per-motor fault while still knowing the bytes were genuinely received.
    """
    if len(raw) != size + 6:
        raise ProtocolError(f'ID {motor_id}: expected {size + 6} bytes, got {len(raw)}')
    if raw[:2] != b'\xff\xff':
        raise ProtocolError(f'ID {motor_id}: bad header')
    if raw[2] != motor_id:
        raise ProtocolError(f'ID {motor_id}: reply from ID {raw[2]}')
    if raw[3] != size + 2:
        raise ProtocolError(f'ID {motor_id}: bad length field {raw[3]}')
    if checksum(raw[2:-1]) != raw[-1]:
        raise ProtocolError(f'ID {motor_id}: checksum mismatch')
    return bytes(raw[5:-1]), raw[4]


def encode_speed(value):
    """Signed speed -> STS sign-magnitude (bit 15 = negative), as teleop.py spin."""
    value = int(value)
    if abs(value) > MAX_SPEED_MAGNITUDE:
        raise ValueError('Speed magnitude too large')
    return abs(value) | (0x8000 if value < 0 else 0)


def decode_sign_magnitude(raw, sign_bit):
    magnitude = raw & ((1 << sign_bit) - 1)
    return -magnitude if raw & (1 << sign_bit) else magnitude


def u16(data, offset):
    return int.from_bytes(data[offset:offset + 2], 'little')


def decode_block(payload):
    """Decode the 31-byte block starting at TORQUE_ENABLE."""
    if len(payload) != BLOCK_SIZE:
        raise ProtocolError('Bad status block size')
    o = -BLOCK_START
    position = u16(payload, o + 0x38)
    if position > ENCODER_MAX:
        raise ProtocolError(f'Encoder value {position} outside 0..{ENCODER_MAX}')
    return dict(
        torque_enabled=payload[o + 0x28] == 1,
        acceleration=payload[o + 0x29],
        goal_position=u16(payload, o + 0x2A),
        goal_speed=decode_sign_magnitude(u16(payload, o + 0x2E), 15),
        position=position,
        speed=decode_sign_magnitude(u16(payload, o + 0x3A), 15),
        load=decode_sign_magnitude(u16(payload, o + 0x3C), 10),
        voltage=payload[o + 0x3E] / 10,
        temperature=payload[o + 0x3F],
        servo_status=payload[o + 0x41],
        moving=payload[o + 0x42] != 0,
        current=decode_sign_magnitude(u16(payload, o + 0x45), 15),
    )
