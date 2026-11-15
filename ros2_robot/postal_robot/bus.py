"""Exclusive ST3215 transport. Invalid replies never become zero positions."""
import threading
import time


class BusError(RuntimeError):
    pass


def packet(motor_id, instruction, parameters):
    body = bytes([motor_id, len(parameters) + 2, instruction]) + bytes(parameters)
    return b'\xff\xff' + body + bytes([(~sum(body)) & 255])


def decode_reply(raw, motor_id, size):
    if (len(raw) != size + 6 or raw[:2] != b'\xff\xff'
            or raw[2] != motor_id or raw[3] != size + 2
            or sum(raw[2:]) & 255 != 255):
        raise BusError(f'ID {motor_id}: incomplete or corrupt reply')
    if raw[4]:
        raise BusError(f'ID {motor_id}: protocol error 0x{raw[4]:02x}')
    return raw[5:-1]


class Bus:
    def __init__(self, device, timeout=.03):
        import serial
        self.serial = serial.Serial(device, 1000000, timeout=timeout,
                                    write_timeout=timeout, exclusive=True)
        self.lock = threading.Lock()

    def read(self, motor_id, address, size):
        with self.lock:
            self.serial.reset_input_buffer()
            self.serial.write(packet(motor_id, 2, [address, size]))
            raw = self.serial.read(size + 6)
            payload = decode_reply(raw, motor_id, size)
            return payload, time.monotonic()

    def close(self):
        self.serial.close()
