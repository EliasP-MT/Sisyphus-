"""Byte-level ST3215 serial test double, selected only by device "sim://".

Used by integration tests on the PC. It is never a source of hardware
feedback: the core reports "SIMULATED SERIAL BUS" in every supervisor state
message while it is in use.
"""
import threading
import time

from . import st3215

ROLE_MODE = {**{i: 0 for i in range(1, 9)}, 9: 1, 10: 1, 11: 1, 12: 1}
START = {1: 2075, 2: 2065, 3: 2024, 4: 2045, 5: 1888, 6: 1910, 7: 2240, 8: 1500,
         9: 1290, 10: 2913, 11: 1678, 12: 2458}


class FakeServo:
    def __init__(self, motor_id, mode, position):
        self.mem = bytearray(0x50)
        self.mem[0x03:0x05] = (777).to_bytes(2, 'little')
        self.mem[0x05], self.mem[0x08], self.mem[0x21] = motor_id, 1, mode
        self.mem[0x0B:0x0D] = (4095).to_bytes(2, 'little')
        self.mem[0x3E], self.mem[0x3F] = 129, 30
        self.position, self.speed, self.t = float(position), 0., time.monotonic()
        self.mem[0x2A:0x2C] = int(position).to_bytes(2, 'little')
        self.silent = False
        self._sync()

    def _u16(self, a):
        return int.from_bytes(self.mem[a:a + 2], 'little')

    def update(self):
        now = time.monotonic()
        dt, self.t = now - self.t, now
        torque = self.mem[0x28] == 1
        if self.mem[0x21] == 0:
            goal, vmax = self._u16(0x2A), max(1, self._u16(0x2E) & 0x7FFF) if torque else 0
            error = goal - self.position
            step = max(-vmax * dt, min(vmax * dt, error)) if torque else 0.
            self.position += step
            self.speed = step / dt if dt > 0 else 0.
        else:
            self.speed = st3215.decode_sign_magnitude(self._u16(0x2E), 15) if torque else 0.
            self.position = (self.position + self.speed * dt) % 4096
        self._sync()

    def _sync(self):
        self.mem[0x38:0x3A] = int(round(self.position)).to_bytes(2, 'little')
        self.mem[0x3A:0x3C] = st3215.encode_speed(int(round(self.speed))).to_bytes(2, 'little')
        self.mem[0x42] = 1 if abs(self.speed) > 1 else 0


class FakeTransport:
    def __init__(self):
        self.servos = {i: FakeServo(i, m, START[i]) for i, m in ROLE_MODE.items()}
        self.lock = threading.Lock()
        self.pending = b''

    def reset_input_buffer(self):
        self.pending = b''

    def write(self, frame):
        frame = bytes(frame)
        if frame[:2] != b'\xff\xff' or st3215.checksum(frame[2:-1]) != frame[-1]:
            return
        motor_id, instruction, params = frame[2], frame[4], frame[5:-1]
        servo = self.servos.get(motor_id)
        if servo is None or servo.silent:
            return
        with self.lock:
            servo.update()
            if instruction == st3215.INST_READ:
                address, size = params
                payload = bytes(servo.mem[address:address + size])
            elif instruction == st3215.INST_WRITE:
                address = params[0]
                servo.mem[address:address + len(params) - 1] = params[1:]
                if address in (st3215.GOAL_POSITION, st3215.GOAL_SPEED):
                    servo.mem[st3215.TORQUE_ENABLE] = 1  # as the real ST3215: a goal write energizes
                payload = b''
            else:
                return
        body = bytes([motor_id, len(payload) + 2, 0]) + payload
        self.pending = b'\xff\xff' + body + bytes([st3215.checksum(body)])

    def read(self, size):
        data, self.pending = self.pending[:size], self.pending[size:]
        return data

    def close(self):
        pass
