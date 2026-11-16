"""Serial protocol used by the ROS 2 base controller for STS wheel servos."""

import serial


class Bus:
    def __init__(self, port):
        self.serial = serial.Serial(port, 1_000_000, timeout=0.1, write_timeout=1, exclusive=True)

    def packet(self, motor, instruction, args, size=0):
        body = [motor, len(args) + 2, instruction, *args]
        self.serial.reset_input_buffer()
        self.serial.write(bytes([255, 255, *body, (~sum(body)) & 255]))
        reply = self.serial.read(size + 6)
        if (
            len(reply) != size + 6
            or reply[:2] != b"\xff\xff"
            or reply[2] != motor
            or reply[3] != size + 2
            or sum(reply[2:]) % 256 != 255
            or reply[4] != 0
        ):
            raise RuntimeError(f"Motor {motor}: invalid/error response {reply.hex()}")
        return reply[5:-1]

    def read(self, motor, address, size):
        return int.from_bytes(self.packet(motor, 2, [address, size], size), "little")

    def write(self, motor, address, value, size=1):
        self.packet(motor, 3, [address, *value.to_bytes(size, "little")])
