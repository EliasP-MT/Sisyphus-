"""Exclusive ST3215 serial transport. Invalid replies never become zero positions.

Writes are restricted to an explicit set of servo IDs and to the RAM registers
in st3215.WRITABLE. Broadcast and EEPROM writes are refused by construction.
"""
import threading
import time

from . import st3215
from .st3215 import ProtocolError, decode_reply, packet


class BusError(RuntimeError):
    pass


class Bus:
    def __init__(self, device, writable_ids, timeout=.02, baudrate=1000000, transport=None):
        if transport is None:
            import serial
            transport = serial.Serial(device, baudrate, timeout=timeout,
                                      write_timeout=timeout, exclusive=True)
        self.serial = transport
        self.writable_ids = frozenset(writable_ids)
        if st3215.BROADCAST_ID in self.writable_ids or not all(1 <= i <= 253 for i in self.writable_ids):
            raise ValueError('Writable IDs must be explicit unicast IDs')
        self.reply_to_writes = {}
        self.lock = threading.Lock()

    def _transact(self, frame, reply_size, motor_id):
        self.serial.reset_input_buffer()
        self.serial.write(frame)
        if reply_size is None:
            return None, 0
        raw = self.serial.read(reply_size + 6)
        try:
            return decode_reply(raw, motor_id, reply_size)
        except ProtocolError as exc:
            raise BusError(str(exc)) from exc

    def read(self, motor_id, address, size):
        """Return (payload, packet_status, monotonic_receive_time)."""
        if motor_id == st3215.BROADCAST_ID:
            raise BusError('Broadcast reads are not supported')
        with self.lock:
            payload, status = self._transact(packet(motor_id, st3215.INST_READ, [address, size]),
                                             size, motor_id)
            return payload, status, time.monotonic()

    def write(self, motor_id, address, value):
        """Write one whitelisted RAM register of one servo. Returns packet status."""
        if motor_id not in self.writable_ids:
            raise BusError(f'Writes to ID {motor_id} are not permitted')
        size = st3215.WRITABLE.get(address)
        if size is None:
            raise BusError(f'Register 0x{address:02x} is not writable by this software')
        value = int(value)
        if not 0 <= value < (1 << (8 * size)):
            raise BusError('Register value out of range')
        data = list(value.to_bytes(size, 'little'))
        replies = self.reply_to_writes.get(motor_id, True)
        with self.lock:
            _, status = self._transact(packet(motor_id, st3215.INST_WRITE, [address] + data),
                                       0 if replies else None, motor_id)
            return status

    def close(self):
        self.serial.close()
