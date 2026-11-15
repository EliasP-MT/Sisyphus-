#!/usr/bin/env python3
"""Read-only arm telemetry and 3D viewer. Never writes servo registers."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--http-port', type=int, default=8890)
    args = parser.parse_args()
    bus = serial.Serial(args.port, 1_000_000, timeout=.025,
                        write_timeout=.5, exclusive=True)
    state = {'timestamp': 0, 'motors': {}, 'errors': {}}
    lock = threading.Lock()
    stop = threading.Event()

    def poll():
        while not stop.is_set():
            motors, errors = {}, {}
            for motor in range(1, 9):
                # READ present-position register, never torque/EEPROM/goal writes.
                body = [motor, 4, 2, 56, 2]
                try:
                    bus.reset_input_buffer()
                    bus.write(bytes([255, 255, *body, (~sum(body)) & 255]))
                    reply = bus.read(8)
                    if (len(reply) != 8 or reply[:2] != b'\xff\xff'
                            or reply[2:5] != bytes([motor, 4, 0])
                            or sum(reply[2:]) % 256 != 255):
                        raise ValueError('Invalid response: ' + reply.hex())
                    value = int.from_bytes(reply[5:7], 'little')
                    motors[motor] = ((4096 - (value & 4095)) & 4095
                                     if value & 32768 else value & 4095)
                except (OSError, ValueError, serial.SerialException) as exc:
                    errors[motor] = str(exc)
            with lock:
                state.update(timestamp=time.time(), motors=motors, errors=errors)
            stop.wait(.1)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/state':
                with lock:
                    data = json.dumps(state).encode()
                mime = 'application/json'
            elif self.path in ('/', '/index.html'):
                data = Path(__file__).with_name('arm-view.html').read_bytes()
                mime = 'text/html; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *_):
            pass

    threading.Thread(target=poll, daemon=True).start()
    server = ThreadingHTTPServer(('127.0.0.1', args.http_port), Handler)
    print(f'Read-only arm view on 127.0.0.1:{args.http_port}', flush=True)
    try:
        server.serve_forever()
    finally:
        stop.set()
        server.server_close()


if __name__ == '__main__':
    main()
