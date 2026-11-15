#!/usr/bin/env python3
"""OCR worker for the station: reads the recipient name from one camera frame.

Runs in the base_ocr virtualenv (PaddleOCR; not available in the ROS container) and
serves 127.0.0.1 only. The station sends one JPEG frame; the function
base_ocr/ocr_base.py:read_recipient_name does the reading (frame= mode: no camera is
opened here, the camera stays owned by the robot).

    base_ocr/.venv/bin/python ros2_robot/postal_station/ocr_worker/worker.py [--port 8894]
"""
import argparse
import base64
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

BASE_OCR = Path(__file__).resolve().parents[3] / 'base_ocr'
sys.path.insert(0, str(BASE_OCR))
import ocr_base  # noqa: E402

STATE = {'ready': False, 'reason': 'loading the OCR model'}


def warm_up():
    try:
        ocr_base._engine()
        STATE.update(ready=True, reason='')
    except Exception as exc:  # model download/initialisation failure is reported, not hidden
        STATE.update(ready=False, reason=f'OCR model unavailable: {exc}')


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, *args):
        pass

    def send(self, code, body):
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        self.send(200 if self.path == '/status' else 404, dict(STATE) if self.path == '/status' else {'error': 'not found'})

    def do_POST(self):
        if self.path != '/read':
            return self.send(404, {'error': 'not found'})
        try:
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length))
            raw = base64.b64decode(body['jpeg_b64'], validate=True)
            frame = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError('undecodable JPEG frame')
            began = time.monotonic()
            name = ocr_base.read_recipient_name(frame=frame)
            self.send(200, {'success': True, 'name': name, 'seconds': round(time.monotonic() - began, 2),
                            'size': [int(frame.shape[1]), int(frame.shape[0])]})
        except Exception as exc:
            self.send(200, {'success': False, 'message': f'{type(exc).__name__}: {exc}'})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8894)
    args = parser.parse_args()
    threading.Thread(target=warm_up, daemon=True).start()
    ThreadingHTTPServer(('127.0.0.1', args.port), Handler).serve_forever()


if __name__ == '__main__':
    main()
