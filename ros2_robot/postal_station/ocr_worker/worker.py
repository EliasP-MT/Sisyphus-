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
MAILBOX_VISION = Path(__file__).resolve().parents[3] / 'mailbox_vision'
sys.path.insert(0, str(BASE_OCR))
import ocr_base  # noqa: E402

sys.path.append(str(MAILBOX_VISION))
from recipient_names import match_recipient  # noqa: E402

_LOCATOR = {}


def mailbox_detector():
    """LetterboxLocator for the arm camera (camera 2), geometry only (no OCR)."""
    if not _LOCATOR:
        import letterbox_locator as ll
        calibration = ll.load_json(MAILBOX_VISION / 'config/arm-camera-intrinsics.json')
        geometry = ll.load_json(MAILBOX_VISION / 'config/letterbox.json')
        _LOCATOR.update(locator=ll.LetterboxLocator(calibration, geometry), ll=ll,
                        calibration=calibration, geometry=geometry)
    return _LOCATOR


def read_label_text(crop):
    """OCR of one rectified label interior: all text lines, top to bottom, joined."""
    with ocr_base._OCR_LOCK:
        results = list(ocr_base._engine().predict(crop))
    if not results:
        return ''
    res = results[0]
    lines = []
    for text, score, poly in zip(res['rec_texts'], res['rec_scores'], res['rec_polys']):
        if text.strip() and float(score) >= 0.3:
            pts = np.asarray(poly, float).reshape(-1, 2)
            lines.append((pts[:, 1].mean(), pts[:, 0].mean(), text.strip()))
    lines.sort()
    return ' '.join(t for _, _, t in lines)


def detect_mailboxes(frame, read_names=True):
    """Detect every mailbox label in one frame (+ read its name): contours drawn + a list."""
    d = mailbox_detector()
    results = d['locator'].locate_all(frame)
    names = json.loads(ocr_base.NAMES_PATH.read_text()) if read_names else []
    drawn = frame.copy()
    labels = []
    for i, r in enumerate(results, 1):
        quarter_turns, ocr_text, name, score = 0, '', None, 0.0
        if read_names:
            # The detector orders the corners by position in the image, i.e. it assumes an upright
            # label. With the wrist rolled, the label is rotated by a multiple of 90 degrees: the turn
            # that makes the name readable gives the label's own frame (and so where its slot is).
            base_corners = np.asarray(r['label_corners_px'], np.float32)
            first_text = ''
            for turns in range(4):
                turned = np.roll(base_corners, -turns, axis=0)
                text = read_label_text(d['ll'].rectify_label(frame, turned, d['geometry']))
                if turns == 0:
                    first_text = text
                found, found_score = match_recipient(text, names)
                if found:
                    quarter_turns, ocr_text, name, score = turns, text, found, found_score
                    break
            else:
                ocr_text = first_text
            if quarter_turns:
                poses = d['ll'].label_pose_candidates(np.roll(base_corners, -quarter_turns, axis=0),
                                                      d['calibration'], d['geometry'])
                if poses:
                    r = dict(label_corners_px=np.roll(base_corners, -quarter_turns, axis=0).tolist(),
                             label_area_px2=r['label_area_px2'], **poses[0], pose_candidates=poses)
        corners = np.int32(r['label_corners_px'])
        cv2.polylines(drawn, [corners], True, (0, 200, 0), 2)
        outline = d['ll'].slot_outline_px(r, d['calibration'], d['geometry'], frame)
        cv2.polylines(drawn, [np.int32(outline[:4])], True, (0, 0, 255), 2)
        x, y = int(corners[:, 0].min()), int(corners[:, 1].min())
        text = f'#{i} ' + (f'{name} ' if name else '') + f'{r["distance_m"] * 100:.0f} cm'
        cv2.putText(drawn, text, (x, max(14, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(drawn, text, (x, max(14, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        labels.append({'id': i, 'ocr_text': ocr_text, 'name': name, 'quarter_turns': quarter_turns,
                       'orientation_known': bool(name) or not read_names, 'match_score': round(float(score or 0), 2),
                       'area_px2': round(float(r['label_area_px2'])),
                       'distance_m': round(float(r['distance_m']), 3),
                       'center_m': [round(float(v), 3) for v in r['label_center_m']],
                       'label_rotation': [[round(float(v), 5) for v in row] for row in r['label_rotation']],
                       'slot_center_m': [round(float(v), 4) for v in r['slot_center_m']],
                       'reprojection_px': round(float(r['reprojection_px']), 2),
                       'ambiguous_poses': len(r.get('pose_candidates', [])) > 1,
                       'corners_px': [[round(float(a), 1), round(float(b), 1)] for a, b in r['label_corners_px']]})
    ok, jpeg = cv2.imencode('.jpg', drawn, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return labels, base64.b64encode(jpeg.tobytes()).decode()

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
        if self.path not in ('/read', '/regions', '/detect'):
            return self.send(404, {'error': 'not found'})
        try:
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length))
            raw = base64.b64decode(body.get('image_b64') or body['jpeg_b64'], validate=True)
            frame = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError('undecodable image')
            if self.path == '/detect':
                began = time.monotonic()
                labels, image = detect_mailboxes(frame)
                return self.send(200, {'success': True, 'labels': labels, 'image_b64': image,
                                       'geometry': mailbox_detector()['geometry'],
                                       'seconds': round(time.monotonic() - began, 3),
                                       'size': [int(frame.shape[1]), int(frame.shape[0])]})
            if self.path == '/regions':
                # Raw text regions (text, score, 4-point polygon) for the SLAM label reader.
                with ocr_base._OCR_LOCK:
                    results = list(ocr_base._engine().predict(frame))
                res = results[0] if results else {'rec_texts': [], 'rec_scores': [], 'rec_polys': []}
                return self.send(200, {'success': True, 'rec_texts': [str(t) for t in res['rec_texts']],
                                       'rec_scores': [float(x) for x in res['rec_scores']],
                                       'rec_polys': [np.asarray(p, float).reshape(-1, 2).tolist()
                                                     for p in res['rec_polys']]})
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
