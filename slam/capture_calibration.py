"""Local webcam calibration capture page; saves unannotated chessboard images."""

import argparse
import json
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2

PAGE = """<!doctype html><html><head><meta name="viewport" content="width=device-width">
<title>Camera calibration capture</title><style>
body{font:18px system-ui;background:#151820;color:#eee;margin:24px;max-width:900px}
img{width:640px;max-width:100%;border:1px solid #555}button{font:inherit;padding:12px;margin:8px 8px 8px 0}
#status{white-space:pre-wrap}small{color:#bbb}</style></head><body>
<h1>Camera calibration</h1><p>9 across &times; 6 down <b>inner corners</b>.
Keep the whole board visible. Move it between captures: centre, edges, near/far, tilt left/right/up/down.</p>
<img src="/stream"><div><button id="capture">Capture (Space)</button></div>
<p id="status">Connecting...</p><p id="result"></p>
<small>Collect 20&ndash;25 sharp, varied images. Only frames with all chessboard corners are saved.
The preview displays corners; saved images contain the original camera pixels.</small>
<script>
async function update(){try{let s=await(await fetch('/status')).json();
document.querySelector('#status').textContent=`Saved: ${s.saved} / 25\n${s.found?'Grid detected — ready to capture':'Grid not detected — show the entire board clearly'}\n${s.error||''}`;}catch(e){}}
async function capture(){let r=await fetch('/capture',{method:'POST'});let s=await r.json();document.querySelector('#result').textContent=s.message;update();}
document.querySelector('#capture').onclick=capture;
document.addEventListener('keydown',e=>{if(e.code==='Space'){e.preventDefault();capture();}});
setInterval(update,500);update();</script></body></html>""".encode("utf-8")


class Capture:
    def __init__(self, device, directory, columns, rows):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.pattern = (columns, rows)
        self.lock = threading.Condition()
        self.frame = self.jpeg = None
        self.found = False
        self.error = None
        self.timestamp = 0
        self.sequence = 0
        self.stop = threading.Event()
        if isinstance(device, str) and device.isdecimal():
            device = int(device)
        backend = cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_V4L2
        self.capture = cv2.VideoCapture(device, backend)
        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError(
                f"Cannot open camera {device}; check device selection and camera permissions"
            )
        if sys.platform != "darwin":
            self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.capture.set(cv2.CAP_PROP_FPS, 15)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        try:
            while not self.stop.is_set():
                ok, frame = self.capture.read()
                if not ok:
                    raise RuntimeError("Camera read failed")
                if frame.shape[:2] != (480, 640):
                    raise RuntimeError(f"Expected 640x480 camera output, got {frame.shape}")
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                found, corners = cv2.findChessboardCorners(
                    gray,
                    self.pattern,
                    cv2.CALIB_CB_ADAPTIVE_THRESH
                    | cv2.CALIB_CB_NORMALIZE_IMAGE
                    | cv2.CALIB_CB_FAST_CHECK,
                )
                preview = frame.copy()
                if found:
                    cv2.drawChessboardCorners(preview, self.pattern, corners, found)
                cv2.putText(
                    preview,
                    "GRID DETECTED" if found else "Show full 9x6 inner-corner grid",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0) if found else (0, 180, 255),
                    2,
                )
                ok, jpg = cv2.imencode(".jpg", preview)
                if not ok:
                    raise RuntimeError("JPEG encoding failed")
                with self.lock:
                    self.frame, self.jpeg, self.found = frame, jpg.tobytes(), found
                    self.timestamp = time.monotonic()
                    self.sequence += 1
                    self.lock.notify_all()
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
                self.lock.notify_all()
        finally:
            self.capture.release()

    def save(self):
        with self.lock:
            if self.error or not self.found or time.monotonic() - self.timestamp > 2:
                return False, self.error or "No recent complete grid detected; image not saved."
            frame = self.frame.copy()
        path = self.directory / f"grid-{time.time_ns()}.png"
        if not cv2.imwrite(str(path), frame):
            return False, "Failed to save image"
        print(f"Saved {path}", flush=True)
        return True, f"Saved {path.name}. Move or tilt the board before the next capture."

    def close(self):
        self.stop.set()
        with self.lock:
            self.lock.notify_all()
        self.thread.join(timeout=3)


def handler(camera):
    class Handler(BaseHTTPRequestHandler):
        def send_json(self, value, status=200):
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(value).encode())

        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(PAGE)
            elif self.path == "/status":
                with camera.lock:
                    data = dict(
                        saved=len(list(camera.directory.glob("grid-*.png"))),
                        found=camera.found and time.monotonic() - camera.timestamp < 2,
                        error=camera.error,
                    )
                self.send_json(data)
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                sequence = -1
                try:
                    while not camera.stop.is_set():
                        with camera.lock:
                            camera.lock.wait_for(
                                lambda: camera.sequence != sequence or camera.error, timeout=5
                            )
                            if camera.error:
                                break
                            sequence, jpeg = camera.sequence, camera.jpeg
                        if jpeg:
                            self.wfile.write(
                                b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
                            )
                            self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path != "/capture":
                self.send_error(404)
                return
            ok, message = camera.save()
            self.send_json(dict(message=message), 200 if ok else 409)

        def log_message(self, *_):
            pass

    return Handler


def preview_cameras(count=4):
    """Show one labeled snapshot per available index; never save images."""
    backend = cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_V4L2
    previews = []
    for index in range(count):
        capture = cv2.VideoCapture(index, backend)
        try:
            if not capture.isOpened():
                continue
            frame = None
            # Allow a few frames for camera startup/exposure.
            for _ in range(5):
                ok, candidate = capture.read()
                if ok:
                    frame = candidate
            if frame is None:
                continue
            previews.append((index, frame))
            print(f"Camera index {index}: {frame.shape[1]}x{frame.shape[0]}", flush=True)
        finally:
            capture.release()
    if not previews:
        raise RuntimeError(
            "No cameras returned images; check USB connection and camera permissions"
        )
    print(
        "Find the USB camera in the labeled windows. Press any key in a preview to close.",
        flush=True,
    )
    try:
        for index, frame in previews:
            cv2.imshow(f"Camera index {index} (use --device {index})", frame)
        cv2.waitKey(0)
    finally:
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--device",
        default="0" if sys.platform == "darwin" else "/dev/video2",
        help="Camera index on macOS (e.g. 0 or 1); device path on Linux",
    )
    parser.add_argument("--output", default="calibration-images")
    parser.add_argument(
        "--list-cameras",
        action="store_true",
        help="Preview camera indices without saving images, then exit",
    )
    parser.add_argument(
        "--scan-count",
        type=int,
        default=4,
        help="Number of indices to preview with --list-cameras (default: 4)",
    )
    args = parser.parse_args()
    if not 1 <= args.scan_count <= 16:
        parser.error("--scan-count must be between 1 and 16")
    if args.list_cameras:
        preview_cameras(args.scan_count)
        return
    camera = Capture(args.device, args.output, 9, 6)
    try:
        with ThreadingHTTPServer(("127.0.0.1", 8767), handler(camera)) as server:
            print("Calibration capture: http://127.0.0.1:8767", flush=True)
            server.serve_forever()
    finally:
        camera.close()


if __name__ == "__main__":

    def terminate(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    try:
        main()
    except KeyboardInterrupt:
        pass
