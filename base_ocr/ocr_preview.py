"""Shared loopback browser preview for ocr_base.py and ocr_arm.py --test.

Only completed OCR frames are streamed. Drawing never changes the OCR input.
No display server, GUI OpenCV build, or additional model is required on the RDK.
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

PAGE = b"""<!doctype html><html><head><meta name="viewport" content="width=device-width">
<title>Live OCR</title><style>
body{font:17px system-ui;background:#151820;color:#eee;margin:20px}
img{width:960px;max-width:100%}pre{white-space:pre-wrap}
</style></head><body><h1>Live OCR at inference speed</h1>
<img src="/stream" alt="Waiting for first completed OCR frame">
<p>Every displayed frame has completed OCR. Green: accepted confidence;
orange: low confidence. Full text and positions appear below.</p>
<pre id="status">Warming up...</pre><script>
async function update(){try{const response=await fetch('/status');
document.getElementById('status').textContent=JSON.stringify(await response.json(),null,2);
}catch(e){document.getElementById('status').textContent='Preview connection lost';}
setTimeout(update,400);}update();</script></body></html>"""


def _detections(reader):
    if hasattr(reader, "last_detections"):
        return reader.last_detections  # Arm: already merged into name regions.
    result = reader.last_result
    if result is None:
        return []
    detections = []
    for text, score, polygon in zip(result["rec_texts"], result["rec_scores"], result["rec_polys"]):
        points = np.asarray(polygon, dtype=float).reshape(-1, 2)
        low, high = points.min(axis=0), points.max(axis=0)
        detections.append(
            dict(
                text=str(text),
                confidence=float(score),
                polygon=points.tolist(),
                bbox=[*low.tolist(), *high.tolist()],
                center=((low + high) / 2).tolist(),
            )
        )
    return detections


def annotate(frame, detections, min_confidence=0.0):
    """Draw text regions on a copy; Unicode strings are also shown in HTML JSON."""
    output = frame.copy()
    for item in detections:
        points = np.asarray(item["polygon"], dtype=np.int32).reshape(-1, 2)
        color = (0, 220, 0) if item["confidence"] >= min_confidence else (0, 160, 255)
        cv2.polylines(output, [points], True, color, 2)
        x, y = points.min(axis=0)
        cv2.putText(
            output,
            f"{item['text']} {item['confidence']:.2f}",
            (int(x), max(15, int(y) - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )
    return output


def _process_frame(reader):
    """Run exactly one OCR request, retaining useful over-budget results."""
    started = time.perf_counter()
    over_budget = False
    read = getattr(reader, "read_text", None) or reader.read_name
    try:
        read()
    except TimeoutError:
        over_budget = True
    if reader.last_frame is None:
        raise RuntimeError("OCR returned no processed image")
    detections = _detections(reader)
    frame = annotate(reader.last_frame, detections, reader.min_confidence)
    cv2.putText(
        frame,
        f"OCR {reader.last_timings['ocr_s']:.3f}s | total {reader.last_elapsed_seconds:.3f}s",
        (10, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )
    ok, jpeg = cv2.imencode(".jpg", frame)
    if not ok:
        raise RuntimeError("Cannot encode OCR preview frame")
    cycle = time.perf_counter() - started
    status = dict(
        state="ready",
        mode="arm" if hasattr(reader, "last_detections") else "base",
        fps=1 / max(cycle, 1e-9),
        frame_cycle_s=cycle,
        timings=dict(reader.last_timings),
        over_budget=over_budget,
        budget_s=reader.max_seconds,
        detections=detections,
    )
    if status["mode"] == "base":
        status["recipient_name"] = reader.last_name
    return jpeg.tobytes(), status


def run_preview(reader, port=8768):
    """Serve a BaseOCR/ArmOCR reader on localhost until Ctrl+C.

    The caller owns the reader and closes it afterward. Camera startup happens
    once before the loop; first inference is marked warmup. No inference queue.
    """
    condition = threading.Condition()
    stopped = threading.Event()
    state = dict(sequence=0, jpeg=None, completed=None, status={"state": "warming up"})

    def process():
        iteration = 0
        while not stopped.is_set():
            try:
                jpeg, status = _process_frame(reader)
                iteration += 1
                status.update(
                    iteration=iteration, warmup=iteration == 1, camera_startup_s=camera_startup
                )
                with condition:
                    state.update(
                        sequence=iteration, jpeg=jpeg, status=status, completed=time.monotonic()
                    )
                    condition.notify_all()
                print(json.dumps(status, ensure_ascii=False), flush=True)
            except Exception as exc:
                with condition:
                    state.update(
                        jpeg=None, completed=None, status={"state": "error", "error": str(exc)}
                    )
                    stopped.set()
                    condition.notify_all()
                print(f"OCR preview error: {exc}", flush=True)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                previous = 0
                try:
                    while not stopped.is_set():
                        with condition:
                            ready = condition.wait_for(
                                lambda: stopped.is_set() or state["sequence"] != previous, timeout=1
                            )
                            if stopped.is_set():
                                return
                            if not ready:
                                continue
                            previous, jpeg = state["sequence"], state["jpeg"]
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + str(len(jpeg)).encode()
                            + b"\r\n\r\n"
                            + jpeg
                            + b"\r\n"
                        )
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
            if path == "/":
                data, content_type = PAGE, "text/html; charset=utf-8"
            elif path == "/status":
                with condition:
                    status = dict(state["status"])
                    status["result_age_s"] = (
                        None
                        if state["completed"] is None
                        else round(time.monotonic() - state["completed"], 2)
                    )
                data, content_type = (
                    json.dumps(status, ensure_ascii=False).encode(),
                    "application/json",
                )
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_):
            pass

    # Bind before opening the camera so a port conflict does not start capture.
    with ThreadingHTTPServer(("127.0.0.1", port), Handler) as server:
        start = time.perf_counter()
        reader._capture()
        camera_startup = time.perf_counter() - start
        worker = threading.Thread(target=process, daemon=True)
        worker.start()
        print(f"OCR preview: http://127.0.0.1:{port} (Ctrl+C to stop)", flush=True)
        try:
            server.serve_forever(poll_interval=0.2)
        finally:
            stopped.set()
            with condition:
                condition.notify_all()
            # Finish in-flight inference before the caller releases the reader.
            worker.join()
