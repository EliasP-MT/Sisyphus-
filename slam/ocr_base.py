"""Base-camera PP-OCRv5 Mobile OCR; no image processing outside PaddleOCR.

Reuse BaseOCR (or read_recipient_name) to avoid reloading models per image.
Call prepare() before latency-sensitive requests. Assumes upright Latin-script
postal addresses with name, street, then postal-code/city lines. Returns '' when
no supported address is found. Recipient selection uses conventional envelope
layout (recipient below/right of sender); it is not a semantic guarantee.
"""

import argparse
import re
import sys
import threading
import time
from dataclasses import dataclass
from functools import lru_cache

# French/continental numeric codes, US ZIP, UK and Canadian postcodes.
_POSTCODE = re.compile(
    r"(?:\b\d{4,6}(?:-\d{4})?\b|\b[A-Z]{1,2}\d[A-Z\d]?\s*\d[A-Z]{2}\b|"
    r"\b[A-Z]\d[A-Z]\s*\d[A-Z]\d\b)",
    re.IGNORECASE,
)
_STREET = re.compile(
    r"\b(?:rue|avenue|av|boulevard|bd|chemin|route|impasse|allée|allee|place|"
    r"street|st|road|rd|lane|ln|drive|way|court|square|strasse|straße|via|calle|"
    r"bp|cedex|po\s*box)\b",
    re.IGNORECASE,
)
_LABEL = re.compile(r"^(?:to|recipient|destinataire|à|a)\s*:?$", re.IGNORECASE)


@dataclass(frozen=True)
class _Line:
    text: str
    x: float
    y: float
    right: float
    bottom: float

    @property
    def height(self):
        return max(1.0, self.bottom - self.y)


def _lines(result, min_confidence):
    """Merge nearby boxes on the same baseline (e.g. split first/last names)."""
    parts = []
    for text, score, polygon in zip(result["rec_texts"], result["rec_scores"], result["rec_polys"]):
        text = text.strip()
        if not text or float(score) < min_confidence:
            continue
        xs, ys = zip(*polygon)
        parts.append(_Line(text, float(min(xs)), float(min(ys)), float(max(xs)), float(max(ys))))
    rows = []
    for part in sorted(parts, key=lambda p: (p.y, p.x)):
        for i, row in enumerate(rows):
            overlap = min(row.bottom, part.bottom) - max(row.y, part.y)
            gap = max(row.x, part.x) - min(row.right, part.right)
            if overlap >= 0.5 * min(row.height, part.height) and gap <= 2 * max(
                row.height, part.height
            ):
                pieces = sorted((row, part), key=lambda p: p.x)
                rows[i] = _Line(
                    " ".join(p.text for p in pieces),
                    min(row.x, part.x),
                    min(row.y, part.y),
                    max(row.right, part.right),
                    max(row.bottom, part.bottom),
                )
                break
        else:
            rows.append(part)
    return sorted(rows, key=lambda p: (p.y, p.x))


def _previous(line, lines):
    candidates = [
        p
        for p in lines
        if p.bottom <= line.y + 0.25 * line.height
        and p.y < line.y
        and line.y - p.bottom <= 2.5 * max(p.height, line.height)
        and abs(p.x - line.x) <= 2 * max(p.height, line.height)
    ]
    return max(candidates, key=lambda p: p.y, default=None)


def _recipient_name(result, width, height, min_confidence=0.6):
    lines = _lines(result, min_confidence)
    candidates = []
    for postal in lines:
        if not _POSTCODE.search(postal.text):
            continue
        # Require a street above the postal line, then the name above that.
        street = _previous(postal, lines)
        if street is None or not (
            _STREET.search(street.text) or re.match(r"^\d+\s+\D", street.text)
        ):
            continue
        name = _previous(street, lines)
        # Permit one extra street/building line between name and street.
        if name is not None and (_STREET.search(name.text) or re.match(r"^\d", name.text)):
            name = _previous(name, lines)
        if name is None or not any(c.isalpha() for c in name.text):
            continue
        if _LABEL.fullmatch(name.text) or _POSTCODE.search(name.text) or _STREET.search(name.text):
            continue
        # Conventional recipient block is farther right/down than the return address.
        rank = name.x / width + name.y / height
        candidates.append((rank, name.text))
    candidates.sort(reverse=True)
    if not candidates or (len(candidates) > 1 and candidates[0][0] - candidates[1][0] < 0.1):
        return ""
    return candidates[0][1]


class _CameraStream:
    """Continuously drain one open camera; keep only its latest full frame."""

    def __init__(self, device, target):
        import cv2

        self.condition = threading.Condition()
        self.stop = threading.Event()
        self.frame = None
        self.received = 0.0
        self.error = None
        self.capture = cv2.VideoCapture(
            device, cv2.CAP_AVFOUNDATION if target == "mac" else cv2.CAP_V4L2
        )
        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError(f"Cannot open base camera {device}")
        try:
            if target == "robot":
                self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.capture.set(cv2.CAP_PROP_FPS, 15)
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
        except Exception:
            self.capture.release()
            raise

    def _run(self):
        try:
            while not self.stop.is_set():
                ok, frame = self.capture.read()
                if not ok:
                    raise RuntimeError("Base camera capture failed")
                with self.condition:
                    self.frame = frame
                    self.received = time.monotonic()
                    self.condition.notify_all()
        except Exception as exc:
            with self.condition:
                self.error = str(exc)
                self.condition.notify_all()
        finally:
            self.capture.release()

    def snapshot(self):
        with self.condition:
            ready = self.condition.wait_for(
                lambda: (
                    self.error
                    or self.stop.is_set()
                    or (self.frame is not None and time.monotonic() - self.received < 0.5)
                ),
                timeout=5,
            )
            if not ready or self.error or self.stop.is_set():
                raise RuntimeError(self.error or "No recent base-camera frame")
            # A copy protects the raw frame from overlays/caller mutation.
            return self.frame.copy()

    def close(self):
        self.stop.set()
        with self.condition:
            self.condition.notify_all()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            # Unblock a backend stuck in read during shutdown.
            self.capture.release()
            self.thread.join(timeout=2)


class BaseOCR:
    """Persistent CPU OCR with base-camera capture. No RDK BPU backend.

    max_seconds includes camera capture, OCR and address selection, excludes
    construction/prepare. A slow request raises TimeoutError after inference;
    this checks the budget, it cannot interrupt native Paddle inference.
    Use a stable /dev/v4l/by-id path if USB camera numbering can change.
    """

    def __init__(
        self,
        device=None,
        *,
        target="robot",
        cpu_threads=4,
        min_confidence=0.6,
        max_seconds=1.0,
        det_model_dir=None,
        rec_model_dir=None,
    ):
        if target not in ("auto", "robot", "mac"):
            raise ValueError("target must be auto, robot, or mac")
        self.target = (
            ("mac" if sys.platform == "darwin" else "robot") if target == "auto" else target
        )
        if self.target == "mac":
            device = 0 if device is None else device
            if isinstance(device, str) and device.isdecimal():
                device = int(device)
            if isinstance(device, bool) or not isinstance(device, int) or device < 0:
                raise ValueError("On macOS, device must be a nonnegative camera index, e.g. 0 or 1")
        elif device is None:
            device = "/dev/video0"
        if max_seconds <= 0 or not 0 <= min_confidence <= 1:
            raise ValueError("max_seconds must be positive; confidence must be in [0, 1]")
        from paddleocr import PaddleOCR

        self.ocr = PaddleOCR(
            text_detection_model_name="PP-OCRv5_mobile_det",
            text_recognition_model_name="PP-OCRv5_mobile_rec",
            text_detection_model_dir=det_model_dir,
            text_recognition_model_dir=rec_model_dir,
            use_doc_orientation_classify=False,
            use_doc_unwarping=False,
            use_textline_orientation=False,
            device="cpu",
            enable_mkldnn=False,
            cpu_threads=cpu_threads,
            text_recognition_batch_size=8,
        )
        self.device = device
        self.min_confidence = min_confidence
        self.max_seconds = max_seconds
        self.last_elapsed_seconds = None
        self.last_timings = {}
        self.last_result = None
        self.last_frame = None
        self.last_name = ""
        self.last_output = None
        self.last_camera_open_seconds = 0.0
        self._camera = None
        self._lock = threading.Lock()

    def _capture(self):
        self.last_camera_open_seconds = 0.0
        if self._camera is None:
            start = time.perf_counter()
            self._camera = _CameraStream(self.device, self.target)
            self.last_camera_open_seconds = time.perf_counter() - start
        return self._camera.snapshot()

    def _read(self, image):
        self.last_output = None
        self.last_result = None
        self.last_frame = None
        self.last_name = ""
        self.last_timings = {}
        self.last_camera_open_seconds = 0.0
        acquisition_start = time.perf_counter()
        if image is None:
            frame = self._capture()
        elif isinstance(image, str) or hasattr(image, "__fspath__"):
            import cv2

            frame = cv2.imread(str(image))
            if frame is None:
                raise ValueError(f"Cannot read image: {image}")
        else:
            frame = image
        if getattr(frame, "ndim", 0) != 3 or frame.shape[2] != 3:
            raise ValueError("Expected an unprocessed OpenCV BGR image")
        acquisition_end = time.perf_counter()
        self.last_frame = frame
        self.last_timings["camera_open_s"] = self.last_camera_open_seconds
        self.last_timings["capture_or_load_s"] = max(
            0.0, acquisition_end - acquisition_start - self.last_camera_open_seconds
        )
        # No crop, resize, threshold, or additional model here.
        results = list(self.ocr.predict(frame))
        ocr_end = time.perf_counter()
        self.last_timings["ocr_s"] = ocr_end - acquisition_end
        self.last_result = results[0] if results else None
        self.last_output = self._extract_result(self.last_result, frame.shape[1], frame.shape[0])
        self.last_name = self.last_output if isinstance(self.last_output, str) else ""
        self.last_timings["selection_s"] = time.perf_counter() - ocr_end
        return self.last_output

    def _extract_result(self, result, width, height):
        return "" if result is None else _recipient_name(result, width, height, self.min_confidence)

    def prepare(self):
        """Warm models using one real full-frame base-camera capture."""
        with self._lock:
            self._read(None)

    def read_name(self, image=None):
        """Capture the base camera (default), or read a path/BGR image; return str.

        '' means no confident supported address. Hardware/runtime failures and
        requests exceeding the latency budget raise exceptions, never a name.
        """
        with self._lock:
            start = time.perf_counter()
            try:
                name = self._read(image)
            finally:
                self.last_elapsed_seconds = time.perf_counter() - start
                self.last_timings["total_s"] = self.last_elapsed_seconds
            if self.last_elapsed_seconds >= self.max_seconds:
                raise TimeoutError(
                    f"OCR took {self.last_elapsed_seconds:.3f}s; budget is <{self.max_seconds:.3f}s"
                )
            return name

    def close(self):
        with self._lock:
            if self._camera is not None:
                self._camera.close()
                self._camera = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


@lru_cache(maxsize=1)
def _default_reader(target="robot", device=None):
    return BaseOCR(target=target, device=device)


def read_recipient_name(image=None, *, target="robot", device=None) -> str:
    """Public convenience function; caches models. Default input is base camera.

    Defaults to the robot backend. Explicit target='auto' selects by OS.
    On Mac, device is a
    camera index; on the robot it is a device path. Select the physical base/USB
    camera explicitly: indices do not identify whether a camera is built-in.
    For explicit warmup, timing or cleanup use BaseOCR.
    """
    return _default_reader(target, device).read_name(image)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target",
        choices=("auto", "robot", "mac"),
        default="robot",
        help="Run locally with robot/Linux or macOS capture (default: robot)",
    )
    parser.add_argument(
        "--device",
        help="Robot base-camera path (default /dev/video0); "
        "Mac USB-camera index (default 0; verify which camera it selects)",
    )
    parser.add_argument("--image", help="Use a saved full image instead of camera")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--test",
        action="store_true",
        help="Visualize continuous live OCR in the browser; otherwise run headlessly",
    )
    mode.add_argument("--once", action="store_true", help="Read one name and exit (the default)")
    parser.add_argument("--port", type=int, default=8768, help="Loopback preview HTTP port")
    parser.add_argument(
        "--benchmark",
        type=int,
        default=0,
        metavar="N",
        help="Warm up then measure N calls; timings go to stderr",
    )
    args = parser.parse_args(argv)
    if args.benchmark < 0:
        parser.error("--benchmark must be nonnegative")
    if args.test and (args.image or args.benchmark):
        parser.error("--test cannot be combined with --image or --benchmark")
    if args.once and args.benchmark:
        parser.error("--once cannot be combined with --benchmark")
    with BaseOCR(device=args.device, target=args.target) as reader:
        if args.test:
            from ocr_preview import run_preview

            try:
                run_preview(reader, port=args.port)
            except KeyboardInterrupt:
                pass
        elif args.benchmark:
            # Warm up on the same source being measured.
            reader._read(args.image)
            for _ in range(args.benchmark):
                try:
                    print(reader.read_name(args.image))
                except TimeoutError as exc:
                    print(str(exc), file=sys.stderr)
                print(
                    " ".join(f"{key}={value:.3f}" for key, value in reader.last_timings.items()),
                    file=sys.stderr,
                )
        else:
            print(reader.read_name(args.image))


if __name__ == "__main__":
    main()
