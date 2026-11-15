"""Capture the sixth base-camera frame and return its first text line as a known name."""

import argparse
import json
import threading
import unicodedata
from difflib import SequenceMatcher
from functools import lru_cache
from pathlib import Path

import cv2

NAMES_PATH = Path(__file__).with_name("recipients.json")
_OCR_LOCK = threading.Lock()


@lru_cache(maxsize=1)
def _engine():
    from paddleocr import PaddleOCR

    return PaddleOCR(
        text_detection_model_name="PP-OCRv5_mobile_det",
        text_recognition_model_name="PP-OCRv5_mobile_rec",
        use_doc_orientation_classify=False,
        use_doc_unwarping=False,
        use_textline_orientation=False,
        device="cpu",
        enable_mkldnn=False,
        cpu_threads=4,
        text_recognition_batch_size=8,
    )


def _normalize(text):
    return "".join(
        c
        for c in unicodedata.normalize("NFKD", text).casefold()
        if c.isalnum() and not unicodedata.combining(c)
    )


def _known_name(text, names):
    text = _normalize(text)
    if not text:
        return ""
    scores = sorted(
        ((SequenceMatcher(None, text, _normalize(name)).ratio(), name) for name in names),
        reverse=True,
    )
    if not scores or scores[0][0] < 0.7:
        return ""
    if len(scores) > 1 and scores[0][0] - scores[1][0] < 0.08:
        return ""
    return scores[0][1]


def _first_line(result):
    # PaddleOCR result order is not an API guarantee: use original-image geometry.
    boxes = []
    for text, score, polygon in zip(result["rec_texts"], result["rec_scores"], result["rec_polys"]):
        if not text.strip():
            continue
        xs, ys = zip(*polygon)
        boxes.append(
            dict(
                text=text.strip(),
                score=float(score),
                x=float(min(xs)),
                right=float(max(xs)),
                y=float(min(ys)),
                bottom=float(max(ys)),
            )
        )
    if not boxes:
        return ""
    boxes.sort(key=lambda b: (b["y"], b["x"]))
    first = boxes.pop(0)
    height = max(1.0, first["bottom"] - first["y"])
    # Only nearby regions sharing the first line's baseline may join it.
    row = [first]
    changed = True
    while changed:
        changed = False
        for box in boxes[:]:
            h = max(1.0, box["bottom"] - box["y"])
            overlap = min(first["bottom"], box["bottom"]) - max(first["y"], box["y"])
            if overlap < 0.5 * min(height, h):
                continue
            gap = min(max(part["x"], box["x"]) - min(part["right"], box["right"]) for part in row)
            if gap <= 2 * min(height, h):
                row.append(box)
                boxes.remove(box)
                changed = True
    if any(part["score"] < 0.6 for part in row):
        return ""  # Do not replace an uncertain first line with another line.
    return " ".join(part["text"] for part in sorted(row, key=lambda b: b["x"]))


def read_recipient_name(device=None, *, target="robot", names_path=NAMES_PATH) -> str:
    """Open camera, discard five frames, OCR the sixth, return a canonical name.

    Robot default: /dev/video0 (verify it is physically the base camera).
    Mac USB test: target='mac', device=0. The camera is always released.
    Only the topmost detected text line is considered; adjacent first/last name
    regions are joined. Empty, uncertain or ambiguous database matches return ''.
    Capture/model/configuration failures raise exceptions. No address filtering,
    image crop, external preprocessing, background capture or latency exception.
    """
    if target not in ("robot", "mac"):
        raise ValueError("target must be robot or mac")
    names = json.loads(Path(names_path).read_text())
    if (
        not isinstance(names, list)
        or not names
        or any(not isinstance(n, str) or not _normalize(n) for n in names)
    ):
        raise ValueError("Names JSON must be a nonempty list of names")
    if len({_normalize(n) for n in names}) != len(names):
        raise ValueError("Names JSON contains ambiguous normalized duplicates")
    if target == "mac":
        device = 0 if device is None else device
        if isinstance(device, str) and device.isdecimal():
            device = int(device)
        if type(device) is not int or device < 0:
            raise ValueError("Mac device must be a nonnegative camera index")
    elif device is None:
        device = "/dev/video0"
    camera = cv2.VideoCapture(device, cv2.CAP_AVFOUNDATION if target == "mac" else cv2.CAP_V4L2)
    try:
        if not camera.isOpened():
            raise RuntimeError(f"Cannot open base camera {device}")
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        for number in range(1, 7):
            ok, frame = camera.read()
            if not ok or frame is None:
                raise RuntimeError(f"Camera failed on frame {number} of 6")
    finally:
        camera.release()
    with _OCR_LOCK:
        results = list(_engine().predict(frame))
    return _known_name(_first_line(results[0]), names) if results else ""


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", choices=("robot", "mac"), default="robot")
    parser.add_argument("--device")
    parser.add_argument("--names", type=Path, default=NAMES_PATH)
    args = parser.parse_args()
    print(read_recipient_name(args.device, target=args.target, names_path=args.names))
