"""Full-frame PP-OCRv5 Mobile arm-camera OCR; returns all text and pixel positions.

No address filtering, cropping, or additional models. Coordinates use the
original image, x right/y down. Nearby words on the same line are merged;
geometry cannot distinguish separate names written with ordinary word spacing.
"""

import argparse
import json
import math
import sys
from functools import lru_cache

import cv2
import numpy as np

from ocr_base import BaseOCR


def merge_text_regions(result, *, merge_gap=0.9, min_confidence=0.0):
    """Return JSON-compatible text groups, retaining original OCR polygons.

    merge_gap is the maximum gap between neighboring regions in units of the
    smaller region's text height. Different rows/angles and larger gaps remain
    separate. No semantic name interpretation or splitting inside OCR regions.
    """
    if not math.isfinite(merge_gap) or merge_gap < 0:
        raise ValueError("merge_gap must be finite and nonnegative")
    if not 0 <= min_confidence <= 1:
        raise ValueError("min_confidence must be in [0, 1]")
    if result is None:
        return []
    words = []
    for text, confidence, polygon in zip(
        result["rec_texts"], result["rec_scores"], result["rec_polys"]
    ):
        text = text.strip()
        if not text or float(confidence) < min_confidence:
            continue
        points = np.asarray(polygon, dtype=float).reshape(4, 2)
        # PaddleOCR orders quadrilateral corners TL, TR, BR, BL.
        direction = (points[1] - points[0]) + (points[2] - points[3])
        length = np.linalg.norm(direction)
        if length < 1e-6 or not np.isfinite(points).all():
            continue
        direction /= length
        if direction[0] < 0:
            direction = -direction
        normal = np.array([-direction[1], direction[0]])
        height = float(np.ptp(points @ normal))
        if height < 1:
            continue
        words.append(
            dict(
                text=text,
                confidence=float(confidence),
                points=points,
                center=points.mean(axis=0),
                direction=direction,
                height=height,
            )
        )

    def adjacent(a, b):
        if a["direction"] @ b["direction"] < math.cos(math.radians(12)):
            return False
        height = min(a["height"], b["height"])
        if max(a["height"], b["height"]) > 2 * height:
            return False
        direction = a["direction"] + b["direction"]
        direction /= np.linalg.norm(direction)
        normal = np.array([-direction[1], direction[0]])
        if abs((a["center"] - b["center"]) @ normal) > 0.45 * height:
            return False
        pa, pb = a["points"] @ direction, b["points"] @ direction
        gap = max(pa.min(), pb.min()) - min(pa.max(), pb.max())
        return -0.2 * height <= gap <= merge_gap * height

    # Merge only when every member remains on the same baseline; this avoids
    # chains drifting across rows or joining differently angled labels.
    groups = []
    for word in sorted(words, key=lambda w: (w["center"][1], w["center"][0])):
        groups.append([word])
    changed = True
    while changed:
        changed = False
        for i in range(len(groups)):
            for j in range(i + 1, len(groups)):
                a, b = groups[i], groups[j]
                if not any(adjacent(x, y) for x in a for y in b):
                    continue
                direction = np.mean([w["direction"] for w in a + b], axis=0)
                direction /= np.linalg.norm(direction)
                normal = np.array([-direction[1], direction[0]])
                baselines = [w["center"] @ normal for w in a + b]
                if np.ptp(baselines) > 0.45 * min(w["height"] for w in a + b):
                    continue
                groups[i] += groups.pop(j)
                changed = True
                break
            if changed:
                break
    output = []
    for group in groups:
        direction = np.mean([w["direction"] for w in group], axis=0)
        group.sort(key=lambda w: w["center"] @ direction)
        points = np.concatenate([w["points"] for w in group])
        low, high = points.min(axis=0), points.max(axis=0)
        polygon = cv2.boxPoints(cv2.minAreaRect(points.astype(np.float32)))
        output.append(
            dict(
                text=" ".join(w["text"] for w in group),
                confidence=min(w["confidence"] for w in group),
                center=((low + high) / 2).tolist(),
                bbox=[*low.tolist(), *high.tolist()],
                polygon=polygon.astype(float).tolist(),
                words=[
                    dict(text=w["text"], confidence=w["confidence"], polygon=w["points"].tolist())
                    for w in group
                ],
            )
        )
    return sorted(output, key=lambda g: (g["center"][1], g["center"][0]))


class ArmOCR(BaseOCR):
    """Same persistent engine/capture/timing as BaseOCR, without address logic.

    Supply the physical arm camera device explicitly for camera captures.
    read_text(image) also accepts a path or an unmodified OpenCV BGR image.
    All nonempty OCR results are retained by default, with their confidences.
    """

    def __init__(self, device=None, *, target="robot", merge_gap=0.9, **kwargs):
        merge_text_regions(
            None, merge_gap=merge_gap, min_confidence=kwargs.setdefault("min_confidence", 0.0)
        )
        self.merge_gap = merge_gap
        self.last_detections = []
        self._arm_device_missing = device is None
        super().__init__(device=device, target=target, **kwargs)

    def _capture(self):
        if self._arm_device_missing:
            raise ValueError(
                "Specify the arm camera device; no fallback to the base camera is allowed"
            )
        return super()._capture()

    def _extract_result(self, result, width, height):
        self.last_detections = merge_text_regions(
            result, merge_gap=self.merge_gap, min_confidence=self.min_confidence
        )
        return self.last_detections

    def read_text(self, image=None) -> list[dict]:
        """Return text/positions, [] if absent; same latency-budget check as base.

        On TimeoutError, last_detections and last_timings remain available.
        """
        return super().read_name(image)


@lru_cache(maxsize=1)
def _reader(device, target, merge_gap):
    return ArmOCR(device=device, target=target, merge_gap=merge_gap)


def read_arm_text(image=None, *, device=None, target="robot", merge_gap=0.9) -> list[dict]:
    """Importable cached-engine convenience API. Prefer ArmOCR for lifecycle control."""
    return _reader(device, target, merge_gap).read_text(image)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", choices=("robot", "mac", "auto"), default="robot")
    parser.add_argument(
        "--device", help="Explicit arm-camera /dev/video* path or Mac USB camera index"
    )
    parser.add_argument("--image", help="Saved full image instead of camera capture")
    parser.add_argument(
        "--test", action="store_true", help="Visualize live merged text at OCR inference speed"
    )
    parser.add_argument("--port", type=int, default=8769)
    parser.add_argument(
        "--merge-gap", type=float, default=0.9, help="Maximum word gap in text-height units"
    )
    parser.add_argument("--min-confidence", type=float, default=0.0)
    parser.add_argument("--benchmark", type=int, default=0)
    args = parser.parse_args(argv)
    if not args.image and args.device is None:
        parser.error("--device is required to select the arm camera")
    if args.test and (args.image or args.benchmark):
        parser.error("--test cannot be combined with --image or --benchmark")
    if args.benchmark < 0:
        parser.error("--benchmark must be nonnegative")
    with ArmOCR(
        device=args.device,
        target=args.target,
        merge_gap=args.merge_gap,
        min_confidence=args.min_confidence,
    ) as reader:
        if args.test:
            from ocr_preview import run_preview

            try:
                run_preview(reader, port=args.port)
            except KeyboardInterrupt:
                pass
        elif args.benchmark:
            reader._read(args.image)
            for _ in range(args.benchmark):
                try:
                    print(json.dumps(reader.read_text(args.image), ensure_ascii=False))
                except TimeoutError as exc:
                    print(str(exc), file=sys.stderr)
                    print(json.dumps(reader.last_detections, ensure_ascii=False))
                print(json.dumps(reader.last_timings), file=sys.stderr)
        else:
            print(json.dumps(reader.read_text(args.image), ensure_ascii=False))


if __name__ == "__main__":
    main()
