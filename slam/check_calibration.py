"""Check saved camera intrinsics by measuring a known chessboard's distance."""

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent


@dataclass(frozen=True)
class BoardDistance:
    center_range_m: float
    depth_m: float
    perpendicular_m: float
    reprojection_rms_px: float


def estimate_board_distance(
    image, calibration, *, columns=None, rows=None, square_m=None, max_reprojection_px=2.0
):
    pattern = calibration.get("inner_corners", [9, 6])
    columns = pattern[0] if columns is None else columns
    rows = pattern[1] if rows is None else rows
    square_m = calibration.get("square_size_m") if square_m is None else square_m
    if (
        not isinstance(columns, int)
        or not isinstance(rows, int)
        or min(columns, rows) < 3
        or square_m is None
        or not np.isfinite(square_m)
        or square_m <= 0
    ):
        raise ValueError("Supply at least 3x3 inner corners and a measured positive square size")
    if not np.isfinite(max_reprojection_px) or max_reprojection_px <= 0:
        raise ValueError("Maximum reprojection error must be positive")
    if (
        not isinstance(image, np.ndarray)
        or image.size == 0
        or image.dtype != np.uint8
        or image.ndim not in (2, 3)
        or (image.ndim == 3 and image.shape[2] != 3)
    ):
        raise ValueError("Expected a nonempty uint8 grayscale/BGR image")
    if (image.shape[1], image.shape[0]) != (calibration["width"], calibration["height"]):
        raise ValueError("Image resolution must match calibration")
    k = np.array(
        [
            [calibration["fx"], 0, calibration["cx"]],
            [0, calibration["fy"], calibration["cy"]],
            [0, 0, 1],
        ],
        dtype=float,
    )
    distortion = np.asarray(calibration["distortion"], dtype=float)
    if not np.isfinite(k).all() or not np.isfinite(distortion).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError("Invalid camera intrinsics")
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray, (columns, rows), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    if not found:
        return None
    corners = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
    )
    world = np.zeros((columns * rows, 3), dtype=float)
    world[:, :2] = (
        np.mgrid[:columns, :rows].T.reshape(-1, 2) - [(columns - 1) / 2, (rows - 1) / 2]
    ) * square_m
    ok, rvec, tvec = cv2.solvePnP(world, corners, k, distortion, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or not np.isfinite(tvec).all() or not np.isfinite(rvec).all():
        raise ValueError("Board pose estimation failed")
    rotation, _ = cv2.Rodrigues(rvec)
    if np.any((world @ rotation.T + tvec.reshape(1, 3))[:, 2] <= 0):
        raise ValueError("Board estimated behind camera")
    projected, _ = cv2.projectPoints(world, rvec, tvec, k, distortion)
    rms = float(
        np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - corners.reshape(-1, 2)) ** 2, axis=1)))
    )
    if not np.isfinite(rms) or rms > max_reprojection_px:
        raise ValueError(f"Unreliable grid fit: {rms:.2f} px RMS")
    t = tvec.ravel()
    return BoardDistance(float(np.linalg.norm(t)), float(t[2]), float(abs(rotation[:, 2] @ t)), rms)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", choices=("arm", "base"), default="arm")
    parser.add_argument("--intrinsics", type=Path)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image")
    source.add_argument("--device", help="Mac camera index or Linux /dev/video* path")
    parser.add_argument("--columns", type=int)
    parser.add_argument("--rows", type=int)
    parser.add_argument("--square-m", type=float)
    parser.add_argument(
        "--distance-m", type=float, help="Tape-measured optical-center to board-center distance"
    )
    args = parser.parse_args(argv)
    if args.distance_m is not None and (not np.isfinite(args.distance_m) or args.distance_m <= 0):
        parser.error("--distance-m must be positive")
    filename = "arm-camera-intrinsics.json" if args.camera == "arm" else "camera-intrinsics.json"
    capture = None
    try:
        calibration = json.loads((args.intrinsics or ROOT / "config" / filename).read_text())
        if args.device is not None:
            device = int(args.device) if args.device.isdecimal() else args.device
            capture = cv2.VideoCapture(
                device, cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_V4L2
            )
            if not capture.isOpened():
                raise ValueError(f"Cannot open camera {args.device}")
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, calibration["width"])
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, calibration["height"])
        while True:
            if capture is None:
                image = cv2.imread(args.image)
            else:
                ok, image = capture.read()
                if not ok:
                    raise ValueError("Camera capture failed")
            result = estimate_board_distance(
                image, calibration, columns=args.columns, rows=args.rows, square_m=args.square_m
            )
            output = {"state": "grid_not_found"} if result is None else asdict(result)
            if result is not None and args.distance_m is not None:
                output["error_m"] = result.center_range_m - args.distance_m
            print(json.dumps(output), flush=True)
            if capture is None:
                break
            text = (
                "Grid not found"
                if result is None
                else f"{result.center_range_m:.3f} m to board center"
            )
            cv2.putText(image, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow("Calibration distance check (Q to quit)", image)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
    except (OSError, ValueError, KeyError, cv2.error) as exc:
        parser.exit(2, f"{exc}\n")
    finally:
        if capture is not None:
            capture.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
