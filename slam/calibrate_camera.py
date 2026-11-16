"""Calibrate the base or arm camera from saved chessboard images."""

import argparse
import glob
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "images", nargs="+", help="Images of a chessboard at varied positions/angles"
    )
    parser.add_argument("--columns", type=int, default=9, help="Inner corners horizontally")
    parser.add_argument("--rows", type=int, default=6, help="Inner corners vertically")
    parser.add_argument("--square-m", type=float, required=True)
    parser.add_argument(
        "--camera",
        choices=("base", "arm"),
        default="base",
        help="Camera whose images are supplied (default: base)",
    )
    parser.add_argument("--output", help="Override camera-specific output JSON path")
    args = parser.parse_args()
    if args.square_m <= 0 or min(args.columns, args.rows) < 3:
        parser.error("Specify positive square size and at least 3x3 inner corners")
    images = []
    for name in args.images:
        matches = sorted(glob.glob(name)) if glob.has_magic(name) else [name]
        if not matches or any(not Path(p).is_file() for p in matches):
            parser.error(
                f"No readable image files found for {name!r}. Capture chessboard images first "
                "with capture_calibration.py --output calibration-images/arm "
                "(set --device to your camera)."
            )
        images.extend(matches)
    filename = "arm-camera-intrinsics.json" if args.camera == "arm" else "camera-intrinsics.json"
    output = (
        Path(args.output) if args.output else Path(__file__).resolve().parent / "config" / filename
    )
    if output.exists():
        raise FileExistsError(
            f"Refusing to overwrite existing measurements: {output}; "
            "use --output to save a new calibration separately"
        )
    import cv2
    import numpy as np

    grid = np.zeros((args.rows * args.columns, 3), dtype=np.float32)
    grid[:, :2] = np.mgrid[0 : args.columns, 0 : args.rows].T.reshape(-1, 2) * args.square_m
    world, pixels, size = [], [], None
    for name in images:
        image = cv2.imread(name)
        if image is None:
            raise ValueError(f"Cannot read {name}")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        current = (gray.shape[1], gray.shape[0])
        if size and size != current:
            raise ValueError("All calibration images must have the same resolution")
        size = current
        ok, corners = cv2.findChessboardCorners(gray, (args.columns, args.rows))
        if ok:
            corners = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            world.append(grid.copy())
            pixels.append(corners)
    if len(world) < 12:
        raise ValueError(f"Only {len(world)} usable images; capture at least 12 varied views")
    error, matrix, distortion, _, _ = cv2.calibrateCamera(world, pixels, size, None, None)
    config = dict(
        fx=float(matrix[0, 0]),
        fy=float(matrix[1, 1]),
        cx=float(matrix[0, 2]),
        cy=float(matrix[1, 2]),
        width=size[0],
        height=size[1],
        distortion=distortion.reshape(-1).tolist(),
        camera=args.camera,
        square_size_m=args.square_m,
        inner_corners=[args.columns, args.rows],
        usable_images=len(world),
        rms_reprojection_px=float(error),
        validated=False,
    )
    if args.camera == "base":
        config.update(height_m=0.0, pitch_down_deg=0.0)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(config, indent=2) + "\n")
    print(f"Saved {output}; RMS reprojection error: {error:.3f} pixels")
    if args.camera == "base":
        print("For floor geometry, measure height/pitch and validate floor-position estimates.")
    else:
        print(
            "Arm intrinsics saved. Camera-to-arm mounting transform requires separate calibration."
        )


if __name__ == "__main__":
    main()
