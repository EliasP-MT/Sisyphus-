"""Build the source-only RDK runtime bundle; never include local state or binaries."""

import argparse
import tarfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULES = (
    "base_geometry.py",
    "calibrate_camera.py",
    "camera_feed.py",
    "capture_calibration.py",
    "check_calibration.py",
    "initialize_mailboxes.py",
    "run_robot_slam.py",
    "letterbox_locator.py",
    "ocr_arm.py",
    "ocr_base.py",
    "ocr_preview.py",
    "recipient_names.py",
    "rdk_drive.py",
    "rover.py",
)


def bundle_files(include_calibration=False):
    files = [ROOT / name for name in MODULES]
    files += [
        ROOT / name
        for name in (
            "README.md",
            "DEPLOYMENT.md",
            "requirements.txt",
            "requirements-ocr.txt",
            "rdk-drive.sh",
            "rdk-camera.sh",
            "postal-slam.service",
            "config/letterbox.json",
            "config/recipients.json",
        )
    ]
    files += sorted((ROOT / "mailbox_slam").glob("*.py"))
    files += sorted((ROOT / "mailbox_slam").glob("*.md"))
    if include_calibration:
        files += [
            ROOT / "config" / name
            for name in ("camera-intrinsics.json", "arm-camera-intrinsics.json")
        ]
    for path in files:
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"Missing or unsafe runtime file: {path.relative_to(ROOT)}")
        if "__pycache__" in path.parts or "station_data" in path.parts:
            raise ValueError(f"Generated/private runtime file selected: {path}")
    return files


def build_bundle(output, include_calibration=False):
    files = bundle_files(include_calibration)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(output, "w:gz") as archive:
        for path in files:
            archive.add(path, arcname=str(path.relative_to(ROOT)), recursive=False)
    return len(files)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--include-calibration", action="store_true")
    args = parser.parse_args()
    try:
        count = build_bundle(args.output, args.include_calibration)
    except (OSError, ValueError) as exc:
        parser.exit(2, f"{exc}\n")
    print(f"Built source bundle with {count} files: {args.output}")


if __name__ == "__main__":
    main()
