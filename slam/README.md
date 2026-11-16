# SLAM and mailbox vision

This folder is the self-contained SLAM subproject. Files owned by `main`
(`ros2_robot/`, `arm_control/`, `digital_twin/` and shared root files) are not
modified by this reorganization. Integrate with those interfaces through an
adapter; keep SLAM implementation and configuration here.

**Run all commands below from `slam/`:**

```bash
cd slam
python3 -m pip install -r requirements-dev.txt
python3 -m unittest discover -v
ruff check .
ruff format --check .
python3 scripts/check_repository.py
bash deploy.sh --check
```

The only SLAM-owned file outside this folder is
[the GitHub Actions workflow](../.github/workflows/slam.yml), which runs checks
with `slam/` as its working directory and watches only SLAM paths.

## Layout

| Path | Purpose |
| --- | --- |
| `initialize_mailboxes.py` | Initialization API and recorded-data replay CLI |
| `mailbox_slam/` | Encoder synchronization, geometry-first mapping, graph optimization and survey controller |
| `letterbox_locator.py` | All-label geometry and separate database-matched name reading |
| `ocr_base.py`, `ocr_arm.py`, `ocr_preview.py` | Base-letter OCR, arm-name OCR and opt-in browser visualization |
| `config/` | Measured camera intrinsics, letterbox dimensions and recipient database |
| `calibrate_camera.py`, `capture_calibration.py`, `check_calibration.py` | Calibration collection, fitting and distance validation |
| `tests/` | Model-free, hardware-free regression tests |
| `scripts/`, `deploy.sh` | SLAM-only source bundle and repository checks |
| `requirements*.txt`, `pyproject.toml` | Isolated dependencies and formatter settings |
| `rdk_drive.py`, `rover.py`, `base_geometry.py`, camera/drive shell scripts | Standalone diagnostic interfaces; do not run alongside main's hardware owners |

[Initialization contract](mailbox_slam/INITIALIZATION.md) ·
[Mapping API](mailbox_slam/README.md) · [Deployment](DEPLOYMENT.md)

## Mapping workflow and status

The robot starts at local `(0, 0, 0)` with the arm fixed. The survey proposes a
full base rotation, maps anonymous labels and predicted slots from measured box
dimensions, then approaches each label to a nominal 30 cm camera-to-plane
standoff and confirms its name against `config/recipients.json`.

Repeated geometry and wheel encoders refine the map and robot pose. Slot centers
come from the measured label-to-slot offset, not independent hole detection.
The mapper/controller emits expiring velocity proposals; live hardware I/O,
calibrated arm/wheel constants and independently established path clearance are
still required. Letter pickup, delivery and insertion are later stages.

```bash
python3 -m mailbox_slam.survey_demo \
  --output /tmp/survey-map.json --events /tmp/survey-events.jsonl
python3 initialize_mailboxes.py \
  --config /tmp/survey-events.config.json \
  --settings /tmp/survey-events.survey.json \
  --events /tmp/survey-events.jsonl --output /tmp/replayed-map.json
```

The demo uses synthetic encoders/geometry/names and never moves hardware or
loads OCR. The isolated tests do not discover main's simulator/ROS test suites.

## Mac USB camera OCR

Install OCR dependencies in a compatible Python environment:

```bash
python3 -m pip install -r requirements-ocr.txt
python3 capture_calibration.py --list-cameras
python3 ocr_base.py --target mac --device 0 --test
```

Use your USB camera's index. Open http://127.0.0.1:8768 for continuous processed
OCR frames. `--test` opts into visualization; without it OCR is headless.
Robot remains the default target, with `/dev/video0` for base capture.
Arm capture always requires an explicit device:

```bash
python3 ocr_arm.py --target mac --device 0 --test
# On the robot, with the verified arm-camera device:
python3 ocr_arm.py --device /dev/video2 --test
```

Arm preview uses port 8769. Avoid competing camera owners. Both OCRs use the
same CPU PP-OCRv5 Mobile models; the base returns an address's first line, the
arm returns all text and pixel positions. `--benchmark 5` reports stage timing.
A one-second budget violation raises after processing; the target has not been
demonstrated on hardware. Test visualization retains over-budget results.

## Calibration and constants

```bash
python3 capture_calibration.py --device 0 --output calibration-images/arm
# Capture 20–25 varied views via http://127.0.0.1:8767, then stop capture.
python3 calibrate_camera.py 'calibration-images/arm/*.png' \
  --camera arm --columns 9 --rows 6 --square-m 0.0222
python3 check_calibration.py --camera arm --device 0 --distance-m 0.60
```

Use your actual grid and measured square size. The output paths are inside
`slam/config/`; `--camera base` selects `camera-intrinsics.json`, and `arm`
selects `arm-camera-intrinsics.json`. Existing measurements are never overwritten;
use `--output` for a new calibration. Intrinsics do not determine the optical
camera mounting or arm link transforms.

Place your measured `SLAM_CONFIG` in ignored `slam/robot_constants.py`, using
`mailbox_slam/constants_example.py` as the schema. The CLI's `--constants
robot_constants` import resolves from this subproject when launched here.
Do not use the demo's synthetic constants for a physical robot.

## Relationship to main and deployment

`ros2_robot/` and `arm_control/` remain authoritative hardware interfaces from
main. The standalone ROS/serial tools in this folder are diagnostics, not a
replacement for that stack. No live SLAM-to-main hardware adapter is supplied.

`deploy.sh` packages **only this folder's allowlisted runtime source**, excluding
main-owned code, local constants, models, recordings and state. It extracts into
`~/humanoid-hackathon/slam` on the robot, never into the main checkout's root.
Camera calibration is preserved remotely unless `--include-calibration` is used.
See [DEPLOYMENT.md](DEPLOYMENT.md).

## Live integration with main

[run_robot_slam.py](run_robot_slam.py) connects to main's ROS 2 cameras,
calibrated joint/motor feedback, supervisor and base-command topic. It derives
SLAM geometry from the actual URDF, verified calibration and robot parameters;
no main-owned source is modified. Camera 2 is the arm camera, camera 1 the base.
See [LIVE_ROBOT.md](LIVE_ROBOT.md) for measured mounting requirements, launch,
non-commanding validation, opt-in execution and outputs.
