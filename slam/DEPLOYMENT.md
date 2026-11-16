# Deploy the mail robot source to the RDK X5

Run these commands from `slam/`. Deployment copies SLAM application source only. It does not install packages, start
nodes, open devices, overwrite robot constants, or move motors.

```bash
# Validate/build the exact bundle locally, without a robot connection:
bash deploy.sh --check
# Copy to ~/humanoid-hackathon/slam on the robot:
bash deploy.sh
# Override the SSH destination:
RDK_HOST=sunrise@192.168.127.10 bash deploy.sh
```

The allowlisted bundle contains OCR, calibration tools, letterbox geometry,
recipient names, the mapping/survey modules, manual ROS drive/camera tools,
and requirements. Main-owned robot, station and arm-control code is not bundled. It excludes recordings,
compiled caches, build output, binaries, local bus-role settings, station state,
robot constants, and tests. A YOLO model is not needed.

Camera calibration files are excluded by default to preserve measurements already
on the robot. To deliberately replace them with the checked-in files:

```bash
bash deploy.sh --include-calibration
```

Inspect the source JSON first and verify which physical camera it describes.
`config/letterbox.json` and `config/recipients.json` are normal application inputs
and are deployed. Provide measured `robot_constants.py` separately on the robot;
use `mailbox_slam/constants_example.py` as its schema. Never deploy synthetic
simulation constants as hardware geometry.

Install runtime dependencies in the Python environment used to launch the app:

```bash
python3 -m pip install -r requirements.txt
# Only when running OCR, using a compatible PaddlePaddle wheel for the board:
python3 -m pip install -r requirements-ocr.txt
```

OpenCV and serial can instead come from Ubuntu packages when using the ROS
system environment. ROS 2 Humble (`rclpy`, image/message packages) remains a
system dependency, not a pip requirement. Do not run a second process that opens
a camera or motor bus already owned by the station/another controller.

Manual base check, without wheel motion:

```bash
bash rdk-drive.sh --check
```

Live OCR diagnostics (no wheel/arm commands):

```bash
python3 ocr_base.py --test
python3 ocr_arm.py --device /dev/video2 --test
```

Run these one at a time and verify the device assignment. View through SSH
forwarding of ports 8768/8769. Calibration and letterbox diagnostic commands are
in the root README.

Mailbox initialization is currently an API plus recorded-data replay, not a
ready-made live hardware driver. Wire timestamped encoder/image inputs and
expiring survey proposals through the robot's existing I/O/watchdog. See
[INITIALIZATION.md](mailbox_slam/INITIALIZATION.md). No code in the deployment
script executes recorded velocity proposals. Real arm/wheel calibration,
free-space checks, station/camera clock alignment, and physical trials remain
required before autonomous movement. Letter pickup and insertion are future work.
