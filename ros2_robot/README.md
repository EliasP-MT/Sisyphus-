# Postal robot ROS 2 migration — in progress

This package is currently a read-only hardware foundation, not a replacement
for the complete arm-control application. Do not use it as a motion controller.

## Hardware ownership

Stop `rdk-station.service` and `arm-view.service` before starting feedback.
Only one process owns the serial adapter. pyserial opens it exclusively.
No motor writes are implemented in this version.

Hardware: arm IDs 1–8; left/back/right wheels 11/12/9; dispenser 10.
Operator-confirmed wheel radius 0.05 m, base radius 0.125 m.
Dispenser requested speed: -1000 ticks/s, finite duration per request.

## Run on ROS 2 Humble

Build with colcon as an `ament_python` package, source its install setup, then:

```sh
ros2 run postal_robot motor_feedback --ros-args \
  -p calibration:=/absolute/path/to/calibration.local.json
```

The calibration must contain the existing verified seven-joint mapping.
Keep calibration and pose JSON files outside version control.
`/robot/joint_states` publishes fresh measured radians per calibrated joint.
Messages are partial by design: each sample has its own acquisition time.
Never refresh another joint's age when receiving a partial message.
`/robot/diagnostics` carries raw ticks and fault information for all 12 motors.
Gripper, wheels and dispenser have no invented angular calibration.

Launch cameras as separate processes with stable by-id device paths:

```sh
ros2 run postal_robot usb_camera --ros-args \
  -r __ns:=/robot/front -p device:=/dev/v4l/by-id/DEVICE-video-index0 \
  -p frame_id:=front_camera_optical_frame
```

Images are JPEG at requested 640×480 / 10 Hz. Camera intrinsics and extrinsics
are not yet calibrated; do not treat frames as geometrically registered.

## Validation

`PYTHONPATH=ros2_robot python3 -m unittest discover -s ros2_robot/tests -v`

Verified on the RDK: 12 addressed servo replies; calibrated ROS JointState
received by ROS CLI; images captured from both USB cameras at 640×480.
Camera ROS publication, long-run rates and disconnect recovery remain untested.

## Remaining before acceptance

- Standard trajectory action, existing collision checks and tracked execution.
- Fresh per-motor state cache and atomic named-pose capture.
- Base commands and bounded dispenser operation with lease/watchdog.
- Stop base, retain arm holding torque, require reconnection/rearming on link loss.
- Local web control station, RViz config, robot description and TF.
- Launch files, systemd deployment, dependency installation and startup checks.
- Hardware motion tests supervised by the operator; no automatic mission resume.
- Targeted commits and final migration report; legacy tools retained until acceptance.
