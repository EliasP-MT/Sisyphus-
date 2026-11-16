# Live SLAM with main's ROS 2 robot runtime

`run_robot_slam.py` is the live bridge. All implementation is under `slam/`;
main's controllers, messages, launch files and robot model files are read-only.
It uses the existing single bus owner and does not open a serial port or camera.
Camera 1 is the base camera; **camera 2 is the arm camera**, confirmed by the operator.

## Model and prerequisites

Start main's robot/station stack normally, using its existing bring-up instructions.
Source the workspace containing `postal_robot_interfaces` and ROS 2 Humble.
Install this folder's requirements and a compatible PaddleOCR runtime into the
same Python environment used to launch the bridge.

The bridge reads:

- Main's `robot.yaml`: wheel IDs **11/12/9**, mount directions, signs, dimensions
  and speed limits. It verifies the geometry against `/robot/core`'s live parameters.
- `~/postal_config/calibration.local.json`: hardware-verified joint zeros/signs/scales.
- `~/postal_config/elrobot.urdf`: the actual serial arm chain, fixed offsets and axes.
- `config/arm-camera-intrinsics.json`: the arm optical intrinsics, at matching resolution.
- `config/letterbox.json` and `config/recipients.json`: measured labels/slots and names.

The URDF chain must connect the base coordinate frame to `camera_2_optical_frame`.
If it does not include the camera, pass `--camera-mount measured-mount.json`:

```json
{"parent_link": "ACTUAL_CAMERA_MOUNT_LINK", "T_parent_camera": [[...], [...], [...], [...]]}
```

Supply a measured rigid 4x4 transform, including optical axes x-right/y-down/z-forward.
The placeholder above is not valid runnable calibration. No mount transform is guessed.
`--base-link` defaults to `base_footprint`; select the actual base frame in your URDF.
Joint angles come from main's calibrated `/robot/joint_states` publication.

## Launch without commanding motion

From the repository root:

```bash
cd slam
python3 run_robot_slam.py
```

This subscribes to the real robot but **does not publish base commands**. It
publishes the proposed workflow to `/slam/status`. It will not simulate motion:
the scan can advance only when wheel feedback actually changes.
Use the existing supervisor/UI to establish its normal acknowledged lease and
arm/base readiness. SLAM does not send heartbeats, acknowledge faults, change
motor modes, switch torque, or move the arm. A missing/inactive supervisor blocks startup.

Override local paths as needed with `--robot-yaml`, `--calibration`, `--urdf`,
`--intrinsics`, `--camera-mount`, and `--base-link`. The camera topic defaults to
`/robot/camera_2/image/compressed` and its expected frame to `camera_2_optical_frame`.

## Execute a survey

After validating geometry and direction with the non-commanding mode, positioning
the arm for a useful view, and clearing the arena:

```bash
python3 run_robot_slam.py --execute
```

Do not drive concurrently from the station UI or another base-command publisher.
This prototype relies on exclusive operator ownership of `/robot/base/cmd_vel`;
main does not provide a SLAM-specific arbitration lease. Keep main's station
lease alive. Start at the desired map origin with the arm stationary and torque
holding it. The worker anchors before its first rotation.

Pipeline:

1. Validate models and live wheel parameters; wait for coherent measured feedback,
   fresh arm-camera images and an ACTIVE supervisor with arm/base readiness.
2. Anchor `(0,0,0)` and record the fixed arm pose.
3. Rotate using main's Kiwi axes, stop at overlapping views, and map **all** visible
   label geometry without OCR. Compute predicted slot centers using the known offset.
4. Associate repeat anonymous sightings spatially and optimize robot/landmark poses.
5. Approach each target using expiring bounded base commands, keeping the fixed
   camera aimed at it. Stop nominally 30 cm from its label plane.
6. Reassociate the close target, rectify/read that label and confirm a canonical
   recipient in two independent frames; leave uncertain/duplicate names unresolved.
7. Repeat, save the map and stop. No letter/dispenser/gripper actions are performed.

Telemetry callbacks remain responsive while a separate worker runs vision/OCR.
Partial joint updates retain individual timestamps; wheel data use measured ages
rather than pretending every motor was read at the publication time. Only the
latest camera image is retained. Main currently stamps images after capture and
JPEG encoding, so timing is approximate; stationary capture reduces this error.

An independent 20 Hz timer publishes a command only while its short deadline,
camera freshness and supervisor permissions remain valid. Capture/OCR states
propose zero velocity. Permission loss, arm motion, stale feedback, queue overflow
or camera faults stop progress; restart explicitly after resolving the problem.
Main's own command timeout and supervisor remain authoritative underneath.

## Outputs and limits

- `/slam/status`: JSON phase, target, proposed command, map, execution flag and failures.
- `/slam/image/compressed`: annotated frames after each processed capture, including slots.
- `slam/initialized-mailboxes.json`: atomic map snapshots and final workflow/error status.

Inspect using `ros2 topic echo /slam/status`; use an image viewer for the compressed
image topic. No new main web UI panel is installed.

This is a direct goal-following prototype for a cleared demo arena, not an
obstacle-aware navigation stack. Occluded boxes and boxes outside the fixed
camera's vertical field of view cannot be guaranteed discovered. Known dimensions
provide metric depth during rotation; this is not triangulation from pure rotation.
Physical extrinsics, OCR legibility at 30 cm, wheel slip, localization accuracy,
lease behavior and actuation require robot testing. Tests mock the ROS-independent
contracts; the ROS node has not been executed against hardware in this environment.

## Follow-up to the recorded robot run (`output.md`)

The recorded attempt reached `scan_rotate` but sent no wheel command: the
Station lease expired. This is not a completed physical survey. Keep the existing
Station session connected and acknowledged; SLAM deliberately does not mint a
second heartbeat/session or bypass `LINK_LOST`.

The robot's URDF `base_link` is the arm pedestal, not the Kiwi origin. Supply
`--arm-base` for this frame; startup now refuses the silent identity assumption.
The report's camera mount and CAD arm-base values remain provisional local files,
not certified calibration. They have not been copied into source as defaults.

Check the model first, without ROS or motor commands:

```bash
cd ~/humanoid-hackathon/slam
python3 run_robot_slam.py --preflight \
  --robot-yaml ~/postal_ws/src/postal_robot/config/robot.yaml \
  --base-link base_link \
  --camera-mount ~/postal_config/camera-mount.json \
  --arm-base ~/postal_config/arm-base.json
```

After sourcing `~/postal_config/ros.env`, ROS Humble and the workspace, omit
`--preflight` for the live proposal-only run. Add `--execute` only for the cleared
arena trial. `/slam/status` now includes explicit `blockers` even before startup,
plus `geometry_warnings` for documents not marked validated/hardware_verified.
The parameter service uses Humble's `GetParameters` client; a stalled response
now produces an actionable timeout instead of waiting indefinitely.

Validate a known label's measured position before trusting a full map. Repeat
camera calibration and mounting validation as needed. Run the regression suite
with `python3 -m unittest discover -v` from `slam/`; pytest is not required.

## Discovery-only map with a known mailbox count

`--hardcode N` means: scan a full encoder-measured **360 degrees**, expect
exactly N geometrically distinct mailbox landmarks, then stop and export the
map. It never approaches a box or initializes OCR in this mode. Early detection
of N boxes does not truncate the turn. Reobservations are associated with existing
landmarks rather than counted again. Missing or excess landmarks produce
`phase: incomplete`, preserving the partial map and count mismatch for inspection.
It does not discard excess detections to pretend the requested count was met.

Using the robot-local geometry from the run report:

```bash
python3 run_robot_slam.py --execute --hardcode 3 \
  --robot-yaml ~/postal_ws/src/postal_robot/config/robot.yaml \
  --base-link base_link \
  --camera-mount ~/postal_config/camera-mount.json \
  --arm-base ~/postal_config/arm-base.json
```

Keep main's Station session ACTIVE as before. Omitting `--execute` generates
proposals only; it cannot physically scan. `--scan-only` performs the same
geometry-only sweep without specifying a count. The recorded-data CLI
`initialize_mailboxes.py` supports both flags too.

The result is `initialized-mailboxes.json` by default (`--output` overrides it).
It is a **sparse 3D landmark map**, not a reconstructed room mesh or occupancy
map. The initial base is `(0,0,0)`, x forward/y left/z up, and mailbox label/slot
centers are in metres relative to that initial frame. Robot poses remain in the
file so any estimated drift during rotation is visible. Names remain null.
`initialization` includes `scan_complete`, `scan_progress_rad`,
`expected_mailboxes`, `detected_mailboxes`, `count_matches` and the final phase.
Physical coverage still depends on visibility, fixed camera height, calibration
and wheel slip; the count does not certify unseen boxes or reject every false positive.
