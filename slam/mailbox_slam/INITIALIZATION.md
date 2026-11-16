# Geometry-first mailbox initialization

This is the initialization workflow implemented for the fixed-arm postal robot.
It is separate from letter pickup, delivery, slot servoing and insertion.

1. Anchor the robot base at **map x=0, y=0, yaw=0 before moving**. The map axes
   follow the initial base heading; these are local coordinates, not geographic.
2. Keep the measured arm pose fixed. Rotate the base through an encoder-measured
   full turn, stopping at overlapping viewpoints to capture geometry.
3. Use **all** results from `letterbox_locator.locate_all(image, read_names=False)`.
   Discover anonymous mailbox landmarks from known label dimensions; no OCR
   model is loaded. Map the label and predict its associated slot using the
   measured rigid offset in `config/letterbox.json`.
4. Match repeat sightings geometrically, one-to-one, and jointly optimize the
   base trajectory and landmark poses with wheel constraints.
5. Select an unnamed mailbox, compute a base approach goal for a nominal **30 cm
   camera-to-label-plane** distance, approach it, stop and settle.
6. Reassociate the visible target geometrically. Only then call the locator's
   `read_label_name`, and match its text against `config/recipients.json`.
   Require the same unambiguous database name in two independent frames by
   default. Never force unknown text to a name or rename a different mailbox.
7. Continue with other anonymous mailboxes. Export named labels, predicted slot
   centers/normals, the corrected robot trajectory and initialization status.

## Inputs already available and inputs still required

The defaults use the measured values currently in `config/letterbox.json`:
6x5 cm outer label, slot center offset `[0.1125, 0, 0]` metres in label axes,
and 14.5x5 cm slot size. Camera intrinsics can come from
`config/arm-camera-intrinsics.json`; recipients are in `config/recipients.json`.
These files are inputs, not rewritten by initialization.

Calibrated wheel geometry/scaling, arm link transforms, joint zeros and the
camera optical mounting transform are still required in `SLAM_CONFIG` (see
`constants_example.py`). This implementation does not infer them from teleop
ranges or assume an uncalibrated default physical robot.

Use the same clock for wheel samples, measured arm joints and image capture
stamps. Frames are mapped at **capture time**, not at OCR completion time.
The camera must use the calibrated image geometry. The fixed-arm check defaults
to a maximum 0.03 rad difference from the recorded start pose. No arm motion
commands are generated during initialization.

## Callable implementation

```python
from initialize_mailboxes import MailboxDiscovery, MailboxSurvey
from robot_constants import SLAM_CONFIG

mapping = MailboxDiscovery(SLAM_CONFIG)
survey = MailboxSurvey(mapping)

# Supply actual measured encoder samples in the configured order/names:
mapping.add_wheel_sample(t0, wheel_ticks)
mapping.add_joint_sample(t0, joint_ticks)
survey.start(t0)  # Must be BEFORE the first rotation, not after first detection.

# Continue feeding timestamped wheel/joint samples throughout the session.
proposal = survey.step(now)
# proposal['velocity_base'] is [vx, vy, wz] in base axes, m/s and rad/s.
# proposal['expires_at'] is the time by which the command MUST expire.
# Send it only through your existing base controller/watchdog in a cleared arena.
if proposal["capture"]:
    # Base has been commanded to stop and encoder feedback confirms settling.
    frame, capture_timestamp = get_fresh_arm_camera_frame()
    report = survey.observe(frame, capture_timestamp)

mapping.save_map("initialized-mailboxes.json")
state = survey.status()
```

The placeholders `wheel_ticks`, `joint_ticks`, `now` and
`get_fresh_arm_camera_frame` belong to your hardware I/O adapter. This module
**does not open devices or publish motor commands**. The existing base controller
can consume the proposed velocities, but a live station/camera/base adapter with
measured constants is not supplied or validated by this change. Never replay
recorded velocity proposals to hardware.

A typical application runs encoder reception continuously into a queue, drains
those samples into this single-threaded mapper, calls `step` frequently, and
captures only when requested. During slow OCR, keep telemetry reception active
outside the mapper. Every capture state proposes zero velocity, so the base's
existing timeout must keep it stopped while processing. Do not replace capture
stamps with network arrival time or stop receiving wheel telemetry during OCR.

## State and motion contract

- `scan_rotate`: zero translation, bounded CCW yaw proposal; progress comes from
  net encoder yaw, so back-and-forth rotation does not count as a full sweep.
- `scan_settle` / `scan_capture`: stopped wheels and fixed arm; geometry only.
- `approach`: aim the fixed camera toward the target, then follow its approach
  goal with bounded Kiwi XY commands. Refresh geometry during the approach.
- `approach_settle` / `approach_capture`: stop and update geometry; unknown boxes
  cannot create new landmarks once the discovery sweep is complete.
- `name_settle` / `name_capture`: stopped, nearby target; explicit OCR + database
  confirmation. A name cannot be attached without a geometric match in that frame.
- `complete`: every discovered mailbox has a confirmed name.
- `incomplete`: no boxes or some boxes unread/unreachable; examine `failed`.
- `fault`: stale telemetry, arm movement, invalid clock or phase timeout; zero
  velocity proposal, with a reason. Restart after resolving the cause.

`SurveySettings` exposes sweep spacing/speed, settle time, approach speed,
30 cm standoff, visual freshness, retries, tolerances and command expiration.
The full-sweep tolerance is 0.025 rad by default. Initial yaw is zero by
convention; encoder slip can still make the physical sweep inaccurate.

Approach goals account for camera offset from the base rotation center and
camera heading relative to the base. They are normal to the mailbox face near
arrival. Goals where the fixed camera cannot see the whole label at the desired
distance are rejected; the initializer does not move the arm to compensate.
The 30 cm reference is the camera optical center, **not the bumper or gripper**.
Label resolution/OCR legibility at that distance must be measured on the real
camera; digital rectification cannot recover missing optical detail. The
standoff is adjustable, not an accuracy guarantee.

## Map association and limitations

`MailboxDiscovery` uses anonymous IDs and spatial/rotation gates, with Hungarian
one-to-one association. Its default position gate is 12 cm and nearest-match
margin is 2.5 cm. Adjacent candidates that cannot be distinguished are rejected;
known-size identical rectangles have no unique ID before OCR. Excess wheel slip
or crowded boxes can still break geometric association and need validation.
Ambiguous planar poses are retained by the locator and rejected for new landmarks
when near-equal solutions imply materially different normals.

Mailboxes are assumed stationary and visible somewhere in the initial sweep.
This is **not** coverage exploration: hidden boxes, boxes outside the fixed
camera's vertical view, and occluded faces cannot be certified absent. A full
rotation alone cannot prove that every physical mailbox has been found.

Slot positions have `slot_source: known_rigid_offset`. They are geometric
predictions, not independently detected holes or final insertion targets.
The exported `T_map_tag` and slot positions update together after optimization.
Names are canonical database strings; inconsistent, tied or duplicate-recipient
assignments remain unconfirmed instead of being silently attached.

There is no obstacle/free-space sensor in this mapping pipeline. Direct goal
following must only be executed in an independently cleared demo area; mailbox
positions do not establish that intervening floor space is free. Obstacle-aware
navigation and final slot alignment remain separate work. No motors were moved
while developing/testing this implementation.

## Demo and recorded-data replay

```bash
# Four synthetic mailboxes around a stationary-starting base; no hardware/OCR.
python3 -m mailbox_slam.survey_demo \
  --output /tmp/survey-map.json --events /tmp/survey-events.jsonl

# Replay the same geometry/encoder/name workflow, still no hardware:
python3 initialize_mailboxes.py \
  --config /tmp/survey-events.config.json \
  --settings /tmp/survey-events.survey.json \
  --events /tmp/survey-events.jsonl --output /tmp/survey-replayed.json

# Your own measured constants and recorded arm images:
python3 initialize_mailboxes.py --constants robot_constants \
  --events recording/survey.jsonl --output initialized-mailboxes.json
```

`--calibration`, `--geometry` and `--recipients` override the input files.
If constants omit intrinsics, the CLI uses `config/arm-camera-intrinsics.json`.
Label dimensions always come from the selected letterbox geometry.
`--print-proposals` prints control outputs for inspection; replay never executes them.
The output also includes `initialization` state/failed targets and survey settings.

Replay records: `wheels`/`joints` with measured `positions`; `start` to anchor
origin; `tick` to advance the controller; `image` with a relative image `path`;
or `locator` with precomputed `results` and optional recorded `ocr_text`.
Each has a `timestamp`. At a shared timestamp, feed encoders before control/image
records. Image records are accepted only when the state machine requests one.
Injected locator/OCR records are trusted recordings, not fresh optical checks.

```bash
python3 -m unittest discover -v
```

Tests and the demo are synthetic: they exercise full rotation, spatial discovery,
slot transforms, wheel-drift correction, goal following, database confirmations,
fixed-arm checks and watchdog-related stop proposals. They do not establish
real-world navigation clearance, physical calibration accuracy or OCR reliability.
