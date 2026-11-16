# Mailbox mapping API

The supported workflow is geometry-first: discover anonymous boxes while rotating
with a fixed arm, then approach and read canonical database names. See
[INITIALIZATION.md](INITIALIZATION.md) for the complete control/I/O contract.

```python
from mailbox_slam import MailboxDiscovery, MailboxSurvey
from robot_constants import SLAM_CONFIG

mapping = MailboxDiscovery(SLAM_CONFIG)
mapping.add_wheel_sample(t0, wheel_positions)
mapping.add_joint_sample(t0, joint_positions)
survey = MailboxSurvey(mapping)
survey.start(t0)

# Feed continuous measured encoder samples, then call from your control loop:
proposal = survey.step(now)
if proposal["capture"]:
    survey.observe(frame, capture_timestamp)
mapping.save_map("initialized-mailboxes.json")
```

The survey returns velocity proposals, not motor commands. Capture states stop
and settle before processing. The mapper is single-threaded; keep asynchronous
telemetry reception in an external queue while OCR runs. The camera image stamp
must refer to capture time and share a clock with measured encoders.

## Configuration

`constants_example.py` contains deliberately empty hardware placeholders. Import
actual constants into `SLAM_CONFIG`, or load JSON with `load_config`.

| Section | Required values |
| --- | --- |
| `camera` | `fx`, `fy`, `cx`, `cy`, `width`, `height`, `distortion` |
| `arm` | `T_base_arm`, serial `joints` list, `T_last_camera` |
| Each joint | `name`, `T_parent_joint`, `axis`, `zero_tick`, `ticks_per_revolution`, `sign`, `modulus`, `max_rad_s` |
| Each wheel | `name`, `position_xy_m`, `rolling_direction_xy`, `radius_m`, `ticks_per_revolution`, `sign`, `modulus`, `max_rad_s` |

Label/slot dimensions come only from `config/letterbox.json` or the explicit
`geometry` argument to `MailboxDiscovery`. The CLI can use the saved arm
intrinsics when constants omit them. Defaults in `settings` are software tuning
values, not measured uncertainty guarantees; unknown keys are rejected.

`T_a_b` maps coordinates in frame b into frame a. Base axes are x forward/y
left/z up. Camera optical axes are x right/y down/z forward. FK multiplies
`T_base_arm`, then each `T_parent_joint @ rotation(axis, measured_angle)`, then
`T_last_camera`. Joint axes are in the joint frame. Radian inputs must already
have physical zero/sign offsets applied and be unwrapped; tick inputs are
converted using the configured zero/sign/scaling.

Wheel geometry must constrain all three planar motion axes. Positive configured
wheel travel follows the unit `rolling_direction_xy`; `sign` converts raw ticks
to that convention. Ticks per revolution include gearing. Use `modulus: null`
for continuous accumulated ticks; otherwise supply the actual wrap range. Speed
limits reject implausible jumps and ambiguous modulo intervals.

## Modules and outputs

- `encoders.py`: validated telemetry, interpolation, Kiwi odometry and arm FK.
- `graph.py`: sparse robust optimization of planar base and 6-DoF label poses.
- `discovery.py`: anonymous spatial association, predicted slots and deferred names.
- `survey.py`: rotate/settle/capture/approach/read state machine.
- `station.py`: read-only ST3215 present-position bridge for decoded NormaCore frames.
- `mapping.py`: shared state and atomic JSON export.
- `survey_demo.py`: the single hardware-free end-to-end example.

`process_image` maps geometry without loading OCR. `read_target_name` explicitly
reassociates the label before reading it. `process_locator_results` and
`process_observations` accept validated external measurements for adapters/replay.

Export includes corrected base poses, `T_map_tag`, label/slot centers, slot push
directions, canonical names and confirmation counts, configuration and original
observation constraints. Optimization residuals are whitened using configured
weights; a covariance estimate is not provided. A fixed initial base pose anchors
the map. The graph is bounded by `max_frames`, and map resume/relocalization after
process restart is not implemented.

No obstacle map, path-clearance guarantee, live hardware adapter, letter pickup
or insertion controller is included. The geometry/vision assumptions and physical
validation gaps are listed in [INITIALIZATION.md](INITIALIZATION.md).
