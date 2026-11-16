# SLAM on the robot: status and handover

Written for: a teammate finishing the live SLAM run on the RDK X5.

## Where it stands

`run_robot_slam.py` starts on the robot, loads every model and reaches `scan_rotate`.
**The survey has never run end to end**: the station lease expired (`LINK_LOST`) during
the `--execute` attempt, so no wheel command was ever sent. Nothing moved. SLAM refused
to drive without an ACTIVE supervisor, which is correct.

Last run, safe mode (no `--execute`): supervisor ACTIVE, `phase: scan_rotate`, no errors.

## To finish

1. Open the station UI, connect and acknowledge so `/robot/supervisor/state` is `ACTIVE`
   (arm torque on, base enabled). Keep that session open. SLAM sends no heartbeats.
2. Clear the arena, leave the arm in a good viewing pose, torque on. Do not drive from
   the UI at the same time.
3. Run with `--execute` (command below) and watch `/slam/status`.

## Run it (on the robot, over SSH)

ROS 2 Humble and `rclpy` are only on the robot (`ssh sunrise@172.20.10.3`).
`postal-robot` is a user service and is already running
(`systemctl --user status postal-robot`).

```bash
set -a; source ~/postal_config/ros.env; set +a
source /opt/ros/humble/setup.bash
source ~/postal_ws/install/setup.bash
cd ~/humanoid-hackathon/slam
python3 run_robot_slam.py --execute \
  --robot-yaml ~/postal_ws/src/postal_robot/config/robot.yaml \
  --base-link base_link \
  --camera-mount ~/postal_config/camera-mount.json \
  --arm-base ~/postal_config/arm-base.json
```

Drop `--execute` for proposals only. Watch with `ros2 topic echo /slam/status`
(same three `source` lines first). Redeploy code with `bash deploy.sh` from `slam/`
(add `--include-calibration` only to overwrite robot-side intrinsics).

## Files that exist only on the robot (not in git)

Both are in `~/postal_config/`. Recreate them from these values if lost.

`camera-mount.json`, camera 2 on the gripper (hand measured, **not validated**):
```json
{"parent_link": "Gripper_Base_v1_1",
 "T_parent_camera": [[-0.01244, -0.99978, 0.01714, 0.06033],
                     [-0.00882, 0.01725, 0.99981, 0.01896],
                     [-0.99988, 0.01229, -0.00903, -0.00092],
                     [0, 0, 0, 1]]}
```
Basis: camera 6 cm from the J7 axis along the negative J6 axis, 2 cm toward the tip,
looking toward the tip, image-up away from the J7 axis, measured with J7 = -0.3 deg.
If any of that is wrong, the labels land in the wrong place. Re-measure or hand-eye
calibrate it before trusting the map.

`arm-base.json`, pose of the URDF `base_link` in the robot Kiwi frame:
```json
{"T_base_arm": [[-0.12398, 0.99228, 0, 0.02688],
                [-0.99228, -0.12398, 0, 0.0025],
                [0, 0, 1, 0.093],
                [0, 0, 0, 1]]}
```
Derived from `arm_control/assets/rover-fixed.local/rover-scene.local.json`
(`hardware_verified: true`). Robot frame: x forward, origin at the centroid of the three
wheel contact points, z = floor. The arm base is 2.7 cm ahead of the wheel centre and its
axes are rotated about 97 deg from the robot's (`base_link` +y is roughly forward).
The old default (identity) was wrong. Origin and floor height come from the CAD and are
not checked on the real robot.

## Changes on this branch

- `--arm-base` option (`T_base_arm` JSON, default identity).
- ROS 2 Humble fix: replaced `rclpy.parameter_client.AsyncParameterClient` with a
  `GetParameters` service client.
- Fault message now names the exact blocker (supervisor state, stale camera, ...).
- Merged `main` into `slam`.

## Open items and risks

- Camera intrinsics: `config/arm-camera-intrinsics.json` is `validated: false`
  (RMS 2.09 px). A colleague is adding better ones.
- Camera mount is unvalidated, see above. Check one known label position against the map.
- Wheel-centre origin and floor height come from CAD. Check on the robot.
- Prototype only: goal-following in a cleared arena, no obstacle avoidance.
- OCR at 30 cm and wheel slip have not been tested on hardware.
- Unit tests (`slam/tests`) were not run after these changes (no pytest in the
  environment used).
