# arm_control — planning and collision libraries

Used by the ROS 2 station planner worker
(`ros2_robot/postal_station/planner_worker/worker.py`); no motor access here.

| File | Role |
| --- | --- |
| `planner.py` | calibration validation, tick/radian conversion (refused, never clipped), IK (ikpy), Ruckig timing, joint/linear plans |
| `collision.py` | PyBullet collision world, scene validation and fingerprints, OMPL RRTConnect joint paths |
| `cad.py`, `import_surface_cad.py`, `inspect_cad_assembly.py`, `simplify_collision.py` | STEP/surface CAD import and collision proxies |
| `build_rover_scene.py`, `register_scene.py`, `benchmark_proxy.py` | rover scene construction, CAD-to-arm registration, proxy benchmarks |
| `plan_recorded.py` | offline preview between recorded poses (no hardware) |
| `COLLISIONS.md` | scene format, margins and validation rules |

Setup (virtual environment + NormaCore ElRobot URDF/meshes in `.sdk/`):

```bash
bash arm_control/setup.sh
cd arm_control && .venv/bin/python -m unittest test_arm test_collision test_simplify_collision test_surface_cad test_rover_registration
```

Local data (never committed): the verified calibration
`calibration-candidate-*.local.json`, `poses*.local.json`, scenes under
`assets/rover-fixed.local/`. Execution on the robot, pose capture and
calibration-protected motion are documented in `docs/ROS2_MIGRATION.md`.

The Station-based calibration UI, commissioning and executor tools were removed
(still in Git history up to commit 934b2b0). Recalibrating the arm
zeros/signs currently requires restoring `calibration_ui.py` from history or a
new ROS-based tool.
