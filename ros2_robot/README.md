# ros2_robot — ROS 2 runtime of the postal robot

Colcon workspace sources (ROS 2 Humble on both the RDK X5 and the PC container):

| Package | Runs on | Content |
| --- | --- | --- |
| `postal_robot_interfaces` | both | messages, services, actions |
| `postal_robot` | RDK | `core` (single servo-bus owner, supervision, base, dispenser, arm execution, poses), `usb_camera`, `robot.launch.py`, `config/robot.yaml` |
| `postal_station` | PC | `web` (local UI + ROS bridge), `arm_motion` (plan/preview/execute), `station.launch.py`, RViz config, `planner_worker/` (arm_control venv) |
| `deploy/` | PC | `deploy_robot.sh`, `postal-robot.service`, `station.sh`, `station_docker.sh`, Fast DDS profile template |

Quick start:

```bash
RDK_HOST=sunrise@172.20.10.3 bash ros2_robot/deploy/deploy_robot.sh --restart   # robot
ROBOT_WIFI_IP=172.20.10.3 bash ros2_robot/deploy/station.sh                      # PC, then http://127.0.0.1:8895/
```

Architecture, safety contract (lease loss, explicit resume, torque rules),
operations, tests and the hardware validation record:
[docs/ROS2_MIGRATION.md](../docs/ROS2_MIGRATION.md).

Calibration, poses and scenes are backed up in `restore/robot-data.tar.gz`.
Run `python3 restore/data.py` after cloning to restore and relocate them.
The SDK is downloaded at its pinned revision by `scripts/restore_pc.sh`.
See [the complete PC restoration guide](../docs/RESTORE_PC.md).
