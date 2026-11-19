# humanoid-hackathon

For the pinned cuRobo environment and safe PC cleanup scope, see
[Recreate the PC setup](docs/RESTORE_PC.md).

## Robot runtime (ROS 2)

The robot runs ROS 2 Humble on its RDK X5; the PC hosts the control station
(local web UI, RViz, collision-checked arm planning). See
[ros2_robot/README.md](ros2_robot/README.md) and the full
[migration document](docs/ROS2_MIGRATION.md).

```bash
RDK_HOST=sunrise@172.20.10.3 bash ros2_robot/deploy/deploy_robot.sh --restart   # robot
ROBOT_WIFI_IP=172.20.10.3 bash ros2_robot/deploy/station.sh                      # PC, then http://127.0.0.1:8895/
```

The arm uses IDs **1–8**, the dispenser servo **10**, and wheels **11 left / 12 back /
9 right**. See [hardware and Ethernet troubleshooting](RDK-TROUBLESHOOTING.md).

## Arm planning and collision libraries

[arm_control/](arm_control/README.md) holds the planning (IK, Ruckig timing),
STEP/surface CAD import and PyBullet/OMPL collision checking reused unchanged by
the station planner worker, plus the rover scene tools. The previous Station-based
tools (Station binaries, arm-view, teleop, calibration UI, hand_control) were
removed after the ROS 2 replacement was validated; they remain in Git history
(last commit containing them: 934b2b0).
