#!/usr/bin/env bash
# Deploy and build the embedded runtime on the RDK (run from the PC).
#   RDK_HOST=sunrise@172.20.10.3 bash ros2_robot/deploy/deploy_robot.sh [--restart] [--enable-boot]
# Never overwrites the robot-side calibration or pose library; use
# --replace-calibration to install a new calibration (old one backed up).
set -euo pipefail
RDK_HOST=${RDK_HOST:-sunrise@172.20.10.3}
HERE=$(cd "$(dirname "$0")/.." && pwd)
REPO=$(cd "$HERE/.." && pwd)
CALIBRATION=${CALIBRATION:-$REPO/arm_control/calibration-candidate-20261115-105050-197423.local.json}
POSES=${POSES:-$REPO/arm_control/poses.local.json}
URDF=$REPO/arm_control/.sdk/hardware/elrobot/simulation/elrobot_follower.urdf
RESTART=0; BOOT=0; REPLACE_CAL=0
for arg in "$@"; do
  case $arg in
    --restart) RESTART=1 ;; --enable-boot) BOOT=1 ;; --replace-calibration) REPLACE_CAL=1 ;;
    *) echo "unknown option $arg" >&2; exit 2 ;;
  esac
done
for f in "$CALIBRATION" "$URDF"; do [[ -f $f ]] || { echo "missing $f" >&2; exit 1; }; done

ssh "$RDK_HOST" 'mkdir -p postal_ws/src postal_config'
rsync -a --delete --exclude __pycache__ --exclude '*.pyc' \
  "$HERE/postal_robot_interfaces" "$HERE/postal_robot" "$RDK_HOST:postal_ws/src/"
# Mesh URIs point at the station-side mount (/opt/postal_description); TF does not need them.
sed 's#filename="assets/#filename="file:///opt/postal_description/assets/#' "$URDF" \
  | ssh "$RDK_HOST" 'cat > postal_config/elrobot.urdf'
if ssh "$RDK_HOST" test -e postal_config/calibration.local.json; then
  if [[ $REPLACE_CAL == 1 ]]; then
    ssh "$RDK_HOST" 'cp -p postal_config/calibration.local.json "postal_config/calibration-backup-$(date +%Y%m%d-%H%M%S).local.json"'
    scp -q "$CALIBRATION" "$RDK_HOST:postal_config/calibration.local.json"
  else
    echo "Keeping robot calibration (differs: $(ssh "$RDK_HOST" sha256sum postal_config/calibration.local.json | cut -c1-12) vs local $(sha256sum "$CALIBRATION" | cut -c1-12))"
  fi
else
  scp -q "$CALIBRATION" "$RDK_HOST:postal_config/calibration.local.json"
fi
if ! ssh "$RDK_HOST" test -e postal_config/poses.local.json && [[ -f $POSES ]]; then
  scp -q "$POSES" "$RDK_HOST:postal_config/poses.local.json"   # first import only
fi
ssh "$RDK_HOST" 'test -e postal_config/ros.env || printf "ROS_DOMAIN_ID=43\nRMW_IMPLEMENTATION=rmw_fastrtps_cpp\nPYTHONUNBUFFERED=1\n" > postal_config/ros.env'
scp -q "$HERE/deploy/postal-robot.service" "$RDK_HOST:.config/systemd/user/postal-robot.service"
ssh "$RDK_HOST" 'set -e; source /opt/ros/humble/setup.bash; cd postal_ws
  colcon build --packages-select postal_robot_interfaces postal_robot --event-handlers console_cohesion- 2>&1 | tail -5
  systemctl --user daemon-reload'
[[ $BOOT == 1 ]] && ssh "$RDK_HOST" 'systemctl --user enable postal-robot.service'
[[ $RESTART == 1 ]] && ssh "$RDK_HOST" 'systemctl --user restart postal-robot.service && sleep 4 && systemctl --user --no-pager status postal-robot.service | head -5'
echo "Deployed to $RDK_HOST"
