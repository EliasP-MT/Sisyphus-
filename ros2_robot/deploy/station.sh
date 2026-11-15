#!/usr/bin/env bash
# Start the PC control station: planner worker (host venv) + ROS station in
# the Humble container (web UI on http://127.0.0.1:8895, RViz).
#   ROBOT_WIFI_IP=172.20.10.3 bash ros2_robot/deploy/station.sh [rviz:=false]
# DDS traffic to the robot is restricted to the PC interface that routes to
# ROBOT_WIFI_IP, so the operational link is the Wi-Fi even when the Ethernet
# service cable is plugged in.
set -euo pipefail
HERE=$(cd "$(dirname "$0")/.." && pwd)
REPO=$(cd "$HERE/.." && pwd)
ROBOT_WIFI_IP=${ROBOT_WIFI_IP:-172.20.10.3}
CALIBRATION=${CALIBRATION:-$REPO/arm_control/calibration-candidate-20261115-105050-197423.local.json}
SCENE=${SCENE:-$REPO/arm_control/assets/rover-fixed.local/rover-scene.local.json}
PC_IP=$(ip -4 route get "$ROBOT_WIFI_IP" | sed -n 's/.* src \([0-9.]*\).*/\1/p')
DEV=$(ip -4 route get "$ROBOT_WIFI_IP" | sed -n 's/.* dev \([^ ]*\).*/\1/p')
[[ -n $PC_IP ]] || { echo "No route to the robot Wi-Fi address $ROBOT_WIFI_IP" >&2; exit 1; }
case $DEV in wl*) ;; *) echo "WARNING: route to $ROBOT_WIFI_IP uses $DEV, not a Wi-Fi interface" >&2 ;; esac
mkdir -p "$HERE/.station_ws"
PROFILE=$HERE/.station_ws/dds.xml
sed -e "s/@PC_IP@/$PC_IP/g" -e "s/@ROBOT_IP@/$ROBOT_WIFI_IP/g" "$HERE/deploy/fastdds_station.xml.in" > "$PROFILE"
echo "Station DDS on $DEV ($PC_IP) -> robot $ROBOT_WIFI_IP, domain ${ROS_DOMAIN_ID:-43}"

"$REPO/arm_control/.venv/bin/python" "$HERE/postal_station/planner_worker/worker.py" \
  --calibration "$CALIBRATION" --scene "$SCENE" > "$HERE/.station_ws/planner.log" 2>&1 &
WORKER=$!
trap 'kill $WORKER 2>/dev/null || true' EXIT
xhost +local: >/dev/null 2>&1 || true
STATION_DDS_PROFILE=$PROFILE bash "$HERE/deploy/station_docker.sh" ros2 launch postal_station station.launch.py "$@"
