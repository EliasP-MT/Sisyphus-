#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
set -u
export ROS_LOCALHOST_ONLY=1
exec /usr/bin/python3 camera_feed.py "$@"
