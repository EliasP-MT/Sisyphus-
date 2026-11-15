#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")/.."
# ROS setup scripts access unset variables on some Humble installations.
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${POSTAL_TWIN_ROS_DOMAIN:-42}"
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$PWD/digital_twin/.runtime/ros_logs"
export OPENBLAS_NUM_THREADS="${POSTAL_TWIN_BLAS_THREADS:-1}"
export OMP_NUM_THREADS="${POSTAL_TWIN_OMP_THREADS:-1}"
mkdir -p "$ROS_LOG_DIR"
exec arm_control/.venv/bin/python -m digital_twin.ros2_bridge "$@"
