#!/usr/bin/env bash
# Run a command in the PC-side ROS 2 Humble container (same distribution as
# the RDK), after building the workspace incrementally.
#   bash ros2_robot/deploy/station_docker.sh ros2 topic list
# Environment: STATION_IMAGE (any image with ROS 2 Humble desktop + colcon),
# ROS_DOMAIN_ID (default 43), SKIP_BUILD=1, STATION_DDS_PROFILE (Fast DDS XML).
set -euo pipefail
HERE=$(cd "$(dirname "$0")/.." && pwd)
REPO=$(cd "$HERE/.." && pwd)
IMAGE=${STATION_IMAGE:-ghcr.io/epflxplore/base@sha256:3ee3faca2959b2eeb904babdabff40928d4e2d7d90afa00055e7a78a950b88b4}
WS=$HERE/.station_ws
DESC=$REPO/arm_control/.sdk/hardware/elrobot/simulation
mkdir -p "$WS/home"
TTY=(); [[ -t 0 && -t 1 ]] && TTY=(-it)
DDS=(); [[ -n ${STATION_DDS_PROFILE:-} ]] && DDS=(-e FASTRTPS_DEFAULT_PROFILES_FILE=/postal/ws/dds.xml -v "$STATION_DDS_PROFILE":/postal/ws/dds.xml:ro)
exec docker run --rm "${TTY[@]}" --net=host --ipc=host --user "$(id -u):$(id -g)" \
  --name "postal-station-$$" \
  -e HOME=/postal/ws/home -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}" -e DISPLAY="${DISPLAY:-}" \
  -e QT_X11_NO_MITSHM=1 -e SKIP_BUILD="${SKIP_BUILD:-0}" -e PYTHONUNBUFFERED=1 "${DDS[@]}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v "$HERE":/postal/src -v "$WS":/postal/ws \
  -v "$DESC":/opt/postal_description:ro \
  -w /postal/ws --entrypoint /bin/bash "$IMAGE" -c '
    source /opt/ros/humble/setup.bash
    if [ "$SKIP_BUILD" != 1 ]; then
      colcon build --base-paths /postal/src --event-handlers console_cohesion- >/postal/ws/build.log 2>&1 \
        || { tail -40 /postal/ws/build.log; exit 1; }
    fi
    source /postal/ws/install/setup.bash
    exec "$@"' bash "$@"
