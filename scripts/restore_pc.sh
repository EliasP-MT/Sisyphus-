#!/usr/bin/env bash
# Restore files and dependencies only; never start the station or command hardware.
set -euo pipefail
REPO=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$REPO"
PYTHON=${PYTHON:-python3.12}
"$PYTHON" -c 'import sys; assert sys.version_info[:2] == (3, 12)'
"$PYTHON" restore/data.py
REV=6b494f77fac366a878ae71222458b196f8400dee
if [[ ! -e arm_control/.sdk ]]; then
  git init arm_control/.sdk
  git -C arm_control/.sdk remote add origin https://github.com/norma-core/norma-core.git
  git -C arm_control/.sdk sparse-checkout set hardware/elrobot/simulation
  git -C arm_control/.sdk fetch --depth 1 --filter=blob:none origin "$REV"
  git -C arm_control/.sdk checkout --detach FETCH_HEAD
else
  [[ $(git -C arm_control/.sdk rev-parse HEAD) == "$REV" ]] || {
    echo 'Existing SDK has another revision; preserved.' >&2; exit 1;
  }
  [[ -z $(git -C arm_control/.sdk status --porcelain) ]] || {
    echo 'Existing SDK has local changes; preserved.' >&2; exit 1;
  }
fi
for component in arm ocr; do
  if [[ $component == arm ]]; then envdir=arm_control/.venv; else envdir=base_ocr/.venv; fi
  if [[ -e $envdir ]]; then
    echo "Existing environment preserved: $envdir. Use a fresh clone to restore pinned packages." >&2
    exit 1
  fi
  "$PYTHON" -m venv "$envdir"
  "$envdir/bin/python" -m pip install --no-cache-dir -r "restore/requirements-$component.lock"
  "$envdir/bin/python" -m pip check
done
"$PYTHON" restore/models.py
IMAGE=ghcr.io/epflxplore/base@sha256:3ee3faca2959b2eeb904babdabff40928d4e2d7d90afa00055e7a78a950b88b4
docker pull "$IMAGE"
echo 'CPU planner, OCR models and ROS image restored.'
echo 'For NVIDIA GPU planning: bash arm_control/curobo/restore.sh'
echo "Launch manually with STATION_IMAGE=$IMAGE ROBOT_WIFI_IP=<robot-ip> bash ros2_robot/deploy/station.sh"
