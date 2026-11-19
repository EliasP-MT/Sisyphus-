#!/usr/bin/env bash
# Recreate the cuRobo environment without starting the station or robot.
set -euo pipefail
export UV_HTTP_TIMEOUT=${UV_HTTP_TIMEOUT:-600}
export UV_CONCURRENT_DOWNLOADS=${UV_CONCURRENT_DOWNLOADS:-4}
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
DEST=${CUROBO_ROOT:-$HOME/hack/curobo-src}
PYTHON=${PYTHON:-python3.12}
REV=71ce1726925adfdcc7d0d39dabec5f2d6bb71acd
if [[ -e "$DEST" ]]; then
  printf 'Destination already exists: %s. Choose an empty CUROBO_ROOT.\n' "$DEST" >&2
  exit 1
fi
"$PYTHON" -c 'import sys; assert sys.version_info[:2] == (3, 12), "Python 3.12 required"'
mkdir -p -- "$DEST"
git init "$DEST/curobo"
git -C "$DEST/curobo" remote add origin https://github.com/NVlabs/curobo.git
git -C "$DEST/curobo" fetch --depth 1 origin "$REV"
git -C "$DEST/curobo" checkout --detach FETCH_HEAD
"$PYTHON" -m venv "$DEST/.venv"
"$DEST/.venv/bin/python" -m pip install --no-cache-dir uv==0.13.8
"$DEST/.venv/bin/uv" pip install --no-cache --index-strategy unsafe-best-match \
  --python "$DEST/.venv/bin/python" -r "$HERE/requirements-linux-py312.lock"
"$DEST/.venv/bin/uv" pip install --no-cache --no-deps --no-build-isolation \
  --python "$DEST/.venv/bin/python" "$DEST/curobo"
"$DEST/.venv/bin/uv" pip check --python "$DEST/.venv/bin/python"
"$DEST/.venv/bin/python" - <<'PY'
import torch, curobo, pybullet, ikpy, ruckig, ompl, OCP
from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
assert torch.cuda.is_available(), 'NVIDIA driver / GPU unavailable'
assert torch.ones(1, device='cuda').sum().item() == 1
print('cuRobo environment restored; GPU:', torch.cuda.get_device_name(0))
PY
printf 'Planner interpreter: %s/.venv/bin/python\n' "$DEST"
