#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
python3 -m venv --without-pip .venv
if [[ ! -f .pip.pyz ]]; then
  curl --fail --location https://bootstrap.pypa.io/pip/pip.pyz -o .pip.pyz
fi
.venv/bin/python .pip.pyz install -r requirements-collision.txt
# Official NormaCore revision providing the ElRobot URDF and meshes. No motor access.
if [[ ! -d .sdk/.git ]]; then
  git init .sdk
  git -C .sdk remote add origin https://github.com/norma-core/norma-core.git
fi
git -C .sdk sparse-checkout set hardware/elrobot/simulation
git -C .sdk fetch --depth 1 --filter=blob:none origin 6b494f77fac366a878ae71222458b196f8400dee
git -C .sdk checkout --detach FETCH_HEAD
.venv/bin/python -c 'import planner, collision; print("arm_control planning environment ready")'
