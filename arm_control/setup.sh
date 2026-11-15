#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
python3 -m venv --without-pip .venv
if [[ ! -f .pip.pyz ]]; then
  curl --fail --location https://bootstrap.pypa.io/pip/pip.pyz -o .pip.pyz
fi
.venv/bin/python .pip.pyz install -r requirements-collision.txt
# Same official revision as hand_control/setup.sh. No Station launch or motor writes.
if [[ ! -d .sdk/.git ]]; then
  git init .sdk
  git -C .sdk remote add origin https://github.com/norma-core/norma-core.git
fi
git -C .sdk sparse-checkout set hardware/elrobot/simulation software/station/shared/station_py shared/gremlin_py target/gen_python
git -C .sdk fetch --depth 1 --filter=blob:none origin 6b494f77fac366a878ae71222458b196f8400dee
git -C .sdk checkout --detach FETCH_HEAD
.venv/bin/python control.py --help
