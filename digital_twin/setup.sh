#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")/.."
if [[ ! -x arm_control/.venv/bin/python ]]; then
  bash arm_control/setup.sh
fi
arm_control/.venv/bin/python arm_control/.pip.pyz install -r digital_twin/requirements.txt
arm_control/.venv/bin/python -c 'from digital_twin.model import build; from digital_twin.common import config; print(build(config())[2])'
