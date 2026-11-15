#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
runtime_dir="${1:-arm_control/runtime.local}"
for runtime_file in calibration.local.json scene.local.json poses.local.json; do
  [[ -f "$runtime_dir/$runtime_file" ]] || { echo "Missing $runtime_dir/$runtime_file: extract the runtime USB bundle first." >&2; exit 1; }
done
export PYTHONDONTWRITEBYTECODE=1
exec arm_control/.venv/bin/python arm_control/calibration_ui.py \
  --config "$runtime_dir/calibration.local.json" \
  --verified-config "$runtime_dir/calibration.local.json" \
  --scene "$runtime_dir/scene.local.json" \
  --poses "$runtime_dir/poses.local.json" \
  --cad-root "$runtime_dir/preview"
