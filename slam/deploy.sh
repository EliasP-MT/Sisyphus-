#!/usr/bin/env bash
# Copy source only. Does not install dependencies, start processes or move motors.
set -euo pipefail
cd "$(dirname "$0")"
check_only=0
include_calibration=()
for argument in "$@"; do
    case "$argument" in
        --check) check_only=1 ;;
        --include-calibration) include_calibration=(--include-calibration) ;;
        *) echo 'Usage: bash deploy.sh [--check] [--include-calibration]' >&2; exit 2 ;;
    esac
done
bundle_dir="$(mktemp -d)"
trap 'rm -rf "$bundle_dir"' EXIT
python3 scripts/build_bundle.py --output "$bundle_dir/runtime.tar.gz" ${include_calibration[@]+"${include_calibration[@]}"}
if ((check_only)); then
    echo 'Source bundle verified; no robot connection was opened.'
    exit 0
fi
rdk_host="${RDK_HOST:-sunrise@172.20.10.3}"
# The remote installation path is fixed; archive contents come from our allowlist.
ssh "$rdk_host" 'mkdir -p ~/humanoid-hackathon/slam'
ssh "$rdk_host" 'cd ~/humanoid-hackathon/slam && tar -xzf -' < "$bundle_dir/runtime.tar.gz"
echo 'Copied source. Install the documented dependencies in your robot environment.'
echo 'No services were started. Robot constants and station state were left untouched.'
