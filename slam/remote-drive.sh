#!/usr/bin/env bash
set -euo pipefail
# Override RDK_HOST if DHCP gives the board a different address.
rdk_host="${RDK_HOST:-sunrise@172.20.10.3}"
if (( $# == 0 )); then
    set -- --keyboard
fi
printf -v rover_args ' %q' "$@"
exec ssh -t "$rdk_host" "cd ~/humanoid-hackathon/slam && ./rdk-drive.sh$rover_args"
