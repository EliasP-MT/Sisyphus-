#!/usr/bin/env bash
set -euo pipefail
rdk_host="${RDK_HOST:-sunrise@172.20.10.3}"
echo 'Camera: http://127.0.0.1:8766 — leave this terminal open; Ctrl+C to stop.'
exec ssh -t -o ExitOnForwardFailure=yes \
    -L 127.0.0.1:8766:127.0.0.1:8766 "$rdk_host" \
    'cd ~/humanoid-hackathon/slam && bash rdk-camera.sh'
