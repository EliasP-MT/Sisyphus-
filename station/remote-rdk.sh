#!/usr/bin/env bash
# Laptop launcher. Uses alternate local ports to coexist with the laptop Station.
set -euo pipefail
echo 'RDK Station: http://127.0.0.1:18889 (TCP client port 18888). Ctrl+C closes the tunnel.'
exec ssh -t -o ConnectTimeout=8 -o ExitOnForwardFailure=yes \
    -L 127.0.0.1:18889:127.0.0.1:8889 \
    -L 127.0.0.1:18888:127.0.0.1:8888 \
    "${RDK_HOST:-sunrise@172.20.10.3}" \
    'systemctl --user start rdk-station.service && echo "Station service running. Keep this terminal open for browser access." && exec sleep infinity'
