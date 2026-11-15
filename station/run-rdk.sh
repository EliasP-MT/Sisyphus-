#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
[[ "$(uname -m)" == aarch64 ]] || { echo 'This launcher requires the ARM64 Station binary on the RDK.' >&2; exit 1; }
exec ./station --config rdk.yaml \
    --web 127.0.0.1:8889 --tcp 127.0.0.1:8888 \
    --max-queue-disk-size 256M --max-memory-usage 128M --normfs-file-size 32M
