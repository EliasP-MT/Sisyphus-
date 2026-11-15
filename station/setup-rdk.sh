#!/usr/bin/env bash
# Run on the laptop: deploy the matching ARM64 binary and existing Python client.
set -euo pipefail
cd "$(dirname "$0")/.."
rdk_host="${RDK_HOST:-sunrise@172.20.10.3}"
archive=station/.cache/station-linux-arm64.tar.gz
checksum=9310eacc99d3247a276dc678f2ac007c4092a3978f053e1ed65dc0b51fb02f49
mkdir -p station/.cache
if [[ ! -f "$archive" ]]; then
    curl -fL --connect-timeout 10 --max-time 180 \
        https://github.com/norma-core/norma-core/releases/download/v0.1.0-beta.10/station-linux-arm64.tar.gz \
        -o "$archive.download"
    printf '%s  %s\n' "$checksum" "$archive.download" | sha256sum -c -
    mv "$archive.download" "$archive"
fi
printf '%s  %s\n' "$checksum" "$archive" | sha256sum -c -
task_tmp=$(mktemp -d)
trap 'rm -rf "$task_tmp"' EXIT
tar -xzf "$archive" -C "$task_tmp" station
ssh "$rdk_host" 'mkdir -p ~/station ~/teleop ~/.config/systemd/user'
scp "$task_tmp/station" "$rdk_host:station/station.new"
scp station/rdk.yaml station/run-rdk.sh "$rdk_host:station/"
scp -r teleop/teleop.py teleop/vendor "$rdk_host:teleop/"
scp station/rdk-station.service "$rdk_host:.config/systemd/user/"
ssh "$rdk_host" 'chmod +x ~/station/station.new ~/station/run-rdk.sh && mv ~/station/station.new ~/station/station && ~/station/station --version'
ssh "$rdk_host" 'systemctl --user daemon-reload'
echo 'Installed. Start with: bash station/remote-rdk.sh'
echo 'Existing calibration/history and the laptop Station installation were not copied or replaced.'
