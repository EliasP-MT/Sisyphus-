#!/usr/bin/env bash
# ExecStartPre of postal-robot.service: write ~/postal_config/fastdds_robot.xml
# whitelisting the current wlan0 address (waits up to 60 s) and loopback.
set -u
DIR=$HOME/postal_config
IFACE=${POSTAL_WIFI_IFACE:-wlan0}
IP=
for _ in $(seq 1 60); do
  IP=$(ip -4 -o addr show "$IFACE" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1)
  [ -n "$IP" ] && break
  sleep 1
done
LIST="        <address>127.0.0.1</address>"
if [ -n "$IP" ]; then
  LIST="        <address>$IP</address>
$LIST"
  echo "postal DDS: Wi-Fi $IFACE $IP + loopback"
else
  echo "postal DDS: WARNING no address on $IFACE; station unreachable until restart" >&2
fi
awk -v list="$LIST" '{ if ($0 == "@WHITELIST@") print list; else print }' \
  "$DIR/fastdds_robot.xml.in" > "$DIR/fastdds_robot.xml"
