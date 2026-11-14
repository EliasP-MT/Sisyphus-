#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
if ! .venv/bin/python -c 'import mediapipe, cv2' >/dev/null 2>&1; then
  echo 'Installe les dépendances avec ./hand_control/setup.sh depuis la racine.' >&2
  exit 1
fi
exec .venv/bin/python control.py "$@"
