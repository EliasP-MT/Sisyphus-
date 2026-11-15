#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")/.."
export OPENBLAS_NUM_THREADS="${POSTAL_TWIN_BLAS_THREADS:-1}"
export OMP_NUM_THREADS="${POSTAL_TWIN_OMP_THREADS:-1}"
exec arm_control/.venv/bin/python -m digital_twin.run "$@"
