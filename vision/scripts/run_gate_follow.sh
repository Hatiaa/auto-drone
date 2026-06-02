#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
DEVICE="${DEVICE:-/dev/ttyTHS1}"
BAUD="${BAUD:-921600}"

exec "${PYTHON_BIN}" "${ROOT_DIR}/scripts/gate_align_hold.py" \
  --device "${DEVICE}" \
  --baud "${BAUD}" \
  --layout "${ROOT_DIR}/config/gate_follow_static_example.yaml" \
  --params "${ROOT_DIR}/camera_params.npz" \
  --camera-id 0 \
  --width 640 \
  --height 480 \
  --mode align_yz_yaw \
  --pose-source live \
  --print-interval 0.3 \
  --enable-x \
  --target-x-m 2.5 \
  --kp-x 0.39 \
  --vx-max 0.195 \
  --x-tol 0.15 \
  --deadband-x 0.08 \
  --kp-y 0.585 \
  --vy-max 0.234 \
  --kp-yaw 0.35 \
  --yaw-rate-max-deg 4 \
  --slew-yaw-deg 5 \
  --yaw-tol-deg 12 \
  "$@"

