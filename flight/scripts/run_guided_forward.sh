#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"

ALTITUDE="${1:-${ALTITUDE:-0.8}}"
DISTANCE="${2:-${DISTANCE:-1.5}}"
SPEED="${3:-${SPEED:-0.2}}"

exec "${PYTHON_BIN}" "${ROOT_DIR}/src/guided_forward_test.py" \
  --altitude "${ALTITUDE}" \
  --distance "${DISTANCE}" \
  --speed "${SPEED}"
