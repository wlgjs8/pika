#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

exec python3 scripts/umi_teleop_publish.py \
  --pedal \
  --swap-lr \
  --target-host 172.28.60.12 \
  "$@"
