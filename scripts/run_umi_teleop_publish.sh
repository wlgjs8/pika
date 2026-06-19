#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [[ -z "${DISPLAY:-}" && -S /tmp/.X11-unix/X0 ]]; then
  export DISPLAY=:0
fi
if [[ -z "${XAUTHORITY:-}" && -r "$HOME/.Xauthority" ]]; then
  export XAUTHORITY="$HOME/.Xauthority"
fi

exec python3 scripts/umi_teleop_publish.py \
  --pedal \
  --swap-lr \
  --target-host 172.28.60.12 \
  # --pedal-toggle \
  "$@"
