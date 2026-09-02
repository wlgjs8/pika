#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=scripts/_venv.sh
source scripts/_venv.sh

# 로컬 OpenCV 창으로 RGB 두 뷰를 원본 해상도로 검수한다.
# 원격 셸에서 실행할 때를 위해 X 디스플레이를 보정한다.
if [[ -z "${DISPLAY:-}" && -S /tmp/.X11-unix/X0 ]]; then
  export DISPLAY=:0
fi
if [[ -z "${XAUTHORITY:-}" && -r "$HOME/.Xauthority" ]]; then
  export XAUTHORITY="$HOME/.Xauthority"
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo "[rgb-review] DISPLAY 가 없습니다 — OpenCV 창을 띄울 수 없습니다." >&2
  echo "[rgb-review] 로컬 모니터가 붙은 세션에서 실행하거나 DISPLAY=:0 을 지정하세요." >&2
  exit 1
fi

exec "${PY_CMD[@]}" scripts/review_rgb_episode.py "$@"
