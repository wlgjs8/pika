#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=scripts/_venv.sh
source scripts/_venv.sh

# 리뷰 서버 URL 에 광고할 LAN IP. 예전 수집 PC(172.28.60.40) 하드코딩을 제거했다 —
# 필요할 때만 지정하고, 없으면 자동 탐지한다.
if [[ -n "${PIKA_VIEW_LAN_IPS:-}" ]]; then
  export PIKA_VIEW_LAN_IPS
fi

PORT="${PIKA_REVIEW_PORT:-8088}"

exec "${PY_CMD[@]}" scripts/review_episode.py \
  --port "$PORT" \
  "$@"
