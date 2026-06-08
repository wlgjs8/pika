#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

export PIKA_VIEW_LAN_IPS="${PIKA_VIEW_LAN_IPS:-172.28.60.40}"

CONDA_ENV="${CONDA_ENV:-pika}"
PORT="${PIKA_REVIEW_PORT:-8088}"

exec conda run --no-capture-output -n "$CONDA_ENV" python scripts/review_episode.py \
  --port "$PORT" \
  "$@"
