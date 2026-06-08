#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

CONDA_ENV="${CONDA_ENV:-pika}"

if command -v conda >/dev/null 2>&1; then
  exec conda run --no-capture-output -n "$CONDA_ENV" python scripts/analyze_data.py "$@"
fi

exec python scripts/analyze_data.py "$@"
