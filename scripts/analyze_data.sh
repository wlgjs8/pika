#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=scripts/_venv.sh
source scripts/_venv.sh

exec "${PY_CMD[@]}" scripts/analyze_data.py "$@"
