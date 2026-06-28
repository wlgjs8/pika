#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/sync_data_to_target.sh [TARGET_IP]

Reads .env from the repo root by default.

Required in .env or environment:
  PASSWD='target_pc_password'

Optional in .env or environment:
  TARGET_IP=192.168.8.50
  TARGET_USER=plaif
  SOURCE_DIR=./data/
  TARGET_DIR=/data/pika/bolt/data/

Examples:
  ./scripts/sync_data_to_target.sh
  ./scripts/sync_data_to_target.sh 192.168.8.50
  TARGET_IP=192.168.8.50 ./scripts/sync_data_to_target.sh
  DRY_RUN=1 ./scripts/sync_data_to_target.sh
EOF
}

if [[ "${1-}" == "-h" || "${1-}" == "--help" ]]; then
  usage
  exit 0
fi

if (( $# > 1 )); then
  usage >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

ARG_TARGET_IP="${1-}"

ENV_FILE="${ENV_FILE:-.env}"
ENV_TARGET_IP="${TARGET_IP-}"
ENV_TARGET_USER="${TARGET_USER-}"
ENV_SOURCE_DIR="${SOURCE_DIR-}"
ENV_TARGET_DIR="${TARGET_DIR-}"
ENV_PASSWD="${PASSWD-}"
ENV_DRY_RUN="${DRY_RUN-}"

if [[ -f "$ENV_FILE" ]]; then
  set -a
  set +u
  # shellcheck disable=SC1090
  source "$ENV_FILE"
  set -u
  set +a
fi

TARGET_IP="${ARG_TARGET_IP:-${ENV_TARGET_IP:-${TARGET_IP:-192.168.8.50}}}"
TARGET_USER="${ENV_TARGET_USER:-${TARGET_USER:-plaif}}"
SOURCE_DIR="${ENV_SOURCE_DIR:-${SOURCE_DIR:-./data/}}"
TARGET_DIR="${ENV_TARGET_DIR:-${TARGET_DIR:-/data/pika/bolt/data/}}"
PASSWD="${ENV_PASSWD:-${PASSWD:-}}"
DRY_RUN="${ENV_DRY_RUN:-${DRY_RUN:-0}}"

if [[ -z "$PASSWD" ]]; then
  echo "ERROR: PASSWD is not set. Add PASSWD='target_pc_password' to $ENV_FILE or export PASSWD." >&2
  exit 1
fi

if [[ ! -d "$SOURCE_DIR" ]]; then
  echo "ERROR: source directory does not exist: $SOURCE_DIR" >&2
  exit 1
fi

if ! command -v sshpass >/dev/null 2>&1; then
  echo "ERROR: sshpass is not installed." >&2
  echo "Install it with: sudo apt-get update && sudo apt-get install -y sshpass" >&2
  exit 1
fi

if ! command -v rsync >/dev/null 2>&1; then
  echo "ERROR: rsync is not installed." >&2
  echo "Install it with: sudo apt-get update && sudo apt-get install -y rsync" >&2
  exit 1
fi

SOURCE_DIR="${SOURCE_DIR%/}/"
TARGET_DIR="${TARGET_DIR%/}/"
TARGET_DIR_NO_TRAIL="${TARGET_DIR%/}"

SSH_CMD="ssh -o StrictHostKeyChecking=accept-new"
printf -v REMOTE_TARGET_DIR "%q" "$TARGET_DIR_NO_TRAIL"

RSYNC_ARGS=(
  --archive
  --human-readable
  --info=progress2
  --partial
  --stats
)

case "${DRY_RUN,,}" in
  1|true|yes|y|on)
    RSYNC_ARGS+=(--dry-run)
    echo "[sync] dry run enabled"
    ;;
esac

echo "[sync] source: $SOURCE_DIR"
echo "[sync] target: ${TARGET_USER}@${TARGET_IP}:${TARGET_DIR}"

export SSHPASS="$PASSWD"

sshpass -e $SSH_CMD "${TARGET_USER}@${TARGET_IP}" "mkdir -p -- $REMOTE_TARGET_DIR"

sshpass -e rsync "${RSYNC_ARGS[@]}" \
  -e "$SSH_CMD" \
  "$SOURCE_DIR" \
  "${TARGET_USER}@${TARGET_IP}:${TARGET_DIR}"

unset SSHPASS

echo "[sync] done"
