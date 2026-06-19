#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

export PIKA_VIEW_LAN_IPS="${PIKA_VIEW_LAN_IPS:-172.28.60.40}"

CONDA_ENV="${CONDA_ENV:-pika}"
VIEW="${PIKA_VIEW:-none}"
CONFIG="${PIKA_CONFIG:-}"
ARM_NAMES="${PIKA_ARM_NAMES:-right,left}"
COMS="${PIKA_COMS:-/dev/ttyUSB0,/dev/ttyUSB1}"
RS_SNS="${PIKA_RS_SNS:-419122270010,260522277606}"
TRACKER_SNS="${PIKA_TRACKER_SNS:-LHR-29E2B23B,LHR-40B32551}"
PNG_COMPRESSION="${PIKA_PNG_COMPRESSION:-1}"
PNG_DEPTH_COMPRESSION="${PIKA_PNG_DEPTH_COMPRESSION:--1}"
ENCODE_WORKERS="${PIKA_ENCODE_WORKERS:-4}"
SAVE_MAX_PENDING="${PIKA_SAVE_MAX_PENDING:-3}"

exec conda run --no-capture-output -n "$CONDA_ENV" python scripts/collect.py \
  --view "$VIEW" \
  --config "$CONFIG" \
  --require-pose \
  --require-all-trackers \
  --arm-names "$ARM_NAMES" \
  --coms "$COMS" \
  --rs-sns "$RS_SNS" \
  --tracker-sns "$TRACKER_SNS" \
  --png-compression "$PNG_COMPRESSION" \
  --png-depth-compression "$PNG_DEPTH_COMPRESSION" \
  --encode-workers "$ENCODE_WORKERS" \
  --save-max-pending "$SAVE_MAX_PENDING" \
  "$@"
