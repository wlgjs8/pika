#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=scripts/_venv.sh
source scripts/_venv.sh

# 뷰어 URL 에 광고할 LAN IP. 예전 수집 PC(172.28.60.40) 하드코딩을 제거했다 —
# 필요할 때만 지정하고, 없으면 뷰어가 자동 탐지한다.
if [[ -n "${PIKA_VIEW_LAN_IPS:-}" ]]; then
  export PIKA_VIEW_LAN_IPS
fi

VIEW="${PIKA_VIEW:-web}"
PNG_COMPRESSION="${PIKA_PNG_COMPRESSION:-1}"
PNG_DEPTH_COMPRESSION="${PIKA_PNG_DEPTH_COMPRESSION:--1}"
ENCODE_WORKERS="${PIKA_ENCODE_WORKERS:-4}"
SAVE_MAX_PENDING="${PIKA_SAVE_MAX_PENDING:-3}"

# 하드웨어는 config/arms.json 이 유일한 출처다(collect.py 기본값).
# 예전에는 여기서 --config '' 로 arms.json 을 무시하고 --coms /dev/ttyUSB0,/dev/ttyUSB1 을
# 강제했는데, 그 두 포트는 지금 **로봇팔 그리퍼**이고 robotics_lab gripper_server 가
# 점유한다 — 그대로 두면 엉뚱한 장치를 열거나 충돌한다. 게다가 ttyUSB 번호는 재부팅마다
# 바뀐다. arms.json 은 by-path 로 고정돼 있으므로 그쪽을 쓴다.
# 임시로 다른 하드웨어를 쓰려면 아래 env 를 명시할 때만 CLI 인자가 붙는다.
ARGS=()
[[ -n "${PIKA_CONFIG:-}" ]]      && ARGS+=(--config "$PIKA_CONFIG")
[[ -n "${PIKA_ARM_NAMES:-}" ]]   && ARGS+=(--arm-names "$PIKA_ARM_NAMES")
[[ -n "${PIKA_COMS:-}" ]]        && ARGS+=(--coms "$PIKA_COMS")
[[ -n "${PIKA_RS_SNS:-}" ]]      && ARGS+=(--rs-sns "$PIKA_RS_SNS")
[[ -n "${PIKA_TRACKER_SNS:-}" ]] && ARGS+=(--tracker-sns "$PIKA_TRACKER_SNS")

exec "${PY_CMD[@]}" scripts/collect.py \
  --view "$VIEW" \
  --require-pose \
  --require-all-trackers \
  --png-compression "$PNG_COMPRESSION" \
  --png-depth-compression "$PNG_DEPTH_COMPRESSION" \
  --encode-workers "$ENCODE_WORKERS" \
  --save-max-pending "$SAVE_MAX_PENDING" \
  "${ARGS[@]}" \
  "$@"
