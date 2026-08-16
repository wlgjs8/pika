#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=scripts/_venv.sh
source scripts/_venv.sh

# 90fps RGB-only 수집 (어안·depth 비활성, 뷰어 없음).
#
# D405 는 640x480 에서 RGB 90fps 를 네이티브 지원한다(실측 89.5fps, drop 0).
# depth 를 끄면 스트림뿐 아니라 rs.align(depth->color) 도 사라져 프레임당 비용이 줄고,
# 저장 용량도 절반 이하가 된다(실측 93.6 MB/s, 프레임간격 p95 11.12ms).
# 포즈(libsurvive 125~245Hz)와 그리퍼(Sense 시리얼 125Hz 고정)는 이미 90Hz 를 받쳐준다.

# RealSense advanced-mode JSON. **robotics_lab 것을 직접 참조**한다 — 사본을 두지 않으므로
# 그 파일 하나만 고치면 수집·추론이 함께 따라간다(동기화 대상 없음).
RS_JSON="${RS_JSON:-/home/plaif/workspace/robotics_lab/camera_server/config/realsense_d405_90fps.json}"
if [[ ! -f "$RS_JSON" ]]; then
  # 여기서 막지 않으면 realsense_win 이 "advanced json 없음, 건너뜀" 으로 **조용히 넘어가고**
  # 카메라에 남아 있던 직전 설정(예: AE off)으로 수집돼 버린다. fail-closed.
  echo "[collect] RealSense advanced JSON 이 없습니다: $RS_JSON" >&2
  echo "[collect] robotics_lab 경로가 바뀌었으면 RS_JSON=<경로> 로 지정하세요." >&2
  exit 1
fi

HZ="${PIKA_HZ:-90}"                       # 수집(기록) 주파수
RS_FPS="${PIKA_RS_FPS:-90}"               # RealSense 스트림 fps
PNG_COMPRESSION="${PIKA_PNG_COMPRESSION:-1}"
ENCODE_WORKERS="${PIKA_ENCODE_WORKERS:-4}"
SAVE_MAX_PENDING="${PIKA_SAVE_MAX_PENDING:-3}"

# 하드웨어는 config/arms.json 이 유일한 출처다(collect.py 기본값).
# 임시로 다른 하드웨어를 쓸 때만 아래 env 를 지정하면 CLI 인자가 붙는다.
ARGS=()
[[ -n "${PIKA_CONFIG:-}" ]]      && ARGS+=(--config "$PIKA_CONFIG")
[[ -n "${PIKA_ARM_NAMES:-}" ]]   && ARGS+=(--arm-names "$PIKA_ARM_NAMES")
[[ -n "${PIKA_COMS:-}" ]]        && ARGS+=(--coms "$PIKA_COMS")
[[ -n "${PIKA_RS_SNS:-}" ]]      && ARGS+=(--rs-sns "$PIKA_RS_SNS")
[[ -n "${PIKA_TRACKER_SNS:-}" ]] && ARGS+=(--tracker-sns "$PIKA_TRACKER_SNS")

# 어안/depth 를 다시 켜려면: PIKA_FISHEYE=1 / PIKA_DEPTH=1
[[ "${PIKA_FISHEYE:-0}" == "1" ]] || ARGS+=(--no-fisheye)
[[ "${PIKA_DEPTH:-0}"   == "1" ]] || ARGS+=(--no-depth)

echo "[collect] ${HZ}Hz 기록 / RealSense ${RS_FPS}fps RGB-only / json=$RS_JSON"

exec "${PY_CMD[@]}" scripts/collect.py \
  --hz "$HZ" \
  --rs-fps "$RS_FPS" \
  --rs-json "$RS_JSON" \
  --require-pose \
  --require-all-trackers \
  --png-compression "$PNG_COMPRESSION" \
  --encode-workers "$ENCODE_WORKERS" \
  --save-max-pending "$SAVE_MAX_PENDING" \
  "${ARGS[@]}" \
  "$@"
