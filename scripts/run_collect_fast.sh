#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=scripts/_venv.sh
source scripts/_venv.sh
# shellcheck source=scripts/_pedal.sh
source scripts/_pedal.sh

# 90fps RGB-only 수집 (어안·depth 비활성, 수집과 격리된 웹 프리뷰).
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
# 프리뷰 기본값은 localhost MJPEG(10FPS). resize/JPEG/network는 별도 프로세스가
# 담당하고 수집 측 공유 버퍼는 non-blocking/latest-only라 느린 client를 기다리지 않는다.
# 기본은 auto — **같은 명령을 로컬에서도 SSH 에서도 그대로** 쓰기 위한 것이다.
#   로컬(디스플레이 있고 SSH 아님) -> x11 : 기존 OpenCV 창. 'b'=REC 토글, 'q'=창 닫기.
#                                          수집 중 실제로 쓰는 조작이 여기 다 들어있다.
#   SSH / 디스플레이 없음          -> web : 브라우저 MJPEG.
#
# SSH 세션이면 DISPLAY 가 있어도 web 을 고른다. X11 forwarding 으로 영상 창을 띄우면
# 프레임마다 전체 비트맵이 넘어가 링크를 다 먹는다 — 그 상황에서는 JPEG 스트림이 낫다.
#
#   PIKA_PREVIEW=0             전체 프리뷰 끄기(기존 env 호환)
#   PIKA_PREVIEW_MODE=x11      강제 OpenCV 창
#   PIKA_PREVIEW_MODE=web      강제 브라우저
#   PIKA_PREVIEW_MODE=off      전체 프리뷰 끄기
#   PIKA_PREVIEW_HOST=0.0.0.0  web 을 LAN 에 공개(인증 없음 — 신뢰된 망 전용)
#
# web 접속 방법(로컬 주소 / SSH -L 명령 / LAN 주소)은 기동 로그가 찍어준다.
PREVIEW_ENABLED="${PIKA_PREVIEW:-1}"
PREVIEW_MODE="${PIKA_PREVIEW_MODE:-auto}"
if [[ "$PREVIEW_ENABLED" != "1" ]]; then
  PREVIEW_MODE="off"
fi

if [[ "$PREVIEW_MODE" == "auto" ]]; then
  # 디스플레이가 없는데 DISPLAY 만 비어 있는 경우가 있어(tty 로그인 등) 실제 X 소켓을
  # 찾아 채운다. X0 를 박지 않는 이유: 이 PC 는 :1 로 뜬다.
  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" && -z "${SSH_CONNECTION:-}" ]]; then
    for _sock in /tmp/.X11-unix/X*; do
      [[ -S "$_sock" ]] || continue
      export DISPLAY=":${_sock##*/X}"
      [[ -z "${XAUTHORITY:-}" && -r "$HOME/.Xauthority" ]] && export XAUTHORITY="$HOME/.Xauthority"
      break
    done
  fi
  if [[ -n "${SSH_CONNECTION:-}" ]]; then
    PREVIEW_MODE="web";  PREVIEW_WHY="SSH 세션"
  elif [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    PREVIEW_MODE="x11";  PREVIEW_WHY="로컬 디스플레이 ${DISPLAY:-$WAYLAND_DISPLAY}"
  else
    PREVIEW_MODE="web";  PREVIEW_WHY="디스플레이 없음"
  fi
fi
case "$PREVIEW_MODE" in
  web)
    ARGS+=(
      --web-preview
      --web-preview-host "${PIKA_PREVIEW_HOST:-127.0.0.1}"
      --web-preview-port "${PIKA_PREVIEW_PORT:-8765}"
      --web-preview-fps "${PIKA_PREVIEW_FPS:-10}"
      --web-preview-tile-width "${PIKA_PREVIEW_TILE_WIDTH:-320}"
      --web-preview-jpeg-quality "${PIKA_PREVIEW_JPEG_QUALITY:-70}"
    )
    ;;
  x11)
    ARGS+=(--preview)
    ;;
  off)
    ;;
  *)
    echo "[collect] 잘못된 PIKA_PREVIEW_MODE=$PREVIEW_MODE (auto|web|x11|off 중 선택)" >&2
    exit 2
    ;;
esac

echo "[collect] ${HZ}Hz 기록 / RealSense ${RS_FPS}fps RGB-only / json=$RS_JSON"
echo "[collect] 발판(녹화 토글): $PEDAL_DEVICE"
echo "[collect] 프리뷰: $PREVIEW_MODE${PREVIEW_WHY:+ (auto: $PREVIEW_WHY)}${PIKA_PREVIEW_HOST:+ bind=$PIKA_PREVIEW_HOST}"

exec "${PY_CMD[@]}" scripts/collect.py \
  --hz "$HZ" \
  --pedal-device "$PEDAL_DEVICE" \
  --rs-fps "$RS_FPS" \
  --rs-json "$RS_JSON" \
  --require-pose \
  --require-all-trackers \
  --png-compression "$PNG_COMPRESSION" \
  --encode-workers "$ENCODE_WORKERS" \
  --save-max-pending "$SAVE_MAX_PENDING" \
  "${ARGS[@]}" \
  "$@"
