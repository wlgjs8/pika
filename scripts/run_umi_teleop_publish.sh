#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [[ -z "${DISPLAY:-}" && -S /tmp/.X11-unix/X0 ]]; then
  export DISPLAY=:0
fi
if [[ -z "${XAUTHORITY:-}" && -r "$HOME/.Xauthority" ]]; then
  export XAUTHORITY="$HOME/.Xauthority"
fi

# 수신자(robotics_lab policy_runner)가 같은 PC 에서 도는 것이 기본.
# 별도 로봇 PC 로 쏘려면: TARGET_HOST=172.28.61.3 scripts/run_umi_teleop_publish.sh
TARGET_HOST="${TARGET_HOST:-127.0.0.1}"

# 이 저장소의 venv (conda 아님). pika SDK/pysurvive/pyserial 이 여기에만 있다.
PY="${PY:-.venv/bin/python}"

# teleop 클러치용 **1구 발판**을 물리 USB 포트(by-path)로 고정한다.
# 이 PC 에는 발판이 2개다: 1구(teleop) + 3구(robotics_lab rb_gui InitMotion).
# 둘은 VID:PID(3553:b001)도 evdev capability 도 완전히 동일해서 by-path 말고는 구별
# 수단이 없고, /dev/input/by-id 에는 udev 이름 충돌로 3구 쪽 심링크 하나만 생긴다.
# 고정하지 않으면 robotics_lab 발판을 밟았을 때 teleop 이 engage 되어 로봇이 움직인다.
# 발판을 다른 USB 포트로 옮기면 아래 경로가 사라지므로 PEDAL_DEVICE 로 덮어쓸 것
# (경로 확인: python scripts/pedal_test.py).
PEDAL_DEVICE="${PEDAL_DEVICE:-/dev/input/by-path/pci-0000:11:00.0-usb-0:4:1.0-event-kbd}"
if [[ ! -e "$PEDAL_DEVICE" ]]; then
  echo "[umi] 지정된 발판 경로가 없습니다: $PEDAL_DEVICE" >&2
  echo "[umi] python scripts/pedal_test.py 로 확인 후 PEDAL_DEVICE=... 로 지정하세요." >&2
  exit 1
fi

# 클러치로 안 쓰는 나머지 발판까지 점유해 키 누수를 막는다(이벤트는 안 읽음).
# 안 막으면 3구 발판을 밟는 동안 X11 오토리피트로 터미널이 'bbbb...' 로 도배된다.
#
# 평소 순서(robotics_lab `make run` 먼저 → 발행자)에서는 rb_gui 가 이미 3구를 점유하고
# 있어 여기 grab 이 실패하는데, 그건 이미 막혀 있다는 뜻이라 무해하다(경고만 남는다).
# 반대로 **발행자를 먼저 띄우면 rb_gui 가 자기 발판을 못 잡아 InitMotion 발판이 죽는다.**
# 그 순서로 쓸 일이 있으면 MUTE_OTHER_PEDALS=0 으로 끌 것.
MUTE_OTHER_PEDALS="${MUTE_OTHER_PEDALS:-1}"
MUTE_FLAG=()
if [[ "$MUTE_OTHER_PEDALS" != "0" ]]; then
  MUTE_FLAG=(--mute-other-pedals)
fi

# --pose-frame 은 기본 tip. robotics_lab stack_real.yaml 의 umi_dual_cartesian 이
# gripper_offset: [0,0,0] + tip→TCP r_align 로 짝지어져 있으므로 tip 을 유지해야 한다
# (raw 로 보내려면 수신측 legacy fallback 값으로 함께 되돌려야 함).
exec "$PY" scripts/umi_teleop_publish.py \
  --pedal \
  --pedal-device "$PEDAL_DEVICE" \
  "${MUTE_FLAG[@]}" \
  --swap-lr \
  --target-host "$TARGET_HOST" \
  --gripper-port 50382 \
  "$@"
