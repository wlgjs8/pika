#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=scripts/_venv.sh
source scripts/_venv.sh
# shellcheck source=scripts/_pedal.sh  (1구 발판 고정 — 수집과 공유)
source scripts/_pedal.sh

if [[ -z "${DISPLAY:-}" && -S /tmp/.X11-unix/X0 ]]; then
  export DISPLAY=:0
fi
if [[ -z "${XAUTHORITY:-}" && -r "$HOME/.Xauthority" ]]; then
  export XAUTHORITY="$HOME/.Xauthority"
fi

# 수신자(robotics_lab policy_runner)가 같은 PC 에서 도는 것이 기본.
# 별도 로봇 PC 로 쏘려면: TARGET_HOST=172.28.61.3 scripts/run_umi_teleop_publish.sh
TARGET_HOST="${TARGET_HOST:-127.0.0.1}"

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
# 이 표준 래퍼는 양팔 전용이다. config 의 트래커가 하나라도 없으면 SINGLE 로 조용히
# 내려가지 않고 Sense 연결/UDP 송신 전에 실패시킨다. 단팔 진단은 Python 스크립트를
# --require-all-trackers 없이 직접 실행한다.
# 다른 사용자의 베이스스테이션 제외는 **발행자 기본값**이다
# (pika_win/libsurvive_config.py 의 EXCLUDED_IDS — 텔레옵·수집·캘리브가 공유하는 단일 출처).
# 여기서 다시 지정하지 않는다. 임시로 바꾸려면:
#   PIKA_LIGHTHOUSE_EXCLUDE_IDS="" scripts/run_umi_teleop_publish.sh        # 제외 없음
#   PIKA_LIGHTHOUSE_EXCLUDE_IDS="5a2c575b 다른id" scripts/run_umi_teleop_publish.sh

exec "${PY_CMD[@]}" scripts/umi_teleop_publish.py \
  --pedal \
  --pedal-device "$PEDAL_DEVICE" \
  "${MUTE_FLAG[@]}" \
  --require-all-trackers \
  --swap-lr \
  --target-host "$TARGET_HOST" \
  --gripper-port 50382 \
  "$@"
