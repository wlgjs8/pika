#!/usr/bin/env bash
# 공통 프렐류드: 이 저장소가 쓰는 발판(1구) 경로 결정. 수집/teleop 스크립트가 source 한다.
#
# 이 리그에는 PCsensor 발판이 2개 있고 **소프트웨어로 구별이 불가능**하다:
# VID:PID(3553:b001)·bcdDevice·인터페이스 구성·evdev capability 비트맵이 전부 동일하고
# 시리얼이 없다. 물리 USB 포트(by-path)만이 유일한 식별자다.
#   1구 = pika (수집 녹화 토글 / teleop 클러치)   ← 이 파일이 고르는 것
#   3구 = robotics_lab rb_gui (a/c=InitMotion, b=record toggle)
# 수집과 teleop 을 동시에 돌리지 않으므로 1구를 두 용도가 공유한다.
#
# /dev/input/by-id 를 쓰면 안 된다 — udev 가 이름 충돌을 피해 먼저 잡힌 하나에만
# 심링크를 만들고 나머지에는 아예 만들지 않아서, 꽂는 순서에 따라 다른 발판을 가리키고
# 두 번째 발판은 주소 지정 자체가 불가능하다.
#
# 결정 순서 (2026-09-04: 경로 하드코딩이 USB 포트 이동으로 깨져서 바꿨다):
#   1) PEDAL_DEVICE 환경변수
#   2) config/pedal_device.local 파일 (한 번 정해두면 유지. *.local 은 gitignore 됨)
#   3) 발판이 **정확히 1개**면 그것        ← 포트만 옮긴 경우 그냥 동작
#   4) 0개 또는 2개 이상이면 **실패**      ← 절대 추측하지 않는다
#
# (3)에서 추측하지 않는 이유: 두 발판이 구별 불가라 잘못 고르면 텔레옵 클러치가 엉뚱한
# 발판에 붙는다. 밟아도 안 움직이거나, 더 나쁘게는 3구를 밟았을 때 팔이 움직인다.

_PEDAL_LOCAL="config/pedal_device.local"
if [[ -z "${PEDAL_DEVICE:-}" && -s "$_PEDAL_LOCAL" ]]; then
  PEDAL_DEVICE="$(head -n1 "$_PEDAL_LOCAL" | tr -d '[:space:]')"
  echo "[pedal] $_PEDAL_LOCAL 에서 읽음: $PEDAL_DEVICE" >&2
fi

if [[ -z "${PEDAL_DEVICE:-}" ]]; then
  _pedal_found=()
  while IFS= read -r line; do
    [[ -n "$line" ]] && _pedal_found+=("$line")
  done < <("${PY_CMD[@]}" - <<'PYEOF' 2>/dev/null
import os, sys
sys.path.insert(0, os.getcwd())   # 호출 스크립트가 저장소 루트로 cd 한 뒤 source 한다
try:
    from pika_win.pedal import find_pedal_devices
    for p in find_pedal_devices():
        print(p)
except Exception:
    pass
PYEOF
)
  if [[ ${#_pedal_found[@]} -eq 1 ]]; then
    PEDAL_DEVICE="${_pedal_found[0]}"
    echo "[pedal] 발판 1개 자동 선택: $PEDAL_DEVICE" >&2
  elif [[ ${#_pedal_found[@]} -eq 0 ]]; then
    echo "[pedal] 발판(FootSwitch)을 찾지 못했습니다. USB 연결을 확인하세요." >&2
    echo "[pedal]   확인: ${PY_CMD[*]} scripts/pedal_test.py" >&2
    exit 1
  else
    echo "[pedal] 발판이 ${#_pedal_found[@]}개 있어 자동 선택하지 않습니다 —" >&2
    echo "[pedal] 둘은 VID:PID 가 같고 시리얼이 없어 소프트웨어로 구별할 수 없습니다." >&2
    echo "[pedal] 어느 것이 1구인지 확인한 뒤(밟으면 표시됩니다):" >&2
    echo "[pedal]   ${PY_CMD[*]} scripts/pedal_test.py" >&2
    echo "[pedal] 그 경로를 지정해서 다시 실행하세요:" >&2
    for _p in "${_pedal_found[@]}"; do
      echo "[pedal]   PEDAL_DEVICE=$_p $0" >&2
    done
    echo "[pedal] 매번 지정하기 싫으면 한 번만 기록해 두세요:" >&2
    echo "[pedal]   echo <경로> > $_PEDAL_LOCAL" >&2
    exit 1
  fi
fi

if [[ ! -e "$PEDAL_DEVICE" ]]; then
  echo "[pedal] 지정된 발판 경로가 없습니다: $PEDAL_DEVICE" >&2
  echo "[pedal] ${PY_CMD[*]} scripts/pedal_test.py 로 확인 후 PEDAL_DEVICE=... 로 지정하세요." >&2
  exit 1
fi
