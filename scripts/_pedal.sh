#!/usr/bin/env bash
# 공통 프렐류드: 이 저장소가 쓰는 발판(1구) 경로 결정. 수집/teleop 스크립트가 source 한다.
#
# 이 리그에는 PCsensor 발판이 2개 있고 **소프트웨어로 구별이 불가능**하다:
# VID:PID(3553:b001)·bcdDevice·인터페이스 구성·evdev capability 비트맵이 전부 동일하고
# 시리얼이 없다. 물리 USB 포트(by-path)만이 유일한 식별자다.
#   1구 = pika (수집 녹화 토글 / teleop 클러치)   ← 이 파일이 고정하는 것
#   3구 = robotics_lab rb_gui (a/c=InitMotion, b=record toggle)
# 수집과 teleop 을 동시에 돌리지 않으므로 1구를 두 용도가 공유한다.
#
# /dev/input/by-id 를 쓰면 안 된다 — udev 가 이름 충돌을 피해 먼저 잡힌 하나에만
# 심링크를 만들고 나머지에는 아예 만들지 않아서, 꽂는 순서에 따라 다른 발판을 가리키고
# 두 번째 발판은 주소 지정 자체가 불가능하다.
#
# 발판을 다른 USB 포트로 옮기면 아래 경로가 사라진다 → PEDAL_DEVICE 로 덮어쓸 것
# (경로 확인: .venv/bin/python scripts/pedal_test.py).

PEDAL_DEVICE="${PEDAL_DEVICE:-/dev/input/by-path/pci-0000:79:00.4-usb-0:1.1:1.0-event-kbd}"

if [[ ! -e "$PEDAL_DEVICE" ]]; then
  echo "[pedal] 지정된 발판 경로가 없습니다: $PEDAL_DEVICE" >&2
  echo "[pedal] .venv/bin/python scripts/pedal_test.py 로 확인 후 PEDAL_DEVICE=... 로 지정하세요." >&2
  exit 1
fi
