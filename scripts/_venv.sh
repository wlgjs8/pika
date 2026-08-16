#!/usr/bin/env bash
# 공통 프렐류드: 파이썬 인터프리터 결정. 각 실행 스크립트가 source 한다.
#
# 이 PC 에는 conda 가 없다. 저장소 로컬 venv(.venv)를 쓴다:
#   uv venv --python 3.10 .venv
#   uv pip install --python .venv/bin/python pyserial numpy opencv-python h5py pyrealsense2 pillow
#   uv pip install --python .venv/bin/python --no-deps agx-pypika
#
# 다른 인터프리터를 쓰려면 PY 로 덮어쓴다:
#   PY="conda run --no-capture-output -n pika python" ./scripts/run_collect_web.sh
#
# 주의: PY 는 공백이 들어갈 수 있으므로(위 conda 예시) 호출부에서 배열로 펼쳐 쓴다.

PY="${PY:-.venv/bin/python}"
read -r -a PY_CMD <<< "$PY"

if [[ ! -x "${PY_CMD[0]}" ]] && ! command -v "${PY_CMD[0]}" >/dev/null 2>&1; then
  echo "[pika] 파이썬 인터프리터를 찾지 못했습니다: ${PY_CMD[0]}" >&2
  echo "[pika] 저장소 루트에서 venv 를 만드세요:" >&2
  echo "         uv venv --python 3.10 .venv" >&2
  echo "         uv pip install --python .venv/bin/python pyserial numpy opencv-python h5py pyrealsense2 pillow" >&2
  echo "         uv pip install --python .venv/bin/python --no-deps agx-pypika" >&2
  echo "[pika] 또는 PY=<인터프리터> 로 지정하세요." >&2
  exit 1
fi
