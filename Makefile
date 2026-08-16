# Makefile for PIKA UMI collection

# 기본은 이 저장소의 venv(.venv/bin/python) — 이 PC 관례(robotics_lab/.venv,
# openpi/.venv)와 동일하고 conda 설치가 필요 없다.
#   uv venv --python 3.10 .venv
#   uv pip install --python .venv/bin/python pyserial numpy opencv-python h5py pyrealsense2 pillow
#   uv pip install --python .venv/bin/python --no-deps agx-pypika
#
# conda 환경을 쓰려면:  make run PY="conda run --no-capture-output -n pika python"
PY     ?= .venv/bin/python
SCRIPT ?= scripts/collect.py

.PHONY: run view identify pose-test

# 좌/우 팔 식별 마법사 → config/arms.json 저장 (실행 전 run/view 종료할 것)
identify:
	$(PY) scripts/identify_arms.py

# 트래커 1개=한팔 / 2개=양팔 자동. 양팔 하드웨어는 ARGS로 전달:
#   make run ARGS="--coms /dev/serial/by-path/...,... --rs-sns SN_R,SN_L --tracker-sns LHR-R,LHR-L"
#   (config/arms.json 이 있으면 그게 우선; 보통 make identify 로 생성)
ARGS ?=

# 헤드리스 수집 + 진단 로깅
run:
	$(PY) $(SCRIPT) $(ARGS)

# 수집 + rerun 라이브 뷰어(브라우저, 양팔이면 양쪽 표시). 네이티브 창은 VIEW=spawn
VIEW ?= web
view:
	$(PY) $(SCRIPT) --view $(VIEW) $(ARGS)

# 포즈 백엔드 스모크 테스트 (libsurvive, SteamVR 불필요)
pose-test:
	$(PY) scripts/pose_test_survive.py $(ARGS)
