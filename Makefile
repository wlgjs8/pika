# Makefile for PIKA UMI collection
# Usage: `make run`  (works on Windows + Linux/macOS)

# Override on the command line, e.g. `make run ENV=other`
CONDA  ?= conda
ENV    ?= pika
SCRIPT ?= scripts/collect.py

.PHONY: run view identify

# 좌/우 팔 식별 마법사 → config/arms.json 저장 (실행 전 run/view 종료할 것)
identify:
	"$(CONDA)" run --no-capture-output -n $(ENV) python scripts/identify_arms.py

# 트래커 1개=한팔 / 2개=양팔 자동. 양팔 하드웨어는 ARGS로 전달:
#   make run ARGS="--coms COM3,COM5 --rs-sns SN_R,SN_L --tracker-sns LHR-R,LHR-L"
#   (config/arms.json 이 있으면 그게 우선; 보통 make identify 로 생성)
ARGS ?=

# 헤드리스 수집 + 진단 로깅
run:
	"$(CONDA)" run --no-capture-output -n $(ENV) python $(SCRIPT) $(ARGS)

# 수집 + rerun 라이브 뷰어(브라우저, 양팔이면 양쪽 표시). 네이티브 창은 VIEW=spawn
VIEW ?= web
view:
	"$(CONDA)" run --no-capture-output -n $(ENV) python $(SCRIPT) --view $(VIEW) $(ARGS)
