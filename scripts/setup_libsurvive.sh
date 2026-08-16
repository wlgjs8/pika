#!/usr/bin/env bash
# libsurvive 빌드 + pysurvive 바인딩 연결 (Ubuntu, 헤드리스).
#
# GUI(freeglut/GL viewer) 는 빌드하지 않는다 — 이 셋업은 survive-cli / pysurvive
# 만 쓰는 헤드리스 경로다.
#
# apt 의존성과 udev 규칙은 root 가 필요하므로 여기서 실행하지 않고 안내만 한다:
#   sudo apt-get install -y libusb-1.0-0-dev liblapacke-dev libopenblas-dev
#   sudo install -m 644 <libsurvive>/useful_files/81-vive.rules /etc/udev/rules.d/
#   sudo udevadm control --reload-rules && sudo udevadm trigger
set -euo pipefail

LIBSURVIVE_PATH="${LIBSURVIVE_PATH:-$HOME/workspace/libsurvive}"

if ! pkg-config --exists libusb-1.0 2>/dev/null && [ ! -f /usr/include/libusb-1.0/libusb.h ]; then
  echo "!! libusb-1.0 개발 헤더가 없습니다. 먼저 실행하세요:" >&2
  echo "   sudo apt-get install -y libusb-1.0-0-dev liblapacke-dev libopenblas-dev" >&2
  exit 1
fi

echo "=== [1/3] clone ==="
if [ ! -d "$LIBSURVIVE_PATH" ]; then
  git clone https://github.com/cntools/libsurvive.git "$LIBSURVIVE_PATH"
fi

echo "=== [2/4] 로컬 패치 적용 ==="
# 업스트림 libsurvive 는 USB 전송 에러 1회마다 transfer 를 재제출한 뒤 그대로
# disconnect 경로로 떨어져, 방금 띄운 in-flight transfer 를 free 한다(use-after-free).
# 그 결과 libusb 가 usbi_mutex_lock assertion 으로 프로세스 전체를 abort 시킨다.
# 이 패치 없이는 전송 에러 한 번에 수집이 통째로 죽는다.
PATCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../patches" && pwd)"
for p in "$PATCH_DIR"/libsurvive-*.patch; do
  [ -e "$p" ] || continue
  if git -C "$LIBSURVIVE_PATH" apply --check --reverse "$p" 2>/dev/null; then
    echo "  이미 적용됨: $(basename "$p")"
  else
    git -C "$LIBSURVIVE_PATH" apply "$p"
    echo "  적용: $(basename "$p")"
  fi
done

echo "=== [3/4] build (Release, GUI 제외) ==="
cmake -S "$LIBSURVIVE_PATH" -B "$LIBSURVIVE_PATH/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$LIBSURVIVE_PATH/build" -j"$(nproc)"

echo "=== [4/4] pysurvive 가 libsurvive.so 를 찾도록 심링크 ==="
# pysurvive.CustomLibraryLoader 는 패키지 디렉터리를 탐색한다 → 여기에 걸어두면
# LD_LIBRARY_PATH 없이도 임포트된다.
ln -sf "$LIBSURVIVE_PATH/build/libsurvive.so" \
       "$LIBSURVIVE_PATH/bindings/python/pysurvive/libsurvive.so"

echo "=== DONE ==="
echo "산출물: $LIBSURVIVE_PATH/build/survive-cli, libsurvive.so"
echo "udev 규칙(28de:2300 트래커 권한)이 아직이면:"
echo "  sudo install -m 644 $LIBSURVIVE_PATH/useful_files/81-vive.rules /etc/udev/rules.d/"
echo "  sudo udevadm control --reload-rules && sudo udevadm trigger"
