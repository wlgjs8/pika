#!/usr/bin/env bash
# libsurvive 빌드 + udev 규칙 설치 (Ubuntu 22.04 / WSL2). root로 실행.
set -euo pipefail

USER_NAME=vision
USER_HOME="/home/${USER_NAME}"

echo "=== [1/4] apt 의존성 설치 ==="
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y build-essential cmake libusb-1.0-0-dev freeglut3-dev \
  liblapacke-dev libopenblas-dev libatlas-base-dev zlib1g-dev git usbutils \
  python3-dev python3-numpy

echo "=== [2/4] libsurvive clone + build (user: ${USER_NAME}) ==="
su - "${USER_NAME}" -c 'set -e; cd ~; if [ ! -d libsurvive ]; then git clone https://github.com/cntools/libsurvive.git; fi; cd libsurvive; rm -rf bin; make -j"$(nproc)"'

echo "=== [3/4] udev 규칙 설치 ==="
cp "${USER_HOME}/libsurvive/useful_files/81-vive.rules" /etc/udev/rules.d/
udevadm control --reload-rules || true
udevadm trigger || true

echo "=== [4/4] 빌드 산출물 확인 ==="
ls -la "${USER_HOME}/libsurvive/bin/" 2>/dev/null || echo "!! bin 디렉터리 없음 — 빌드 실패 가능"
echo "=== DONE ==="
