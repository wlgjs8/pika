"""PIKA 그리퍼 어안(fisheye) 카메라 리더 — V4L2 UVC, 백그라운드 스레드로 최신 프레임 보관.

DECXIN/Sunplus UVC 어안 카메라(BGR). RealSense 리더(realsense_win.py)와 같은
'최신 프레임 1장 보관' 패턴. 두 어안 카메라는 USB 시리얼이 동일("01.00.00")해
by-id 로 구분 불가 → RealSense 와 같은 USB 허브에 물려 있다는 점을 이용해
by-path(USB 포트)로 팔에 매핑한다(resolve_fisheye_node 참조).

전송은 MJPG(저대역폭, 같은 허브의 RealSense 와 USB 대역폭 경합 최소화),
저장은 recorder 에서 BGR 디코드 후 PNG 무손실 인코딩(RealSense color 와 동일 경로).
"""
import glob
import logging
import os
import re
import threading
import time

import cv2

log = logging.getLogger("pika.fisheye")


def _hub_of(port):
    """RealSense physical_port / by-path 문자열에서 (PCI 컨트롤러, 루트 포트) 추출.

    반환은 by-path 링크에 그대로 대조할 수 있는 접두 문자열이다.
      physical_port(sysfs): '/sys/.../0000:79:00.4/usb10/10-1/10-1.2/...'
        -> 'pci-0000:79:00.4-usb-0:1.'
      by-path:              'pci-0000:13:00.0-usb-0:3.1.1:1.0-video...'
        -> 'pci-0000:13:00.0-usb-0:3.'

    **PCI 주소를 반드시 포함해야 한다.** xHCI 컨트롤러가 둘 이상이면 루트 포트
    번호만으로는 구분이 안 된다(컨트롤러 A의 포트 1과 B의 포트 1이 둘 다 'usb-0:1.'
    에 매칭 → 정렬상 앞선 쪽의 카메라를 잘못 집는다). 한 RealSense 의 USB3 버스와
    짝 어안의 USB2 버스는 버스 번호는 다르지만 같은 PCI 주소를 공유하므로
    이 접두는 두 스트림을 정확히 같은 물리 포트로 묶는다.
    """
    if not port:
        return None
    # sysfs: '.../<pci_addr>/usb<bus>/<bus>-<rootport>[/...]'
    m = re.search(r"(0000:[0-9a-f]{2}:[0-9a-f]{2}\.\d)/usb\d+/\d+-(\d+)", port)
    if m:
        return f"pci-{m.group(1)}-usb-0:{m.group(2)}."
    # 이미 by-path 형태: 'pci-<addr>-usb-0:<rootport>.<...>'
    m = re.search(r"(pci-0000:[0-9a-f]{2}:[0-9a-f]{2}\.\d-usb-0:\d+\.)", port)
    if m:
        return m.group(1)
    return None


def _v4l_name(dev):
    """/dev/videoN 의 V4L2 디바이스 이름(예: 'DECXIN CAMERA'). 실패 시 None."""
    base = os.path.basename(dev)
    try:
        with open(f"/sys/class/video4linux/{base}/name", encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return None


def resolve_fisheye_node(realsense_port, model_hint="DECXIN"):
    """RealSense 의 USB 허브와 같은 허브에 물린 어안 카메라 capture 노드(/dev/videoN) 반환.

    같은 컨트롤러·같은 루트 포트(_hub_of 접두) + 모델명이 model_hint 인
    video-index0 노드를 찾는다. 없으면 None. (Windows/by-path 미존재 환경에서도 None)
    """
    if os.name == "nt":
        return None
    prefix = _hub_of(realsense_port)
    if not prefix:
        return None
    for link in sorted(glob.glob("/dev/v4l/by-path/*-video-index0")):
        if prefix not in link:
            continue
        dev = os.path.realpath(link)
        name = _v4l_name(dev)
        if name and model_hint.upper() in name.upper():
            return dev
    return None


class FisheyeCamera:
    def __init__(self, device, width=640, height=480, fps=30, fourcc="MJPG"):
        # device: '/dev/videoN'(str), N(int), 또는 인덱스 문자열
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.fourcc = fourcc
        self.cap = None
        self._frame = None
        self._ts = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def _candidates(self):
        d = self.device
        cands = [d]
        if isinstance(d, str) and d.startswith("/dev/video"):
            try:
                cands.append(int(d.replace("/dev/video", "")))
            except ValueError:
                pass
        elif isinstance(d, str) and d.isdigit():
            cands.append(int(d))
        return cands

    def connect(self):
        cap = None
        for cand in self._candidates():
            c = cv2.VideoCapture(cand, cv2.CAP_V4L2)
            if c.isOpened():
                cap = c
                break
            c.release()
        if cap is None:
            raise RuntimeError(f"[fisheye] open 실패: {self.device}")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap = cap
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        log.info("[fisheye] %s open %dx%d@%d %s", self.device, aw, ah, self.fps, self.fourcc)
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="Fisheye", daemon=True)
        self._thread.start()
        return self

    def _loop(self):
        while self._running:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                time.sleep(0.005)
                continue
            with self._lock:
                self._frame, self._ts = frame, time.time()

    def get_frame(self):
        """(bgr or None, timestamp)."""
        with self._lock:
            if self._frame is None:
                return None, None
            return self._frame.copy(), self._ts

    def disconnect(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
