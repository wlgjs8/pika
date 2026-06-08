#!/usr/bin/env python
"""에피소드 1개 기록 — 포즈/그리퍼/RealSense를 동기화해 HDF5로 저장.

실행 예:
  conda run -n pika python scripts\\record_episode.py --duration 5 --name ep001
  conda run -n pika python scripts\\record_episode.py --duration 5 --no-realsense   # 디버그
전제: SteamVR 실행(포즈), PIKA Sense USB 시리얼 연결, RealSense 연결.
"""
import argparse
import glob
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from pika_win.recorder import EpisodeRecorder  # noqa: E402
from pika_win.sdk_logging import quiet_pika_sdk_info  # noqa: E402


def _default_com():
    if os.name == "nt":
        return "COM3"
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    if by_id:
        return by_id[0]
    tty_usb = sorted(glob.glob("/dev/ttyUSB*"))
    if tty_usb:
        return tty_usb[0]
    return "/dev/ttyUSB0"


def _default_rs_sn():
    value = os.environ.get("PIKA_RS_SN", "")
    if value:
        return value
    return "260522277606" if os.name == "nt" else ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--name", default="episode")
    ap.add_argument("--out", default=os.path.join(REPO_ROOT, "data"))
    ap.add_argument("--com", default=_default_com())
    ap.add_argument("--rs-sn", default=_default_rs_sn())
    ap.add_argument("--no-pose", action="store_true")
    ap.add_argument("--no-sense", action="store_true")
    ap.add_argument("--no-realsense", action="store_true")
    a = ap.parse_args()

    quiet_pika_sdk_info()
    rec = EpisodeRecorder(
        out_dir=a.out, record_hz=a.hz, com_port=a.com,
        realsense_sn=(a.rs_sn or None),
        use_pose=not a.no_pose, use_sense=not a.no_sense,
        use_realsense=not a.no_realsense)
    rec.start()
    try:
        rec.record(duration=a.duration, name=a.name)
    finally:
        rec.stop()


if __name__ == "__main__":
    main()
