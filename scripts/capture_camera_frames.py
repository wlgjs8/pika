#!/usr/bin/env python
"""카메라 인덱스별 프레임 1장 캡처 -> PNG 저장 (어안 vs RealSense-RGB 식별용).

실행: conda run -n pika python scripts\\capture_camera_frames.py
"""
import os
import sys

import cv2

OUT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INDICES = [int(a) for a in sys.argv[1:]] or [0, 2]

for idx in INDICES:
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"index {idx}: open 실패")
        cap.release()
        continue
    for _ in range(8):          # 워밍업
        cap.read()
    ok, frame = cap.read()
    if ok and frame is not None:
        out = os.path.join(OUT_DIR, f"camera{idx}.png")
        cv2.imwrite(out, frame)
        print(f"index {idx}: saved {out}  shape={frame.shape}")
    else:
        print(f"index {idx}: read 실패")
    cap.release()
