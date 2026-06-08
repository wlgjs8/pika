#!/usr/bin/env python
"""SteamVR 포즈의 (1) 달성 가능 폴링 Hz, (2) 실제 갱신(값 변화) Hz 측정.

PIKA SDK 대비 어느 레이트까지 가능한지 확인용.
실행: conda run -n pika python scripts\\measure_pose_hz.py
정확한 '갱신 Hz'를 보려면 측정 중 트래커를 살살 움직이세요(정지 시 값이 반복되어 과소측정됨).
"""
import sys
import time

import openvr

DURATION = 5.0

def main():
    try:
        vr = openvr.init(openvr.VRApplication_Background)
    except Exception as e:
        sys.exit(f"openvr.init 실패 (SteamVR 실행 확인): {e}")
    n = openvr.k_unMaxTrackedDeviceCount
    polls = changes = 0
    last = None
    t0 = time.perf_counter()
    try:
        while time.perf_counter() - t0 < DURATION:
            poses = vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, n)
            cur = None
            for i in range(n):
                if vr.getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_GenericTracker:
                    p = poses[i]
                    if p.bPoseIsValid:
                        m = p.mDeviceToAbsoluteTracking
                        cur = (round(m[0][3], 6), round(m[1][3], 6), round(m[2][3], 6))
                    break
            polls += 1
            if cur is not None and cur != last:
                changes += 1
                last = cur
    finally:
        el = time.perf_counter() - t0
        openvr.shutdown()
    print(f"duration   = {el:.2f} s")
    print(f"poll rate  = {polls/el:,.0f} Hz  (폴링 헤드룸 — 원하는 어떤 target_hz도 가능)")
    print(f"update rate= {changes/el:,.0f} Hz  (실제 값 변화 — 트래커 native 추적 레이트 근사)")


if __name__ == "__main__":
    main()
