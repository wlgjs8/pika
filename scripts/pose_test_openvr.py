#!/usr/bin/env python
"""SteamVR + pyopenvr 트래커 6DoF 포즈 검증 스크립트 (Windows 네이티브).

WSL/libsurvive에서 실패했던 포즈 추적을, Windows의 SteamVR 경로로 대체해 검증한다.
- 전제: SteamVR가 실행 중(헤드리스 설정), 동글이 Windows에 연결, 트래커 전원 ON + 페어링됨.
- 실행: conda run -n pika python scripts\\pose_test_openvr.py
- 포즈 포맷: position (x,y,z) [m] + quaternion (x,y,z,w)  ← PIKA SDK와 동일 포맷
"""
import sys
import time
import math

try:
    import openvr
except ImportError:
    sys.exit("openvr 미설치: conda activate pika 후 pip install openvr")

CLASS_NAMES = {
    openvr.TrackedDeviceClass_HMD: "HMD",
    openvr.TrackedDeviceClass_Controller: "Controller",
    openvr.TrackedDeviceClass_GenericTracker: "Tracker",
    openvr.TrackedDeviceClass_TrackingReference: "BaseStation",
}


def mat34_to_pos_quat(m):
    """HmdMatrix34_t(3x4) -> (pos xyz, quat xyzw)."""
    x, y, z = m[0][3], m[1][3], m[2][3]
    r00, r01, r02 = m[0][0], m[0][1], m[0][2]
    r10, r11, r12 = m[1][0], m[1][1], m[1][2]
    r20, r21, r22 = m[2][0], m[2][1], m[2][2]
    tr = r00 + r11 + r22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw, qx, qy, qz = 0.25 * s, (r21 - r12) / s, (r02 - r20) / s, (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2
        qw, qx, qy, qz = (r21 - r12) / s, 0.25 * s, (r01 + r10) / s, (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2
        qw, qx, qy, qz = (r02 - r20) / s, (r01 + r10) / s, 0.25 * s, (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2
        qw, qx, qy, qz = (r10 - r01) / s, (r02 + r20) / s, (r12 + r21) / s, 0.25 * s
    return (x, y, z), (qx, qy, qz, qw)


def main():
    print("openvr init (SteamVR 실행 중이어야 함)...")
    try:
        vr = openvr.init(openvr.VRApplication_Background)
    except Exception as e:
        sys.exit(f"openvr.init 실패 — SteamVR 실행/헤드리스 설정 확인: {e}")
    print("connected. Ctrl-C 로 종료.\n")
    valid_count = 0
    try:
        while True:
            poses = vr.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
            lines = []
            for i in range(openvr.k_unMaxTrackedDeviceCount):
                cls = vr.getTrackedDeviceClass(i)
                if cls not in CLASS_NAMES:
                    continue
                p = poses[i]
                if not p.bDeviceIsConnected:
                    continue
                name = CLASS_NAMES[cls]
                if cls in (openvr.TrackedDeviceClass_GenericTracker,
                           openvr.TrackedDeviceClass_Controller):
                    if p.bPoseIsValid:
                        pos, q = mat34_to_pos_quat(p.mDeviceToAbsoluteTracking)
                        try:
                            sn = vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)
                        except Exception:
                            sn = "?"
                        valid_count += 1
                        lines.append(
                            f"[{i}] {name} {sn}  pos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})  "
                            f"quat=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f})")
                    else:
                        lines.append(f"[{i}] {name}  POSE INVALID (result={p.eTrackingResult})")
                elif cls == openvr.TrackedDeviceClass_TrackingReference:
                    lines.append(f"[{i}] BaseStation OK")
            print(" | ".join(lines) if lines else
                  "연결된 트래커/베이스스테이션 없음 — 트래커 전원/페어링/SteamVR 확인",
                  flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        openvr.shutdown()
        print(f"\nshutdown. valid pose samples = {valid_count}")


if __name__ == "__main__":
    main()
