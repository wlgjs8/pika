#!/usr/bin/env python
"""대화형 좌/우 팔 식별 마법사 → config/arms.json 저장.

각 손을 물리적으로 조작시켜 (트래커=움직임, 그리퍼=쥐기, RealSense=가리기)
어느 하드웨어가 어느 손에 묶이는지 자동 식별하고, left/right 번들을 config에 고정한다.
한 번 저장하면 collect.py 가 실행 시 이 config 를 읽어 항상 같은 매핑을 쓴다.

⚠️ 실행 전 collect.py(make run/make view)를 반드시 종료하세요(장치 점유 충돌).
Linux에서는 가능한 경우 /dev/serial/by-id/... 경로를 com_port에 저장한다.
실행: conda run -n pika python scripts\\identify_arms.py
"""
import glob
import json
import os
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
CONFIG_PATH = os.path.join(REPO_ROOT, "config", "arms.json")

import numpy as np  # noqa: E402
from pika_win.sdk_logging import quiet_pika_sdk_info  # noqa: E402


def _ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return ""


def _confirm_or_manual(auto, options, label):
    opts = "/".join(str(o) for o in options)
    print(f"  → 자동 식별: {label} = {auto}")
    ans = _ask(f"    맞으면 Enter, 아니면 직접 입력 [{opts}]: ")
    return ans if ans else auto


def _countdown(msg, seconds):
    print(f"\n{msg}")
    for s in range(3, 0, -1):
        print(f"  {s}...", end="", flush=True)
        time.sleep(1)
    print(f"  측정 시작! ({seconds:.0f}초)")


# ----------------------------------------------------------------- trackers
def detect_trackers():
    import openvr
    vr = openvr.init(openvr.VRApplication_Background)
    n = openvr.k_unMaxTrackedDeviceCount
    seen = set()
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < 1.0:
        poses = vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, n)
        for i in range(n):
            if vr.getTrackedDeviceClass(i) != openvr.TrackedDeviceClass_GenericTracker:
                continue
            if poses[i].bDeviceIsConnected:
                try:
                    seen.add(vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String))
                except Exception:
                    pass
        time.sleep(0.05)
    return vr, openvr, sorted(seen)


def _tracker_positions(vr, openvr):
    n = openvr.k_unMaxTrackedDeviceCount
    poses = vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, n)
    out = {}
    for i in range(n):
        if vr.getTrackedDeviceClass(i) != openvr.TrackedDeviceClass_GenericTracker:
            continue
        p = poses[i]
        if not (p.bDeviceIsConnected and p.bPoseIsValid):
            continue
        try:
            sn = vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)
        except Exception:
            continue
        m = p.mDeviceToAbsoluteTracking
        out[sn] = np.array([m[0][3], m[1][3], m[2][3]])
    return out


def identify_tracker(vr, openvr, serials, hand, duration=4.0):
    _countdown(f"[{hand}] 손의 트래커만 크게 움직이세요 (다른 손은 가만히).", duration)
    motion = {sn: 0.0 for sn in serials}
    last = {}
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < duration:
        cur = _tracker_positions(vr, openvr)
        for sn, pos in cur.items():
            if sn in last:
                motion[sn] += float(np.linalg.norm(pos - last[sn]))
            last[sn] = pos
        time.sleep(0.02)
    ranked = sorted(motion.items(), key=lambda kv: kv[1], reverse=True)
    print("  움직임량:", {k: round(v, 3) for k, v in ranked})
    auto = ranked[0][0]
    return _confirm_or_manual(auto, serials, f"{hand} tracker")


# ----------------------------------------------------------------- grippers (COM)
def detect_com_candidates():
    from serial.tools import list_ports
    stable = _linux_stable_serial_paths()
    out = []
    for p in list_ports.comports():
        d = (p.description or "")
        if "CH340" in d or "USB-SERIAL" in d.upper():
            out.append(stable.get(os.path.realpath(p.device), p.device))
    return sorted(out)


def _linux_stable_serial_paths():
    if os.name == "nt":
        return {}
    out = {}
    for path in sorted(glob.glob("/dev/serial/by-id/*")):
        out.setdefault(os.path.realpath(path), path)
    return out


def identify_com(senses, coms, hand, duration=4.0):
    _countdown(f"[{hand}] 손의 그리퍼를 '꽉 쥐었다 펴기' 반복하세요.", duration)
    series = {c: [] for c in coms}
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < duration:
        for c in coms:
            try:
                v = senses[c].get_encoder_data().get("angle")
            except Exception:
                v = None
            if v is not None and v == v:
                series[c].append(v)
        time.sleep(0.02)
    spread = {c: (max(v) - min(v)) if len(v) >= 2 else 0.0 for c, v in series.items()}
    ranked = sorted(spread.items(), key=lambda kv: kv[1], reverse=True)
    print("  각도 변동폭:", {k: round(v, 1) for k, v in ranked})
    return _confirm_or_manual(ranked[0][0], coms, f"{hand} COM")


# ----------------------------------------------------------------- realsense
def identify_rs(rs_objs, sns, hand, duration=4.0):
    _countdown(f"[{hand}] 손의 RealSense(D405) 앞에 손바닥을 가까이 댔다 떼기 반복.", duration)
    series = {sn: [] for sn in sns}
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < duration:
        for sn in sns:
            try:
                _, depth, _ = rs_objs[sn].get_frames()
                if depth is not None:
                    valid = depth[depth > 0]
                    if valid.size:
                        series[sn].append(float(valid.mean()))
            except Exception:
                pass
        time.sleep(0.03)
    spread = {sn: (max(v) - min(v)) if len(v) >= 2 else 0.0 for sn, v in series.items()}
    ranked = sorted(spread.items(), key=lambda kv: kv[1], reverse=True)
    print("  depth 평균 변동폭:", {k: round(v, 1) for k, v in ranked})
    return _confirm_or_manual(ranked[0][0], sns, f"{hand} RealSense SN")


# ----------------------------------------------------------------- main
def main():
    quiet_pika_sdk_info()
    print("=" * 64)
    print("좌/우 팔 식별 마법사 — 각 손을 물리적으로 조작해 번들을 고정합니다.")
    print("⚠️ collect.py(make run/make view)가 실행 중이면 먼저 종료하세요.")
    if _ask("준비됐으면 Enter (취소: q): ").lower() == "q":
        return

    vr = openvr = None
    senses, rs_objs = {}, {}
    try:
        # ---- 연결 ----
        vr, openvr, trackers = detect_trackers()
        print(f"\n트래커 {len(trackers)}개: {trackers}")
        if len(trackers) < 2:
            print("트래커가 2개 미만 — 양팔 식별 불가. 종료.")
            return

        coms = detect_com_candidates()
        print(f"Sense COM 후보: {coms}")
        from pika.sense import Sense
        for c in coms:
            s = Sense(port=c)
            s.connect()
            senses[c] = s

        import pyrealsense2 as rs
        rs_sns = [d.get_info(rs.camera_info.serial_number) for d in rs.context().query_devices()]
        print(f"RealSense SN: {rs_sns}")
        from pika_win.realsense_win import RealSenseD4xx
        for sn in rs_sns:
            rs_objs[sn] = RealSenseD4xx(serial=sn).connect()
        time.sleep(1.0)

        # ---- 식별: 트래커 / 그리퍼 / RealSense (각 2개 → 오른손 식별, 왼손=나머지) ----
        print("\n" + "-" * 64 + "\n[1/3] 트래커")
        r_tracker = identify_tracker(vr, openvr, trackers, "오른")
        l_tracker = next(s for s in trackers if s != r_tracker)

        print("\n" + "-" * 64 + "\n[2/3] 그리퍼(COM)")
        r_com = identify_com(senses, coms, "오른")
        l_com = next(c for c in coms if c != r_com)

        print("\n" + "-" * 64 + "\n[3/3] RealSense")
        r_rs = identify_rs(rs_objs, rs_sns, "오른")
        l_rs = next(s for s in rs_sns if s != r_rs)

        # ---- 저장 ----
        config = {"arms": {
            "right": {"tracker_sn": r_tracker, "com_port": r_com, "realsense_sn": r_rs},
            "left":  {"tracker_sn": l_tracker, "com_port": l_com, "realsense_sn": l_rs},
        }}
        print("\n" + "=" * 64)
        print(json.dumps(config, indent=2, ensure_ascii=False))
        if _ask("\n이 매핑으로 저장할까요? (Enter=저장, q=취소): ").lower() == "q":
            print("취소됨 — 저장 안 함.")
            return
        os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(config, f, indent=2, ensure_ascii=False)
        print(f"저장 완료: {CONFIG_PATH}")
        print("이제 make run / make view 가 이 config 를 읽어 좌/우를 고정합니다.")
    finally:
        for o in rs_objs.values():
            try:
                o.disconnect()
            except Exception:
                pass
        for s in senses.values():
            try:
                s.disconnect()
            except Exception:
                pass
        if openvr is not None:
            try:
                openvr.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
    os._exit(0)
