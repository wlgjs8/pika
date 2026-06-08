#!/usr/bin/env python
"""현재 연결된 하드웨어 열거 — arms config 작성 보조.

열거만 수행(장치를 점유하지 않음)하므로 collect.py 실행 중에도 안전:
  - Vive 트래커 시리얼 (SteamVR/openvr, Background)
  - RealSense 장치 시리얼 (pyrealsense2 context)
  - Windows COM 또는 Linux /dev/serial/by-id, /dev/ttyUSB 포트 목록

실행: conda run -n pika python scripts\\detect_hardware.py
"""
import glob
import os
import sys
import time


def trackers():
    try:
        import openvr
    except Exception as e:
        return None, f"openvr import 실패: {e}"
    try:
        vr = openvr.init(openvr.VRApplication_Background)
    except Exception as e:
        return None, f"SteamVR init 실패(실행 중인지 확인): {e}"
    n = openvr.k_unMaxTrackedDeviceCount
    seen = {}
    t0 = time.perf_counter()
    try:
        while time.perf_counter() - t0 < 1.0:
            poses = vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, n)
            for i in range(n):
                if vr.getTrackedDeviceClass(i) != openvr.TrackedDeviceClass_GenericTracker:
                    continue
                p = poses[i]
                try:
                    sn = vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)
                except Exception:
                    sn = f"dev{i}"
                seen[sn] = bool(p.bDeviceIsConnected and p.bPoseIsValid)
            time.sleep(0.05)
    finally:
        openvr.shutdown()
    return seen, None


def realsenses():
    try:
        import pyrealsense2 as rs
    except Exception as e:
        return None, f"pyrealsense2 import 실패: {e}"
    try:
        ctx = rs.context()
        out = []
        for d in ctx.query_devices():
            out.append({
                "serial": d.get_info(rs.camera_info.serial_number),
                "name": d.get_info(rs.camera_info.name),
            })
        return out, None
    except Exception as e:
        return None, f"RealSense 열거 실패: {e}"


def com_ports():
    try:
        from serial.tools import list_ports
    except Exception as e:
        return None, f"pyserial import 실패: {e}"
    out = []
    for p in list_ports.comports():
        desc = (p.description or "") + " " + (p.manufacturer or "")
        out.append({
            "device": p.device,
            "description": p.description,
            "is_sense_candidate": ("CH340" in desc) or ("USB-SERIAL" in desc.upper()),
        })
    return out, None


def linux_serial_by_id():
    if os.name == "nt":
        return []
    return [
        {
            "path": path,
            "target": os.path.realpath(path),
            "is_sense_candidate": "ch340" in path.lower() or "usb-serial" in path.lower(),
        }
        for path in sorted(glob.glob("/dev/serial/by-id/*"))
    ]


def linux_tty_usb():
    if os.name == "nt":
        return []
    return sorted(glob.glob("/dev/ttyUSB*"))


def main():
    print("=" * 60)
    tk, err = trackers()
    print("[Vive 트래커]")
    if err:
        print("  ", err)
    else:
        for sn in sorted(tk):
            print(f"   {sn}   pose_valid={tk[sn]}")

    print("\n[RealSense 장치]")
    rsl, err = realsenses()
    if err:
        print("  ", err)
    else:
        for d in rsl:
            print(f"   SN={d['serial']}   {d['name']}")

    print("\n[COM 포트]")
    cps, err = com_ports()
    if err:
        print("  ", err)
    else:
        for p in cps:
            mark = " <- Sense 후보" if p["is_sense_candidate"] else ""
            print(f"   {p['device']:6s}  {p['description']}{mark}")

    if os.name != "nt":
        print("\n[Linux stable serial paths]")
        by_id = linux_serial_by_id()
        if by_id:
            for item in by_id:
                mark = " <- Sense 후보" if item["is_sense_candidate"] else ""
                print(f"   {item['path']} -> {item['target']}{mark}")
        else:
            print("   /dev/serial/by-id/* 없음")
        tty_usb = linux_tty_usb()
        print("   ttyUSB:", ", ".join(tty_usb) if tty_usb else "(없음)")
    print("=" * 60)
    print("이 값들로 config/arms.json 의 left/right 번들을 채우세요.")
    if os.name != "nt":
        print("Linux에서는 COM3 대신 /dev/serial/by-id/... 경로를 com_port에 쓰는 것을 권장합니다.")


if __name__ == "__main__":
    main()
    sys.exit(0)
