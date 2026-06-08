#!/usr/bin/env python
"""PIKA Sense 시리얼(COM 포트) Windows 동작 검증 — 공식 pika.Sense 사용.

읽기 전용: 버전/엔코더(그리퍼 각도)/command + 원시 JSON(IMU 포함 스키마) 덤프.
실행: conda run -n pika python scripts\\test_sense_serial.py [COM3]
"""
import os
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from pika_win.sdk_logging import quiet_pika_sdk_info  # noqa: E402

quiet_pika_sdk_info()

from pika.sense import Sense  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"


def find_raw_accessor(s):
    for attr in ("serial_comm", "_serial_comm", "serial", "comm", "_comm", "_serial"):
        obj = getattr(s, attr, None)
        if obj is not None and hasattr(obj, "get_latest_data"):
            return attr, obj
    return None, None


def main():
    print(f"Sense(port={PORT}) connect...")
    s = Sense(port=PORT)
    ok = s.connect()
    print("connect ->", ok)
    print("public methods:", [m for m in dir(s) if not m.startswith("_")])

    for fn in ("get_version",):
        if hasattr(s, fn):
            try:
                print(f"{fn}():", getattr(s, fn)())
            except Exception as e:
                print(f"{fn}() err: {e}")

    attr, raw = find_raw_accessor(s)
    print("raw-data accessor:", attr)

    print("\n--- 15 samples @5Hz (그리퍼 정지 상태여도 값이 유효한지/IMU 스키마 확인) ---")
    for i in range(15):
        line = {}
        for fn in ("get_encoder_data", "get_command_state"):
            if hasattr(s, fn):
                try:
                    line[fn] = getattr(s, fn)()
                except Exception as e:
                    line[fn] = f"err:{e}"
        if raw is not None:
            try:
                line["raw"] = raw.get_latest_data()
            except Exception as e:
                line["raw"] = f"err:{e}"
        print(f"[{i:02d}]", line)
        time.sleep(0.2)

    s.disconnect()
    print("disconnected.")


if __name__ == "__main__":
    main()
