#!/usr/bin/env python
"""발판(FootSwitch) 진단 — 연결된 발판을 전부 나열하고, 밟으면 어느 것인지 표시.

발판이 여러 개일 때 `/dev/input/by-id` 는 **하나에만** 심링크를 만든다(같은 VID:PID +
시리얼 없음 → udev 이름 충돌 회피). 그래서 by-id 로 자동 탐지하면 나머지 발판은
영원히 인식되지 않는다. 이 스크립트는 sysfs 이름으로 전부 찾아 확인시켜 준다.

실행: .venv/bin/python scripts/pedal_test.py [--seconds 30]
"""
import argparse
import os
import struct
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

from umi_teleop_publish import PedalClutch  # noqa: E402

EVENT = struct.Struct("llHHi")
EV_KEY = 0x01


def _sysfs(path, field):
    # find_devices() 는 안정적인 by-path 심링크를 돌려주므로 sysfs 조회 전에 eventN 으로 푼다.
    node = os.path.basename(os.path.realpath(path))
    try:
        with open(f"/sys/class/input/{node}/device/{field}", encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return "?"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=30.0)
    a = ap.parse_args()

    paths = PedalClutch.find_devices()
    if not paths:
        print("발판을 찾지 못했습니다. lsusb 로 3553:b001 이 보이는지, "
              "/dev/input/event* 권한(plugdev)이 있는지 확인하세요.")
        return 1

    print(f"발판 키보드 노드 {len(paths)}개:")
    fds = {}
    for p in paths:
        phys = _sysfs(p, "phys")
        name = _sysfs(p, "name")
        try:
            fds[os.open(p, os.O_RDONLY | os.O_NONBLOCK)] = (p, phys)
            state = "열림"
        except OSError as e:
            state = f"열기 실패({e})"
        print(f"  {p:20s} phys={phys:28s} name='{name}'  {state}")
    print(f"\n{a.seconds:.0f}초 동안 발판을 하나씩 밟아보세요 (어느 것이 반응하는지 표시됩니다).")
    print("robotics_lab rb_gui 가 켜져 있으면 그쪽이 발판 하나를 배타적으로 잡고 있어\n"
          "여기서 이벤트가 안 보일 수 있습니다.\n")

    seen = set()
    t0 = time.time()
    try:
        while time.time() - t0 < a.seconds:
            for fd, (p, phys) in fds.items():
                try:
                    data = os.read(fd, EVENT.size * 64)
                except (BlockingIOError, OSError):
                    continue
                for off in range(0, len(data) - (len(data) % EVENT.size), EVENT.size):
                    _, _, etype, code, value = EVENT.unpack_from(data, off)
                    if etype != EV_KEY or value == 2:
                        continue
                    seen.add(p)
                    print(f"  [{time.time() - t0:5.1f}s] {p}  phys={phys}  "
                          f"key_code={code} {'눌림' if value else '뗌'}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        for fd in fds:
            os.close(fd)

    print(f"\n반응한 발판: {sorted(seen) or '없음'}")
    quiet = [p for p in paths if p not in seen]
    if quiet:
        print(f"반응 없던 발판: {quiet}")
    print("\n특정 발판만 쓰려면:  scripts/run_umi_teleop_publish.sh --pedal-device <경로>")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
