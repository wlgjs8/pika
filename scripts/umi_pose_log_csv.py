#!/usr/bin/env python
"""UMI 트래커 raw 포즈 CSV 로거 — 발판을 밟는 순간부터 고유 Hz로 기록.

umi_teleop_publish.py 가 UDP 로 '송신'하는 것과 달리, 이 스크립트는 로봇/네트워크
없이 PoseSteamVR 폴러(기본 target 250Hz)의 스냅샷을 '새 샘플일 때만' CSV 한 줄로
남긴다. 발행 rate 로 리샘플하지 않으므로 기록 Hz = 폴러가 실제로 달성한 고유 Hz.
포즈는 raw(steamvr_world, 그리퍼 오프셋 미적용)가 기본 — --gripper-offset 시
PIKA 물리 오프셋([0.172,0,-0.076]m, 트래커 로컬)을 적용해 기록.

트리거(--trigger):
  start  = 발판 첫 밟음부터 종료(Ctrl-C)까지 계속 기록 (기본)
  held   = 밟고 있는 동안만 기록
  toggle = 밟을 때마다 기록 on/off

CSV: '#' 메타 주석 몇 줄 + 헤더 1줄 + 데이터.  (pandas: read_csv(..., comment="#"))
  ts_epoch   샘플 폴링 시각(time.time(), PoseSteamVR snapshot timestamp)
  ts_mono    기록 시점 time.monotonic() (수신부/다른 로그와 교차정렬용)
  pedal      그 순간 발판 눌림(0/1)
  <side>_valid, <side>_x, _y, _z, _qx, _qy, _qz, _qw   (side = left, right)
  트래커 미검출 side 는 valid=0 + 빈 필드.

실행(.40, SteamVR 실행 + 트래커 + 발판 연결):
  conda run -n pika python scripts/umi_pose_log_csv.py --pedal-device auto
키보드 폴백(발판 없이 테스트): --no-pedal 이면 시작 즉시 기록.
"""
import argparse
import csv
import json
import logging
import math
import os
import sys
import time
from datetime import datetime

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

log = logging.getLogger("pika.umi_pose_log")

SIDES = ("left", "right")
FIELDS = ["ts_epoch", "ts_mono", "pedal"] + [
    f"{side}_{k}" for side in SIDES for k in ("valid", "x", "y", "z", "qx", "qy", "qz", "qw")
]


# ----------------------------- 순수 로직 (하드웨어 불필요, selftest 가능) -----------------------------
class RecordGate:
    """발판 held 상태 → 기록 on/off 상태기계. mode: start | held | toggle."""

    def __init__(self, mode):
        if mode not in ("start", "held", "toggle"):
            raise ValueError(f"unknown trigger mode: {mode}")
        self.mode = mode
        self.recording = False
        self._prev_held = False

    def update(self, held):
        held = bool(held)
        if self.mode == "held":
            self.recording = held
        elif self.mode == "start":
            if held:
                self.recording = True
        else:  # toggle: 누름 edge 마다 뒤집기
            if held and not self._prev_held:
                self.recording = not self.recording
        self._prev_held = held
        return self.recording


def _finite7(values):
    return (
        isinstance(values, (list, tuple))
        and len(values) == 7
        and all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
    )


def build_row(ts_epoch, ts_mono, pedal, side_poses):
    """side_poses: {"left": [x..qw]|None, "right": ...} → CSV row (list, FIELDS 순서)."""
    row = [f"{ts_epoch:.6f}", f"{ts_mono:.6f}", int(bool(pedal))]
    for side in SIDES:
        p = side_poses.get(side)
        if _finite7(p):
            row.append(1)
            row.extend(f"{float(v):.7f}" for v in p)
        else:
            row.append(0)
            row.extend("" for _ in range(7))
    return row


def snapshot_to_side_poses(snap, sn_to_side):
    """PoseSteamVR 스냅샷(serial→pose dict) → {side: [x,y,z,qx,qy,qz,qw]}."""
    out = {}
    for sn, pd in snap.items():
        side = sn_to_side.get(sn)
        if side is None or not isinstance(pd, dict) or not pd.get("valid"):
            continue
        out[side] = list(pd.get("position", [])) + list(pd.get("rotation", []))
    return out


def load_tracker_map(config_path, swap_lr=False):
    """config/arms.json → {tracker_sn: side}. swap_lr 시 좌/우 맞바꿈."""
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    sn_to_side = {}
    for name in SIDES:
        d = cfg.get("arms", {}).get(name) or {}
        sn = d.get("tracker_sn")
        if sn:
            out = {"left": "right", "right": "left"}[name] if swap_lr else name
            sn_to_side[sn] = out
    if not sn_to_side:
        raise SystemExit(f"[arms] {config_path} 에 tracker_sn 이 없음 — --tracker-sns 로 지정하세요")
    return sn_to_side


def selftest():
    # RecordGate: start = 첫 밟음 이후 계속
    g = RecordGate("start")
    assert [g.update(h) for h in (0, 0, 1, 0, 0)] == [False, False, True, True, True]
    # held = 밟는 동안만
    g = RecordGate("held")
    assert [g.update(h) for h in (0, 1, 1, 0, 1)] == [False, True, True, False, True]
    # toggle = 누름 edge 마다
    g = RecordGate("toggle")
    assert [g.update(h) for h in (0, 1, 1, 0, 1, 0)] == [False, True, True, True, False, False]
    # row: 한쪽 미검출 → valid=0 + 빈 필드, 자릿수 고정
    row = build_row(1.5, 2.5, True, {"left": [0.1, 0.2, 0.3, 0, 0, 0, 1]})
    assert len(row) == len(FIELDS) and row[2] == 1 and row[3] == 1 and row[11] == 0
    assert row[4] == "0.1000000" and row[18] == ""
    # NaN pose 는 무효
    row = build_row(1.5, 2.5, 0, {"left": [float("nan")] * 7})
    assert row[3] == 0
    # 스냅샷 매핑 + swap
    snap = {
        "LHR-A": {"valid": True, "position": [1, 2, 3], "rotation": [0, 0, 0, 1]},
        "LHR-B": {"valid": False, "position": [9, 9, 9], "rotation": [0, 0, 0, 1]},
    }
    sp = snapshot_to_side_poses(snap, {"LHR-A": "left", "LHR-B": "right"})
    assert "left" in sp and "right" not in sp and sp["left"] == [1, 2, 3, 0, 0, 0, 1]
    print("selftest OK")


# ----------------------------- main -----------------------------
def get_arguments():
    ap = argparse.ArgumentParser(description="UMI 트래커 raw 포즈 CSV 로거 (발판 트리거)")
    ap.add_argument("--selftest", action="store_true", help="하드웨어 없이 순수 로직 검증 후 종료")
    ap.add_argument("--config", default=os.path.join(REPO_ROOT, "config", "arms.json"))
    ap.add_argument("--tracker-sns", nargs=2, metavar=("LEFT_SN", "RIGHT_SN"), default=None,
                    help="arms.json 대신 좌/우 트래커 SN 직접 지정")
    ap.add_argument("--swap-lr", action="store_true", help="좌/우 매핑 스왑(teleop --swap-lr 와 동일 의미)")
    ap.add_argument("--out", default=None,
                    help="CSV 경로(기본 data/umi_pose_logs/umi_pose_<시각>.csv)")
    ap.add_argument("--pose-hz", type=float, default=250.0, help="PoseSteamVR target 폴링 Hz")
    ap.add_argument("--trigger", choices=("start", "held", "toggle"), default="start",
                    help="start=첫 밟음부터 계속(기본) / held=밟는 동안 / toggle=밟을 때마다 on/off")
    ap.add_argument("--pedal-device", default="auto",
                    help="발판 evdev 경로(기본 auto=/dev/input/by-id/*FootSwitch*event-kbd)")
    ap.add_argument("--no-pedal", action="store_true", help="발판 없이 시작 즉시 기록(테스트용)")
    ap.add_argument("--gripper-offset", action="store_true",
                    help="PIKA 그리퍼 오프셋([0.172,0,-0.076]m, 트래커 로컬) 적용해 기록(기본 raw)")
    ap.add_argument("--flush-sec", type=float, default=0.5, help="CSV flush 주기")
    ap.add_argument("--verbose", action="store_true")
    args, _ = ap.parse_known_args()
    return args


def main():
    a = get_arguments()
    if a.selftest:
        selftest()
        return
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO, format="%(message)s")

    if a.tracker_sns:
        pair = (a.tracker_sns[1], a.tracker_sns[0]) if a.swap_lr else tuple(a.tracker_sns)
        sn_to_side = {pair[0]: "left", pair[1]: "right"}
    else:
        sn_to_side = load_tracker_map(a.config, swap_lr=a.swap_lr)
    log.info("[map] tracker→side: %s", sn_to_side)

    out_path = a.out or os.path.join(
        REPO_ROOT, "data", "umi_pose_logs",
        f"umi_pose_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)

    from pika_win.pose_steamvr import PoseSteamVR  # openvr 의존 — 지연 임포트
    pose = PoseSteamVR(target_hz=a.pose_hz, apply_gripper_offset=a.gripper_offset).connect()
    log.info("[pose] SteamVR 연결 (target %.0fHz, gripper_offset=%s)", a.pose_hz, a.gripper_offset)

    if a.no_pedal:
        clutch = None
        log.info("[pedal] 미사용 — 시작 즉시 기록")
    else:
        from umi_teleop_publish import PedalClutch  # 동일 scripts/ 의 발판 클러치 재사용
        clutch = PedalClutch(a.pedal_device, toggle=False).start()  # raw held 상태만 사용
        log.info("[pedal] device=%s trigger=%s", clutch.path, a.trigger)

    gate = RecordGate(a.trigger)
    poll_sleep = 1.0 / (4.0 * a.pose_hz) if a.pose_hz > 0 else 0.0005
    rows = 0
    last_ts = None
    last_flush = last_report = time.perf_counter()
    t_first = None

    with open(out_path, "w", newline="", encoding="utf-8") as f:
        f.write(f"# schema: pika.umi_pose_log.v1\n")
        f.write(f"# pose_frame: steamvr_world  pose_format: x,y,z,qx,qy,qz,qw\n")
        f.write(f"# gripper_offset_applied: {a.gripper_offset}  target_hz: {a.pose_hz:g}  trigger: {a.trigger}\n")
        f.write(f"# tracker_map: {json.dumps(sn_to_side)}\n")
        w = csv.writer(f)
        w.writerow(FIELDS)
        log.info("[log] → %s  (발판을 밟으면 기록 시작)", out_path)
        try:
            while True:
                held = bool(a.no_pedal) if clutch is None else clutch.update()["left"]
                recording = gate.update(held)
                snap = pose.get_pose()
                # 트래커 1개면 pose dict 자체가 옴 → serial 키 dict 로 정규화
                if isinstance(snap, dict) and snap and "position" in snap:
                    snap = {snap.get("device_name", "?"): snap}
                if recording and isinstance(snap, dict) and snap:
                    ts = max(pd.get("timestamp", 0.0) for pd in snap.values())
                    if ts != last_ts:  # 새 폴링 스냅샷일 때만 기록 → 고유 Hz
                        last_ts = ts
                        w.writerow(build_row(ts, time.monotonic(), held, snapshot_to_side_poses(snap, sn_to_side)))
                        rows += 1
                        if t_first is None:
                            t_first = time.perf_counter()
                            log.info("[log] 기록 시작 (pedal)")
                now = time.perf_counter()
                if now - last_flush >= a.flush_sec:
                    last_flush = now
                    f.flush()
                if now - last_report >= 2.0:
                    last_report = now
                    eff = getattr(pose, "effective_hz", 0.0)
                    if t_first is not None:
                        log.info("[log] rows=%d  write_hz=%.0f  poller_hz=%.0f  pedal=%d",
                                 rows, rows / max(now - t_first, 1e-6), eff, held)
                    else:
                        log.info("[log] 대기중 (pedal=%d, poller_hz=%.0f, trackers=%s)",
                                 held, eff, pose.get_devices())
                if poll_sleep:
                    time.sleep(poll_sleep)
        except KeyboardInterrupt:
            pass
        finally:
            f.flush()
            if clutch is not None:
                clutch.close()
            pose.disconnect()
            dur = (time.perf_counter() - t_first) if t_first else 0.0
            log.info("[log] 종료 — rows=%d  duration=%.1fs  avg_hz=%.1f  → %s",
                     rows, dur, rows / dur if dur > 0 else 0.0, out_path)


if __name__ == "__main__":
    main()
