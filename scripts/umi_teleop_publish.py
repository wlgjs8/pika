#!/usr/bin/env python
"""UMI 라이브 텔레오퍼레이션 포즈 퍼블리셔 (Windows/SteamVR → Linux robotics_lab).

collect.py가 HDF5로 '기록'만 하는 것과 달리, 이 스크립트는 양팔 트래커의
6DoF 포즈 + 그리퍼 + 클러치(deadman) 상태를 매 틱 UDP JSON으로 스트리밍한다.
robotics_lab policy_runner 의 `umi_dual_cartesian` 액션소스(UdpUmiPoseReader)가
이를 받아 relative-init 방식으로 RB 양팔을 pgmode 시뮬레이션에서 구동한다.

와이어 스키마 (robotics_lab UdpUmiPoseReader._sample_from_udp_packet 와 1:1):
  {"t": <monotonic>,
   "left":  {"pose": [x,y,z,qx,qy,qz,qw], "gripper": <0..1>, "deadman": <bool>},
   "right": {"pose": [...],               "gripper": <0..1>, "deadman": <bool>}}
- pose 프레임 = steamvr_world (TrackingUniverseStanding). 상대 모션만 쓰이므로
  월드 정렬/측정 캘리브레이션 불필요 (teleop은 무캘리브레이션).
- 수신부는 side(left/right) 마다 별도 포트에 bind 하므로, 같은 결합 패킷을
  좌/우 두 목적지 포트로 각각 보낸다 (각 리더가 자기 side만 추출).
- pose 가 유효하지 않은(미검출) side 는 패킷에서 생략 → 해당 팔은 Hold.

deadman(클러치)은 PikaAnyArm 트리거와 동일한 토글 의미:
  켜짐 = 클러치 engage(켜는 순간 robotics_lab이 init 스냅샷), 꺼짐 = 해제/Hold.
  키보드 키로 토글 (Windows msvcrt / Linux cbreak). 좌/우 개별 또는 공유.

실행(Windows, SteamVR 실행 + 트래커 + Sense 연결):
  conda run -n pika python scripts\\umi_teleop_publish.py \
    --target-host 192.168.8.x --left-port 50380 --right-port 50381

키: [space]=양팔 클러치 토글, [a]=좌팔, [l]=우팔, [q]=종료.
"""
import argparse
import json
import logging
import math
import os
import socket
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

log = logging.getLogger("pika.umi_teleop")

SIDES = ("left", "right")


# ----------------------------- 순수 패킷 빌더 (하드웨어/openvr 불필요, 테스트 가능) -----------------------------
def _pose_valid(pose):
    return (
        isinstance(pose, (list, tuple))
        and len(pose) == 7
        and all(isinstance(v, (int, float)) and not math.isnan(v) for v in pose)
    )


def normalize_gripper(value, open_val, closed_val):
    """그리퍼 스칼라(angle 등) → 0..1 (1=closed). 범위 미확정 시 None 반환."""
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return None
    if open_val is None or closed_val is None or open_val == closed_val:
        return None
    frac = (float(value) - float(open_val)) / (float(closed_val) - float(open_val))
    return max(0.0, min(1.0, frac))


def build_packet(t, sides):
    """sides: {name: {"pose":[7]|None, "gripper":float|None, "deadman":bool}} → wire dict.

    pose 가 유효하지 않은 side 는 생략한다(수신부가 None→Hold 처리).
    """
    packet = {"t": float(t)}
    for name in SIDES:
        s = sides.get(name)
        if not s or not _pose_valid(s.get("pose")):
            continue
        packet[name] = {
            "pose": [float(v) for v in s["pose"]],
            "gripper": float(s.get("gripper") or 0.0),
            "deadman": bool(s.get("deadman")),
        }
    return packet


# ----------------------------- 키보드 클러치 토글 (Win/Linux) -----------------------------
class KeyboardClutch:
    """단일 키 non-blocking 폴링으로 좌/우/양팔 클러치 토글 + 종료 키."""

    def __init__(self, left_key="a", right_key="l", both_key=" ", quit_key="q"):
        self.left_key = left_key.lower()
        self.right_key = right_key.lower()
        self.both_key = both_key.lower()
        self.quit_key = quit_key.lower()
        self.engaged = {"left": False, "right": False}
        self.quit = False
        self._fd = None
        self._old = None
        self._termios = None
        self._msvcrt = None

    def start(self):
        if os.name == "nt":
            import msvcrt
            self._msvcrt = msvcrt
            return self
        if not sys.stdin.isatty():
            log.warning("[clutch] stdin이 tty가 아님 — 키 입력 비활성(양팔 클러치를 강제 ON)")
            self.engaged = {"left": True, "right": True}
            return self
        import termios
        import tty
        self._termios = termios
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def _poll_keys(self):
        if os.name == "nt":
            keys = []
            while self._msvcrt.kbhit():
                keys.append(self._msvcrt.getwch().lower())
            return keys
        if self._old is None:
            return []
        import select
        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch:
                keys.append(ch.lower())
        return keys

    def update(self):
        for ch in self._poll_keys():
            if ch == self.quit_key:
                self.quit = True
            elif ch == self.both_key:
                new = not (self.engaged["left"] and self.engaged["right"])
                self.engaged["left"] = self.engaged["right"] = new
            elif ch == self.left_key:
                self.engaged["left"] = not self.engaged["left"]
            elif ch == self.right_key:
                self.engaged["right"] = not self.engaged["right"]
        return self.engaged

    def close(self):
        if self._old is not None:
            self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old)
            self._old = None


# ----------------------------- arms.json 로더 (collect.build_arms 의 최소 버전) -----------------------------
def load_arms(config_path):
    """config/arms.json → ArmSpec 리스트(좌/우). pika_win.recorder.ArmSpec 사용."""
    from pika_win.recorder import ArmSpec
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    arms = []
    for name in SIDES:  # left, right 순서 고정
        d = cfg.get("arms", {}).get(name)
        if not d:
            continue
        arms.append(ArmSpec(
            name,
            com_port=d.get("com_port") or None,
            realsense_sn=d.get("realsense_sn") or None,
            tracker_sn=d.get("tracker_sn") or None,
        ))
    if not arms:
        raise SystemExit(f"[arms] {config_path} 에서 left/right arm을 찾지 못함")
    return arms


def selftest():
    """하드웨어/openvr 없이 순수 패킷 빌더·그리퍼 정규화 검증."""
    # 유효 양팔 → 두 side 모두 포함, float 캐스팅
    pkt = build_packet(1.0, {
        "left": {"pose": [0.1, 0.2, 0.3, 0, 0, 0, 1], "gripper": 0.4, "deadman": True},
        "right": {"pose": [0.5, 0.6, 0.7, 0, 0, 0, 1], "gripper": None, "deadman": False},
    })
    assert set(pkt) == {"t", "left", "right"}, pkt
    assert pkt["left"]["deadman"] is True and pkt["right"]["deadman"] is False
    assert pkt["right"]["gripper"] == 0.0  # None → 0.0
    assert isinstance(pkt["left"]["pose"][0], float)
    # 무효 pose side(미검출/NaN) 는 생략
    pkt2 = build_packet(2.0, {
        "left": {"pose": [float("nan")] * 7, "gripper": 0.0, "deadman": True},
        "right": {"pose": [1, 1, 1, 0, 0, 0, 1], "gripper": 1.0, "deadman": True},
    })
    assert "left" not in pkt2 and "right" in pkt2, pkt2
    # 길이 오류 pose 도 생략
    assert "left" not in build_packet(3.0, {"left": {"pose": [0, 0, 0], "deadman": True}})
    # 그리퍼 정규화
    assert normalize_gripper(5, 0, 10) == 0.5
    assert normalize_gripper(99, 0, 10) == 1.0
    assert normalize_gripper(-5, 0, 10) == 0.0
    assert normalize_gripper(None, 0, 10) is None
    assert normalize_gripper(5, 0, 0) is None  # 범위 미확정
    print("selftest OK")


def get_arguments():
    ap = argparse.ArgumentParser(description="UMI 라이브 텔레오퍼레이션 포즈 퍼블리셔")
    ap.add_argument("--selftest", action="store_true",
                    help="하드웨어 없이 패킷 빌더 검증 후 종료")
    ap.add_argument("--config", default=os.path.join(REPO_ROOT, "config", "arms.json"))
    ap.add_argument("--target-host", default="127.0.0.1",
                    help="robotics_lab(수신) 호스트 IP")
    ap.add_argument("--left-port", type=int, default=50380)
    ap.add_argument("--right-port", type=int, default=50381)
    ap.add_argument("--rate", type=float, default=120.0, help="발행 Hz")
    ap.add_argument("--no-sense", action="store_true", help="Sense(그리퍼) 미연결")
    ap.add_argument("--grip-open", type=float, default=None, help="그리퍼 open 각도(미지정=자동 범위)")
    ap.add_argument("--grip-closed", type=float, default=None, help="그리퍼 closed 각도")
    ap.add_argument("--left-key", default="a")
    ap.add_argument("--right-key", default="l")
    ap.add_argument("--both-key", default=" ")
    ap.add_argument("--start-engaged", action="store_true",
                    help="시작 시 양팔 클러치 ON (키 입력 없이 즉시 추종)")
    ap.add_argument("--verbose", action="store_true")
    args, _ = ap.parse_known_args()
    return args


def main():
    a = get_arguments()
    if a.selftest:
        selftest()
        return
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(message)s")
    from pika_win.recorder import EpisodeRecorder  # openvr 의존 — main 안에서 지연 임포트

    arms = load_arms(a.config)
    rec = EpisodeRecorder(out_dir=os.path.join(REPO_ROOT, "data", "_umi_teleop_tmp"),
                          arms=arms, use_realsense=False, use_sense=not a.no_sense,
                          use_pose=True, require_pose=True)
    rec.start()
    names = rec.arm_names()
    log.info("[umi] 활성 팔: %s", names)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    targets = [(a.target_host, a.left_port), (a.target_host, a.right_port)]
    log.info("[umi] 송신 대상: %s (좌/우 동일 패킷)", targets)

    clutch = KeyboardClutch(a.left_key, a.right_key, a.both_key).start()
    if a.start_engaged:
        clutch.engaged = {"left": True, "right": True}
    log.info("[umi] 키: [%s]=양팔 [%s]=좌 [%s]=우 [q]=종료",
             a.both_key if a.both_key.strip() else "space", a.left_key, a.right_key)

    # 그리퍼 자동 범위(open/closed 미지정 시 관측 min/max 누적)
    grange = {n: [a.grip_open, a.grip_closed] for n in SIDES}
    period = 1.0 / a.rate if a.rate > 0 else 0.0
    last_log = 0.0
    try:
        while not clutch.quit:
            tick = time.perf_counter()
            engaged = clutch.update()
            frame = rec.read_frame()
            sides = {}
            for ai, name in enumerate(names):
                if name not in SIDES:
                    continue
                arm = frame["arms"][ai]
                pose = arm.get("pose")
                grip_angle = arm.get("gripper", [None])[0]
                # 자동 범위 갱신
                if a.grip_open is None and grip_angle is not None and not (
                        isinstance(grip_angle, float) and math.isnan(grip_angle)):
                    lo, hi = grange[name]
                    grange[name] = [
                        grip_angle if lo is None else min(lo, grip_angle),
                        grip_angle if hi is None else max(hi, grip_angle),
                    ]
                gn = normalize_gripper(grip_angle, grange[name][0], grange[name][1])
                sides[name] = {"pose": pose, "gripper": gn, "deadman": engaged.get(name, False)}

            packet = build_packet(time.monotonic(), sides)
            data = json.dumps(packet).encode("utf-8")
            for tgt in targets:
                sock.sendto(data, tgt)

            now = time.time()
            if a.verbose and now - last_log > 0.5:
                last_log = now
                active = [n for n in SIDES if n in packet]
                log.debug("[umi] eff_pose_hz=%.0f engaged=%s sides=%s",
                          getattr(rec.pose, "effective_hz", 0.0), engaged, active)

            rem = period - (time.perf_counter() - tick)
            if rem > 0:
                time.sleep(rem)
    except KeyboardInterrupt:
        pass
    finally:
        clutch.close()
        sock.close()
        rec.stop()
        log.info("[umi] 종료")


if __name__ == "__main__":
    main()
