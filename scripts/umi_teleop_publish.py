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
import glob
import json
import logging
import math
import os
import socket
import struct
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


# ----------------------------- USB 발판(FootSwitch) 클러치 (Linux evdev, stdin 무의존) -----------------------------
class PedalClutch:
    """USB FootSwitch(evdev)를 클러치로. 기본 momentary(밟는 동안 양팔 engage),
    --pedal-toggle 시 밟을 때마다 토글. stdin을 안 읽으므로 원격/비-tty 실행에 안전.

    PCsensor FootSwitch는 키보드형 키 이벤트(KEY_PRESS value=1 / KEY_RELEASE value=0)를
    보낸다. momentary는 마지막 키 상태(눌림=1)를 engage로, toggle은 누름 edge마다 뒤집는다.
    `key/code` 필터 없이 EV_KEY 이벤트를 그대로 사용(발판이 어떤 키를 보내든 동작).
    """
    EV_KEY = 0x01
    EVENT = struct.Struct("llHHi")  # input_event: timeval(2 long) + type + code + value

    def __init__(self, device="auto", toggle=False):
        self.device_arg = device
        self.toggle = bool(toggle)
        self.fd = None
        self.path = None
        self.held = False        # momentary: 발판 눌림 상태
        self.engaged_both = False  # toggle: 누적 상태
        self.quit = False        # 인터페이스 호환(발판엔 종료 키 없음)

    def _resolve_device(self):
        if self.device_arg not in ("auto", ""):
            return self.device_arg
        cands = sorted(glob.glob("/dev/input/by-id/*FootSwitch*event-kbd")) or \
            sorted(glob.glob("/dev/input/by-id/*[Ff]oot[Ss]witch*event-kbd"))
        return cands[0] if cands else None

    def start(self):
        path = self._resolve_device()
        if not path:
            raise RuntimeError("FootSwitch evdev 장치를 찾지 못함 (/dev/input/by-id/*FootSwitch*event-kbd). "
                               "--pedal-device 로 직접 지정하거나 권한(input 그룹) 확인.")
        try:
            self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except PermissionError as e:
            raise RuntimeError(f"{path} 읽기 권한 없음 ({e}). 'sudo usermod -aG input $USER' 후 재로그인 또는 udev 규칙 필요.")
        self.path = path
        return self

    def update(self):
        # 쌓인 이벤트를 모두 소비해 상태 갱신
        while True:
            try:
                data = os.read(self.fd, self.EVENT.size * 64)
            except BlockingIOError:
                break
            except OSError:
                break
            if not data:
                break
            usable = len(data) - (len(data) % self.EVENT.size)
            for off in range(0, usable, self.EVENT.size):
                _, _, etype, _code, value = self.EVENT.unpack_from(data, off)
                if etype != self.EV_KEY:
                    continue
                if value == 1:      # 누름
                    self.held = True
                    if self.toggle:
                        self.engaged_both = not self.engaged_both
                elif value == 0:    # 뗌
                    self.held = False
        on = self.engaged_both if self.toggle else self.held
        return {"left": on, "right": on}

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None


# ----------------------------- 로봇측 Pika Gripper 추종 (POSITION_CTRL) -----------------------------
def gripper_send_decision(rad, last, now, min_rad, max_rad, deadband_rad, min_period):
    """보낼 모터각(rad) 결정 — 클램프 + 데드밴드 + 레이트리밋. (순수함수, selftest 가능)

    rad: Sense 인코더 각도(rad, None/NaN=미정), last: (last_monotonic, last_rad)|None.
    반환: 송신할 rad 또는 None(스킵).
    """
    if rad is None or (isinstance(rad, float) and math.isnan(rad)):
        return None
    clamped = max(float(min_rad), min(float(max_rad), float(rad)))
    if last is not None:
        last_t, last_rad = last
        if now - last_t < min_period:
            return None
        if abs(clamped - last_rad) < deadband_rad:
            return None
    return clamped


class _LogThrottle(logging.Filter):
    """같은 로거의 메시지를 period당 1건으로 제한 (pika SDK telemetry 파싱 에러 스팸 방지)."""

    def __init__(self, period_sec=2.0):
        super().__init__()
        self.period_sec = period_sec
        self._last = 0.0

    def filter(self, record):
        now = time.monotonic()
        if now - self._last >= self.period_sec:
            self._last = now
            return True
        return False


class GripperFollower:
    """Sense 인코더 각도(rad)를 로봇에 장착된 Pika Gripper 모터각으로 추종.

    매뉴얼상 Sense/Gripper 파라미터 동일 → 기본 1:1 rad 패스스루(범위 클램프만).
    ports 의 side 키는 '로봇팔 기준'(= 패킷 out_name, --swap-lr 적용 후)이다.
    시리얼 오류는 teleop 본체를 죽이지 않고 스로틀 WARN으로만 보고한다.
    """

    def __init__(self, ports, min_rad=0.0, max_rad=1.75, deadband_rad=0.005, max_hz=60.0):
        self.ports = dict(ports)            # {"left": "/dev/...", "right": "/dev/..."}
        self.min_rad = float(min_rad)
        self.max_rad = float(max_rad)
        self.deadband_rad = float(deadband_rad)
        self.min_period = 1.0 / max_hz if max_hz > 0 else 0.0
        self.grippers = {}
        self._last_sent = {}                # side -> (monotonic, rad)
        self._warned = {}                   # side -> last warn monotonic

    def start(self):
        from pika.gripper import Gripper    # pika SDK — 지연 임포트
        # 24V 미인가 등으로 telemetry 가 깨질 때 SDK 가 에러를 틱마다 찍음 → 스로틀
        logging.getLogger("pika.serial_comm").addFilter(_LogThrottle(2.0))
        for side, port in self.ports.items():
            g = Gripper(port=port)
            if not g.connect():
                raise RuntimeError(f"[gripper:{side}] {port} 연결 실패")
            if not g.enable():
                raise RuntimeError(f"[gripper:{side}] {port} enable 실패")
            self.grippers[side] = g
            log.info("[gripper] %s ← %s 연결+enable", side, port)
        return self

    def update(self, side, rad):
        g = self.grippers.get(side)
        if g is None:
            return
        now = time.monotonic()
        decided = gripper_send_decision(
            rad, self._last_sent.get(side), now,
            self.min_rad, self.max_rad, self.deadband_rad, self.min_period)
        if decided is None:
            return
        try:
            g.set_motor_angle(decided)
            self._last_sent[side] = (now, decided)
        except Exception as exc:
            if now - self._warned.get(side, 0.0) > 2.0:
                self._warned[side] = now
                log.warning("[gripper:%s] 송신 실패: %s", side, exc)

    def close(self):
        for side, g in self.grippers.items():
            for fn in (g.disable, g.disconnect):
                try:
                    fn()
                except Exception:
                    pass
        self.grippers = {}


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
    # 그리퍼 추종 판단: 클램프/데드밴드/레이트리밋/무효입력
    assert gripper_send_decision(None, None, 0.0, 0.0, 1.75, 0.005, 1 / 60) is None
    assert gripper_send_decision(float("nan"), None, 0.0, 0.0, 1.75, 0.005, 1 / 60) is None
    assert gripper_send_decision(0.5, None, 0.0, 0.0, 1.75, 0.005, 1 / 60) == 0.5
    assert gripper_send_decision(9.0, None, 0.0, 0.0, 1.75, 0.005, 1 / 60) == 1.75   # 상한 클램프
    assert gripper_send_decision(-1.0, None, 0.0, 0.0, 1.75, 0.005, 1 / 60) == 0.0   # 하한 클램프
    assert gripper_send_decision(0.5, (0.0, 0.5), 1.0, 0.0, 1.75, 0.005, 1 / 60) is None      # 데드밴드
    assert gripper_send_decision(0.6, (0.99, 0.5), 1.0, 0.0, 1.75, 0.005, 1 / 60) is None     # 레이트리밋
    assert gripper_send_decision(0.6, (0.0, 0.5), 1.0, 0.0, 1.75, 0.005, 1 / 60) == 0.6       # 통과
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
    ap.add_argument("--swap-lr", action="store_true",
                    help="좌/우 트래커↔로봇팔 매핑을 스왑(로봇을 마주보고 조작할 때 미러)")
    ap.add_argument("--pedal", action="store_true",
                    help="USB 발판(FootSwitch)을 클러치로 사용(키보드 대신, stdin 무의존)")
    ap.add_argument("--pedal-device", default="auto",
                    help="발판 evdev 경로(기본 auto=/dev/input/by-id/*FootSwitch*event-kbd)")
    ap.add_argument("--pedal-toggle", action="store_true",
                    help="발판을 밟을 때마다 토글(기본은 밟는 동안만 engage하는 momentary)")
    ap.add_argument("--start-engaged", action="store_true",
                    help="시작 시 양팔 클러치 ON (키 입력 없이 즉시 추종)")
    ap.add_argument("--left-gripper-port", default="/dev/ttyUSB2",
                    help="로봇 왼팔에 장착된 Pika Gripper 시리얼 포트(지정 시 추종 활성). "
                         "side 는 로봇팔 기준(= --swap-lr 적용 후 out_name)")
    ap.add_argument("--right-gripper-port", default="/dev/ttyUSB3",
                    help="로봇 오른팔에 장착된 Pika Gripper 시리얼 포트")
    ap.add_argument("--gripper-min-rad", type=float, default=0.0,
                    help="그리퍼 모터각 하한(rad)")
    ap.add_argument("--gripper-max-rad", type=float, default=1.75,
                    help="그리퍼 모터각 상한(rad) — Sense open 실측 ~1.71rad 기준 여유")
    ap.add_argument("--gripper-deadband-rad", type=float, default=0.005,
                    help="이 변화량 미만이면 재송신 생략")
    ap.add_argument("--gripper-rate", type=float, default=60.0,
                    help="그리퍼 POSITION_CTRL 최대 송신 Hz")
    ap.add_argument("--gripper-engaged-only", action="store_true",
                    help="클러치 engage 동안만 그리퍼 추종(기본은 항상 추종)")
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

    if a.pedal:
        clutch = PedalClutch(a.pedal_device, toggle=a.pedal_toggle).start()
        mode = "토글(밟을 때마다)" if a.pedal_toggle else "momentary(밟는 동안)"
        log.info("[umi] 발판 클러치: %s  device=%s  (%s)", "ON", clutch.path, mode)
    else:
        clutch = KeyboardClutch(a.left_key, a.right_key, a.both_key).start()
        if a.start_engaged:
            clutch.engaged = {"left": True, "right": True}
        log.info("[umi] 키: [%s]=양팔 [%s]=좌 [%s]=우 [q]=종료",
                 a.both_key if a.both_key.strip() else "space", a.left_key, a.right_key)

    # 로봇측 Pika Gripper 추종 (옵션) — side 키는 로봇팔 기준(out_name)
    gripper_ports = {}
    if a.left_gripper_port:
        gripper_ports["left"] = a.left_gripper_port
    if a.right_gripper_port:
        gripper_ports["right"] = a.right_gripper_port
    
    gripper_follow = None
    if gripper_ports:
        gripper_follow = GripperFollower(
            gripper_ports, a.gripper_min_rad, a.gripper_max_rad,
            a.gripper_deadband_rad, a.gripper_rate).start()
        log.info("[gripper] 추종 활성: %s (%s, max %.0fHz, 데드밴드 %.3frad)",
                 gripper_ports, "engage 시에만" if a.gripper_engaged_only else "항상",
                 a.gripper_rate, a.gripper_deadband_rad)
        
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
                out_name = {"left": "right", "right": "left"}[name] if a.swap_lr else name
                sides[out_name] = {"pose": pose, "gripper": gn, "deadman": engaged.get(name, False)}
                # 로봇측 그리퍼 추종: Sense 인코더 rad → 같은 로봇팔(out_name)의 Gripper 모터각
                if gripper_follow is not None and (
                        not a.gripper_engaged_only or engaged.get(name, False)):
                    grip_rad = (math.radians(grip_angle)
                                if isinstance(grip_angle, (int, float)) else None)
                    gripper_follow.update(out_name, grip_rad)

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
        if gripper_follow is not None:
            gripper_follow.close()
        sock.close()
        rec.stop()
        log.info("[umi] 종료")


if __name__ == "__main__":
    main()
