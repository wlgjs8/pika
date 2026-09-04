#!/usr/bin/env python
"""UMI 라이브 텔레오퍼레이션 포즈 퍼블리셔 (Vive Tracker → Linux robotics_lab).

collect.py가 HDF5로 '기록'만 하는 것과 달리, 이 스크립트는 양팔 트래커의
6DoF 포즈 + 그리퍼 + 클러치(deadman) 상태를 매 틱 UDP JSON으로 스트리밍한다.
robotics_lab policy_runner 의 `umi_dual_cartesian` 액션소스(UdpUmiPoseReader)가
이를 받아 relative-init 방식으로 RB 양팔을 pgmode 시뮬레이션에서 구동한다.

와이어 스키마 (robotics_lab UdpUmiPoseReader._sample_from_udp_packet 와 1:1,
gripper_rad 는 수신측 그리퍼 브리지 umi_gripper_follow.py 전용 추가 필드):
  {"t": <monotonic>,
   "left":  {"pose": [x,y,z,qx,qy,qz,qw], "gripper": <0..1>,
             "gripper_rad": <rad>, "deadman": <bool>},
   "right": {"pose": [...], ...}}
- pose 프레임 = **백엔드가 정한 월드**. 기본 백엔드는 --pose-backend survive
  (libsurvive, GUI 불필요) 이고 자체 scene solve 로 원점을 잡는다. steamvr 백엔드는
  TrackingUniverseStanding 이라 둘은 일치하지 않는다. 상대 모션만 쓰이므로 텔레옵에는
  월드 정렬/측정 캘리브레이션이 불필요하다.
- 발행은 기본 **소스 이벤트 구동**(--publish-mode event): 어느 side 든 pose_seq 가
  전진하면 그 side 의 포트로 보낸다. 고정 케이던스는 소스보다 느리면 샘플을 버리고
  빠르면 복제하는데 둘 다 필터가 없어서, 실측(2026-09-03, 좌 125Hz/우 246Hz 소스에
  발행 198Hz)에서 우측 샘플의 1/4 이 유실되고 좌측의 37% 가 중복이었다.
- pose 원점(기본 --pose-frame tip) = PIKA SDK 공식 변환을 적용한 그리퍼 핑거팁 라인
  (T_raw·R_corr·Trans(0.172,0,-0.076), 축 x=전방/y=좌/z=상). relative-init 텔레옵의
  회전 피벗 = 발행 원점이므로, 로봇 URDF TCP(그리퍼 팁)와 일치해야 직관적 조작이 된다.
  --pose-frame tracker 로 구(raw 트래커 원점) 동작 복귀 가능 — 이때는 robotics_lab
  수신측 r_align/gripper_offset config 도 함께 되돌려야 한다.
- 수신부는 side(left/right) 마다 별도 포트에 bind 하므로, 같은 결합 패킷을
  좌/우 두 목적지 포트로 각각 보낸다 (각 리더가 자기 side만 추출).
- pose 가 유효하지 않은(미검출) side 는 패킷에서 생략 → 해당 팔은 Hold.
- 로봇 장착 Pika Gripper 는 이제 robotics_lab PC(예: 172.28.60.12)에 직결 →
  여기서는 시리얼 구동 대신 Sense 인코더 raw rad 를 "gripper_rad" 로 같은
  패킷에 실어 그리퍼 포트(기본 50382)에도 송신한다. 수신/시리얼 구동은
  robotics_lab/scripts/umi_gripper_follow.py 가 담당(클램프·데드밴드·레이트리밋
  로직도 그쪽으로 이전). side 는 로봇팔 기준(--swap-lr 적용 후 out_name).

deadman(클러치)은 PikaAnyArm 트리거와 동일한 토글 의미:
  켜짐 = 클러치 engage(켜는 순간 robotics_lab이 init 스냅샷), 꺼짐 = 해제/Hold.
  키보드 키로 토글 (Windows msvcrt / Linux cbreak). 좌/우 개별 또는 공유.

실행(트래커 + Sense 연결. survive 백엔드는 SteamVR/GUI 불필요):
  .venv/bin/python scripts/umi_teleop_publish.py \
    --target-host 192.168.8.x --left-port 50380 --right-port 50381

키: [space]=양팔 클러치 토글, [a]=좌팔, [l]=우팔, [q]=종료.
"""
import argparse
import datetime
import glob
import json
import logging
import math
import os
import re
import socket
import struct
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from pika_win.usb_topology import (  # noqa: E402
    realsense_sn_for_tracker, sense_port_for_tracker)
from pika_win.pedal import find_pedal_devices, open_pedal  # noqa: E402
from pika_win.sdk_logging import quiet_pika_sdk_info  # noqa: E402

log = logging.getLogger("pika.umi_teleop")

SIDES = ("left", "right")

# 한국 표준시(KST, UTC+9) — 서버 시스템 TZ 와 무관하게 항상 KST 로 타임스탬프
KST = datetime.timezone(datetime.timedelta(hours=9), name="KST")
# 송신 간격이 이 값을 넘으면 로그 라인에 [GAP] 토큰을 붙여 grep 으로 찾기 쉽게 한다(ms)
GAP_THRESHOLD_MS = 50.0
# pose source(폴링 스레드)의 timestamp 가 이 시간 이상 안 바뀌면 스레드/openvr stall 로 보고
# [POSESTALL] 토큰. 정상 폴링은 ~4ms 갱신이므로 100ms 면 확실한 정지.
POSE_STALL_MS = 100.0


def _kst_now():
    return datetime.datetime.now(KST)


class PacketLogger:
    """매 UDP 송신을 KST 기준으로 한 줄씩 기록(실행마다 새 파일).

    timing 진단용 — 페달을 밟고 있는데도 수신측(.12)에서 50ms 이상 간격이
    생기는 원인을 송신측에서 잡기 위해, 모든 패킷의 KST 벽시계 시각 + 직전
    송신과의 간격(dt_ms) + 활성 side/deadman + 패킷 JSON 을 전부 남긴다.
    dt_ms 가 GAP_THRESHOLD_MS 를 넘으면 라인에 [GAP] 토큰을 붙인다.
    """

    def __init__(self, log_dir):
        os.makedirs(log_dir, exist_ok=True)
        stamp = _kst_now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(log_dir, f"umi_teleop_publish_{stamp}_KST.log")
        # line-buffered: 매 줄 flush → 크래시/강제종료에도 직전까지 보존
        self.fh = open(self.path, "w", buffering=1, encoding="utf-8")
        self._last_perf = None
        # side -> [마지막 pose_ts 값, 그 값이 마지막으로 '바뀐' perf 시각]
        self._pose_ts_seen = {}
        self.fh.write(f"# umi_teleop_publish packet log  started={_kst_now().isoformat()}\n")
        self.fh.write("# fields: <KST wallclock>  dt_ms=<직전 송신과 간격>  "
                      "mono=<packet t>  sides=<활성>  deadman=<L?R?>  "
                      "GAP_token(>50ms)  <packet json>\n")

    def log(self, perf_now, packet):
        dt_ms = -1.0 if self._last_perf is None else (perf_now - self._last_perf) * 1000.0
        self._last_perf = perf_now
        wall = _kst_now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        active = [n for n in SIDES if n in packet]
        dead = "".join(
            f"{n[0].upper()}{int(bool(packet.get(n, {}).get('deadman')))}" for n in SIDES
        )
        gap = " [GAP]" if dt_ms > GAP_THRESHOLD_MS else ""
        # 추적 품질: 포함된 side 별 eTrackingResult. 200=OK. 하나라도 200 아니면
        # [TRK!] 토큰 → grep '\[TRK!\]' 로 "추적 손실 중 멈춤" 을 즉시 찾는다.
        trks = {n: packet[n].get("tracking_result") for n in active
                if isinstance(packet.get(n, {}).get("tracking_result"), int)}
        trk = ""
        if trks:
            trk = "  trk=" + ",".join(f"{n[0].upper()}{v}" for n, v in trks.items())
            if any(v != 200 for v in trks.values()):
                trk += " [TRK!]"
        # pose source 정지 감지: pose_ts 가 POSE_STALL_MS 이상 안 바뀐 side
        stalls = []
        for n in active:
            pts = packet.get(n, {}).get("pose_ts")
            if not isinstance(pts, (int, float)):
                continue
            seen = self._pose_ts_seen.get(n)
            if seen is None or pts != seen[0]:
                self._pose_ts_seen[n] = [pts, perf_now]
            elif (perf_now - seen[1]) * 1000.0 > POSE_STALL_MS:
                stalls.append(f"{n[0].upper()}{int((perf_now - seen[1]) * 1000)}ms")
        if stalls:
            trk += "  posestall=" + ",".join(stalls) + " [POSESTALL]"
        # 포즈 중복(같은 샘플 재독): pose_fresh=False 인 side. grep '\[DUP\]' 로
        # "발행레이트>트래커 갱신레이트라 같은 pose 가 반복 송신된" 순간을 찾는다.
        # (posestall=폴링 스레드 정지, dup=값 미갱신 — 서로 다른 현상)
        dups = [n for n in active if packet.get(n, {}).get("pose_fresh") is False]
        if dups:
            trk += "  dup=" + ",".join(n[0].upper() for n in dups) + " [DUP]"
        self.fh.write(
            f"{wall}  dt_ms={dt_ms:7.2f}  mono={packet.get('t', 0.0):.6f}  "
            f"sides={','.join(active) or '-'}  deadman={dead}{gap}{trk}  "
            f"{json.dumps(packet, separators=(',', ':'))}\n"
        )

    def close(self):
        try:
            self.fh.write(f"# closed={_kst_now().isoformat()}\n")
            self.fh.close()
        except (OSError, ValueError):
            pass


# ----------------------------- 순수 패킷 빌더 (하드웨어/openvr 불필요, 테스트 가능) -----------------------------
class PosePublishGate:
    """언제 패킷을 보낼지 정하고 side 별 pose_seq 회계를 한다 (하드웨어 불필요).

    고정 케이던스 발행은 소스 갱신률과 아무 관계가 없어서, 소스가 빠르면 샘플을
    **버리고** 느리면 같은 포즈를 **복제한다**. 둘 다 필터 없이 일어난다:
    실측 2026-09-03 (좌 125Hz / 우 246Hz 소스, 발행 198Hz) 에서 우측은 발행의 24%
    에서 pose_seq 가 2씩 뛰었고(매 4번째 샘플 유실 = 안티에일리어싱 없는 데시메이션),
    좌측은 37%가 중복이었다. 중복은 수신측 이동평균도 희석시킨다 — 8패킷 창에 서로
    다른 포즈가 5.4개뿐이라 노이즈 저감이 이상적 대비 1.28배 나빴다.

    event 모드는 그래서 소스 이벤트에 발행을 맞춘다: 어느 side 든 pose_seq 가 마지막
    송신 대비 전진하면 보낸다. 포즈가 안 바뀌어도 max_hold_sec 이 지나면 보내는데,
    클러치(deadman)와 그리퍼는 포즈와 무관하게 바뀌므로 트래커가 멈춘 동안 수신측이
    굶으면 안 되기 때문이다.

    폴링이 소스보다 느리면 event 모드에서도 샘플이 유실된다 — 그건 조용히 넘어가지
    않고 `skipped` 로 세어 호출부가 경고할 수 있게 한다.
    """

    def __init__(self, sides, event_mode=True, max_hold_sec=0.02):
        self.sides = tuple(sides)
        self.event_mode = bool(event_mode)
        self.max_hold_sec = float(max_hold_sec)
        self.last_sent_seq = {n: None for n in self.sides}
        self.dup = {n: 0 for n in self.sides}
        self.sent = {n: 0 for n in self.sides}
        self.skipped = {n: 0 for n in self.sides}
        self._last_send = {n: None for n in self.sides}

    def freshness(self, sides):
        """side -> 이 포즈가 마지막 '송신' 대비 새 것인가. seq 가 없으면 True(구 소스 보존)."""
        out = {}
        for nm, s in sides.items():
            seq = s.get("pose_seq")
            if isinstance(seq, int):
                prev = self.last_sent_seq.get(nm)
                out[nm] = (prev is None) or (seq != prev)
            else:
                out[nm] = True
        return out

    def sides_to_send(self, fresh_flags, now):
        """side -> 이번에 그 side 의 포트로 보낼 것인가.

        판정이 **side 별**인 것이 핵심이다. 좌/우는 각자의 UDP 포트로 나가고 수신측은
        자기 side 만 읽으므로, 한쪽이 갱신됐다고 다른 쪽까지 보내면 그쪽에는 같은 포즈가
        중복 송신된다 — 그리고 그 중복이 수신측 이동평균을 희석시킨다(8패킷 창에 서로
        다른 포즈 5.4개, 노이즈 저감 1.28배 손해). 좌우 갱신률이 다른 동안에는 전역
        판정으로 이 문제를 못 없앤다.
        """
        if not self.event_mode:
            return {nm: True for nm in fresh_flags}
        out = {}
        for nm, fresh in fresh_flags.items():
            last = self._last_send.get(nm)
            out[nm] = bool(fresh) or last is None or (now - last) >= self.max_hold_sec
        return out

    def commit(self, sides, fresh_flags, send_flags, now, stamp_fresh=True):
        """송신 확정 — 실제로 보낸 side 만 seq 를 전진시키고 중복/유실을 센다."""
        for nm, s in sides.items():
            if stamp_fresh:
                s["pose_fresh"] = fresh_flags[nm]
            if not send_flags.get(nm):
                continue
            self._last_send[nm] = now
            seq = s.get("pose_seq")
            if not isinstance(seq, int):
                continue
            prev = self.last_sent_seq.get(nm)
            if prev is not None and seq - prev > 1:
                self.skipped[nm] += seq - prev - 1
            self.last_sent_seq[nm] = seq
            self.sent[nm] += 1
            if not fresh_flags[nm]:
                self.dup[nm] += 1


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
    """sides: {name: {"pose":[7]|None, "gripper":float|None,
    "gripper_rad":float|None, "deadman":bool}} → wire dict.

    pose 가 유효하지 않은 side 는 생략한다(수신부가 None→Hold 처리).
    gripper_rad(Sense raw rad) 는 유효(finite)할 때만 포함 — 수신측 그리퍼
    브리지가 None/누락을 Hold 로 처리한다.
    """
    packet = {"t": float(t)}
    for name in SIDES:
        s = sides.get(name)
        if not s or not _pose_valid(s.get("pose")):
            continue
        entry = {
            "pose": [float(v) for v in s["pose"]],
            "gripper": float(s.get("gripper") or 0.0),
            "deadman": bool(s.get("deadman")),
        }
        rad = s.get("gripper_rad")
        if isinstance(rad, (int, float)) and math.isfinite(rad):
            entry["gripper_rad"] = float(rad)
        # tracking_result(eTrackingResult): 200=Running_OK. 진단용으로 와이어에 실어
        # 송신 로그에서 "pose 정지 = 추적 손실(201/300)인지" 바로 보이게 한다.
        # 수신측은 모르는 키를 무시하므로 스키마 호환(추가만).
        tr = s.get("tracking_result")
        if isinstance(tr, int):
            entry["tracking_result"] = tr
        # pose_ts: pose source(폴링 스레드)가 이 트래커를 마지막 '폴링' 한 time.time().
        # 폴 시각이라 값이 멈춰도 advance 한다 → 폴링 스레드 live 판별엔 쓰되,
        # "중복 샘플" 판별엔 못 쓴다(그건 pose_seq 로).
        pts = s.get("pose_ts")
        if isinstance(pts, (int, float)) and math.isfinite(pts):
            entry["pose_ts"] = float(pts)
        # 중복(같은 샘플 재독) 신호. dedup 활성 시 main 이 채운다:
        #   pose_seq   = 값 변화 시에만 증가하는 시퀀스(소스에서 전달)
        #   pose_fresh = 직전 '송신' 대비 이 side 의 pose_seq 가 전진했는지
        # 수신측은 pose_fresh=False 면 pose 를 '새 측정' 으로 쓰지 말고 Hold
        # (deadman/gripper 는 그대로 적용). 모르는 키는 무시되므로 구 수신기 무영향.
        seq = s.get("pose_seq")
        if isinstance(seq, int):
            entry["pose_seq"] = seq
        pf = s.get("pose_fresh")
        if pf is not None:
            entry["pose_fresh"] = bool(pf)
        packet[name] = entry
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

    def __init__(self, device="auto", toggle=False, debounce_sec=0.05, grab=True,
                 mute_others=False):
        self.device_arg = device
        self.toggle = bool(toggle)
        self.debounce_sec = max(0.0, float(debounce_sec))
        # 기본 배타 점유: 안 하면 발판이 키보드로도 동작해 포커스된 창/터미널에
        # 'b' 를 계속 입력한다(PCsensor 기본 키맵 + X11 오토리피트 → 밟고 있는 동안 도배).
        self.grab = bool(grab)
        # 클러치로 쓰지 않는 나머지 발판까지 점유해 키 누수만 막는다(이벤트는 안 읽음).
        self.mute_others = bool(mute_others)
        self._mute_fds = []
        self.fds = {}            # path -> fd (발판이 여러 개면 전부 수신)
        self.path = None         # 표시용(여러 개면 쉼표 결합)
        self._raw_by_fd = {}     # fd -> 그 장치의 raw 눌림 상태
        self.held = False        # momentary: 발판 눌림 상태(디바운스 후 committed)
        self.engaged_both = False  # toggle: 누적 상태
        self.quit = False        # 인터페이스 호환(발판엔 종료 키 없음)
        # 접점 바운스 제거: raw 키 상태가 debounce_sec 동안 안정될 때만 commit.
        # 격한/빠른 모션의 진동이 spurious press/release edge 를 내도 한 번의
        # 논리 edge 로 합쳐져 deadman 이 떨리지 않는다.
        self._raw_held = False
        self._last_edge_mono = 0.0

    find_devices = staticmethod(find_pedal_devices)

    def _resolve_devices(self):
        if self.device_arg not in ("auto", ""):
            return [p.strip() for p in self.device_arg.split(",") if p.strip()]
        found = self.find_devices()
        if len(found) > 1:
            # 발판이 여러 개면 **자동 선택하지 않는다.** 이 PC 는 teleop 용 1구 발판과
            # robotics_lab(rb_gui InitMotion) 용 3구 발판을 함께 쓰는데, 둘은 VID:PID·
            # evdev capability 가 완전히 동일해 물리 포트(by-path) 말고는 구별할 수단이
            # 없다. 잘못 고르면 robotics_lab 발판을 밟았을 때 teleop 클러치가 걸려
            # 로봇이 움직인다 → 모호하면 기동을 거부하는 쪽이 안전하다.
            raise RuntimeError(
                "발판이 여러 개 감지되어 자동 선택할 수 없습니다(VID:PID·capability 동일). "
                "--pedal-device 로 하나를 지정하세요:\n  " + "\n  ".join(found) +
                "\n어느 것이 어느 발판인지는 `python scripts/pedal_test.py` 로 확인하세요.")
        return found

    def start(self):
        paths = self._resolve_devices()
        if not paths:
            raise RuntimeError(
                "FootSwitch evdev 장치를 찾지 못함. `python scripts/pedal_test.py` 로 "
                "장치를 확인하거나 --pedal-device 로 직접 지정하고, 권한(plugdev 그룹 / "
                "udev 99-footswitch.rules)을 확인하세요.")
        errors = []
        for path in paths:
            try:
                fd = open_pedal(path, grab=self.grab)
            except OSError as e:
                errors.append(f"{path}: {e}")
                continue
            self.fds[path] = fd
            self._raw_by_fd[fd] = False
        if not self.fds:
            raise RuntimeError("발판 evdev 열기 실패 — " + "; ".join(errors) +
                               ". 'sudo usermod -aG plugdev $USER' 후 재로그인이 필요할 수 있습니다.")
        self.path = ",".join(self.fds)
        if self.mute_others:
            self._mute_other_pedals()
        if len(self.fds) > 1:
            # 여러 발판을 동시에 수신한다(어느 것을 밟아도 클러치가 걸린다).
            # 한 개만 쓰려면 --pedal-device 로 경로를 지정할 것.
            log.info("[umi] 발판 %d개 동시 수신: %s", len(self.fds), self.path)
        if errors:
            log.warning("[umi] 일부 발판을 열지 못함: %s", "; ".join(errors))
        return self

    def _mute_other_pedals(self):
        """클러치로 안 쓰는 나머지 발판을 점유만 해서 키 누수를 막는다(이벤트는 안 읽음).

        이 리그의 3구 발판은 robotics_lab rb_gui 전용이라 평소엔 rb_gui 가 점유한다.
        발행자만 단독 실행하면 아무도 안 잡아서, 그 발판을 밟는 동안 X11 오토리피트로
        터미널이 'bbbb...' 로 도배된다. rb_gui 가 이미 잡고 있으면 여기 grab 은 실패하는데,
        그건 이미 막혀 있다는 뜻이므로 정상이다(경고만 남기고 넘어간다).
        """
        mine = {os.path.realpath(p) for p in self.fds}
        for path in find_pedal_devices():
            if os.path.realpath(path) in mine:
                continue
            try:
                self._mute_fds.append(open_pedal(path, grab=True))
                log.info("[umi] 발판 키 누수 차단(점유만): %s", path)
            except OSError as e:
                log.warning("[umi] %s 누수 차단 실패: %s", path, e)

    def update(self):
        now = time.monotonic()
        # 쌓인 이벤트를 모두 소비해 raw 키 상태 갱신(바운스 edge 포함).
        # raw 가 바뀔 때마다 _last_edge_mono 를 갱신 → 바운스 동안엔 계속 리셋.
        for fd in list(self._raw_by_fd):
            while True:
                try:
                    data = os.read(fd, self.EVENT.size * 64)
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
                    if value == 1:
                        self._raw_by_fd[fd] = True
                    elif value == 0:
                        self._raw_by_fd[fd] = False
                    # value==2(오토리피트)·동일값 반복은 무시
        # 여러 발판 중 하나라도 눌려 있으면 눌린 것으로 본다.
        raw_any = any(self._raw_by_fd.values())
        if raw_any != self._raw_held:
            self._raw_held = raw_any
            self._last_edge_mono = now
        # raw 상태가 debounce_sec 동안 안정되면 commit. toggle 은 commit 된
        # 누름 edge(상승)에서만 누적 상태를 뒤집는다 → 바운스로 인한 다중 토글 제거.
        if self._raw_held != self.held and (now - self._last_edge_mono) >= self.debounce_sec:
            pressed_edge = self._raw_held and not self.held
            self.held = self._raw_held
            if self.toggle and pressed_edge:
                self.engaged_both = not self.engaged_both
        on = self.engaged_both if self.toggle else self.held
        return {"left": on, "right": on}

    def close(self):
        for fd in list(self._raw_by_fd) + self._mute_fds:
            try:
                os.close(fd)
            except OSError:
                pass
        self.fds.clear()
        self._raw_by_fd.clear()
        self._mute_fds.clear()


# ----------------------------- arms.json 로더 (collect.build_arms 의 최소 버전) -----------------------------
def resolve_com_port(name, com_port, tracker_sn):
    """arms.json 의 com_port 해석. "auto"(또는 미지정)면 트래커와 같은 USB 체인에서 찾는다.

    by-path 를 박아두면 포트를 옮길 때마다 깨지고, 더 나쁘게는 **조용히 교차 배선**된다
    (2026-09-04: left 의 com_port 가 right 유닛의 그리퍼를 가리켰는데 연결은 성공했다).
    트래커와 그리퍼는 같은 PIKA Sense 유닛 = 같은 USB 체인이므로 그 결속으로 찾는 것이
    유일하게 안전하다.
    """
    if com_port and str(com_port).lower() != "auto":
        return com_port
    if not tracker_sn:
        raise SystemExit(f"[arms] {name}: com_port=auto 인데 tracker_sn 이 없습니다 — "
                         f"자동 탐색은 트래커 기준입니다")
    port = sense_port_for_tracker(tracker_sn)
    if not port:
        raise SystemExit(
            f"[arms] {name}: 트래커 {tracker_sn} 과 같은 USB 체인에서 그리퍼 시리얼을 "
            f"찾지 못했습니다. 트래커가 연결돼 있는지 확인하거나 arms.json 에 "
            f"com_port 를 명시하세요 (ls -l /dev/serial/by-path/)")
    log.info("[arms] %s: com_port 자동 = %s (트래커 %s 와 같은 체인)", name, port, tracker_sn)
    return port


def resolve_realsense_sn(name, sn, tracker_sn):
    """"auto"(또는 미지정)면 트래커와 같은 유닛의 RealSense 시리얼을 찾는다.

    RealSense 는 USB3 라 트래커(USB2)와 다른 버스에 올라오지만, 커널의 port/peer 가
    같은 물리 허브의 두 트리를 이어주므로 유닛 결속은 유지된다.
    """
    if sn and str(sn).lower() != "auto":
        return sn
    if not tracker_sn:
        return None
    found = realsense_sn_for_tracker(tracker_sn)
    if found:
        log.info("[arms] %s: realsense_sn 자동 = %s (트래커 %s 와 같은 유닛)",
                 name, found, tracker_sn)
    else:
        log.warning("[arms] %s: 트래커 %s 와 같은 유닛의 RealSense 를 찾지 못했습니다",
                    name, tracker_sn)
    return found


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
            com_port=resolve_com_port(name, d.get("com_port"), d.get("tracker_sn")),
            realsense_sn=resolve_realsense_sn(name, d.get("realsense_sn"),
                                              d.get("tracker_sn")),
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
    # gripper_rad: finite 일 때만 포함 (None/NaN → 키 생략 = 수신측 Hold)
    pkt3 = build_packet(4.0, {
        "left": {"pose": [0, 0, 0, 0, 0, 0, 1], "gripper": 0.2,
                 "gripper_rad": 0.7, "deadman": True},
        "right": {"pose": [0, 0, 0, 0, 0, 0, 1], "gripper": 0.2,
                  "gripper_rad": float("nan"), "deadman": True},
    })
    assert pkt3["left"]["gripper_rad"] == 0.7
    assert "gripper_rad" not in pkt3["right"]
    assert "gripper_rad" not in build_packet(5.0, {
        "left": {"pose": [0, 0, 0, 0, 0, 0, 1], "deadman": True}})["left"]
    # 그리퍼 정규화
    assert normalize_gripper(5, 0, 10) == 0.5
    assert normalize_gripper(99, 0, 10) == 1.0
    assert normalize_gripper(-5, 0, 10) == 0.0
    assert normalize_gripper(None, 0, 10) is None
    assert normalize_gripper(5, 0, 0) is None  # 범위 미확정
    # (그리퍼 추종 클램프/데드밴드/레이트리밋 판단은 robotics_lab/scripts/
    #  umi_gripper_follow.py 로 이전 — 해당 selftest 도 그쪽에 있음)
    # 트래커→그리퍼 팁 공식 변환 (pose_math 는 백엔드 중립 — openvr/pysurvive 불필요)
    from pika_win.pose_math import (
        TIP_ROTATION_QUAT, TIP_TRANSLATION, apply_tip_transform, quat_rotate_vec)
    # raw 항등 포즈 → 원점은 raw frame 레버암 R_corr·t = [0, -0.0126, +0.1876] 로 이동
    pos, quat = apply_tip_transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    expect = quat_rotate_vec(TIP_ROTATION_QUAT, TIP_TRANSLATION)
    assert all(abs(p - e) < 1e-12 for p, e in zip(pos, expect)), (pos, expect)
    assert abs(pos[0]) < 1e-9 and abs(pos[1] + 0.0126) < 1e-3 and abs(pos[2] - 0.1876) < 1e-3, pos
    assert abs(sum(c * c for c in quat) - 1.0) < 1e-9
    # 접근축(팁 frame x)은 raw 트래커 +z에서 정확히 20°
    approach_raw = quat_rotate_vec(quat, (1.0, 0.0, 0.0))
    angle = math.degrees(math.acos(max(-1.0, min(1.0, approach_raw[2]))))
    assert abs(angle - 20.0) < 1e-6, angle
    # 병진은 raw 포즈 회전을 따라간다 (z축 90° 회전 시 레버암도 90° 회전)
    s = math.sin(math.pi / 4)
    pos_rot, _ = apply_tip_transform((1.0, 2.0, 3.0), (0.0, 0.0, s, s))
    lever = quat_rotate_vec((0.0, 0.0, s, s), expect)
    assert all(abs(p - (b + l)) < 1e-9 for p, b, l in zip(pos_rot, (1.0, 2.0, 3.0), lever))
    print("selftest OK (tip-transform 포함)")


def get_arguments(argv=None):
    ap = argparse.ArgumentParser(description="UMI 라이브 텔레오퍼레이션 포즈 퍼블리셔")
    ap.add_argument("--selftest", action="store_true",
                    help="하드웨어 없이 패킷 빌더 검증 후 종료")
    ap.add_argument("--config", default=os.path.join(REPO_ROOT, "config", "arms.json"))
    ap.add_argument("--target-host", default="127.0.0.1",
                    help="robotics_lab(수신) 호스트 IP")
    ap.add_argument("--left-port", type=int, default=50380)
    ap.add_argument("--right-port", type=int, default=50381)
    ap.add_argument("--publish-mode", choices=("event", "fixed"), default="event",
                    help="event(기본)=포즈 소스가 갱신될 때 발행. fixed=--rate 고정 케이던스"
                         "(구 동작). 고정 케이던스는 소스보다 느리면 샘플을 버리고 빠르면 "
                         "같은 포즈를 복제하는데, 둘 다 필터 없이 일어난다 — 실측 2026-09-03: "
                         "좌 125Hz/우 246Hz 소스에 발행 198Hz 라 우측은 24%의 발행에서 "
                         "pose_seq 가 2씩 뛰었고(=매 4번째 샘플 유실), 좌측은 37%가 중복이었다.")
    ap.add_argument("--rate", type=float, default=200.0,
                    help="발행 Hz (--publish-mode fixed 에서만)")
    ap.add_argument("--poll-hz", type=float, default=500.0,
                    help="event 모드 폴링 Hz. 가장 빠른 트래커보다 넉넉히 높아야 한다"
                         "(느리면 다시 샘플을 버리며, 그때는 [SKIP] 경고가 뜬다).")
    ap.add_argument("--max-hold-ms", type=float, default=20.0,
                    help="event 모드에서 포즈 갱신이 없어도 이 시간이 지나면 발행한다 — "
                         "클러치(deadman)/그리퍼는 포즈와 무관하게 바뀌므로 트래커가 "
                         "멈춰도 수신측이 굶지 않게 한다.")
    ap.add_argument("--no-dedup-pose", dest="dedup_pose", action="store_false",
                    help="포즈 중복 표시(pose_fresh) 비활성. 기본 활성: 발행 레이트가 "
                         "트래커 native 갱신(~120Hz)보다 빨라 같은 pose 가 반복 송신되면 "
                         "그 패킷의 해당 side 에 pose_fresh=false 를 실어 수신측이 Hold 하게 "
                         "한다(케이던스/그리퍼/deadman 은 유지). 비활성 시 구 동작(중복 그대로).")
    ap.set_defaults(dedup_pose=True)
    ap.add_argument("--no-sense", action="store_true", help="Sense(그리퍼) 미연결")
    ap.add_argument("--grip-open", type=float, default=0, help="그리퍼 open 각도(미지정=자동 범위)")
    ap.add_argument("--grip-closed", type=float, default=98, help="그리퍼 closed 각도")
    ap.add_argument("--left-key", default="a")
    ap.add_argument("--right-key", default="l")
    ap.add_argument("--both-key", default=" ")
    ap.add_argument("--swap-lr", action="store_true",
                    help="좌/우 트래커↔로봇팔 매핑을 스왑(로봇을 마주보고 조작할 때 미러)")
    ap.add_argument("--pose-backend", choices=("survive", "steamvr"), default="survive",
                    help="포즈 백엔드: survive=libsurvive(GUI 불필요, 기본), "
                         "steamvr=OpenVR(SteamVR 실행 필요)")
    # ---- libsurvive 튜닝 (백엔드 survive 일 때만 적용) --------------------------
    # PoseSurvive 는 원래 이 인자들을 다 받는데 CLI 로 나와 있지 않아, solver/필터/
    # 캘리브레이션을 코드 수정 없이는 A/B 할 수 없었다.
    ap.add_argument("--survive-arg", action="append", default=None, metavar="ARG",
                    help="libsurvive 에 그대로 넘길 인자(반복 가능). 값이 '-' 로 시작하면 "
                         "argparse 가 옵션으로 오인하므로 **= 형식**을 써야 한다: "
                         "--survive-arg=--poser --survive-arg=MPFIT")
    ap.add_argument("--survive-exclude-id", action="append", default=None, metavar="HEXID",
                    # 기본값은 pika_win.libsurvive_config.EXCLUDED_IDS (단일 출처).
                    help="이 OOTX id 의 라이트하우스를 추적/해에서 제외한다(반복 가능). "
                         "공용 공간에서 **다른 사용자의 베이스스테이션**이 켜져도 우리 해에 "
                         "섞이지 않게 한다. libsurvive 의 lighthouse-N-disable 은 N 이 채널이 "
                         "아니라 **슬롯 인덱스**라서 재캘리브레이션마다 달라질 수 있는데, "
                         "여기서는 매 실행 config 에서 id -> 슬롯을 다시 찾으므로 안전하다.")
    ap.add_argument("--survive-config", default=None,
                    help="라이트하우스 해 저장/재사용 경로(기본 config/libsurvive_config.json). "
                         "실행 간 월드 프레임을 고정하려면 같은 파일을 계속 써야 한다.")
    ap.add_argument("--survive-force-calibrate", action="store_true",
                    help="저장된 라이트하우스 해를 버리고 다시 캘리브레이션한다.")
    ap.add_argument("--survive-target-hz", type=float, default=None,
                    help="포즈 폴링 스레드의 유휴 상한 Hz(기본 250). libsurvive 는 "
                         "이벤트 구동이라 이 값은 '갱신이 없을 때' 만 쓰인다.")
    ap.add_argument("--survive-stale-timeout", type=float, default=None,
                    help="이 시간(초) 동안 기기 갱신이 없으면 valid=False (기본 0.25). "
                         "libsurvive 는 정지한 트래커도 계속 갱신하므로 '갱신 없음'은 "
                         "실제로 추적 손실이다.")
    ap.add_argument("--pose-frame", choices=("tip", "tracker"), default="tip",
                    help="발행 포즈 원점: tip=PIKA 공식 그리퍼 팁 변환 적용(기본, "
                         "URDF 팁 TCP와 짝), tracker=raw 트래커 원점(구 동작)")
    ap.add_argument("--require-all-trackers", action="store_true",
                    help="config에 설정된 tracker SN이 모두 보이지 않으면 Sense 연결과 "
                         "UDP 송신 전에 종료. 표준 run_umi_teleop_publish.sh는 양팔 안전을 "
                         "위해 이 옵션을 기본 적용")
    ap.add_argument("--pedal", action="store_true",
                    help="USB 발판(FootSwitch)을 클러치로 사용(키보드 대신, stdin 무의존)")
    ap.add_argument("--pedal-device", default="auto",
                    help="발판 evdev 경로(기본 auto=/dev/input/by-id/*FootSwitch*event-kbd)")
    ap.add_argument("--mute-other-pedals", action="store_true",
                    help="클러치로 안 쓰는 나머지 발판까지 점유해 키 누수를 막는다(이벤트는 "
                         "읽지 않음). robotics_lab 을 같이 띄우면 rb_gui 가 자기 발판을 이미 "
                         "점유하므로 불필요하다. 발행자만 단독 실행할 때 쓸 것 — 이 상태에서 "
                         "rb_gui 를 나중에 띄우면 그쪽 발판이 비활성된다")
    ap.add_argument("--no-pedal-grab", dest="pedal_grab", action="store_false",
                    help="발판 배타 점유(EVIOCGRAB) 비활성. 기본은 점유해서 발판 키가 "
                         "터미널/창으로 새는 것을 막는다. 진단용으로만 끌 것")
    ap.add_argument("--pedal-toggle", action="store_true",
                    help="발판을 밟을 때마다 토글(기본은 밟는 동안만 engage하는 momentary)")
    ap.add_argument("--pedal-debounce-sec", type=float, default=0.05,
                    help="발판 접점 바운스 제거 창(초, 기본 0.05). raw 키 상태가 이 시간 "
                         "동안 안정될 때만 반영 → 진동/빠른 모션 중 spurious deadman 토글 방지. 0=비활성")
    ap.add_argument("--start-engaged", action="store_true",
                    help="시작 시 양팔 클러치 ON (키 입력 없이 즉시 추종)")
    ap.add_argument("--gripper-port", type=int, default=50382,
                    help="수신측 그리퍼 브리지(robotics_lab umi_gripper_follow.py) UDP 포트. "
                         "같은 패킷을 이 포트에도 추가 송신(gripper_rad 포함). 0=비활성. "
                         "그리퍼는 이제 robotics_lab PC 에 직결 — 시리얼 옵션은 그쪽 스크립트로 이전")
    ap.add_argument("--show-sdk-parse-errors", action="store_true",
                    help="Pika SDK 시리얼 JSON 파싱 오류를 스로틀된 경고로 표시")
    ap.add_argument("--no-packet-log", action="store_true",
                    help="UDP 송신 패킷 로깅 비활성(기본은 실행마다 pika/logs/ 하위에 "
                         "KST 타임스탬프 파일로 모든 송신 패킷을 기록 — timing 진단용)")
    ap.add_argument("--packet-log-dir", default=os.path.join(REPO_ROOT, "logs"),
                    help="패킷 로그 디렉터리(기본 pika/logs)")
    ap.add_argument("--verbose", action="store_true")
    args, _ = ap.parse_known_args(argv)
    return args


def survive_options(a):
    """--survive-* 플래그 -> PoseSurvive kwargs. 지정된 것만 담는다(기본값 보존).

    survive 백엔드가 아닐 때는 비어 있어야 한다 — PoseSteamVR 는 이 키들을 모르고,
    넘기면 TypeError 로 죽는다.
    """
    if getattr(a, "pose_backend", "survive") != "survive":
        return {}
    opts = {}
    if getattr(a, "survive_config", None):
        opts["config_path"] = a.survive_config
    if getattr(a, "survive_force_calibrate", False):
        opts["force_calibrate"] = True
    if getattr(a, "survive_target_hz", None):
        opts["target_hz"] = float(a.survive_target_hz)
    if getattr(a, "survive_stale_timeout", None):
        opts["stale_timeout"] = float(a.survive_stale_timeout)
    from pika_win.libsurvive_config import exclude_args
    extra = list(getattr(a, "survive_arg", None) or [])
    from pika_win.libsurvive_config import default_exclude_ids
    ids = getattr(a, "survive_exclude_id", None)
    extra += exclude_args(ids if ids is not None else default_exclude_ids(),
                          opts.get("config_path"), log=log.info)
    if extra:
        opts["extra_args"] = extra
    return opts


def _create_recorder(a, arms, recorder_cls=None):
    """CLI 안전 옵션을 EpisodeRecorder에 전달한다(하드웨어 없이 테스트 가능)."""
    if recorder_cls is None:
        from pika_win.recorder import EpisodeRecorder
        recorder_cls = EpisodeRecorder
    return recorder_cls(out_dir=os.path.join(REPO_ROOT, "data", "_umi_teleop_tmp"),
                        arms=arms, use_realsense=False, use_fisheye=False,
                        use_sense=not a.no_sense, use_pose=True, require_pose=True,
                        require_all_trackers=a.require_all_trackers,
                        pose_tip_frame=(a.pose_frame == "tip"),
                        pose_backend=a.pose_backend,
                        pose_options=survive_options(a))


def main():
    a = get_arguments()
    if a.selftest:
        selftest()
        return
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(message)s")
    quiet_pika_sdk_info(show_parse_errors=a.show_sdk_parse_errors)
    arms = load_arms(a.config)
    rec = _create_recorder(a, arms)
    rec.start()
    names = rec.arm_names()
    log.info("[umi] 활성 팔: %s  pose_frame=%s", names,
             "gripper_tip(공식 변환)" if a.pose_frame == "tip" else "tracker_raw")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # side -> 그 side 전용 포트. event 모드는 이 포트로 그 side 가 갱신됐을 때만 보낸다.
    side_targets = {"left": (a.target_host, a.left_port),
                    "right": (a.target_host, a.right_port)}
    # 그리퍼 브리지는 양쪽 그리퍼를 함께 읽으므로 어느 side 든 보낼 때 같이 보낸다.
    gripper_target = (a.target_host, a.gripper_port) if a.gripper_port else None
    log.info("[umi] 송신 대상: %s%s (포즈는 side 별, 그리퍼 브리지는 공통)",
             list(side_targets.values()),
             f" + {gripper_target}" if gripper_target else "")

    pkt_log = None if a.no_packet_log else PacketLogger(a.packet_log_dir)
    if pkt_log is not None:
        log.info("[umi] 패킷 로그(KST): %s", pkt_log.path)

    if a.pedal:
        clutch = PedalClutch(a.pedal_device, toggle=a.pedal_toggle,
                             debounce_sec=a.pedal_debounce_sec,
                             grab=a.pedal_grab,
                             mute_others=a.mute_other_pedals).start()
        mode = "토글(밟을 때마다)" if a.pedal_toggle else "momentary(밟는 동안)"
        log.info("[umi] 발판 클러치: %s  device=%s  (%s, %s)", "ON", clutch.path, mode,
                 "배타 점유" if a.pedal_grab else "점유 안 함(키가 터미널로 샘)")
    else:
        clutch = KeyboardClutch(a.left_key, a.right_key, a.both_key).start()
        if a.start_engaged:
            clutch.engaged = {"left": True, "right": True}
        log.info("[umi] 키: [%s]=양팔 [%s]=좌 [%s]=우 [q]=종료",
                 a.both_key if a.both_key.strip() else "space", a.left_key, a.right_key)

    # 그리퍼 자동 범위(open/closed 미지정 시 관측 min/max 누적)
    grange = {n: [a.grip_open, a.grip_closed] for n in SIDES}
    event_mode = a.publish_mode == "event"
    period = 1.0 / a.rate if a.rate > 0 else 0.0
    poll_period = (1.0 / a.poll_hz if a.poll_hz > 0 else 0.0) if event_mode else period
    max_hold = max(0.0, a.max_hold_ms / 1000.0)
    last_skip_warn = 0.0
    log.info("[umi] 발행: %s", f"event (poll {a.poll_hz:.0f}Hz, max_hold {a.max_hold_ms:.0f}ms)"
             if event_mode else f"fixed {a.rate:.0f}Hz")
    last_log = 0.0
    # 송신 판정 + pose_seq 회계. 비교 기준은 '내가 마지막에 보낸 seq' 이지 소스의
    # per-poll fresh 가 아니다 — 그래야 발행레이트>갱신레이트 구간의 재독이 잡힌다.
    gate = PosePublishGate(SIDES, event_mode=event_mode, max_hold_sec=max_hold)
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
                # gripper_rad: Sense 인코더 raw rad — 수신측(umi_gripper_follow.py)이
                # 같은 로봇팔(out_name)의 Gripper 모터각으로 1:1 추종
                grip_rad = (math.radians(grip_angle)
                            if isinstance(grip_angle, (int, float)) else None)
                sides[out_name] = {"pose": pose, "gripper": gn, "gripper_rad": grip_rad,
                                   "deadman": engaged.get(name, False),
                                   "tracking_result": arm.get("tracking_result"),
                                   "pose_ts": arm.get("pose_ts"),
                                   "pose_seq": arm.get("pose_seq")}

            # 각 side 의 pose_seq 가 직전 '송신' 대비 전진했는지 = 그 side 의 pose_fresh.
            # seq 가 없으면(구 소스/무효 pose) fresh=True 로 둬 기존 동작 보존.
            # 판정은 **발행 여부와 무관하게** 매 폴에서 해야 한다: event 모드의 송신
            # 조건이 바로 이 값이기 때문이다.
            fresh_flags = gate.freshness(sides)
            send_flags = gate.sides_to_send(fresh_flags, tick)
            if not any(send_flags.values()):
                rem = poll_period - (time.perf_counter() - tick)
                if rem > 0:
                    time.sleep(rem)
                continue
            gate.commit(sides, fresh_flags, send_flags, tick, stamp_fresh=a.dedup_pose)
            if event_mode and any(gate.skipped.values()) and tick - last_skip_warn > 10.0:
                last_skip_warn = tick
                log.warning("[umi] [SKIP] 폴링(%.0fHz)이 소스를 못 따라가 샘플 유실: %s "
                            "— --poll-hz 를 올리세요", a.poll_hz,
                            ", ".join(f"{n}={gate.skipped[n]}" for n in SIDES if gate.skipped[n]))
            packet = build_packet(time.monotonic(), sides)
            data = json.dumps(packet).encode("utf-8")
            for nm, tgt in side_targets.items():
                if send_flags.get(nm):
                    sock.sendto(data, tgt)
            if gripper_target is not None:
                sock.sendto(data, gripper_target)
            if pkt_log is not None:
                pkt_log.log(time.perf_counter(), packet)

            now = time.time()
            if a.verbose and now - last_log > 0.5:
                last_log = now
                active = [n for n in SIDES if n in packet]
                dupinfo = " ".join(
                    f"{n}:{gate.dup[n]}/{gate.sent[n]}dup"
                    + (f"/{gate.skipped[n]}skip" if gate.skipped[n] else "")
                    for n in SIDES if gate.sent[n]
                )
                log.debug("[umi] eff_pose_hz=%.0f engaged=%s sides=%s %s",
                          getattr(rec.pose, "effective_hz", 0.0), engaged, active, dupinfo)

            rem = poll_period - (time.perf_counter() - tick)
            if rem > 0:
                time.sleep(rem)
    except KeyboardInterrupt:
        pass
    finally:
        clutch.close()
        sock.close()
        rec.stop()
        if pkt_log is not None:
            pkt_log.close()
        log.info("[umi] 종료")


if __name__ == "__main__":
    main()
