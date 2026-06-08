#!/usr/bin/env python
"""키보드 기반 연속 에피소드 수집 (Windows/Linux, 단일/양팔 자동) + 진단 로깅.

- 인식된 Vive 트래커 개수로 모드 자동: 1개=한팔, 2개=양팔(BIMANUAL).
- 키보드 b 키로 녹화 시작/정지 토글. 그리퍼 움직임은 녹화 제어에 사용하지 않음.
- 시작~정지 구간 = 에피소드 1개 → data/data_<시각>/episode_NNN.hdf5 자동 저장.
- rerun 라이브 뷰어(--view)는 팔별로 카메라/포즈/그리퍼를 모두 표시.
- 모든 상태/그리퍼/시리얼 진단은 stdout 로깅.

실행: conda run -n pika python scripts\\collect.py                 (헤드리스 + 진단)
      conda run -n pika python scripts\\collect.py --view web      (브라우저 뷰어)
양팔 매핑은 보통 config/arms.json(make identify 로 생성)으로 고정. CLI 예: --coms COM3,COM4 또는 /dev/serial/by-id/...
전제: SteamVR/OpenVR 또는 호환 포즈 백엔드 실행, PIKA Sense USB 시리얼 연결, RealSense 연결.
종료: Ctrl-C (녹화 중이면 마지막 에피소드 저장)
"""
import argparse
import glob
import logging
import os
import struct
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from pika_win.recorder import EpisodeRecorder, ArmSpec  # noqa: E402
from pika_win.gesture import GripperGestureDetector, calibrate_open_closed  # noqa: E402
from pika_win.sdk_logging import quiet_pika_sdk_info  # noqa: E402
from pika_win.viewer import make_viewer  # noqa: E402

log = logging.getLogger("collect")

MAX_ARMS = 2  # Vive 양팔


def _split(s):
    return [x.strip() for x in s.split(",")] if s else []


def _at(lst, i, default=None):
    return lst[i] if i < len(lst) and lst[i] not in ("", "None") else default


def _default_coms():
    if os.name == "nt":
        return "COM3,COM4"
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    if len(by_id) >= 2:
        return ",".join(by_id[:2])
    tty_usb = sorted(glob.glob("/dev/ttyUSB*"))
    if len(tty_usb) >= 2:
        return ",".join(tty_usb[:2])
    if len(by_id) == 1:
        return by_id[0] + ","
    if len(tty_usb) == 1:
        return tty_usb[0] + ","
    return "/dev/ttyUSB0,/dev/ttyUSB1"


def _default_rs_sns():
    value = os.environ.get("PIKA_RS_SNS", "")
    if value:
        return value
    return "260522277606," if os.name == "nt" else ""


def _looks_like_windows_com(port):
    return isinstance(port, str) and port.upper().startswith("COM")


def build_arms(a):
    """팔 번들 생성. config/arms.json 이 있으면 그것을 우선(좌/우 고정), 없으면 CLI 리스트."""
    # 1) config 파일이 있으면 source of truth (identify_arms.py 로 저장한 좌/우 고정 매핑)
    if a.config and os.path.exists(a.config):
        import json
        with open(a.config, encoding="utf-8") as f:
            cfg = json.load(f).get("arms", {})
        if cfg:
            log.info("[config] 팔 매핑 로드: %s", a.config)
            arms = []
            for name, d in cfg.items():
                com_port = d.get("com_port") or None
                if os.name != "nt" and _looks_like_windows_com(com_port):
                    log.warning("[config][%s] Linux에서 Windows COM 포트(%s)가 설정됨. "
                                "config/arms.json을 /dev/serial/by-id/...로 갱신하거나 --config ''로 CLI 값을 쓰세요.",
                                name, com_port)
                arms.append(ArmSpec(
                    name=name,
                    com_port=com_port,
                    realsense_sn=d.get("realsense_sn") or None,
                    tracker_sn=d.get("tracker_sn") or None,
                ))
            return arms[:MAX_ARMS]

    # 2) config 없으면 CLI 리스트(쉼표 구분). 활성 개수는 런타임 트래커 수가 결정.
    names = _split(a.arm_names) or ["right", "left"]
    coms, rss, tss = _split(a.coms), _split(a.rs_sns), _split(a.tracker_sns)
    arms = []
    for i in range(MAX_ARMS):
        arms.append(ArmSpec(
            name=_at(names, i, f"arm{i}"),
            com_port=_at(coms, i),
            realsense_sn=_at(rss, i),
            tracker_sn=_at(tss, i),
        ))
    return arms


def calibrate(rec, arm_idx, name, seconds=4.0, min_span=20.0, retries=3):
    attempts = max(1, int(retries))
    last_reason = "no samples"
    for attempt in range(1, attempts + 1):
        log.info("[calib][%s] %.0f초간 그리퍼를 '꽉 쥐었다 펴기' 2~3회 반복하세요... (%d/%d)",
                 name, seconds, attempt, attempts)
        samples, rest = [], None
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < seconds:
            v = rec.read_gripper_angle(arm_idx)
            if v is not None and v == v:
                samples.append(v)
                if rest is None:
                    rest = v
            time.sleep(0.02)
        if len(samples) < 5:
            last_reason = f"샘플 부족 n={len(samples)}"
            log.warning("[calib][%s] 실패: %s — Sense(COM) 연결 확인", name, last_reason)
            continue
        lo, hi = min(samples), max(samples)
        span = hi - lo
        if span < min_span:
            last_reason = f"움직임 폭 부족 span={span:.1f} < {min_span:.1f}"
            log.warning("[calib][%s] 실패: %s (range %.1f~%.1f). 실제로 끝까지 쥐었다 펴세요.",
                        name, last_reason, lo, hi)
            continue
        ov, cv = calibrate_open_closed(samples, rest=rest)
        log.info("[calib][%s] open~%.1f  closed~%.1f  (n=%d, range %.1f~%.1f, span=%.1f)",
                 name, ov, cv, len(samples), lo, hi, span)
        return ov, cv
    raise RuntimeError(f"[{name}] 그리퍼 캘리브레이션 실패 — {last_reason}")


class KeyboardToggle:
    """터미널에서 단일 키를 non-blocking으로 읽어 녹화 토글에 사용한다."""

    def __init__(self, key="b"):
        self.key = key.lower()
        self.enabled = True
        self._fd = None
        self._old_term = None
        self._termios = None
        self._msvcrt = None

    def start(self):
        if os.name == "nt":
            import msvcrt
            self._msvcrt = msvcrt
            return True
        if not sys.stdin.isatty():
            self.enabled = False
            return False
        import termios
        import tty
        self._termios = termios
        self._fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return True

    def close(self):
        if self._old_term is not None:
            self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old_term)
            self._old_term = None

    def poll_keys(self):
        if not self.enabled:
            return []
        if os.name == "nt":
            keys = []
            while self._msvcrt.kbhit():
                keys.append(self._msvcrt.getwch().lower())
            return keys
        import select
        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch:
                keys.append(ch.lower())
        return keys

    def pressed(self):
        return self.key in self.poll_keys()


class PedalToggle:
    """Linux evdev FootSwitch key events as a recording toggle source."""

    EV_KEY = 0x01
    KEY_PRESS = 1
    EVENT = struct.Struct("llHHi")

    def __init__(self, device="auto"):
        self.device_arg = device
        self.path = None
        self.fd = None
        self.enabled = False
        self.reason = None

    def _resolve_device(self):
        if self.device_arg in ("", "none", "off"):
            self.reason = "disabled"
            return None
        if self.device_arg != "auto":
            return self.device_arg
        candidates = sorted(glob.glob("/dev/input/by-id/*FootSwitch*event-kbd"))
        if candidates:
            return candidates[0]
        self.reason = "FootSwitch event-kbd device not found"
        return None

    def start(self):
        if os.name == "nt":
            self.reason = "raw pedal input is Linux-only"
            return False
        path = self._resolve_device()
        if not path:
            return False
        try:
            self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except PermissionError as e:
            self.reason = f"{path}: permission denied ({e})"
            return False
        except OSError as e:
            self.reason = f"{path}: {e}"
            return False
        self.path = path
        self.enabled = True
        return True

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        self.enabled = False

    def pressed(self):
        if not self.enabled:
            return False
        pressed = False
        while True:
            try:
                data = os.read(self.fd, self.EVENT.size * 32)
            except BlockingIOError:
                break
            except OSError as e:
                self.reason = str(e)
                self.close()
                break
            if not data:
                break
            usable = len(data) - (len(data) % self.EVENT.size)
            for off in range(0, usable, self.EVENT.size):
                _, _, ev_type, _, value = self.EVENT.unpack_from(data, off)
                if ev_type == self.EV_KEY and value == self.KEY_PRESS:
                    pressed = True
        return pressed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--out", default=os.path.join(REPO_ROOT, "data"))
    ap.add_argument("--window", type=float, default=1.5, help=argparse.SUPPRESS)
    ap.add_argument("--bimanual-toggle-window", type=float, default=0.75,
                    help=argparse.SUPPRESS)
    ap.add_argument("--key-toggle-cooldown", type=float, default=1.5,
                    help="b 키 반복 입력으로 인한 중복 토글 방지 시간(초)")
    ap.add_argument("--pedal-device", default="auto",
                    help="FootSwitch evdev 경로(auto=/dev/input/by-id/*FootSwitch*event-kbd)")
    ap.add_argument("--no-pedal", action="store_true",
                    help="FootSwitch raw input 토글 비활성화")
    ap.add_argument("--min-record-sec", type=float, default=1.0,
                    help="REC 시작 직후 이 시간 안의 정지 토글은 무시")
    ap.add_argument("--start-index", type=int, default=0)
    ap.add_argument("--no-realsense", action="store_true")
    ap.add_argument("--require-pose", action="store_true",
                    help="SteamVR/OpenVR pose와 active tracker pose가 유효하지 않으면 시작하지 않음")
    ap.add_argument("--require-all-trackers", action="store_true",
                    help="설정된 tracker-sns가 모두 보이지 않으면 시작하지 않음")
    ap.add_argument("--pose-valid-timeout", type=float, default=2.0,
                    help="active tracker pose 유효성 확인 timeout(초)")
    # ---- 팔 매핑 config (identify_arms.py 로 생성, 있으면 우선) ----
    ap.add_argument("--config", default=os.path.join(REPO_ROOT, "config", "arms.json"),
                    help="좌/우 팔 번들 config(JSON). 존재하면 CLI 하드웨어 인자보다 우선")
    # ---- 팔별 하드웨어(쉼표 구분, config 없을 때만 사용. [arm0=right, arm1=left]) ----
    ap.add_argument("--arm-names", default="right,left")
    ap.add_argument("--coms", default=_default_coms(), help="Sense serial ports")
    ap.add_argument("--rs-sns", default=_default_rs_sns(), help="RealSense 시리얼(빈칸=auto)")
    ap.add_argument("--tracker-sns", default="", help="Vive 트래커 시리얼(빈칸=순서배정). 좌/우 고정 시 둘 다 지정")
    # ---- 로깅/뷰어 ----
    ap.add_argument("--hb", type=float, default=1.0, help="REC 중 진행 로그 주기(초); 0=끔")
    ap.add_argument("--debug", action="store_true", help="모든 close/open 전이까지 로깅")
    ap.add_argument("--calib-min-span", type=float, default=20.0,
                    help="그리퍼 캘리브레이션 통과에 필요한 최소 angle 변동폭")
    ap.add_argument("--calib-retries", type=int, default=3,
                    help="그리퍼 캘리브레이션 재시도 횟수")
    ap.add_argument("--view", choices=["none", "web", "spawn"], default="none",
                    help="rerun 라이브 뷰어(뷰 전용): web=브라우저, spawn=네이티브 창, none=끔")
    ap.add_argument("--view-img-every", type=int, default=3, help="뷰어 카메라 로깅 간격(프레임)")
    ap.add_argument("--view-mem", default="2GB", help="뷰어 메모리 상한(초과 시 오래된 데이터 폐기)")
    a = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if a.debug else logging.INFO,
        format="%(asctime)s %(levelname)-5s %(name)s | %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stdout,
    )
    quiet_pika_sdk_info()

    # 이 실행(세션) 전용 출력 폴더: data/data_YYYYMMDD_HHMMSS (시작 시각, 초 단위)
    session_dir = os.path.join(a.out, "data_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(session_dir, exist_ok=True)
    file_handler = logging.FileHandler(os.path.join(session_dir, "collect.log"), encoding="utf-8")
    file_handler.setFormatter(logging.Formatter(
        "%(asctime)s %(levelname)-5s %(name)s | %(message)s",
        datefmt="%H:%M:%S",
    ))
    logging.getLogger().addHandler(file_handler)
    log.info("[out] 세션 폴더: %s", session_dir)
    log.info("[out] 로그 파일: %s", os.path.join(session_dir, "collect.log"))

    rec = EpisodeRecorder(out_dir=session_dir, arms=build_arms(a), record_hz=a.hz,
                          use_realsense=not a.no_realsense,
                          require_pose=a.require_pose,
                          require_all_trackers=a.require_all_trackers,
                          pose_valid_timeout=a.pose_valid_timeout)
    viewer = None
    try:
        rec.start()  # ← 트래커 개수로 단일/양팔 자동 결정

        names = rec.arm_names()
        n = rec.n_arms
        dets = []
        for ai, name in enumerate(names):
            ov, cv = calibrate(rec, ai, name, min_span=a.calib_min_span, retries=a.calib_retries)
            det = GripperGestureDetector(ov, cv, double_window=a.window)
            dets.append(det)
            log.info("[gesture][%s] enter_closed=%.1f enter_open=%.1f dir=%+.0f window=%.1fs min_gap=%.0fms",
                     name, det.enter_closed, det.enter_open, det.dir, det.double_window, det.min_pinch_gap * 1e3)
        viewer = make_viewer(a.view, memory_limit=a.view_mem, img_every=a.view_img_every,
                             session_dir=session_dir)
    except Exception:
        if viewer is not None:
            viewer.close()
        rec.stop()
        raise

    recording = False
    frames = []
    ep = a.start_index
    saved = 0
    rec_t0 = 0.0
    period = 1.0 / a.hz
    last_key_toggle = -1e9
    keyboard = KeyboardToggle("b")
    pedal = PedalToggle("none" if a.no_pedal else a.pedal_device)

    def reset_diag():
        return (
            [float("inf")] * n,
            [float("-inf")] * n,
            [0] * n,
            [float("nan")] * n,
        )

    # ---- REC 중 진행 로그 / 진단 누적(팔별, 직전 로그 이후) ----
    last_hb = time.perf_counter()
    a_min, a_max, nan_run, ang_last = reset_diag()

    if not keyboard.start():
        log.warning("[keyboard] stdin이 TTY가 아니라 b 키 제어를 사용할 수 없습니다. Ctrl-C 종료만 가능합니다.")
    if pedal.start():
        log.info("[pedal] FootSwitch 토글 입력: %s", pedal.path)
    elif not a.no_pedal:
        log.warning("[pedal] FootSwitch raw input 비활성: %s", pedal.reason)
        log.warning("[pedal] 권한 문제면 `sudo usermod -aG input $USER` 후 재로그인하거나 `newgrp input`을 실행하세요.")
    toggle_desc = "b 키/FootSwitch" if pedal.enabled else "b 키"
    log.info("준비 완료 ✅  [%s] 모드. %s로 시작/정지. Ctrl-C 종료.",
             "양팔" if n > 1 else "한팔", toggle_desc)
    log.info("[keyboard] terminal focus 상태에서 'b'를 누르면 REC 시작/정지")
    if pedal.enabled:
        log.info("[pedal] FootSwitch를 밟아도 REC 시작/정지")
    log.info("○ IDLE 대기   next=episode_%03d   saved=%d   (%s = REC 시작)",
             ep, saved, toggle_desc)
    try:
        while True:
            tick = time.perf_counter()
            fr = rec.read_frame()
            now = fr["ts"]

            # ---- b 키 입력으로 REC 토글. 그리퍼 움직임은 녹화 제어에 사용하지 않는다. ----
            toggled, toggled_by = False, None
            toggle_sources = []
            if keyboard.pressed():
                toggle_sources.append("keyboard:b")
            if pedal.pressed():
                toggle_sources.append("pedal")
            if toggle_sources:
                if tick - last_key_toggle >= a.key_toggle_cooldown:
                    toggled, toggled_by = True, "+".join(toggle_sources)
                    last_key_toggle = tick
                else:
                    log.info("↳ 토글 반복 입력 무시   source=%s elapsed=%.2fs < %.2fs",
                             "+".join(toggle_sources), tick - last_key_toggle, a.key_toggle_cooldown)

            # ---- 팔별 그리퍼 상태 갱신 + 진단(제어용 아님) ----
            for ai in range(n):
                angle = fr["arms"][ai]["gripper"][0]
                ang_last[ai] = angle
                dets[ai].update(angle, tick)
                if angle != angle:
                    nan_run[ai] += 1
                else:
                    a_min[ai] = min(a_min[ai], angle)
                    a_max[ai] = max(a_max[ai], angle)
                ev = dets[ai].last_event
                if a.debug and ev in ("close", "open"):
                    log.debug("  · [%s] %-5s angle=%.1f state=%s", names[ai], ev, angle, dets[ai].state)

            if toggled:
                if not recording:
                    recording, frames, rec_t0 = True, [], now
                    last_hb = tick
                    a_min, a_max, nan_run, ang_last = reset_diag()
                    log.info("● REC 시작   episode_%03d   by=%s   min_record=%.1fs",
                             ep, toggled_by, a.min_record_sec)
                else:
                    rec_elapsed = now - rec_t0
                    if rec_elapsed < a.min_record_sec:
                        log.info("↳ REC 정지 무시   elapsed=%.2fs < %.2fs   by=%s",
                                 rec_elapsed, a.min_record_sec, toggled_by)
                    else:
                        recording = False
                        path = os.path.join(session_dir, f"episode_{ep:03d}.hdf5")
                        rec.write_episode(path, frames)
                        log.info("■ REC 저장   %s   frames=%d   duration=%.1fs   by=%s",
                                 path, len(frames), rec_elapsed, toggled_by)
                        ep += 1
                        saved += 1
                        frames = []
                        last_hb = tick
                        a_min, a_max, nan_run, ang_last = reset_diag()
                        log.info("○ IDLE 대기   next=episode_%03d   saved=%d   (%s = REC 시작)",
                                 ep, saved, toggle_desc)

            if recording:
                frames.append(fr)

            # ---- 라이브 뷰어(뷰 전용, 팔별, 비활성 시 no-op) ----
            if viewer.enabled:
                per_arm = [(names[ai], fr["arms"][ai]["gripper"][0], dets[ai].is_closed) for ai in range(n)]
                viewer.state(recording, ep, saved, (now - rec_t0) if recording else 0.0, per_arm)
                for ai in range(n):
                    viewer.pose(names[ai], fr["arms"][ai]["pose"], recording)
                    viewer.images(names[ai], fr["arms"][ai])

            # ---- REC 진행 로그: 팔별 각도 변동폭 vs 임계 비교(채터/오트리거 진단) ----
            if recording and a.hb > 0 and tick - last_hb >= a.hb:
                rec_elapsed = now - rec_t0
                log.info("● REC 진행   episode_%03d   t=%.1fs   frames=%d   saved=%d",
                         ep, rec_elapsed, len(frames), saved)
                for ai in range(n):
                    span = (a_max[ai] - a_min[ai]) if a_max[ai] >= a_min[ai] else float("nan")
                    log.info("  [%s] angle=%.1f span[%.1f~%.1f]=%.1f "
                             "vs close=%.1f/open=%.1f | nan=%d state=%s",
                             names[ai],
                             ang_last[ai] if ang_last[ai] == ang_last[ai] else float("nan"),
                             a_min[ai] if a_min[ai] != float("inf") else float("nan"),
                             a_max[ai] if a_max[ai] != float("-inf") else float("nan"),
                             span, dets[ai].enter_closed, dets[ai].enter_open,
                             nan_run[ai], dets[ai].state)
                last_hb = tick
                a_min, a_max, nan_run, ang_last = reset_diag()

            rem = period - (time.perf_counter() - tick)
            if rem > 0:
                time.sleep(rem)
    except KeyboardInterrupt:
        if recording and frames:
            path = os.path.join(session_dir, f"episode_{ep:03d}.hdf5")
            rec.write_episode(path, frames)
            log.info("■ (중단) 저장 %s   (%d frames)", path, len(frames))
    finally:
        log.info("[summary] saved=%d  next=episode_%03d  arms=%s", saved, ep, names)
        for ai in range(n):
            d = dets[ai]
            log.info("[summary][%s] close=%d open=%d nan=%d",
                     names[ai], d.n_close, d.n_open, d.nan_count)
        pedal.close()
        keyboard.close()
        viewer.close()
        rec.stop()


if __name__ == "__main__":
    main()
    # 시리얼/포즈 백그라운드 스레드가 프로세스를 붙잡는 좀비화 방지(하드웨어 정리는 finally에서 끝남).
    os._exit(0)
