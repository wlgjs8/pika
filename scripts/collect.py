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
import atexit
import gc
import glob
import logging
import os
import queue
import signal
import struct
import sys
import threading
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from pika_win.recorder import EpisodeRecorder, ArmSpec  # noqa: E402
from pika_win.episode_writer import EpisodeWriterProcess  # noqa: E402
from pika_win.gesture import GripperGestureDetector, calibrate_open_closed  # noqa: E402
from pika_win.sdk_logging import quiet_pika_sdk_info  # noqa: E402
from pika_win.viewer import make_viewer  # noqa: E402

log = logging.getLogger("collect")

MAX_ARMS = 2  # Vive 양팔


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def _ignore_shutdown_signals():
    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            previous[sig] = signal.getsignal(sig)
            signal.signal(sig, signal.SIG_IGN)
        except (OSError, ValueError):
            pass
    return previous


class CollectRunLock:
    """collect.py 동시 실행 방지용 PID 락."""
    def __init__(self, path):
        self.path = path
        self.fd = None

    def acquire(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        while True:
            try:
                self.fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
                break
            except FileExistsError:
                pid, cmd = self._read_existing()
                if pid and self._pid_alive(pid):
                    detail = f"pid={pid}"
                    if cmd:
                        detail += f" cmd={cmd}"
                    raise RuntimeError(
                        f"[lock] collect.py가 이미 실행 중입니다 ({detail}). "
                        "기존 수집을 종료한 뒤 다시 실행하세요."
                    )
                try:
                    os.unlink(self.path)
                except FileNotFoundError:
                    pass
        body = f"pid={os.getpid()}\ncmd={' '.join(sys.argv)}\n"
        os.write(self.fd, body.encode("utf-8", errors="replace"))
        atexit.register(self.release)
        return self

    def release(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass

    def _read_existing(self):
        try:
            with open(self.path, encoding="utf-8") as f:
                lines = f.read().splitlines()
        except OSError:
            return None, ""
        values = {}
        for line in lines:
            if "=" in line:
                k, v = line.split("=", 1)
                values[k.strip()] = v.strip()
        try:
            pid = int(values.get("pid", ""))
        except ValueError:
            pid = None
        return pid, values.get("cmd", "")

    @staticmethod
    def _pid_alive(pid):
        if pid == os.getpid():
            return True
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True


class EpisodeSaveWorker:
    """에피소드 저장을 단일 worker 스레드 + bounded queue 로 처리.

    worker 스레드는 compact(점진 인코딩된) 프레임에서 payload 를 만들고(recorder.build_payload),
    실제 HDF5 디스크 쓰기는 별도 프로세스(writer)에 위임한 뒤 완료까지 대기한다. 대기 동안
    GIL 을 놓으므로 수집 루프(메인 스레드)가 30Hz 로 계속 돈다(저장-캡처 GIL/디스크 격리).
    """
    def __init__(self, recorder, logger, writer, max_pending=2, status_interval=1.0):
        self.recorder = recorder
        self.writer = writer
        self.log = logger
        self.max_pending = int(max_pending)
        self.status_interval = float(status_interval)
        self._lock = threading.Lock()
        self._queue = queue.Queue(maxsize=self.max_pending)
        self._idle = threading.Event()
        self._idle.set()
        self._active = None
        self._errors = []
        self._done_count = 0
        self._last_status = 0.0
        self._thread = threading.Thread(target=self._run, name="EpisodeSaveWorker", daemon=False)
        self._thread.start()

    def pending_count(self):
        with self._lock:
            active = 1 if self._active is not None else 0
        return active + self._queue.qsize()

    def can_start_recording(self):
        self._raise_if_error()
        return self.pending_count() < self.max_pending

    def status(self):
        with self._lock:
            active = self._active
            done = self._done_count
            failed = len(self._errors)
        queued = list(self._queue.queue)
        pending = (1 if active is not None else 0) + len(queued)
        if active is None:
            active_text = "-"
        else:
            start_perf = active.get("save_start_perf", active["enqueue_perf"])
            elapsed = time.perf_counter() - start_perf
            active_text = f"episode_{active['ep']:03d} frames={active['frames']} elapsed={elapsed:.1f}s"
        queued_text = ",".join(f"episode_{item['ep']:03d}" for item in queued)
        return f"pending={pending} active={active_text} queued=[{queued_text}] done={done} failed={failed}"

    def log_status(self, force=False):
        now = time.perf_counter()
        if force or now - self._last_status >= self.status_interval:
            self._last_status = now
            self.log.info("[saveq] %s", self.status())

    def _raise_if_error(self):
        if self._errors:
            err = self._errors.pop(0)
            raise RuntimeError(f"[saveq] background save failed: {err}") from err

    def enqueue(self, ep, path, frames, block=True, reason="enqueue"):
        item = {
            "ep": ep,
            "path": path,
            "frames": len(frames),
            "frames_obj": frames,
            "enqueue_perf": time.perf_counter(),
        }
        while True:
            if self.pending_count() >= self.max_pending:
                if not block:
                    raise RuntimeError(f"[saveq] queue full: pending={self.pending_count()}")
                self.log.info("[saveq] enqueue 대기   reason=%s   %s", reason, self.status())
                time.sleep(self.status_interval)
                self._raise_if_error()
                continue
            try:
                self._queue.put(item, block=False)
                self._idle.clear()
                self.log.info("[saveq] enqueue ep=%03d frames=%d pending=%d path=%s",
                              ep, item["frames"], self.pending_count(), path)
                return
            except queue.Full:
                # Worker가 아직 active로 가져가지 않은 queued item 수가 limit에 닿은 경우.
                if not block:
                    raise RuntimeError(f"[saveq] queue full: pending={self.pending_count()}")
                self.log.info("[saveq] enqueue 대기   reason=%s   %s", reason, self.status())
                time.sleep(self.status_interval)
                self._raise_if_error()

    def _run(self):
        while True:
            item = self._queue.get()
            if item is None:
                self._queue.task_done()
                return
            with self._lock:
                self._active = item
            try:
                item["save_start_perf"] = time.perf_counter()
                # compact 프레임 → payload(인코딩 Future resolve) 후 즉시 원본 참조 해제,
                # 디스크 쓰기는 writer 프로세스에 위임(완료까지 블록, 대기 중 GIL 해제).
                payload = self.recorder.build_payload(item["frames_obj"])
                item["frames_obj"] = None
                self.writer.write(item["path"], payload)
                payload = None
                save_elapsed = time.perf_counter() - item["save_start_perf"]
                queue_elapsed = item["save_start_perf"] - item["enqueue_perf"]
                fps = item["frames"] / max(save_elapsed, 1e-6)
                size_mb = os.path.getsize(item["path"]) / (1024 * 1024)
                self.log.info("[saveq] 완료   ep=%03d frames=%d save_elapsed=%.1fs "
                              "save_fps=%.1f queue_wait=%.1fs size=%.1fMiB pending=%d path=%s",
                              item["ep"], item["frames"], save_elapsed, fps,
                              queue_elapsed, size_mb, self.pending_count(), item["path"])
                with self._lock:
                    self._done_count += 1
            except BaseException as exc:  # noqa: BLE001  background error 전달
                with self._lock:
                    self._errors.append(exc)
                self.log.exception("[saveq] 실패   ep=%03d path=%s", item["ep"], item["path"])
            finally:
                item.pop("frames_obj", None)
                with self._lock:
                    self._active = None
                self._queue.task_done()
                if self.pending_count() == 0:
                    self._idle.set()

    def wait_idle(self, reason):
        self.log.info("[saveq] 대기 시작   reason=%s   %s", reason, self.status())
        while self.pending_count() > 0:
            self.log_status(force=True)
            self._idle.wait(timeout=self.status_interval)
        self.log.info("[saveq] 대기 종료   reason=%s   pending=0", reason)
        self._raise_if_error()

    def close(self):
        self.wait_idle("save worker close")
        self._queue.put(None)
        self._thread.join(timeout=5.0)


def append_timing_gap_record(record_path, episode_idx, episode_path, frames, duration_s, gap_events):
    if not gap_events:
        return
    exists = os.path.exists(record_path)
    record_dir = os.path.dirname(record_path)
    if record_dir:
        os.makedirs(record_dir, exist_ok=True)
        rel_episode_path = os.path.relpath(episode_path, record_dir)
    else:
        rel_episode_path = episode_path
    max_gap = max(event["dt"] for event in gap_events)
    gap_detail = ",".join(
        f"{event['frame_idx']}:{event['dt']:.3f}s" for event in gap_events
    )
    with open(record_path, "a", encoding="utf-8") as f:
        if not exists:
            f.write("# Episodes with timestamp frame gaps detected during collection.\n")
            f.write("# frame_idx is the index of the frame after the gap.\n")
            f.write("episode\tpath\tframes\tduration_s\tgap_count\tmax_gap_s\tgaps\n")
        f.write(
            f"episode_{episode_idx:03d}\t{rel_episode_path}\t{len(frames)}\t"
            f"{duration_s:.3f}\t{len(gap_events)}\t{max_gap:.3f}\t{gap_detail}\n"
        )


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
                    fisheye_dev=d.get("fisheye_dev") or None,
                ))
            # config 가 fisheye_dev 를 안 줘도 CLI --fisheye-devs 로 override 가능
            fde = _split(a.fisheye_devs)
            if fde:
                for i, arm in enumerate(arms):
                    arm.fisheye_dev = _at(fde, i, arm.fisheye_dev)
            return arms[:MAX_ARMS]

    # 2) config 없으면 CLI 리스트(쉼표 구분). 활성 개수는 런타임 트래커 수가 결정.
    names = _split(a.arm_names) or ["right", "left"]
    coms, rss, tss = _split(a.coms), _split(a.rs_sns), _split(a.tracker_sns)
    fde = _split(a.fisheye_devs)
    arms = []
    for i in range(MAX_ARMS):
        arms.append(ArmSpec(
            name=_at(names, i, f"arm{i}"),
            com_port=_at(coms, i),
            realsense_sn=_at(rss, i),
            tracker_sn=_at(tss, i),
            fisheye_dev=_at(fde, i),
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
            try:
                self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old_term)
            except OSError as e:
                log.warning("[keyboard] terminal restore failed: %s", e)
            finally:
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
    ap.add_argument("--arm-bolt-colors", default="right=black,left=gray",
                    help="이번 세션에서 각 팔이 집는 볼트 색 (예: 'right=black,left=gray'=normal, "
                         "'right=gray,left=black'=swap). 매 에피소드 HDF5 attr `arm_bolt_colors`로 기록 → "
                         "변환기가 색-grounded 프롬프트 배정. 박스는 색-매칭(coordinated)이라 별도 표기 불필요. "
                         "미지정/레거시 데이터는 normal로 간주.")
    ap.add_argument("--no-realsense", default=False, action="store_true")
    ap.add_argument("--no-fisheye", default=False, action="store_true",
                    help="그리퍼 어안 카메라 수집 비활성화")
    ap.add_argument("--fisheye-devs", default="",
                    help="팔별 어안 디바이스(쉼표; /dev/videoN 또는 인덱스). "
                         "빈칸=RealSense 와 같은 USB 허브에서 자동 매핑")
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
    ap.add_argument("--skip-gripper-calib", action="store_true",
                    help="skip startup gripper open/close calibration (tracker-only captures; nominal open/closed)")
    ap.add_argument("--view", choices=["none", "web", "spawn"], default="none",
                    help="rerun 라이브 뷰어(뷰 전용): web=브라우저, spawn=네이티브 창, none=끔")
    ap.add_argument("--view-img-every", type=int, default=3, help="뷰어 카메라 로깅 간격(프레임)")
    ap.add_argument("--view-mem", default="2GB", help="뷰어 메모리 상한(초과 시 오래된 데이터 폐기)")
    ap.add_argument("--save-max-pending", type=int, default=2,
                    help="저장 worker의 최대 pending 에피소드 수(active 포함). 2=저장 중 1개 + 대기 1개")
    ap.add_argument("--encode-workers", type=int, default=0,
                    help="에피소드 저장 시 PNG 인코딩 worker 수. 0=auto, 1~2는 수집 루프 CPU 여유 확보에 유리")
    ap.add_argument("--png-compression", type=int, default=1,
                    help="color/fisheye PNG compression level(0~9). 0도 무손실이며 가장 빠르고 파일이 큼")
    ap.add_argument("--png-depth-compression", type=int, default=-1,
                    help="depth PNG compression level(0~9). 음수=OpenCV 기본값")
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    logging.basicConfig(
        level=logging.DEBUG if a.debug else logging.INFO,
        format="%(asctime)s %(levelname)-5s %(name)s | %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stdout,
    )
    quiet_pika_sdk_info()

    lock_path = os.path.join(a.out, ".collect.lock")
    try:
        run_lock = CollectRunLock(lock_path).acquire()
    except RuntimeError as e:
        log.error("%s", e)
        sys.exit(2)

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

    arms = build_arms(a)
    viewer = None
    try:
        viewer = make_viewer(a.view, memory_limit=a.view_mem, img_every=a.view_img_every,
                             session_dir=session_dir,
                             arm_names=[arm.name for arm in arms])
    except Exception:
        log.exception("[viewer] 초기화 실패")
        raise

    rec = EpisodeRecorder(out_dir=session_dir, arms=arms, record_hz=a.hz,
                          use_realsense=not a.no_realsense,
                          use_fisheye=not a.no_fisheye,
                          require_pose=a.require_pose,
                          require_all_trackers=a.require_all_trackers,
                          pose_valid_timeout=a.pose_valid_timeout,
                          png_compression=a.png_compression,
                          png_depth_compression=(
                              None if a.png_depth_compression < 0 else a.png_depth_compression),
                          encode_workers=(None if a.encode_workers <= 0 else a.encode_workers),
                          arm_bolt_colors=a.arm_bolt_colors)
    log.info("[collect] arm_bolt_colors=%s (per-session bolt assignment -> HDF5 attr)", a.arm_bolt_colors)
    log.info("[save] max_pending=%d encode_workers=%d png_compression=%d depth_png=%s",
             max(1, a.save_max_pending), rec.encode_workers, rec.png_compression,
             "opencv-default" if rec.png_depth_compression is None else rec.png_depth_compression)
    # 하드웨어 스레드(RealSense/pose/fisheye)가 시작되기 전(메인 스레드 단독) 시점에
    # 디스크 쓰기 전담 writer 프로세스를 fork 로 생성 → fork-after-threads 위험 회피.
    writer = EpisodeWriterProcess()
    try:
        rec.start()  # ← 트래커 개수로 단일/양팔 자동 결정

        names = rec.arm_names()
        n = rec.n_arms
        dets = []
        for ai, name in enumerate(names):
            if a.skip_gripper_calib:
                _rest = rec.read_gripper_angle(ai)
                _rest = _rest if (_rest is not None and _rest == _rest) else 0.0
                ov, cv = _rest + 30.0, _rest - 30.0
                log.info("[calib][%s] SKIPPED (--skip-gripper-calib): open~%.1f closed~%.1f (nominal)", name, ov, cv)
            else:
                ov, cv = calibrate(rec, ai, name, min_span=a.calib_min_span, retries=a.calib_retries)
            det = GripperGestureDetector(ov, cv, double_window=a.window)
            dets.append(det)
            log.info("[gesture][%s] enter_closed=%.1f enter_open=%.1f dir=%+.0f window=%.1fs min_gap=%.0fms",
                     name, det.enter_closed, det.enter_open, det.dir, det.double_window, det.min_pinch_gap * 1e3)
    except BaseException:
        if viewer is not None:
            viewer.close()
        rec.stop()
        writer.close()
        raise

    recording = False
    frames = []
    ep = a.start_index
    saved = 0
    saveq = EpisodeSaveWorker(rec, log, writer,
                              max_pending=max(1, a.save_max_pending), status_interval=1.0)
    rec_t0 = 0.0
    period = 1.0 / a.hz
    gap_warn_sec = max(period * 2.5, 0.10)
    timing_gap_path = os.path.join(session_dir, "timing_gaps.txt")
    gap_events = []
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
    log.info("[timing] frame gap 기록 파일: %s", timing_gap_path)
    log.info("○ IDLE 대기   next=episode_%03d   saved=%d   (%s = REC 시작)",
             ep, saved, toggle_desc)

    # ---- 캡처 루프 stall 진단/완화 ----------------------------------------
    # read_frame 의 모든 장치 읽기는 논블로킹(백그라운드 스레드+최신값 버퍼)이므로,
    # 멀티초 gap 은 캡처 스레드가 '안 돌아간' 것 = (a) 대용량 객체 churn 의 GC 일시정지,
    # (b) swap-in 페이지폴트 가 유력. 둘을 구분하기 위해 gap 시 read_frame 소요/직전 iter
    # 소요/VmSwap 변화를 함께 로깅하고, 녹화 중에는 GC 를 끄고 IDLE 에서만 수거한다.
    def _vmswap_kb():
        try:
            with open("/proc/self/status", encoding="utf-8") as f:
                for line in f:
                    if line.startswith("VmSwap:"):
                        return int(line.split()[1])
        except OSError:
            return -1
        return 0

    _gc_pause = {"t0": 0.0}

    def _gc_cb(phase, info):
        if phase == "start":
            _gc_pause["t0"] = time.perf_counter()
        elif phase == "stop":
            dt = time.perf_counter() - _gc_pause["t0"]
            if dt > 0.2:
                log.warning("[gc] pause %.0fms gen=%s collected=%s",
                            dt * 1e3, info.get("generation"), info.get("collected"))

    gc.callbacks.append(_gc_cb)
    gc.disable()  # 녹화 중 GC 일시정지 방지(수거는 IDLE 전환 시 명시적으로)
    prev_read_ms = 0.0
    prev_total_ms = 0.0
    prev_swap = _vmswap_kb()
    try:
        while True:
            tick = time.perf_counter()
            fr = rec.read_frame()
            read_ms = (time.perf_counter() - tick) * 1e3
            now = fr["ts"]
            started_this_tick = False

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
                    if saveq.can_start_recording():
                        recording, frames, rec_t0, gap_events = True, [], 0.0, []
                        started_this_tick = True
                        last_hb = tick
                        a_min, a_max, nan_run, ang_last = reset_diag()
                        log.info("● REC 시작   episode_%03d   by=%s   min_record=%.1fs   saveq=%s",
                                 ep, toggled_by, a.min_record_sec, saveq.status())
                    else:
                        log.info("↳ REC 시작 무시   by=%s   save queue full   %s",
                                 toggled_by, saveq.status())
                else:
                    rec_elapsed = (now - rec_t0) if rec_t0 > 0.0 else 0.0
                    if rec_elapsed < a.min_record_sec:
                        log.info("↳ REC 정지 무시   elapsed=%.2fs < %.2fs   by=%s",
                                 rec_elapsed, a.min_record_sec, toggled_by)
                    else:
                        recording = False
                        path = os.path.join(session_dir, f"episode_{ep:03d}.hdf5")
                        append_timing_gap_record(timing_gap_path, ep, path, frames, rec_elapsed, gap_events)
                        if gap_events:
                            log.warning("[timing] gap episode recorded   episode_%03d   count=%d   file=%s",
                                        ep, len(gap_events), timing_gap_path)
                        saveq.enqueue(ep, path, frames, block=True, reason="REC 정지 저장")
                        log.info("■ REC 저장 enqueue   %s   frames=%d   duration=%.1fs   by=%s",
                                 path, len(frames), rec_elapsed, toggled_by)
                        ep += 1
                        saved += 1
                        frames = []
                        gap_events = []
                        last_hb = tick
                        a_min, a_max, nan_run, ang_last = reset_diag()
                        # 녹화 중 비활성화한 GC 를 IDLE 진입 시 명시 수거(다음 녹화 전 청소).
                        gc.collect()
                        prev_swap = _vmswap_kb()
                        log.info("○ IDLE 대기   next=episode_%03d   saved=%d   (%s = REC 시작)",
                                 ep, saved, toggle_desc)

            if recording:
                if started_this_tick:
                    pass
                else:
                    if not frames:
                        rec_t0 = now
                    else:
                        dt = now - frames[-1]["ts"]
                        if dt > gap_warn_sec:
                            gap_events.append({
                                "frame_idx": len(frames),
                                "dt": dt,
                                "elapsed": (now - rec_t0) if rec_t0 > 0.0 else 0.0,
                            })
                            cur_swap = _vmswap_kb()
                            log.warning("[timing] frame gap   episode_%03d   dt=%.3fs   "
                                        "frames=%d   target_dt=%.3fs   "
                                        "| 직전iter read=%.0fms total=%.0fms   swap=%dKB(Δ%+d)   "
                                        "saveq=%s",
                                        ep, dt, len(frames), period,
                                        prev_read_ms, prev_total_ms,
                                        cur_swap, cur_swap - prev_swap, saveq.status())
                    # 캡처 즉시 이미지 PNG 인코딩(백그라운드 풀)하여 compact 프레임만 보관.
                    # 원본 raw 이미지는 이 tick 의 fr 와 함께 다음 tick 에 해제됨.
                    frames.append(rec.encode_frame(fr))
            else:
                saveq.log_status()

            # ---- 라이브 뷰어(뷰 전용, 팔별, 비활성 시 no-op) ----
            if viewer.enabled:
                per_arm = [(names[ai], fr["arms"][ai]["gripper"][0], dets[ai].is_closed) for ai in range(n)]
                viewer_elapsed = (now - rec_t0) if recording and rec_t0 > 0.0 else 0.0
                viewer.state(recording, ep, saved, viewer_elapsed, per_arm)
                for ai in range(n):
                    viewer.pose(names[ai], fr["arms"][ai]["pose"], recording)
                    viewer.images(names[ai], fr["arms"][ai])

            # ---- REC 진행 로그: 팔별 각도 변동폭 vs 임계 비교(채터/오트리거 진단) ----
            if recording and a.hb > 0 and tick - last_hb >= a.hb:
                rec_elapsed = (now - rec_t0) if rec_t0 > 0.0 else 0.0
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
            # gap 진단용: 직전 iteration 의 read_frame/전체 소요와 swap 점유 추적
            prev_read_ms = read_ms
            prev_total_ms = (time.perf_counter() - tick) * 1e3
            prev_swap = _vmswap_kb()
    except KeyboardInterrupt:
        log.info("[shutdown] Ctrl-C 감지: 저장 큐 flush 후 종료합니다.")
        if recording and frames:
            recording = False
            path = os.path.join(session_dir, f"episode_{ep:03d}.hdf5")
            rec_elapsed = (frames[-1]["ts"] - rec_t0) if rec_t0 > 0.0 else 0.0
            append_timing_gap_record(timing_gap_path, ep, path, frames, rec_elapsed, gap_events)
            if gap_events:
                log.warning("[timing] gap episode recorded   episode_%03d   count=%d   file=%s",
                            ep, len(gap_events), timing_gap_path)
            saveq.enqueue(ep, path, frames, block=True, reason="중단 저장")
            log.info("■ (중단) 저장 enqueue %s   (%d frames)", path, len(frames))
            ep += 1
            saved += 1
    finally:
        _ignore_shutdown_signals()
        try:
            if saveq.pending_count() > 0:
                log.info("[shutdown] 저장 대기 중: %s", saveq.status())
            saveq.close()
            log.info("[shutdown] 저장 큐 flush 완료")
        except Exception:
            log.exception("[saveq] 종료 전 flush 실패")
        try:
            writer.close()
            log.info("[shutdown] writer 프로세스 종료")
        except Exception:
            log.exception("[writer] 종료 실패")
        log.info("[summary] saved=%d  next=episode_%03d  arms=%s", saved, ep, names)
        for ai in range(n):
            d = dets[ai]
            log.info("[summary][%s] close=%d open=%d nan=%d",
                     names[ai], d.n_close, d.n_open, d.nan_count)
        pedal.close()
        keyboard.close()
        viewer.close()
        rec.stop()
        run_lock.release()


if __name__ == "__main__":
    main()
    # 시리얼/포즈 백그라운드 스레드가 프로세스를 붙잡는 좀비화 방지(하드웨어 정리는 finally에서 끝남).
    os._exit(0)
