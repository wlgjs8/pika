"""libsurvive(pysurvive) 기반 Vive Tracker 포즈 리더 — GUI/SteamVR 불필요.

pose_steamvr.PoseSteamVR 와 **동일한 공개 인터페이스**를 제공한다
(connect / get_pose / get_devices / effective_hz / disconnect, 포즈 dict 스키마 동일).
따라서 recorder / umi_teleop_publish 는 백엔드 객체만 바꿔 끼우면 된다.

SteamVR 경로 대비 차이:
  - **좌표계**: libsurvive 는 자체 scene solve 로 월드 원점을 잡는다. SteamVR 의
    TrackingUniverseStanding 과 일치하지 않는다. `config_path` 로 라이트하우스 해를
    저장/재사용해야 실행 간 프레임이 고정된다(기본값: config/libsurvive_config.json).
    프레임이 바뀌면 기존 SteamVR 수집분/캘리브레이션과 직접 비교할 수 없다.
  - **팁 변환**: PIKA SDK 의 R_corr 는 원래 libsurvive raw frame 기준으로 정의된
    값이다(pika/tracker/vive_tracker.py). 즉 이 백엔드에서는 pose_steamvr 이 달고
    있던 "두 body frame 이 같다"는 가정이 필요 없다.
  - **tracking_result**: libsurvive 에는 OpenVR eTrackingResult 대응물이 없다.
    다운스트림 호환을 위해 `200`(Running_OK) / `201`(Running_OutOfRange) 두 값을
    **합성**한다 — 마지막 디바이스 갱신이 stale_timeout 이내면 200, 아니면 201.
    드라이버가 보고한 값이 아니므로 에피소드 attrs 의 pose_backend 로 구분할 것.
  - 정지한 트래커도 libsurvive 는 계속 포즈를 갱신하므로 "갱신 없음 = 추적 손실"이
    실제 신호가 된다(SteamVR 은 캐시/예측 포즈를 반환해 이 구분이 안 됐다).

기기 키는 **시리얼 번호**(`LHR-...`)다. libsurvive 의 object name 은 코드네임
(`T20` 등)이라 config/arms.json 의 tracker_sn 과 맞지 않는다.
"""
import ctypes
import logging
import os
import sys
import threading
import time


def _bootstrap_pysurvive():
    """pysurvive 임포트 경로 확보 — libsurvive 는 pip 패키지가 아니라 소스 빌드다.

    pysurvive 는 순수 ctypes 바인딩이라 빌드 산출물(libsurvive.so)만 찾으면 된다.
    CustomLibraryLoader 가 패키지 디렉터리를 탐색하므로 setup_libsurvive.sh 가
    거기에 .so 심링크를 만들어 둔다. 경로는 LIBSURVIVE_PATH 로 재정의 가능.
    """
    root = os.environ.get(
        "LIBSURVIVE_PATH",
        os.path.join(os.path.expanduser("~"), "workspace", "libsurvive"))
    bindings = os.path.join(root, "bindings", "python")
    if os.path.isdir(bindings) and bindings not in sys.path:
        sys.path.append(bindings)


_bootstrap_pysurvive()

import pysurvive  # noqa: E402

from .pose_math import (  # noqa: F401
    TIP_ROTATION_QUAT,
    TIP_TRANSLATION,
    SampleSeqTracker,
    apply_tip_transform,
    quat_mul,
    quat_rotate_vec,
)

log = logging.getLogger("pika.pose_survive")

# 합성 tracking_result — pose_steamvr.TRACKING_RESULT_NAMES 와 같은 코드 체계.
TRACKING_OK = 200
TRACKING_OUT_OF_RANGE = 201

# survive_simple_object_get_type() 열거값 중 트래커/피추적 오브젝트.
# LIGHTHOUSE(=1) 는 베이스스테이션이므로 제외해야 한다.
_OBJECT_TYPES = (
    pysurvive.SurviveSimpleObject_OBJECT,
    pysurvive.SurviveSimpleObject_HMD,
)

_DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "libsurvive_config.json")


class PoseSurvive:
    """libsurvive 포즈 리더. PoseSteamVR 와 동일 인터페이스.

    Args:
        target_hz: 갱신 이벤트가 없을 때의 유휴 폴링 상한(이벤트 구동이라 SteamVR
            처럼 고정레이트 폴링이 아니다).
        apply_gripper_offset: True 면 PIKA SDK 공식 트래커→그리퍼 팁 변환 적용.
        config_path: 라이트하우스 해 저장 경로. 실행 간 월드 프레임 고정을 위해
            반드시 같은 파일을 재사용할 것. None 이면 libsurvive 기본(cwd) 사용.
        stale_timeout: 이 시간(초) 동안 디바이스 갱신이 없으면 valid=False,
            tracking_result=201 로 본다.
        extra_args: pysurvive 에 그대로 전달할 추가 인자 리스트.
    """

    def __init__(self, target_hz=250.0, apply_gripper_offset=False,
                 config_path=_DEFAULT_CONFIG, stale_timeout=0.25,
                 force_calibrate=False, warmup_sec=10.0, warmup_expect=None,
                 extra_args=None):
        self.target_hz = float(target_hz)
        self.apply_gripper_offset = bool(apply_gripper_offset)
        self.config_path = config_path
        self.stale_timeout = float(stale_timeout)
        self.force_calibrate = bool(force_calibrate)
        self.warmup_sec = float(warmup_sec)
        # 기대 트래커: 시리얼 목록 또는 개수. 주어지면 전부 붙을 때까지 기다린다.
        self.warmup_expect = list(warmup_expect) if warmup_expect else None
        self.extra_args = list(extra_args) if extra_args else []
        self.ctx = None
        self._latest = {}            # serial -> pose dict (valid/tracking_result 제외)
        self._lock = threading.Lock()
        self._thread = None
        self._running = False
        self._eff_hz = 0.0
        # 값 변화 기반 샘플 시퀀스 — pose_steamvr 과 동일 구현 공유.
        self._seq = SampleSeqTracker()
        # SimpleObject 포인터 -> (serial, is_tracked_object) 캐시.
        # 시리얼/타입 조회는 매 이벤트마다 하기엔 비싸고 값이 변하지 않는다.
        self._obj_meta = {}

    # ---------------- lifecycle ----------------
    def connect(self):
        args = ["pika-pose-survive"]
        if self.config_path:
            os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
            # 옵션명은 --configfile 이다. --config 는 libsurvive 가 인식하지 못해
            # 조용히 무시되고 해가 ~/.config/libsurvive/config.json 에 저장된다
            # (공식 pika SDK 의 vive_tracker.py 도 --config 를 써서 같은 함정에 빠진다).
            args += ["--configfile", self.config_path]
        if self.force_calibrate:
            args += ["--force-calibrate", "1"]
        args += self.extra_args
        self.ctx = pysurvive.SimpleContext(args)
        if not self.ctx or not self.ctx.ptr:
            raise RuntimeError(
                "[pose] libsurvive SimpleContext 초기화 실패 — 트래커 USB 연결과 "
                "hidraw 권한(udev 규칙 81-vive.rules)을 확인하세요.")
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="PoseSurvive", daemon=True)
        self._thread.start()
        self._warmup()
        return self

    def _warmup(self):
        """첫 포즈가 나올 때까지 대기 — libsurvive 는 프로세스마다 콜드 스타트다.

        SteamVR 은 런타임이 이미 추적 중이라 connect 직후 포즈가 나오지만, libsurvive 는
        라이트하우스 획득에 실측 ~1.7s 가 걸린다. 이걸 흡수하지 않으면 호출부의 짧은
        settle(EpisodeRecorder 기본 1.0s) 안에 트래커가 하나도 안 보여 "트래커 없음" 으로
        기동이 실패한다.

        warmup_expect 로 기대 트래커를 주면 **전부 붙을 때까지** 기다린다. 트래커마다
        획득 시각이 다르고(가림/각도에 따라 수 초 차이) 첫 기기 직후 끊으면 양팔 세션이
        한쪽만 잡힌 채 시작된다 — 실제로 그렇게 실패했다. 기대치가 없을 때만 "새 기기가
        더 안 들어오면 종료" 휴리스틱을 쓴다.
        """
        if self.warmup_sec <= 0:
            return
        t0 = time.monotonic()
        deadline = t0 + self.warmup_sec
        want = self.warmup_expect
        n = 0
        settle_until = None
        while time.monotonic() < deadline:
            devs = self.get_devices()
            if want is not None:
                if all(sn in devs for sn in want):
                    n = len(devs)
                    break
            elif len(devs) > n:
                n = len(devs)
                settle_until = time.monotonic() + 0.5   # 추가 기기 유입 대기
            elif settle_until is not None and time.monotonic() >= settle_until:
                break
            n = max(n, len(devs))
            time.sleep(0.05)
        devs = self.get_devices()
        missing = [sn for sn in want if sn not in devs] if want else []
        if missing:
            log.warning("[pose] libsurvive 워밍업 %.1fs 내에 트래커 누락: %s (보이는 것: %s) — "
                        "가림/베이스스테이션 시야 확인", self.warmup_sec, missing, devs)
        elif devs:
            log.info("[pose] libsurvive 워밍업 %.2fs, 트래커 %d개", time.monotonic() - t0, len(devs))
        else:
            log.warning("[pose] libsurvive 워밍업 %.1fs 내에 트래커를 못 찾음 — "
                        "베이스스테이션 전원/시야, 캘리브레이션(config/libsurvive_config.json) 확인",
                        self.warmup_sec)

    def disconnect(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        # libsurvive 컨텍스트는 종료 시 config_path 에 라이트하우스 해를 기록한다.
        self.ctx = None

    # ---------------- internals ----------------
    def _meta(self, obj):
        """SimpleObject -> (serial, 추적 대상 여부). C 포인터 주소 단위로 캐시.

        NextUpdated() 는 매번 새 SimpleObject 래퍼를 만들어 반환하므로 파이썬 객체
        id/repr 로는 캐시가 절대 적중하지 않는다. 캐시 키는 C 포인터 주소여야 한다.
        """
        key = ctypes.cast(obj.ptr, ctypes.c_void_p).value
        cached = self._obj_meta.get(key)
        if cached is not None:
            return cached
        try:
            otype = pysurvive.simple_object_get_type(obj.ptr)
        except Exception:
            otype = None
        serial = None
        try:
            raw = pysurvive.simple_serial_number(obj.ptr)
            if raw:
                serial = raw.decode("utf-8", "replace") if isinstance(raw, bytes) else str(raw)
        except Exception:
            serial = None
        if not serial:
            raw = obj.Name()
            serial = raw.decode("utf-8", "replace") if isinstance(raw, bytes) else str(raw)
        tracked = otype is None or otype in _OBJECT_TYPES
        meta = (serial, tracked)
        self._obj_meta[key] = meta
        if tracked:
            log.info("[pose] libsurvive 오브젝트 인식: %s (type=%s)", serial, otype)
        return meta

    def _loop(self):
        idle_sleep = 1.0 / self.target_hz if self.target_hz > 0 else 0.001
        cnt, t0 = 0, time.perf_counter()
        while self._running:
            try:
                updated = self.ctx.NextUpdated()
            except Exception as e:
                log.error("[pose] libsurvive 갱신 조회 실패: %s", e)
                break
            if not updated:
                # 새 이벤트 없음 — 유휴 폴링 상한만큼 쉰다.
                time.sleep(idle_sleep)
                continue
            serial, tracked = self._meta(updated)
            if not tracked:
                continue   # 라이트하우스(LH0/LH1/LH2) 등은 포즈 스트림에서 제외
            pose, _timecode = updated.Pose()
            pos = (pose.Pos[0], pose.Pos[1], pose.Pos[2])
            # libsurvive Rot 은 [w, x, y, z] — pika 규약 (x, y, z, w) 로 변환.
            quat = (pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0])
            if self.apply_gripper_offset:
                pos, quat = apply_tip_transform(pos, quat)
            ts = time.time()
            seq, sample_ts, fresh = self._seq.update(serial, pos, quat, ts)
            entry = {
                "device_name": serial,
                "timestamp": ts,
                "position": [pos[0], pos[1], pos[2]],
                "rotation": [quat[0], quat[1], quat[2], quat[3]],
                "sample_seq": seq,       # 값 변화 시에만 증가(중복 판별용)
                "sample_ts": sample_ts,  # pose 값이 마지막으로 '바뀐' time.time()
                "fresh": fresh,          # 이 이벤트에서 값이 바뀌었는지(진단용)
                "recv_ts": ts,           # 디바이스 갱신 이벤트를 마지막으로 받은 시각
            }
            with self._lock:
                self._latest[serial] = entry
            cnt += 1
            now = time.perf_counter()
            if now - t0 >= 1.0:
                self._eff_hz = cnt / (now - t0)
                cnt, t0 = 0, now

    def _decorate(self, entry, now):
        """저장된 항목에 읽기 시점 기준 valid/tracking_result 를 붙여 반환.

        libsurvive 는 정지한 트래커도 계속 갱신하므로 '갱신 끊김 = 추적 손실'이다.
        """
        age = now - entry["recv_ts"]
        ok = age <= self.stale_timeout
        out = dict(entry)
        out["valid"] = ok
        out["tracking_result"] = TRACKING_OK if ok else TRACKING_OUT_OF_RANGE
        return out

    # ---------------- public API (PoseSteamVR 호환) ----------------
    def get_pose(self, device_name=None):
        """device_name 지정 시 해당 트래커 포즈, 미지정 시 트래커 1개면 그 포즈, 여러개면 dict."""
        now = time.time()
        with self._lock:
            if device_name is not None:
                entry = self._latest.get(device_name)
                return self._decorate(entry, now) if entry else None
            items = list(self._latest.items())
        if len(items) == 1:
            return self._decorate(items[0][1], now)
        return {sn: self._decorate(e, now) for sn, e in items}

    def get_devices(self):
        """현재 추적이 살아 있는(stale 아닌) 트래커 시리얼 목록."""
        now = time.time()
        with self._lock:
            return [sn for sn, e in self._latest.items()
                    if now - e["recv_ts"] <= self.stale_timeout]

    @property
    def effective_hz(self):
        return self._eff_hz
