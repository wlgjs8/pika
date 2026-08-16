"""SteamVR(OpenVR) 기반 Vive Tracker 포즈 리더 — PIKA SDK의 pysurvive 경로 대체.

설계는 PIKA SDK와 동일 패턴:
  - 백그라운드 스레드가 target_hz로 폴링하여 '최신 포즈'를 보관
  - get_pose(device_name) 으로 최신값 조회 (event-driven 내부 + 고정레이트 샘플)
포즈 포맷도 PIKA PoseData와 동일:
  - position [x, y, z] (m), rotation [x, y, z, w] (쿼터니언)
옵션:
  - apply_gripper_offset=True 시 PIKA SDK 공식 트래커→그리퍼 팁 변환을 적용해
    포즈 원점을 그리퍼 핑거팁 라인(축: x=전방/접근, y=좌, z=상)으로 옮긴다:
      T_pub = T_tracker(raw) · R_corr · Trans(0.172, 0, -0.076)
      R_corr = Rx(-20°) · [Ry(-90°) · Rx(-90°)]   (pika_sdk vive_tracker.py 하드코딩과 동일)
    주의: R_corr 는 libsurvive raw frame 기준 정의다. 본 리더는 OpenVR 경로이므로
    두 body frame 동일성은 캘리브레이션 클립으로 확정 전까지 가정이다
    (기대값: raw frame 레버암 [0,-0.0126,+0.1876]m ≈18.8cm, 접근축 = 트래커 +z에서 20°).
"""
import threading
import time

import openvr

# 트래커→팁 변환/쿼터니언 수학은 백엔드 중립이라 pose_math 한 곳에만 둔다.
# 기존 `from pika_win.pose_steamvr import apply_tip_transform, ...` 호출부 호환을
# 위해 여기서 재수출한다.
from .pose_math import (  # noqa: F401
    TIP_ROTATION_QUAT,
    TIP_TRANSLATION,
    SampleSeqTracker,
    apply_tip_transform,
    mat34_to_pos_quat,
    quat_mul,
    quat_rotate_vec,
)

# OpenVR ETrackingResult 코드 → 사람이 읽는 이름. Running_OK(200) 만 정상 추적.
# 그 외(특히 201 OutOfRange / 300 Fallback)는 pose 가 valid 로 보여도 추적을
# 사실상 놓친 상태 → pose 값이 얼어붙는 원인.
TRACKING_RESULT_NAMES = {
    1: "Uninitialized",
    100: "Calibrating_InProgress",
    101: "Calibrating_OutOfRange",
    200: "Running_OK",
    201: "Running_OutOfRange",
    300: "Fallback_RotationOnly",
}


class PoseSteamVR:
    def __init__(self, target_hz=250.0,
                 origin=openvr.TrackingUniverseStanding,
                 device_class=openvr.TrackedDeviceClass_GenericTracker,
                 apply_gripper_offset=False):
        self.target_hz = float(target_hz)
        self.origin = origin
        self.device_class = device_class
        self.apply_gripper_offset = apply_gripper_offset
        self.vr = None
        self._latest = {}            # serial -> pose dict
        self._lock = threading.Lock()
        self._thread = None
        self._running = False
        self._eff_hz = 0.0           # 실제 달성 폴링 Hz
        # 값 변화 기반 샘플 시퀀스. openvr 가 새 디바이스 데이터 없이 캐시/예측 pose 를
        # 반환하면 값이 비트 동일 → seq 동결. 다운스트림은 timestamp(폴 시각)가 아니라
        # 이 seq 로 "새 측정 vs 중복 재독" 을 구분한다. pose_survive 와 동일 구현 공유.
        self._seq = SampleSeqTracker()

    def connect(self):
        self.vr = openvr.init(openvr.VRApplication_Background)
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="PoseSteamVR", daemon=True)
        self._thread.start()
        return self

    def _loop(self):
        period = 1.0 / self.target_hz if self.target_hz > 0 else 0.0
        cnt, t0 = 0, time.perf_counter()
        n = openvr.k_unMaxTrackedDeviceCount
        while self._running:
            t = time.perf_counter()
            poses = self.vr.getDeviceToAbsoluteTrackingPose(self.origin, 0, n)
            ts = time.time()
            snap = {}
            for i in range(n):
                if self.vr.getTrackedDeviceClass(i) != self.device_class:
                    continue
                p = poses[i]
                if not (p.bDeviceIsConnected and p.bPoseIsValid):
                    continue
                pos, quat = mat34_to_pos_quat(p.mDeviceToAbsoluteTracking)
                if self.apply_gripper_offset:
                    pos, quat = apply_tip_transform(pos, quat)
                try:
                    sn = self.vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)
                except Exception:
                    sn = "dev%d" % i
                # eTrackingResult: bPoseIsValid 가 True 라도 추적 품질이 떨어지면
                # SteamVR 은 마지막/예측 pose 를 유지(Running_OutOfRange/Fallback)한다.
                # 이 값으로 "pose 정지 = 트래커 추적 손실" vs "사용자가 손을 가만히 둠"
                # (둘 다 Running_OK) 을 구분한다. 200=Running_OK, 201=Running_OutOfRange,
                # 300=Fallback_RotationOnly (TRACKING_RESULT_NAMES 참조).
                try:
                    tr = int(p.eTrackingResult)
                except Exception:
                    tr = -1
                # 값 변화 기반 샘플 시퀀스(중복 판별의 신뢰 신호). SampleSeqTracker 참조.
                seq, sample_ts, fresh = self._seq.update(sn, pos, quat, ts)
                snap[sn] = {
                    "device_name": sn,
                    "timestamp": ts,
                    "position": [pos[0], pos[1], pos[2]],
                    "rotation": [quat[0], quat[1], quat[2], quat[3]],
                    "valid": True,
                    "tracking_result": tr,
                    "sample_seq": seq,      # 값 변화 시에만 증가(중복 판별용)
                    "sample_ts": sample_ts, # pose 값이 마지막으로 '바뀐' time.time()
                    "fresh": fresh,         # 이 폴에서 값이 바뀌었는지(진단용)
                }
            with self._lock:
                self._latest = snap
            cnt += 1
            if t - t0 >= 1.0:
                self._eff_hz = cnt / (t - t0)
                cnt, t0 = 0, t
            if period:
                rem = period - (time.perf_counter() - t)
                if rem > 0:
                    time.sleep(rem)

    def get_pose(self, device_name=None):
        """device_name 지정 시 해당 트래커 포즈, 미지정 시 트래커 1개면 그 포즈, 여러개면 dict."""
        with self._lock:
            if device_name is not None:
                return self._latest.get(device_name)
            if len(self._latest) == 1:
                return next(iter(self._latest.values()))
            return dict(self._latest)

    def get_devices(self):
        with self._lock:
            return list(self._latest.keys())

    @property
    def effective_hz(self):
        return self._eff_hz

    def disconnect(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self.vr is not None:
            openvr.shutdown()
            self.vr = None
