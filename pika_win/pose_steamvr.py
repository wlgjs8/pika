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
import math
import threading
import time

import openvr

# PIKA SDK 공식: 트래커 원점 -> 그리퍼 팁 (보정 후 그리퍼 frame 기준 병진, meter)
# (구 GRIPPER_OFFSET 동일 수치 — 단, raw 트래커 frame이 아니라 R_corr 적용 후 frame에서의 값)
TIP_TRANSLATION = (0.172, 0.0, -0.076)


def _rpy_to_quat(roll, pitch, yaw):
    """R = Rz(yaw)·Ry(pitch)·Rx(roll) 의 쿼터니언 (x,y,z,w) — pika_sdk xyzrpy2Mat 규약."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_mul(a, b):
    """쿼터니언 곱 a⊗b ((x,y,z,w), 회전 합성: b를 a의 로컬 frame에서 추가 적용)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


# R_corr = Rx(-20°)·[Ry(-90°)·Rx(-90°)] — pika_sdk 와 동일하게 합성
_DEG = math.pi / 180.0
TIP_ROTATION_QUAT = quat_mul(
    _rpy_to_quat(-20.0 * _DEG, 0.0, 0.0),
    _rpy_to_quat(-90.0 * _DEG, -90.0 * _DEG, 0.0),
)


def apply_tip_transform(pos, quat):
    """raw 트래커 포즈 → 공식 그리퍼 팁 포즈. T_pub = T_raw · R_corr · Trans(TIP_TRANSLATION).

    병진은 R_corr 적용 후 frame에서 정의되므로 raw frame 레버암은
    R_corr·t (= quat_rotate_vec(TIP_ROTATION_QUAT, TIP_TRANSLATION) ≈ [0,-0.0126,+0.1876]).
    """
    lever_local = quat_rotate_vec(TIP_ROTATION_QUAT, TIP_TRANSLATION)
    off = quat_rotate_vec(quat, lever_local)
    return (
        (pos[0] + off[0], pos[1] + off[1], pos[2] + off[2]),
        quat_mul(quat, TIP_ROTATION_QUAT),
    )


def mat34_to_pos_quat(m):
    """OpenVR HmdMatrix34_t -> (pos (x,y,z), quat (x,y,z,w))."""
    x, y, z = m[0][3], m[1][3], m[2][3]
    r00, r01, r02 = m[0][0], m[0][1], m[0][2]
    r10, r11, r12 = m[1][0], m[1][1], m[1][2]
    r20, r21, r22 = m[2][0], m[2][1], m[2][2]
    tr = r00 + r11 + r22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw, qx, qy, qz = 0.25 * s, (r21 - r12) / s, (r02 - r20) / s, (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2
        qw, qx, qy, qz = (r21 - r12) / s, 0.25 * s, (r01 + r10) / s, (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2
        qw, qx, qy, qz = (r02 - r20) / s, (r01 + r10) / s, 0.25 * s, (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2
        qw, qx, qy, qz = (r10 - r01) / s, (r02 + r20) / s, (r12 + r21) / s, 0.25 * s
    return (x, y, z), (qx, qy, qz, qw)


def quat_rotate_vec(q, v):
    """쿼터니언 q(x,y,z,w)로 벡터 v를 회전. v' = q v q*."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


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
                snap[sn] = {
                    "device_name": sn,
                    "timestamp": ts,
                    "position": [pos[0], pos[1], pos[2]],
                    "rotation": [quat[0], quat[1], quat[2], quat[3]],
                    "valid": True,
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
