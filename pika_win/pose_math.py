"""포즈 백엔드 공용 수학/보조 — 백엔드 모듈(openvr/pysurvive) 임포트 없음.

pose_steamvr(OpenVR)와 pose_survive(libsurvive) 두 리더가 이 모듈을 공유한다.
트래커→그리퍼 팁 변환은 PIKA SDK(pika/tracker/vive_tracker.py)의 하드코딩과
수치가 동일해야 하므로 정의를 여기 한 곳에만 둔다.
"""
import math

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


class SampleSeqTracker:
    """값 변화 기반 샘플 시퀀스 — '새 측정 vs 중복 재독' 판별의 신뢰 신호.

    포즈 폴링(기본 250Hz)이 트래커 native 갱신(~120Hz)보다 빨라 같은 값이 여러 번
    읽힌다. timestamp(폴 시각)로는 정지/중복을 구분 못 하므로 다운스트림은
    이 seq 를 본다. 두 백엔드가 동일 의미를 갖도록 여기서 한 번만 구현한다.
    """

    def __init__(self):
        self._state = {}   # key -> {"key": tuple, "seq": int, "sample_ts": float}

    def update(self, device, pos, quat, ts):
        """(seq, sample_ts, fresh) 반환. 포즈 값이 직전과 비트 동일하면 seq/sample_ts 동결."""
        key = (pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3])
        prev = self._state.get(device)
        if prev is None or key != prev["key"]:
            seq = 0 if prev is None else prev["seq"] + 1
            sample_ts, fresh = ts, True
        else:
            seq, sample_ts, fresh = prev["seq"], prev["sample_ts"], False
        self._state[device] = {"key": key, "seq": seq, "sample_ts": sample_ts}
        return seq, sample_ts, fresh
