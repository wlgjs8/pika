"""카메라를 마스터 클럭으로 pose/gripper 를 보간하는 후처리 타임싱크.

수집은 90 Hz 틱마다 각 스트림의 '최신값'을 샘플-홀드로 묶는다(행 단위 단일 timestamp).
그 결과 한 행 안에서 pose(<=4 ms 묵음)와 이미지(<=11 ms 묵음)가 서로 다른 순간의 것이고,
이송 속도 0.5~1 m/s 에서 라벨 기준 5~11 mm 오정렬이 된다. UMI 계열의 표준 처방
(FastUMI/WHED/TacUMI: 카메라가 싱크 마스터)대로, 각 행의 카메라 프레임 수신 시각(rs_ts)
시점의 pose/gripper 를 보간으로 복원한다.

원본(pose/gripper)은 그대로 두고 파생(pose_synced/gripper_synced)만 추가한다 — 지연 상수
캘리브레이션 등 개선이 생기면 원본에서 다시 만들 수 있어야 하기 때문.

pose format: x,y,z,qx,qy,qz,qw (survive_world). 회전은 shortest-path slerp.
"""

import numpy as np

POSE_DIM = 7


def _slerp(q0, q1, u):
    """단위 쿼터니언 shortest-path slerp. q*: (4,) [qx,qy,qz,qw], u: 스칼라 0..1."""
    d = float(np.dot(q0, q1))
    if d < 0.0:                       # 같은 회전의 부호 반전 표현 -> 짧은 길로
        q1 = -q1
        d = -d
    if d > 0.9995:                    # 근접: nlerp 로 충분(수치 안정)
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th = np.arccos(np.clip(d, -1.0, 1.0))
    return (np.sin((1 - u) * th) * q0 + np.sin(u * th) * q1) / np.sin(th)


def _dedup_samples(ts, values):
    """(틱 단위로 저장된) 스트림에서 실제 샘플열만 추린다: ts 가 증가한 행만.

    pose_sample_ts 는 '값이 바뀐 시각'이라 같은 샘플을 재독한 행에서는 반복된다.
    NaN ts(소스 없음)와 NaN 값 행은 버린다.
    """
    ts = np.asarray(ts, np.float64)
    values = np.asarray(values, np.float64)
    ok = np.isfinite(ts) & np.all(np.isfinite(values), axis=-1)
    ts, values = ts[ok], values[ok]
    if len(ts) == 0:
        return ts, values
    keep = np.concatenate([[True], np.diff(ts) > 0])
    return ts[keep], values[keep]


def interp_to(ts_target, ts_src, values, is_pose):
    """values(ts_src 시각의 샘플들)를 ts_target 각 시각으로 보간. 범위 밖은 끝값 클램프.

    반환: (len(ts_target), D). 소스 샘플이 2개 미만이면 None (보간 불가 -> 호출측 스킵).
    """
    ts_src, values = _dedup_samples(ts_src, values)
    if len(ts_src) < 2:
        return None
    ts_target = np.asarray(ts_target, np.float64)
    out = np.empty((len(ts_target), values.shape[1]), np.float64)
    idx = np.clip(np.searchsorted(ts_src, ts_target, side="right") - 1, 0, len(ts_src) - 2)
    t0, t1 = ts_src[idx], ts_src[idx + 1]
    u = np.clip((ts_target - t0) / np.maximum(t1 - t0, 1e-9), 0.0, 1.0)
    if is_pose:
        out[:, :3] = values[idx, :3] + u[:, None] * (values[idx + 1, :3] - values[idx, :3])
        for i in range(len(ts_target)):
            out[i, 3:7] = _slerp(values[idx[i], 3:7], values[idx[i] + 1, 3:7], float(u[i]))
    else:
        out[:] = values[idx] + u[:, None] * (values[idx + 1] - values[idx])
    return out


def sync_arm(pose, pose_sample_ts, gripper, gripper_ts, rs_ts):
    """한 팔 분량을 카메라 시각으로 재앵커. 반환 dict 는 만들 수 있었던 것만 담는다."""
    rs_ts = np.asarray(rs_ts, np.float64)
    out = {}
    if np.isfinite(rs_ts).sum() < 2:
        return out                     # 카메라 타임스탬프가 없으면 마스터가 없다
    p = interp_to(rs_ts, pose_sample_ts, np.asarray(pose, np.float64), is_pose=True)
    if p is not None:
        out["pose_synced"] = p.astype(np.float32)
    g = interp_to(rs_ts, gripper_ts, np.asarray(gripper, np.float64), is_pose=False)
    if g is not None:
        out["gripper_synced"] = g.astype(np.float32)
    return out
