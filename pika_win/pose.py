"""포즈 백엔드 선택 — libsurvive(기본) / SteamVR(OpenVR).

두 리더는 공개 인터페이스가 같아서(connect/get_pose/get_devices/effective_hz/
disconnect) 호출부는 이 팩토리만 쓰면 된다. 백엔드 모듈은 각각 pysurvive/openvr
를 임포트하므로 **선택된 백엔드만** 지연 임포트한다.
"""
BACKENDS = ("survive", "steamvr")

# 에피소드 HDF5 attrs 로 기록할 백엔드 표기 — 좌표계와 tracking_result 의미가
# 백엔드마다 다르므로 수집분을 나중에 구분할 수 있어야 한다.
BACKEND_LABELS = {
    "survive": "libsurvive",
    "steamvr": "steamvr_openvr",
}


def make_pose_reader(backend="survive", target_hz=250.0,
                     apply_gripper_offset=False, warmup_expect=None, **kwargs):
    """포즈 리더 인스턴스 생성(아직 connect() 는 호출하지 않음).

    Args:
        backend: "survive"(libsurvive, GUI 불필요) 또는 "steamvr"(OpenVR).
        warmup_expect: 기대 트래커 시리얼 목록. libsurvive 는 콜드 스타트라 기기마다
            획득 시각이 달라서, 이걸 주면 전부 붙을 때까지 connect() 가 기다린다.
            SteamVR 은 런타임이 이미 추적 중이라 워밍업 개념이 없어 무시한다.
        kwargs: 백엔드별 추가 인자. survive 는 config_path/stale_timeout/extra_args,
            steamvr 은 origin/device_class 를 받는다.
    """
    backend = (backend or "survive").lower()
    if backend not in BACKENDS:
        raise ValueError(f"알 수 없는 포즈 백엔드: {backend!r} (가능: {', '.join(BACKENDS)})")
    if backend == "survive":
        from .pose_survive import PoseSurvive
        return PoseSurvive(target_hz=target_hz,
                           apply_gripper_offset=apply_gripper_offset,
                           warmup_expect=warmup_expect, **kwargs)
    from .pose_steamvr import PoseSteamVR
    return PoseSteamVR(target_hz=target_hz,
                       apply_gripper_offset=apply_gripper_offset, **kwargs)
