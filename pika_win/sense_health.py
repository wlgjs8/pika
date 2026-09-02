"""PIKA Sense 연결 직후 실제 엔코더 텔레메트리 검증.

공식 SDK의 ``Sense`` 객체는 연결되지 않았거나 아직 데이터를 한 번도 받지
않았을 때도 초기 엔코더 값 ``0``을 반환한다. 따라서 공개 getter 값만 보면
정상적으로 닫힌 Sense와 잘못된/무응답 시리얼 포트를 구별할 수 없다.

여기서는 SDK 시리얼 리더가 마지막으로 파싱한 원시 프레임을 확인한다. 실제
``AS5047`` 프레임으로 수신된 0은 정상이고, SDK 객체의 초기 기본값은 통과하지
않는다.
"""
import math
import time


def sense_encoder_sample(sense):
    """유효한 최신 AS5047 샘플을 ``{angle, rad}``로 반환, 없으면 ``None``."""
    comm = getattr(sense, "serial_comm", None)
    get_latest = getattr(comm, "get_latest_data", None)
    if not callable(get_latest):
        return None
    try:
        raw = get_latest()
    except Exception:
        return None
    if not isinstance(raw, dict):
        return None
    encoder = raw.get("AS5047")
    if not isinstance(encoder, dict):
        return None

    angle = encoder.get("angle")
    rad = encoder.get("rad")
    if (isinstance(angle, bool) or not isinstance(angle, (int, float)) or
            isinstance(rad, bool) or not isinstance(rad, (int, float))):
        return None
    if not (math.isfinite(float(angle)) and math.isfinite(float(rad))):
        return None
    return {"angle": float(angle), "rad": float(rad)}


def wait_for_sense_encoder(sense, timeout=2.0, poll_sec=0.02):
    """제한 시간 동안 첫 유효 AS5047 프레임을 기다린다."""
    deadline = time.monotonic() + max(0.0, float(timeout))
    while True:
        sample = sense_encoder_sample(sense)
        if sample is not None:
            return sample
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        time.sleep(min(max(0.001, float(poll_sec)), remaining))
