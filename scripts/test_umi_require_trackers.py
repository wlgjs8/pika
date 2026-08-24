#!/usr/bin/env python
"""UMI 양팔 fail-fast 옵션 회귀 테스트(하드웨어 불필요).

실행: .venv/bin/python scripts/test_umi_require_trackers.py
"""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from pika_win.recorder import ArmSpec, EpisodeRecorder  # noqa: E402
from scripts.umi_teleop_publish import _create_recorder, get_arguments  # noqa: E402


RIGHT_SN = "LHR-RIGHT-TEST"
LEFT_SN = "LHR-LEFT-TEST"


class _FakeRecorder:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


def test_cli_wiring():
    default = get_arguments([])
    strict = get_arguments(["--require-all-trackers"])
    assert default.require_all_trackers is False
    assert strict.require_all_trackers is True

    rec = _create_recorder(strict, [], recorder_cls=_FakeRecorder)
    assert rec.kwargs["require_pose"] is True
    assert rec.kwargs["require_all_trackers"] is True


def test_missing_tracker_fails():
    rec = EpisodeRecorder.__new__(EpisodeRecorder)
    rec.require_pose = True
    rec.require_all_trackers = True
    rec.arms_cfg = [
        ArmSpec("right", tracker_sn=RIGHT_SN),
        ArmSpec("left", tracker_sn=LEFT_SN),
    ]

    def raise_start_error(message):
        raise RuntimeError(message)

    rec._raise_start_error = raise_start_error
    try:
        rec._validate_required_trackers([RIGHT_SN])
    except RuntimeError as exc:
        message = str(exc)
        assert f"expected=['{RIGHT_SN}', '{LEFT_SN}']" in message
        assert f"detected=['{RIGHT_SN}']" in message
        assert f"missing=['{LEFT_SN}']" in message
    else:
        raise AssertionError("누락 트래커가 있는데 시작 검사가 통과했습니다")

    rec._validate_required_trackers([RIGHT_SN, LEFT_SN])


if __name__ == "__main__":
    test_cli_wiring()
    test_missing_tracker_fails()
    print("ALL TESTS PASSED")
