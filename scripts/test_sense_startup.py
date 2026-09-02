#!/usr/bin/env python
"""Sense 시작 fail-fast와 identify 후보 필터 회귀 테스트(하드웨어 불필요).

실행: .venv/bin/python scripts/test_sense_startup.py
"""
import math
import os
import sys
import tempfile
from unittest import mock

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from pika_win.recorder import ArmSpec, EpisodeRecorder, _ArmIO  # noqa: E402
from pika_win.sense_health import sense_encoder_sample  # noqa: E402
from scripts.identify_arms import (  # noqa: E402
    _linux_stable_serial_paths,
    connect_sense_candidates,
)


class _FakeSerialComm:
    def __init__(self, raw):
        self.raw = raw

    def get_latest_data(self):
        if isinstance(self.raw, BaseException):
            raise self.raw
        return self.raw


class _FakeSense:
    profiles = {}
    instances = []

    def __init__(self, port):
        self.port = port
        self.profile = dict(self.profiles[port])
        self.serial_comm = _FakeSerialComm(self.profile.get("raw", {}))
        self.is_connected = False
        self.disconnected = False
        self.instances.append(self)

    def connect(self):
        exc = self.profile.get("exception")
        if exc is not None:
            raise exc
        result = self.profile.get("connect", True)
        self.is_connected = bool(self.profile.get("is_connected", result))
        return result

    def disconnect(self):
        self.disconnected = True
        self.is_connected = False


def _raw(angle=12.5, rad=0.25):
    return {"Command": 0, "AS5047": {"angle": angle, "rad": rad}}


def _make_recorder(*ports):
    tmp = tempfile.TemporaryDirectory(prefix="pika-sense-startup-")
    specs = [ArmSpec(f"arm{i}", com_port=port) for i, port in enumerate(ports)]
    rec = EpisodeRecorder(
        out_dir=tmp.name,
        arms=specs,
        use_pose=False,
        use_sense=True,
        use_realsense=False,
        use_fisheye=False,
        sense_valid_timeout=0.0,
    )
    rec.active = [_ArmIO(spec) for spec in specs]
    return rec, tmp


def _expect_start_failure(profiles, expected):
    _FakeSense.profiles = profiles
    _FakeSense.instances = []
    rec, tmp = _make_recorder(*profiles)
    try:
        try:
            rec._connect_senses(sense_cls=_FakeSense)
        except RuntimeError as exc:
            assert expected in str(exc), str(exc)
        else:
            raise AssertionError("잘못된 Sense가 있는데 시작 검사가 통과했습니다")
        assert all(s.disconnected for s in _FakeSense.instances)
    finally:
        rec.stop()
        tmp.cleanup()


def test_raw_sample_validation():
    valid_zero = type("S", (), {"serial_comm": _FakeSerialComm(_raw(0.0, 0.0))})()
    assert sense_encoder_sample(valid_zero) == {"angle": 0.0, "rad": 0.0}

    missing = type("S", (), {"serial_comm": _FakeSerialComm({})})()
    assert sense_encoder_sample(missing) is None
    invalid = type(
        "S", (), {"serial_comm": _FakeSerialComm(_raw(math.nan, 0.0))}
    )()
    assert sense_encoder_sample(invalid) is None


def test_recorder_rejects_connect_failure_and_default_zero():
    _expect_start_failure({"missing": {"connect": False}}, "Sense 연결 실패")
    _expect_start_failure(
        {"exception": {"exception": OSError("serial unavailable")}},
        "Sense 연결 예외",
    )
    _expect_start_failure({"wrong-device": {"raw": {}}}, "AS5047 텔레메트리가 없습니다")
    _expect_start_failure(
        {"bad-frame": {"raw": _raw(math.nan, 0.0)}},
        "AS5047 텔레메트리가 없습니다",
    )


def test_recorder_accepts_real_zero_frame():
    _FakeSense.profiles = {"closed-sense": {"raw": _raw(0.0, 0.0)}}
    _FakeSense.instances = []
    rec, tmp = _make_recorder("closed-sense")
    try:
        rec._connect_senses(sense_cls=_FakeSense)
        assert rec.active[0].sense.is_connected
    finally:
        rec.stop()
        tmp.cleanup()


def test_second_arm_failure_cleans_up_first():
    _expect_start_failure(
        {"right": {"raw": _raw()}, "left": {"raw": {}}},
        "[arm1]",
    )


def test_identify_filters_non_sense_ch340_ports():
    ports = ["robot-left", "robot-right", "sense-right", "sense-left"]
    _FakeSense.profiles = {
        "robot-left": {"raw": {}},
        "robot-right": {"raw": {}},
        "sense-right": {"raw": _raw(75.4, 1.32)},
        "sense-left": {"raw": _raw(73.9, 1.29)},
    }
    _FakeSense.instances = []
    senses = connect_sense_candidates(ports, sense_cls=_FakeSense, telemetry_timeout=0.0)
    try:
        assert set(senses) == {"sense-right", "sense-left"}
        rejected = [s for s in _FakeSense.instances if s.port.startswith("robot-")]
        assert rejected and all(s.disconnected for s in rejected)
    finally:
        for sense in senses.values():
            sense.disconnect()


def test_linux_stable_paths_use_by_path():
    if os.name == "nt":
        return
    with tempfile.TemporaryDirectory(prefix="pika-by-path-") as tmp:
        tty = os.path.join(tmp, "ttyUSB3")
        link = os.path.join(tmp, "pci-test-usb-0:3.1.4:1.0-port0")
        open(tty, "wb").close()
        os.symlink(tty, link)
        with mock.patch("scripts.identify_arms.glob.glob", return_value=[link]) as glob_fn:
            mapping = _linux_stable_serial_paths()
        glob_fn.assert_called_once_with("/dev/serial/by-path/*")
        assert mapping == {os.path.realpath(tty): link}


if __name__ == "__main__":
    test_raw_sample_validation()
    test_recorder_rejects_connect_failure_and_default_zero()
    test_recorder_accepts_real_zero_frame()
    test_second_arm_failure_cleans_up_first()
    test_identify_filters_non_sense_ch340_ports()
    test_linux_stable_paths_use_by_path()
    print("ALL TESTS PASSED")
