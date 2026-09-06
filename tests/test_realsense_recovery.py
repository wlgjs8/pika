"""RealSenseD4xx 기동 검증 + 복구 사다리 회귀 테스트.

실제 카메라 없이 사다리의 '순서'와 '종료 조건'만 검증한다. pipeline/센서/펌웨어
호출은 하드웨어가 있어야만 재현되는 물림 상태를 만들 수 없으므로 스텁으로 대체하고,
_recover 가 (a) 경증 rung 부터 시도하는지, (b) 한 rung 이 살리면 다음 rung 을
건너뛰는지, (c) 둘 다 실패하면 조용히 통과하지 않고 예외로 막는지를 본다.
"""
import logging
import unittest

from pika_win.realsense_win import RealSenseD4xx


class _FakeCam(RealSenseD4xx):
    """pipeline/센서/펌웨어 호출을 기록만 하는 스텁. heals_at 단계에서 프레임이 돈다."""

    def __init__(self, heals_at=None, raises_at=None):
        super().__init__(serial="TEST", first_frame_timeout=0.05)
        self.calls = []
        self.heals_at = heals_at
        self.raises_at = raises_at
        self._healed = heals_at == "start"

    def _apply_advanced_json(self):
        self.calls.append("json")

    def _start_pipeline(self):
        self.calls.append("start")
        if self._healed:
            with self._lock:
                self._frames = 1

    def _stop_pipeline(self):
        self.calls.append("stop")
        with self._lock:
            self._frames = 0

    def _rung(self, name):
        self.calls.append(name)
        if self.raises_at == name:
            raise RuntimeError(f"{name} 실패(테스트)")
        if self.heals_at == name:
            self._healed = True

    def _reopen_sensor(self):
        self._rung("reopen")

    def _hardware_reset(self):
        self._rung("reset")


class RealSenseRecoveryTests(unittest.TestCase):
    def test_healthy_connect_skips_recovery(self):
        cam = _FakeCam(heals_at="start")
        cam.connect()
        self.assertEqual(cam.calls, ["json", "start"])

    def test_sensor_reopen_recovers_without_hardware_reset(self):
        cam = _FakeCam(heals_at="reopen")
        cam.connect()
        self.assertEqual(cam.calls, ["json", "start", "stop", "reopen", "start"])

    def test_falls_back_to_hardware_reset(self):
        cam = _FakeCam(heals_at="reset")
        cam.connect()
        self.assertEqual(
            cam.calls,
            ["json", "start", "stop", "reopen", "start", "stop", "reset", "start"],
        )

    def test_rung_exception_does_not_abort_the_ladder(self):
        # 경증 rung 이 예외를 던져도 중증 rung 까지는 가야 한다.
        cam = _FakeCam(heals_at="reset", raises_at="reopen")
        cam.connect()
        self.assertIn("reset", cam.calls)

    def test_both_rungs_fail_raises_instead_of_silent_black(self):
        cam = _FakeCam()
        with self.assertRaises(RuntimeError) as ctx:
            cam.connect()
        self.assertIn("프레임이 오지 않습니다", str(ctx.exception))
        self.assertEqual(cam.calls[-1], "stop")   # 실패 시 pipeline 을 남기지 않는다

    def test_disabled_check_skips_verification(self):
        cam = _FakeCam()
        cam.first_frame_timeout = 0.0
        cam.connect()
        self.assertEqual(cam.calls, ["json", "start"])

    def test_timeouts_are_counted_and_warned_at_most_once_per_interval(self):
        cam = _FakeCam()
        cam.warn_interval = 60.0
        with self.assertLogs("pika.realsense", level=logging.WARNING) as logs:
            for _ in range(5):
                cam._note_timeout(RuntimeError("Frame didn't arrive within 2000"))
        self.assertEqual(len(logs.output), 1)
        self.assertEqual(cam.health()["timeouts"], 5)
        self.assertEqual(cam.health()["frames"], 0)
