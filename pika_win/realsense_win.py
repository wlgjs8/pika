"""Intel RealSense (D405 등) 리더 — pyrealsense2 직접 사용, 백그라운드 스레드로 최신 프레임 보관.

color(BGR8) + depth(Z16, color에 정렬) 스트림. Windows 네이티브 동작.

**프레임 0 은 예외가 아니라 "검은 화면"으로 나타난다.** pipeline.start() 는 성공하는데
wait_for_frames 만 타임아웃하는 물림 상태가 실제로 있고(2026-09-06 D405 2대), 그때
collect 프리뷰는 None 을 검은 타일로 대체해 어두운 장면과 구분이 안 된다. 그래서
여기서 (a) 타임아웃을 세어 주기적으로 warn 하고, (b) 기동 시 첫 프레임 도착을 확인해
안 오면 복구 사다리를 자동으로 태운다(_recover 참조).
"""
import logging
import os
import re
import threading
import time

import numpy as np
import pyrealsense2 as rs

log = logging.getLogger("pika.realsense")


def _default_advanced_json():
    """수집·추론이 동일 카메라 세팅을 쓰도록 하는 rs400 advanced-mode JSON 경로.

    우선순위: 인자 > 환경변수 PIKA_REALSENSE_JSON > repo 내 config 기본 파일.
    파일이 없으면 None(=미적용, 드라이버 기본값 유지).
    """
    env = os.environ.get("PIKA_REALSENSE_JSON")
    if env:
        return env
    repo_default = os.path.join(
        os.path.dirname(__file__), "..", "config", "realsense_d405_advanced.json"
    )
    return repo_default if os.path.isfile(repo_default) else None


class RealSenseD4xx:
    # hardware_reset 후 USB 재열거를 기다리는 최대 시간(초). 실측 재열거는 ~1초지만
    # 허브까지 함께 다시 올라오는 경우가 있어 여유를 둔다.
    RESET_SETTLE_TIMEOUT = 20.0

    def __init__(self, serial=None, width=640, height=480, fps=30, align_to_color=True,
                 json_path=None, use_depth=True, first_frame_timeout=6.0,
                 warn_interval=5.0):
        self.serial = serial
        self.width = width
        self.height = height
        self.fps = fps
        self.align_to_color = align_to_color
        self.json_path = json_path if json_path is not None else _default_advanced_json()
        # depth 를 끄면 스트림뿐 아니라 rs.align(depth->color) 도 건너뛴다. align 은
        # 프레임마다 도는 비용이라 고fps 수집에서 무시할 수 없다.
        self.use_depth = bool(use_depth)
        # 기동 시 첫 프레임 도착을 기다리는 시간(초). 0 이하면 검사·복구를 건너뛴다.
        self.first_frame_timeout = float(first_frame_timeout)
        # 프레임이 안 올 때 warn 을 내보내는 최소 간격(초). 매 타임아웃마다 찍으면
        # 2초에 한 줄씩 쌓여 collect.log 가 그것만으로 채워진다.
        self.warn_interval = float(warn_interval)
        self.pipe = None
        self.align = None
        self.physical_port = None
        self.calib = None
        self._color = None
        self._depth = None
        self._ts = 0.0
        self._frames = 0
        self._timeouts = 0
        self._last_warn = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def _find_device(self):
        """serial 에 해당하는 rs.device (없으면 None).

        매번 새 context 로 재열거한다 — hardware_reset 뒤에는 이전 context 가 들고 있던
        핸들이 stale 로 남아, 재사용하면 이미 사라진 장치를 가리킨다.
        """
        for d in rs.context().query_devices():
            sn = d.get_info(rs.camera_info.serial_number)
            if self.serial is None or str(sn) == str(self.serial):
                return d
        return None

    def _apply_advanced_json(self):
        """pipeline start 전에 advanced-mode JSON을 디바이스에 적용.

        실패해도 캡처 자체는 계속(드라이버 기본값으로 폴백). JSON에
        controls-autoexposure-auto=True가 들어있으면 노출은 그대로 auto 유지.
        """
        path = self.json_path
        if not path:
            return
        if not os.path.isfile(path):
            print(f"[RealSense] advanced json 없음, 건너뜀: {path}")
            return
        try:
            with open(path, "r") as f:
                json_str = f.read()
        except Exception as e:
            print(f"[RealSense] advanced json 읽기 실패({path}): {e}")
            return

        try:
            dev = self._find_device()
            if dev is None:
                print(f"[RealSense] advanced json: serial={self.serial} 디바이스 미발견, 건너뜀")
                return
            adv = rs.rs400_advanced_mode(dev)
            tries = 0
            while not adv.is_enabled() and tries < 5:
                adv.toggle_advanced_mode(True)
                time.sleep(5)  # advanced mode 토글은 USB 재열거를 유발 → 재획득
                dev = self._find_device()
                if dev is None:
                    break
                adv = rs.rs400_advanced_mode(dev)
                tries += 1
            adv.load_json(json_str)
            print(f"[RealSense] advanced json 적용 완료 serial={self.serial}: {path}")
            self._enforce_depth_units(dev, json_str)
        except Exception as e:
            print(f"[RealSense] advanced json 적용 실패 serial={self.serial}: {e}")

    def _enforce_depth_units(self, dev, json_str):
        """depth_units(=depth scale, m/LSB)를 JSON의 param-depthunits로 강제 적용.

        D405 펌웨어에서 load_json 은 param-depthunits 를 무시하고 depth-table 의
        이전 값을 유지하므로, JSON 을 단일 소스로 유지하기 위해 RS2_OPTION_DEPTH_UNITS
        를 직접 설정한다. advanced param-depthunits 는 마이크로미터 단위.
        예) 100 -> 100e-6 m = 0.1mm/LSB -> max range 65535*0.1mm ≈ 6.55m.
        """
        m = re.search(r'"param-depthunits"\s*:\s*"?(\d+(?:\.\d+)?)"?', json_str)
        if not m:
            return
        depth_units_m = float(m.group(1)) * 1e-6
        try:
            sensor = dev.first_depth_sensor()
            if not sensor.supports(rs.option.depth_units):
                return
            rng = sensor.get_option_range(rs.option.depth_units)
            val = min(max(depth_units_m, rng.min), rng.max)
            sensor.set_option(rs.option.depth_units, val)
            print(f"[RealSense] depth_units={val} m (param-depthunits={m.group(1)}µm, "
                  f"max range≈{val * 65535:.3f} m) serial={self.serial}")
        except Exception as e:
            print(f"[RealSense] depth_units 설정 실패 serial={self.serial}: {e}")

    def connect(self):
        self._apply_advanced_json()
        self._start_pipeline()
        if self.first_frame_timeout > 0 and not self.wait_first_frame(self.first_frame_timeout):
            self._recover()
        return self

    def _start_pipeline(self):
        self.pipe = rs.pipeline()
        cfg = rs.config()
        if self.serial:
            cfg.enable_device(str(self.serial))
        if self.use_depth:
            cfg.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        profile = self.pipe.start(cfg)
        try:
            self.physical_port = profile.get_device().get_info(rs.camera_info.physical_port)
        except Exception:
            self.physical_port = None
        self.calib = self._read_calibration(profile)
        self.align = rs.align(rs.stream.color) if (self.align_to_color and self.use_depth) else None
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="RealSense", daemon=True)
        self._thread.start()

    def _stop_pipeline(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self.pipe is not None:
            try:
                self.pipe.stop()
            except Exception:
                pass
            self.pipe = None

    def wait_first_frame(self, timeout, poll_sec=0.05):
        """첫 프레임이 도착하면 True, 제한 시간 내 못 받으면 False."""
        deadline = time.monotonic() + max(0.0, float(timeout))
        while True:
            with self._lock:
                if self._frames > 0:
                    return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(poll_sec)

    def _recover(self):
        """pipeline 은 떴는데 첫 프레임이 안 오는 물림 상태를 자동 복구한다.

        2026-09-06 실측 사다리 — 두 단계는 증상 등급이 다르다.
          1) 경증: pipeline 만 물림(커널 raw 캡처 `v4l2-ctl --stream-mmap` 은 정상).
             센서를 직접 open→start→stop→close 로 한 사이클 돌리면 풀린다.
          2) 중증: 커널 raw 캡처마저 멈춘다. USBDEVFS_RESET ioctl 은 효과가 없었고
             hardware_reset() 만 허브까지 재열거시켜 살아났다.
        둘 다 실패하면 예외로 수집을 막는다. 여기서 통과시키면 프레임 0 이 예외 없이
        검은 화면으로만 나타나 세션 전체가 빈 영상으로 끝난다.
        """
        sn = self.serial or "(auto)"
        for step, label in ((self._reopen_sensor, "센서 재개방"),
                            (self._hardware_reset, "hardware_reset")):
            log.warning("[RealSense] serial=%s: %.1f초 안에 첫 프레임 없음 → %s 시도",
                        sn, self.first_frame_timeout, label)
            self._stop_pipeline()
            try:
                step()
            except Exception as e:
                log.warning("[RealSense] serial=%s: %s 실패 — %s", sn, label, e)
                continue
            self._start_pipeline()
            if self.wait_first_frame(self.first_frame_timeout):
                log.warning("[RealSense] serial=%s: 복구 성공(%s) — 프레임 재개", sn, label)
                return
        self._stop_pipeline()
        raise RuntimeError(
            f"[RealSense] serial={sn}: pipeline 은 start 했지만 "
            f"{self.first_frame_timeout:g}초 안에 프레임이 오지 않습니다 "
            "(센서 재개방·hardware_reset 모두 실패). USB 케이블/허브 전원을 "
            "물리적으로 재연결하세요."
        )

    def _reopen_sensor(self):
        """센서를 직접 open→start→stop→close 한 사이클. pipeline 계층만 물린 경우를 푼다."""
        dev = self._find_device()
        if dev is None:
            raise RuntimeError("장치 미발견")
        sensor = dev.query_sensors()[0]
        profiles = sensor.get_stream_profiles()
        target = None
        for prof in profiles:
            if prof.stream_type() != rs.stream.color:
                continue
            video = prof.as_video_stream_profile()
            if video.width() == self.width and video.height() == self.height:
                target = prof
                break
        if target is None:
            if not profiles:
                raise RuntimeError("스트림 프로파일 없음")
            target = profiles[0]
        sensor.open(target)
        started = False
        try:
            sensor.start(lambda _f: None)
            started = True
            time.sleep(1.0)
        finally:
            if started:
                try:
                    sensor.stop()
                except Exception:
                    pass
            sensor.close()

    def _hardware_reset(self):
        """펌웨어 리셋 → USB 재열거 대기 → advanced JSON 재적용."""
        dev = self._find_device()
        if dev is None:
            raise RuntimeError("장치 미발견")
        dev.hardware_reset()
        # 리셋 직후에 조회하면 아직 사라지지 않은 stale 항목이 잡힌다. 먼저 확실히
        # 내려갈 시간을 준 뒤 시리얼이 다시 보일 때까지 폴링한다.
        time.sleep(2.0)
        deadline = time.monotonic() + self.RESET_SETTLE_TIMEOUT
        while True:
            try:
                if self._find_device() is not None:
                    break
            except Exception:
                pass
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"리셋 후 {self.RESET_SETTLE_TIMEOUT:g}초 안에 재열거되지 않음")
            time.sleep(0.5)
        time.sleep(1.5)   # /dev 노드(udev) 생성까지 여유
        # 리셋은 advanced-mode 로 올린 설정을 날린다 — JSON 을 단일 소스로 유지하려면 재적용.
        self._apply_advanced_json()

    def _note_timeout(self, exc):
        """wait_for_frames 타임아웃을 세고 warn_interval 마다 한 번 경고한다.

        예전에는 조용히 continue 해서 카메라가 죽어도 로그가 한 줄도 남지 않았다.
        프리뷰는 검은 타일이라 어두운 장면과 구분이 안 되고, 결국 수집이 끝난 뒤
        HDF5 를 열어봐야 빈 영상인 걸 알게 됐다.
        """
        with self._lock:
            self._timeouts += 1
            timeouts, frames, last_ts = self._timeouts, self._frames, self._ts
        now = time.monotonic()
        if now - self._last_warn < self.warn_interval:
            return
        self._last_warn = now
        age = f"{time.time() - last_ts:.1f}초 전" if last_ts else "한 번도 없음"
        log.warning("[RealSense] serial=%s 프레임 없음 — timeout %d회 / 수신 %d장 "
                    "(마지막 프레임 %s): %s", self.serial or "(auto)", timeouts, frames, age, exc)

    def health(self):
        """{frames, timeouts, last_ts, age_sec} — 수집 루프/프리뷰의 상태 표시용."""
        with self._lock:
            frames, timeouts, ts = self._frames, self._timeouts, self._ts
        return {
            "frames": frames,
            "timeouts": timeouts,
            "last_ts": ts or None,
            "age_sec": (time.time() - ts) if ts else None,
        }

    def _loop(self):
        while self._running:
            try:
                frames = self.pipe.wait_for_frames(2000)
            except Exception as e:
                self._note_timeout(e)
                continue
            if self.align is not None:
                frames = self.align.process(frames)
            c = frames.get_color_frame()
            if not c:
                continue
            d = frames.get_depth_frame() if self.use_depth else None
            if self.use_depth and not d:
                continue
            color = np.asanyarray(c.get_data())   # HxWx3 BGR8
            depth = np.asanyarray(d.get_data()) if d else None   # HxW uint16
            with self._lock:
                self._color, self._depth, self._ts = color, depth, time.time()
                self._frames += 1

    @staticmethod
    def _intr_to_dict(intr):
        return {
            "width": int(intr.width),
            "height": int(intr.height),
            "fx": float(intr.fx),
            "fy": float(intr.fy),
            "ppx": float(intr.ppx),
            "ppy": float(intr.ppy),
            "model": str(intr.model),
            "coeffs": [float(c) for c in intr.coeffs],
        }

    def _read_calibration(self, profile):
        """color/depth intrinsics + depth->color extrinsic + depth_scale (정적).

        depth 를 끈 경우 depth 관련 항목은 만들 수 없으므로 color intrinsics 만 담아
        반환한다(예전에는 예외로 빠져 calib 전체가 None 이 되어 color intrinsics 까지
        잃었다 — 수집분에서 카메라 내부파라미터가 통째로 사라지는 손실).
        """
        if not self.use_depth:
            try:
                cprof = profile.get_stream(rs.stream.color).as_video_stream_profile()
                return {
                    "color_intrinsics": self._intr_to_dict(cprof.get_intrinsics()),
                    "depth_aligned_to_color": False,
                }
            except Exception:
                return None
        try:
            cprof = profile.get_stream(rs.stream.color).as_video_stream_profile()
            dprof = profile.get_stream(rs.stream.depth).as_video_stream_profile()
            extr = dprof.get_extrinsics_to(cprof)   # depth -> color
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = depth_sensor.get_depth_scale()
            baseline_mm = None      # 스테레오 IR 이미저 간 baseline(mm)
            try:
                if depth_sensor.supports(rs.option.stereo_baseline):
                    baseline_mm = float(depth_sensor.get_option(rs.option.stereo_baseline))
            except Exception:
                baseline_mm = None
            return {
                "color_intrinsics": self._intr_to_dict(cprof.get_intrinsics()),
                "depth_intrinsics": self._intr_to_dict(dprof.get_intrinsics()),
                # rotation 9 = column-major 3x3, translation 3 = meters
                "depth_to_color_rotation": [float(r) for r in extr.rotation],
                "depth_to_color_translation": [float(t) for t in extr.translation],
                "depth_scale": float(depth_scale),
                "stereo_baseline_mm": baseline_mm,
                "depth_aligned_to_color": bool(self.align_to_color),
            }
        except Exception:
            return None

    def get_calibration(self):
        return self.calib

    def get_frames(self):
        """(color BGR or None, depth uint16 or None, timestamp)."""
        with self._lock:
            if self._color is None:
                return None, None, None
            depth = self._depth.copy() if self._depth is not None else None
            return self._color.copy(), depth, self._ts

    def disconnect(self):
        self._stop_pipeline()
