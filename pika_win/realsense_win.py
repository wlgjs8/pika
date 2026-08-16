"""Intel RealSense (D405 등) 리더 — pyrealsense2 직접 사용, 백그라운드 스레드로 최신 프레임 보관.

color(BGR8) + depth(Z16, color에 정렬) 스트림. Windows 네이티브 동작.
"""
import os
import re
import threading
import time

import numpy as np
import pyrealsense2 as rs


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
    def __init__(self, serial=None, width=640, height=480, fps=30, align_to_color=True,
                 json_path=None, use_depth=True):
        self.serial = serial
        self.width = width
        self.height = height
        self.fps = fps
        self.align_to_color = align_to_color
        self.json_path = json_path if json_path is not None else _default_advanced_json()
        # depth 를 끄면 스트림뿐 아니라 rs.align(depth->color) 도 건너뛴다. align 은
        # 프레임마다 도는 비용이라 고fps 수집에서 무시할 수 없다.
        self.use_depth = bool(use_depth)
        self.pipe = None
        self.align = None
        self.physical_port = None
        self.calib = None
        self._color = None
        self._depth = None
        self._ts = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

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

        def _find_dev():
            ctx = rs.context()
            for d in ctx.query_devices():
                sn = d.get_info(rs.camera_info.serial_number)
                if self.serial is None or str(sn) == str(self.serial):
                    return d
            return None

        try:
            dev = _find_dev()
            if dev is None:
                print(f"[RealSense] advanced json: serial={self.serial} 디바이스 미발견, 건너뜀")
                return
            adv = rs.rs400_advanced_mode(dev)
            tries = 0
            while not adv.is_enabled() and tries < 5:
                adv.toggle_advanced_mode(True)
                time.sleep(5)  # advanced mode 토글은 USB 재열거를 유발 → 재획득
                dev = _find_dev()
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
        return self

    def _loop(self):
        while self._running:
            try:
                frames = self.pipe.wait_for_frames(2000)
            except Exception:
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
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self.pipe is not None:
            try:
                self.pipe.stop()
            except Exception:
                pass
            self.pipe = None
