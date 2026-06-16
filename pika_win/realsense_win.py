"""Intel RealSense (D405 등) 리더 — pyrealsense2 직접 사용, 백그라운드 스레드로 최신 프레임 보관.

color(BGR8) + depth(Z16, color에 정렬) 스트림. Windows 네이티브 동작.
"""
import threading
import time

import numpy as np
import pyrealsense2 as rs


class RealSenseD4xx:
    def __init__(self, serial=None, width=640, height=480, fps=30, align_to_color=True):
        self.serial = serial
        self.width = width
        self.height = height
        self.fps = fps
        self.align_to_color = align_to_color
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

    def connect(self):
        self.pipe = rs.pipeline()
        cfg = rs.config()
        if self.serial:
            cfg.enable_device(str(self.serial))
        cfg.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        profile = self.pipe.start(cfg)
        try:
            self.physical_port = profile.get_device().get_info(rs.camera_info.physical_port)
        except Exception:
            self.physical_port = None
        self.calib = self._read_calibration(profile)
        self.align = rs.align(rs.stream.color) if self.align_to_color else None
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
            d = frames.get_depth_frame()
            c = frames.get_color_frame()
            if not d or not c:
                continue
            color = np.asanyarray(c.get_data())   # HxWx3 BGR8
            depth = np.asanyarray(d.get_data())   # HxW uint16
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
        """color/depth intrinsics + depth->color extrinsic + depth_scale (정적)."""
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
            return self._color.copy(), self._depth.copy(), self._ts

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
