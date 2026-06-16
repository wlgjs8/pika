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
