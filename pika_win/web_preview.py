"""수집과 격리된 최신-프레임 MJPEG 프리뷰.

수집 프로세스는 최대 ``fps`` 번/초 공유 버퍼에 non-blocking copy 만 시도한다.
버퍼를 프리뷰 프로세스가 읽는 중이면 기다리지 않고 그 프리뷰 프레임을 버린다.
resize/JPEG/HTTP/socket write 는 모두 별도 프로세스에서 실행되므로 느린 client가
수집 루프에 backpressure를 전달할 수 없다.
"""

from __future__ import annotations

import ctypes
import json
import multiprocessing as mp
import os
import signal
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


# 이보다 오래된 RealSense 프레임은 "얼어붙은 마지막 1장"으로 보고 타일에 표시한다.
# 90Hz 수집에서 정상 나이는 20ms 미만이라 오탐 여지가 없다.
STALE_FRAME_MS = 500.0

_INDEX_HTML = """<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>PIKA collect preview</title>
  <style>
    html, body { margin: 0; min-height: 100%; background: #111; color: #ddd;
                 font-family: Consolas, "Malgun Gothic", sans-serif; }
    main { display: grid; place-items: center; gap: 10px; padding: 12px; }
    #status { min-height: 1.4em; font-size: 14px; }
    img { display: block; max-width: calc(100vw - 24px); max-height: calc(100vh - 60px);
          width: auto; height: auto; background: #222; image-rendering: auto; }
  </style>
</head>
<body><main>
  <div id="status">preview 연결 중...</div>
  <img id="preview" src="/stream.mjpg" alt="PIKA camera preview">
</main>
<script>
const statusNode = document.getElementById('status');
const preview = document.getElementById('preview');
preview.onerror = () => {
  statusNode.textContent = 'stream 재연결 중...';
  setTimeout(() => { preview.src = '/stream.mjpg?t=' + Date.now(); }, 1000);
};
async function refreshStatus() {
  try {
    const r = await fetch('/healthz?t=' + Date.now(), {cache: 'no-store'});
    const s = await r.json();
    statusNode.textContent = `${s.state} | clients=${s.clients} | ` +
      `published=${s.published} dropped=${s.dropped}`;
  } catch (_) {
    statusNode.textContent = 'server 재연결 중...';
  }
}
refreshStatus();
setInterval(refreshStatus, 1000);
</script></body></html>
""".encode("utf-8")


def _no_cache(handler):
    handler.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
    handler.send_header("Pragma", "no-cache")
    handler.send_header("Expires", "0")
    handler.send_header("X-Accel-Buffering", "no")


class _PreviewHub:
    """프리뷰 자식 프로세스 안에서 encoder와 HTTP handler가 공유하는 상태."""

    def __init__(self, shared):
        self.shared = shared
        self.stop = shared["stop"]
        self.condition = threading.Condition()
        self.clients_guard = threading.Lock()
        self.latest = None

    def add_client(self, delta):
        with self.clients_guard:
            current = max(0, int(self.shared["clients"].value) + int(delta))
            self.shared["clients"].value = current

    def set_jpeg(self, sequence, jpeg):
        with self.condition:
            self.latest = (int(sequence), jpeg)
            self.condition.notify_all()

    def wait_jpeg(self, previous, timeout=1.0):
        with self.condition:
            if self.latest is None or self.latest[0] == previous:
                self.condition.wait(timeout)
            if self.latest is None or self.latest[0] == previous:
                return None
            return self.latest

    def health(self):
        state_code = int(self.shared["state"].value)
        state = {0: "STARTING", 1: "IDLE", 2: "REC", 3: "STOPPING"}.get(
            state_code, "UNKNOWN"
        )
        return {
            "ok": True,
            "state": state,
            "episode": int(self.shared["episode"].value),
            "collect_hz": round(float(self.shared["collect_hz"].value), 1),
            "clients": int(self.shared["clients"].value),
            "published": int(self.shared["published"].value),
            "dropped": int(self.shared["dropped"].value),
            "sequence": int(self.shared["sequence"].value),
        }


def _make_handler(hub):
    class PreviewHandler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_GET(self):  # noqa: N802 -- BaseHTTPRequestHandler API
            path = self.path.split("?", 1)[0]
            if path == "/":
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(_INDEX_HTML)))
                _no_cache(self)
                self.end_headers()
                self.wfile.write(_INDEX_HTML)
                return
            if path == "/healthz":
                body = json.dumps(hub.health(), separators=(",", ":")).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                _no_cache(self)
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/stream.mjpg":
                self._stream()
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def _stream(self):
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            _no_cache(self)
            self.end_headers()
            self.connection.settimeout(2.0)
            previous = -1
            hub.add_client(1)
            try:
                while not hub.stop.is_set():
                    item = hub.wait_jpeg(previous, timeout=0.5)
                    if item is None:
                        continue
                    sequence, jpeg = item
                    header = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(jpeg)}\r\n".encode("ascii")
                        + f"X-Sequence: {sequence}\r\n\r\n".encode("ascii")
                    )
                    self.wfile.write(header)
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    previous = sequence
            except (BrokenPipeError, ConnectionError, OSError):
                pass
            finally:
                hub.add_client(-1)

        def log_message(self, fmt, *args):
            # 매 MJPEG 연결/해제 로그가 수집 stdout을 더럽히지 않게 조용히 처리한다.
            return

    return PreviewHandler


class _ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def _encoder_loop(shared, hub, names, source_shape, tile_width, jpeg_quality):
    import cv2  # 자식 프로세스에서만 무거운 영상 처리를 수행한다.
    import numpy as np

    n_arms, source_height, source_width, _ = source_shape
    raw = np.frombuffer(shared["frames"], dtype=np.uint8).reshape(source_shape)
    local = np.empty(source_shape, dtype=np.uint8)
    tile_height = max(1, int(round(source_height * tile_width / source_width)))
    order = sorted(range(n_arms), key=lambda i: 0 if names[i] == "left" else 1)
    last_sequence = 0

    while not shared["stop"].wait(0.01):
        if shared["clients"].value <= 0:
            continue
        sequence = int(shared["sequence"].value)
        if sequence == 0 or sequence == last_sequence:
            continue

        # lock은 원본 copy 동안만 잡고 resize/JPEG는 lock 밖에서 수행한다.
        with shared["frame_lock"]:
            sequence = int(shared["sequence"].value)
            if sequence == 0 or sequence == last_sequence:
                continue
            np.copyto(local, raw)
            capture_ts = float(shared["capture_ts"].value)
            state_code = int(shared["state"].value)
            episode = int(shared["episode"].value)
            collect_hz = float(shared["collect_hz"].value)
            dropped = int(shared["dropped"].value)
            arm_age_ms = list(shared["arm_age_ms"])

        tiles = []
        for i in order:
            tile = cv2.resize(local[i], (tile_width, tile_height),
                              interpolation=cv2.INTER_AREA)
            age = arm_age_ms[i]
            mark = None
            if age < 0.0:
                mark = "NO FRAMES"
            elif age > STALE_FRAME_MS:
                mark = f"STALE {age / 1000.0:.1f}s"
            if mark:
                cv2.putText(tile, mark, (6, tile_height - 8), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.rectangle(tile, (1, 1), (tile_width - 2, tile_height - 2),
                              (0, 0, 255), 2)
            tiles.append(tile)
        canvas = cv2.hconcat(tiles) if len(tiles) > 1 else tiles[0]
        bar = np.zeros((34, canvas.shape[1], 3), dtype=np.uint8)
        state = "REC" if state_code == 2 else "IDLE"
        color = (0, 200, 0) if state_code == 2 else (0, 0, 230)
        age_ms = max(0.0, (time.time() - capture_ts) * 1000.0)
        label = (
            f"{state} ep{episode:03d}  collect {collect_hz:4.1f}Hz  "
            f"age {age_ms:4.0f}ms  drop {dropped}"
        )
        cv2.putText(bar, label, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                    color, 1, cv2.LINE_AA)
        output = cv2.vconcat([bar, canvas])
        ok, encoded = cv2.imencode(
            ".jpg", output, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)]
        )
        if ok:
            hub.set_jpeg(sequence, encoded.tobytes())
            last_sequence = sequence


def _preview_process_main(shared, names, source_shape, tile_width, jpeg_quality,
                          host, port, ready):
    # SSH/터미널의 Ctrl-C는 수집 부모만 처리한다. 자식이 공유 Event/Lock 사용 중
    # KeyboardInterrupt로 죽으면 부모의 close()가 해당 락에서 멈출 수 있다.
    try:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    except (AttributeError, OSError, ValueError):
        pass
    try:
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
    except (AttributeError, OSError, ValueError):
        pass
    try:
        try:
            os.nice(10)
        except (AttributeError, OSError):
            pass

        hub = _PreviewHub(shared)
        server = _ReusableThreadingHTTPServer((host, int(port)), _make_handler(hub))
        encoder = threading.Thread(
            target=_encoder_loop,
            args=(shared, hub, names, source_shape, tile_width, jpeg_quality),
            name="WebPreviewEncoder",
            daemon=True,
        )
        encoder.start()
        ready.send(("ok", int(server.server_address[1])))
        ready.close()
        server.timeout = 0.2
        while not shared["stop"].is_set():
            server.handle_request()
            if not encoder.is_alive():
                raise RuntimeError("web preview encoder thread stopped")
        server.server_close()
        with hub.condition:
            hub.condition.notify_all()
        encoder.join(timeout=1.0)
    except BaseException as exc:  # noqa: BLE001 -- 오류를 부모에 전달하고 프리뷰만 종료
        try:
            ready.send(("error", f"{type(exc).__name__}: {exc}"))
            ready.close()
        except (BrokenPipeError, EOFError, OSError):
            pass


class WebPreview:
    """별도 프로세스 MJPEG 프리뷰의 부모측 non-blocking bridge."""

    def __init__(self, names, fps=10.0, tile_width=320, jpeg_quality=70,
                 host="127.0.0.1", port=8765, source_width=640, source_height=480,
                 episode=0, startup_timeout=5.0):
        self.names = list(names)
        if not self.names:
            raise ValueError("web preview에는 팔이 하나 이상 필요합니다")
        self.fps = float(fps)
        self.tile_width = int(tile_width)
        self.jpeg_quality = int(jpeg_quality)
        if not (0.1 <= self.fps <= 30.0):
            raise ValueError("web preview fps는 0.1~30이어야 합니다")
        if not (80 <= self.tile_width <= 1280):
            raise ValueError("web preview tile width는 80~1280이어야 합니다")
        if not (20 <= self.jpeg_quality <= 95):
            raise ValueError("web preview JPEG quality는 20~95여야 합니다")
        if not (0 <= int(port) <= 65535):
            raise ValueError("web preview port는 0~65535여야 합니다")

        self.source_shape = (
            len(self.names), int(source_height), int(source_width), 3
        )
        self._period = 1.0 / self.fps
        self._next_publish = 0.0
        self._prev_tick_ts = None
        self._hz = 0.0
        self._closed = False

        ctx = mp.get_context("spawn")
        nbytes = 1
        for size in self.source_shape:
            nbytes *= size
        self._shared = {
            "frames": ctx.RawArray(ctypes.c_ubyte, nbytes),
            "frame_lock": ctx.Lock(),
            "stop": ctx.Event(),
            "clients": ctx.RawValue(ctypes.c_int, 0),
            "sequence": ctx.RawValue(ctypes.c_ulonglong, 0),
            "capture_ts": ctx.RawValue(ctypes.c_double, 0.0),
            "state": ctx.RawValue(ctypes.c_int, 0),
            "episode": ctx.RawValue(ctypes.c_int, int(episode)),
            "collect_hz": ctx.RawValue(ctypes.c_double, 0.0),
            "published": ctx.RawValue(ctypes.c_ulonglong, 0),
            "dropped": ctx.RawValue(ctypes.c_ulonglong, 0),
            # 팔별 마지막 RealSense 프레임 나이(ms). 음수 = 프레임 자체가 없음.
            # 인코더가 이걸로 죽은 타일에 사유를 새긴다(검은 타일 = 어두운 장면 오인 방지).
            "arm_age_ms": ctx.RawArray(ctypes.c_double, len(self.names)),
        }
        self._shared["arm_age_ms"][:] = [-1.0] * len(self.names)
        recv_ready, send_ready = ctx.Pipe(duplex=False)
        self._ready = recv_ready
        self._process = ctx.Process(
            target=_preview_process_main,
            args=(self._shared, self.names, self.source_shape, self.tile_width,
                  self.jpeg_quality, str(host), int(port), send_ready),
            name="WebPreview",
            daemon=True,
        )
        self._process.start()
        send_ready.close()
        if not self._ready.poll(float(startup_timeout)):
            self.close()
            self._ready.close()
            raise RuntimeError("web preview server 시작 timeout")
        status, detail = self._ready.recv()
        self._ready.close()
        if status != "ok":
            self.close()
            raise RuntimeError(f"web preview server 시작 실패: {detail}")
        self.host = str(host)
        self.port = int(detail)

    @property
    def is_alive(self):
        return (not self._closed and self._process is not None
                and self._process.is_alive())

    def stats(self):
        return {
            "clients": int(self._shared["clients"].value),
            "published": int(self._shared["published"].value),
            "dropped": int(self._shared["dropped"].value),
            "sequence": int(self._shared["sequence"].value),
            "collect_hz": float(self._shared["collect_hz"].value),
        }

    def publish(self, frame, recording=False, episode=0):
        """최신 프레임을 전달한다. 어떤 경우에도 lock이나 네트워크를 기다리지 않는다."""
        if not self.is_alive:
            return False

        ts = float(frame.get("ts") or time.time())
        if self._prev_tick_ts is not None:
            inst = 1.0 / max(ts - self._prev_tick_ts, 1e-6)
            self._hz = inst if self._hz == 0.0 else 0.98 * self._hz + 0.02 * inst
        self._prev_tick_ts = ts
        self._shared["state"].value = 2 if recording else 1
        self._shared["episode"].value = int(episode)
        self._shared["collect_hz"].value = float(self._hz)

        # client가 없으면 raw frame copy조차 하지 않는다.
        if self._shared["clients"].value <= 0:
            return True
        now = time.perf_counter()
        if now < self._next_publish:
            return True
        self._next_publish = now + self._period

        lock = self._shared["frame_lock"]
        if not lock.acquire(block=False):
            self._shared["dropped"].value += 1
            return True
        try:
            import numpy as np

            target = np.frombuffer(
                self._shared["frames"], dtype=np.uint8
            ).reshape(self.source_shape)
            arms = frame.get("arms") or []
            ages = self._shared["arm_age_ms"]
            for idx in range(len(self.names)):
                arm = arms[idx] if idx < len(arms) else {}
                image = arm.get("realsense_color")
                if image is None or image.shape != self.source_shape[1:]:
                    target[idx].fill(0)
                    ages[idx] = -1.0
                else:
                    np.copyto(target[idx], image)
                    rs_ts = arm.get("rs_ts")
                    # rs_ts 를 안 주는 호출자(합성 프레임)는 나이를 알 수 없으니
                    # 0(=신선)으로 두고 오탐을 내지 않는다.
                    ages[idx] = (ts - float(rs_ts)) * 1000.0 if rs_ts else 0.0
            self._shared["capture_ts"].value = ts
            self._shared["sequence"].value += 1
            self._shared["published"].value += 1
        finally:
            lock.release()
        return True

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._shared["state"].value = 3
        process = self._process
        if process is not None:
            # 이미 비정상 종료한 자식의 공유 동기화 객체를 건드리지 않는다.
            if process.is_alive():
                self._shared["stop"].set()
            process.join(timeout=3.0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.0)
            process.close()
            self._process = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
