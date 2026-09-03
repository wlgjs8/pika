import json
import socket
import time
import unittest
import urllib.request

import numpy as np

from pika_win.web_preview import WebPreview


def _frame(ts=None):
    left = np.zeros((480, 640, 3), dtype=np.uint8)
    right = np.zeros((480, 640, 3), dtype=np.uint8)
    left[:, :, 1] = 180
    right[:, :, 2] = 180
    return {
        "ts": time.time() if ts is None else ts,
        "arms": [
            {"realsense_color": right},
            {"realsense_color": left},
        ],
    }


def _health(preview):
    url = f"http://127.0.0.1:{preview.port}/healthz"
    with urllib.request.urlopen(url, timeout=2.0) as response:
        return json.loads(response.read())


class WebPreviewTests(unittest.TestCase):
    def test_http_stream_produces_jpeg_and_health(self):
        preview = WebPreview(
            ["right", "left"], fps=20, tile_width=160, jpeg_quality=65, port=0
        )
        stream = None
        try:
            self.assertEqual(_health(preview)["state"], "STARTING")
            stream = urllib.request.urlopen(
                f"http://127.0.0.1:{preview.port}/stream.mjpg", timeout=3.0
            )

            deadline = time.monotonic() + 2.0
            while _health(preview)["clients"] < 1 and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(_health(preview)["clients"], 1)

            self.assertTrue(preview.publish(_frame(), recording=True, episode=7))
            boundary = stream.readline()
            self.assertEqual(boundary, b"--frame\r\n")
            headers = {}
            while True:
                line = stream.readline()
                if line == b"\r\n":
                    break
                key, value = line.decode("ascii").split(":", 1)
                headers[key.lower()] = value.strip()
            jpeg = stream.read(int(headers["content-length"]))
            self.assertTrue(jpeg.startswith(b"\xff\xd8"))
            self.assertTrue(jpeg.endswith(b"\xff\xd9"))

            health = _health(preview)
            self.assertEqual(health["state"], "REC")
            self.assertEqual(health["episode"], 7)
            self.assertEqual(health["published"], 1)
        finally:
            if stream is not None:
                stream.close()
            preview.close()
        self.assertFalse(preview.is_alive)

    def test_publish_never_waits_for_busy_preview_buffer(self):
        preview = WebPreview(["right", "left"], fps=10, port=0)
        lock = preview._shared["frame_lock"]
        try:
            # 실제 HTTP client 없이도 publish 경로를 활성화해 contention을 재현한다.
            preview._shared["clients"].value = 1
            lock.acquire()
            started = time.perf_counter()
            self.assertTrue(preview.publish(_frame()))
            elapsed = time.perf_counter() - started
            self.assertLess(elapsed, 0.05)
            self.assertEqual(preview.stats()["dropped"], 1)
            self.assertEqual(preview.stats()["published"], 0)
        finally:
            lock.release()
            preview.close()

    def test_no_client_skips_raw_frame_copy(self):
        preview = WebPreview(["right", "left"], fps=10, port=0)
        try:
            self.assertTrue(preview.publish(_frame()))
            self.assertEqual(preview.stats()["published"], 0)
            self.assertEqual(preview.stats()["dropped"], 0)
        finally:
            preview.close()

    def test_busy_port_fails_without_leaking_process(self):
        occupied = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        occupied.bind(("127.0.0.1", 0))
        occupied.listen(1)
        port = occupied.getsockname()[1]
        try:
            with self.assertRaisesRegex(RuntimeError, "시작 실패"):
                WebPreview(["right", "left"], port=port)
        finally:
            occupied.close()


if __name__ == "__main__":
    unittest.main()
