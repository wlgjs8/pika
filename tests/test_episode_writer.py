import os
import signal
import time
import unittest

from pika_win.episode_writer import EpisodeWriterProcess


class EpisodeWriterProcessTests(unittest.TestCase):
    @unittest.skipUnless(os.name == "posix", "POSIX signal behavior")
    def test_writer_child_ignores_terminal_interrupt(self):
        writer = EpisodeWriterProcess()
        try:
            # 프로세스가 _writer_loop에 들어가 signal handler를 설치할 시간을 준다.
            time.sleep(0.1)
            os.kill(writer._proc.pid, signal.SIGINT)
            time.sleep(0.1)
            self.assertTrue(writer._proc.is_alive())
        finally:
            writer.close()


if __name__ == "__main__":
    unittest.main()
