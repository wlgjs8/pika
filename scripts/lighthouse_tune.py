#!/usr/bin/env python
"""베이스스테이션 배치 튜닝용 실시간 계기판.

스테이션을 옮기거나 켜/끄면서 **그 자리가 나아졌는지**를 즉시 숫자로 본다.
배치 작업에서 알고 싶은 것은 포즈 값이 아니라 아래 넷이고, 이 도구는 그것만 본다:

  1. 갱신률(Hz)        — 그 트래커가 실제로 데이터를 받고 있는가
  2. 정지 시 반복정밀도 — 가만히 둔 트래커가 얼마나 떠는가 (배치 품질의 본체)
  3. 팁 지터           — 그 떨림이 그리퍼 팁에서 몇 mm 인가. 회전 노이즈가 188 mm
                         레버로 증폭되므로 원점 지터보다 항상 크다. 로봇이 실제로
                         따라가는 값은 이쪽이다.
  4. 점프              — 한 샘플에 몇 mm 튀는가. 2026-09-04 텔레옵 진동의 직접 원인이
                         raw 380~591 mm 짜리 점프였다. 평균은 이걸 못 보여준다.

두 트래커를 **고정된 지그에 같이 올려두면** `sep`(트래커 간 거리)이 최고의 단일 지표다:
진짜 거리는 상수이므로, 측정된 sep 의 변동 = 순수한 재구성 오차다. 사람이 "가만히
들고 있었다"는 가정이 필요 없다.

조작:  m=현재 배치 기록(라벨)   r=통계 리셋   q=종료

주의: 저장된 라이트하우스 해는 기본적으로 **임시 사본**으로 돌려 건드리지 않는다.
"""
import argparse
import collections
import csv
import re
import math
import os
import shutil
import signal
import sys
import tempfile
import threading
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from pika_win.libsurvive_config import default_exclude_ids, exclude_args  # noqa: E402
from pika_win.pose_math import (  # noqa: E402
    TIP_ROTATION_QUAT, TIP_TRANSLATION, quat_rotate_vec,
)

LIVE_CONFIG = os.path.join(REPO_ROOT, "config", "libsurvive_config.json")
LEVER_M = math.dist((0.0, 0.0, 0.0), quat_rotate_vec(TIP_ROTATION_QUAT, TIP_TRANSLATION))


ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


class CLogTap:
    """libsurvive 는 C 레벨에서 **stdout** 으로 로그를 낸다(실측 확인: stderr 로 잡으면
    한 줄도 안 들어온다). fd 1 을 파이프로 돌려 'Adding lighthouse ch N' / 'acc err'
    같은 줄을 계기판이 읽고, 계기판 자신은 원본 fd 사본에 직접 그린다."""

    def __init__(self):
        self.lines = collections.deque(maxlen=400)
        self._saved = None
        self.render_fd = None

    def start(self):
        r, w = os.pipe()
        self._saved = os.dup(1)      # 계기판 출력용 원본 사본
        self.render_fd = self._saved
        os.dup2(w, 1)
        os.close(w)
        threading.Thread(target=self._pump, args=(r,), daemon=True).start()
        return self

    def _pump(self, r):
        with os.fdopen(r, "r", errors="replace") as fh:
            for line in fh:
                self.lines.append(ANSI.sub("", line).rstrip("\n"))

    def write(self, text):
        os.write(self.render_fd, text.encode("utf-8", "replace"))

    def stop(self):
        if self._saved is not None:
            os.dup2(self._saved, 1)
            os.close(self._saved)
            self._saved = None


class Stats:
    """한 트래커의 롤링 통계. 정지 구간의 산포가 곧 반복정밀도다."""

    def __init__(self, window_sec=4.0):
        self.window_sec = window_sec
        self.pos = collections.deque()      # (t, (x,y,z))
        self.tip = collections.deque()
        self.quat = collections.deque()
        self.seq = collections.deque()
        self.max_jump_mm = 0.0
        self.invalid = 0
        self._last_pos = None

    def add(self, t, pos, quat, seq, valid):
        if not valid:
            self.invalid += 1
            return
        off = quat_rotate_vec(quat, quat_rotate_vec(TIP_ROTATION_QUAT, TIP_TRANSLATION))
        tip = (pos[0] + off[0], pos[1] + off[1], pos[2] + off[2])
        if self._last_pos is not None:
            self.max_jump_mm = max(self.max_jump_mm, math.dist(pos, self._last_pos) * 1000)
        self._last_pos = pos
        for dq, v in ((self.pos, pos), (self.tip, tip), (self.quat, quat), (self.seq, seq)):
            dq.append((t, v))
        cutoff = t - self.window_sec
        for dq in (self.pos, self.tip, self.quat, self.seq):
            while dq and dq[0][0] < cutoff:
                dq.popleft()

    def hz(self):
        if len(self.seq) < 2:
            return 0.0
        dt = self.seq[-1][0] - self.seq[0][0]
        return (self.seq[-1][1] - self.seq[0][1]) / dt if dt > 0 else 0.0

    @staticmethod
    def _spread(dq):
        if len(dq) < 3:
            return (0.0, 0.0, 0.0), 0.0
        cols = list(zip(*[v for _, v in dq]))
        std = tuple((sum((c - sum(col) / len(col)) ** 2 for c in col) / len(col)) ** 0.5 * 1000
                    for col in cols)
        # 창 안의 축별 최대-최소 = 그 구간의 반복정밀도(peak-to-peak).
        rng = max((max(col) - min(col)) for col in cols) * 1000
        return std, rng

    def position(self):
        return self._spread(self.pos)

    def tip_jitter(self):
        return self._spread(self.tip)

    def rot_std_deg(self):
        if len(self.quat) < 3:
            return 0.0
        ref = self.quat[-1][1]
        angs = []
        for _, q in self.quat:
            d = sum(a * b for a, b in zip(q, ref))
            angs.append(2 * math.degrees(math.acos(min(1.0, abs(d)))))
        m = sum(angs) / len(angs)
        return (sum((a - m) ** 2 for a in angs) / len(angs)) ** 0.5

    def reset(self):
        self.max_jump_mm = 0.0
        self.invalid = 0


def read_key():
    import select
    if not sys.stdin.isatty():
        return None
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--window", type=float, default=100.0, help="롤링 통계 창(초)")
    ap.add_argument("--refresh", type=float, default=100.0, help="화면 갱신 Hz")
    ap.add_argument("--csv", default=None, help="샘플을 CSV 로 기록할 경로")
    ap.add_argument("--use-live-config", action="store_true",
                    help="저장된 해를 임시 사본이 아니라 원본으로 사용(종료 시 갱신됨)")
    ap.add_argument("--exclude-id", action="append", default=None, metavar="HEXID",
                    help="이 OOTX id 의 라이트하우스를 제외(반복 가능). 다른 사용자의 "
                         "스테이션이 켜져 있어도 우리 구성만으로 측정하려면 필요하다.")
    ap.add_argument("--fresh", action="store_true",
                    help="저장된 해 없이 시작 — 어떤 채널이 실제로 켜져 있는지만 볼 때")
    a = ap.parse_args()

    cfg = LIVE_CONFIG
    tmp = None
    if not a.use_live_config:
        tmp = tempfile.NamedTemporaryFile(suffix=".json", delete=False)
        tmp.close()
        if not a.fresh and os.path.exists(LIVE_CONFIG):
            shutil.copy(LIVE_CONFIG, tmp.name)
        cfg = tmp.name

    # timeout/kill 로 죽어도 finally 가 돌아 USB 를 놓도록 SIGTERM 을 예외로 바꾼다.
    # 안 그러면 프로세스가 장치를 잡은 채 남아 다음 실행이 "트래커를 못 찾음" 으로
    # 실패한다(2026-09-04 에 실제로 그렇게 30분을 날렸다).
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    tap = CLogTap().start()
    try:
        from pika_win.pose_survive import PoseSurvive
        ids = a.exclude_id if a.exclude_id is not None else default_exclude_ids()
        extra = exclude_args(ids, LIVE_CONFIG, log=lambda m: tap.write(m + "\n"))
        pose = PoseSurvive(apply_gripper_offset=False, config_path=cfg,
                           warmup_sec=12.0, extra_args=extra).connect()
    except Exception as e:
        tap.stop()
        raise SystemExit(f"libsurvive 연결 실패: {e}")

    stats, marks = {}, []
    writer = fh = None
    if a.csv:
        fh = open(a.csv, "w", newline="")
        writer = csv.writer(fh)
        writer.writerow(["t", "serial", "seq", "x", "y", "z", "qx", "qy", "qz", "qw", "valid"])

    old_term = None
    if sys.stdin.isatty():
        import termios, tty
        old_term = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    t0 = time.time()
    last_draw = 0.0
    last_seq = {}
    label = "(배치 1)"
    try:
        while True:
            snap = pose.get_pose()
            if isinstance(snap, dict) and "position" in snap:
                snap = {snap["device_name"]: snap}
            elif not isinstance(snap, dict):
                snap = {}
            now = time.time()
            for sn, p in snap.items():
                if p.get("sample_seq") == last_seq.get(sn):
                    continue                      # 같은 샘플 재독 — 통계 오염 방지
                last_seq[sn] = p.get("sample_seq")
                st = stats.setdefault(sn, Stats(a.window))
                pos, q = tuple(p["position"]), tuple(p["rotation"])
                st.add(now - t0, pos, q, p.get("sample_seq", 0), p.get("valid", False))
                if writer:
                    writer.writerow([f"{now - t0:.4f}", sn, p.get("sample_seq"),
                                     *[f"{v:.6f}" for v in pos], *[f"{v:.6f}" for v in q],
                                     int(bool(p.get("valid")))])

            k = read_key()
            if k == "q":
                break
            if k == "r":
                for st in stats.values():
                    st.reset()
            if k == "m":
                marks.append((now - t0, label, {sn: (st.hz(), st.tip_jitter()[1])
                                                for sn, st in stats.items()}))

            if now - last_draw >= 1.0 / a.refresh:
                last_draw = now
                draw(now - t0, stats, tap, marks, pose)
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        if old_term is not None:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        pose.disconnect()
        tap.stop()
        if fh:
            fh.close()
        if tmp:
            os.unlink(tmp.name)
        print("\n종료. 저장된 해는 " + ("갱신되었습니다." if a.use_live_config else "건드리지 않았습니다."))
        if a.csv:
            print(f"CSV: {a.csv}")


def draw(t, stats, tap, marks, pose):
    out = ["\x1b[2J\x1b[H"]
    out.append(f"베이스스테이션 배치 튜너   t={t:6.1f}s   레버={LEVER_M * 1000:.0f}mm"
               f"    [m]기록 [r]리셋 [q]종료")
    out.append("")
    chans = sorted({l.split("channel ")[1].split(";")[0].strip()
                    for l in tap.lines if "OOTX not set for LH in channel" in l}
                   | {l.split("ch ")[1].split(" ")[0] for l in tap.lines
                      if "Adding lighthouse ch" in l})
    ids = [l.split("Got OOTX packet ")[1] for l in tap.lines if "Got OOTX packet" in l]
    out.append(f"  라이트하우스: 채널 {chans or '—'}   OOTX {sorted(set(ids)) or '—'}")
    solve = [l for l in tap.lines if "acc err" in l]
    if solve:
        out.append(f"  최근 해: {solve[-1].strip()[:96]}")
    out.append("")
    out.append(f"  {'트래커':16s}{'Hz':>7}{'위치 p2p':>10}{'  위치 std x/y/z (mm)':>24}"
               f"{'회전std':>9}{'팁 p2p':>9}{'최대점프':>10}{'무효':>6}")
    out.append("  " + "-" * 92)
    live = []
    for sn, st in sorted(stats.items()):
        pstd, pp2p = st.position()
        _, tp2p = st.tip_jitter()
        out.append(f"  {sn:16s}{st.hz():7.1f}{pp2p:9.2f}m"
                   f"m  {pstd[0]:6.2f}/{pstd[1]:5.2f}/{pstd[2]:5.2f}"
                   f"{st.rot_std_deg():9.3f}{tp2p:8.2f}mm{st.max_jump_mm:9.1f}{st.invalid:6d}")
        if len(st.pos) > 2:
            live.append(st.pos[-1][1])
    out.append("")
    if len(live) == 2:
        a_dq, b_dq = [stats[k].pos for k in sorted(stats)][:2]
        n = min(len(a_dq), len(b_dq))
        if n > 3:
            vals = [math.dist(a_dq[-i][1], b_dq[-i][1]) for i in range(1, n + 1)]
            mean = sum(vals) / len(vals)
            std = (sum((v - mean) ** 2 for v in vals) / len(vals)) ** 0.5
            out.append(f"  트래커 간 거리 sep = {mean * 1000:8.2f} mm   변동(std) = {std * 1000:6.3f} mm"
                       f"   ← 지그에 고정했다면 이 변동이 순수 재구성 오차")
    out.append("")
    if marks:
        out.append("  기록된 배치:")
        for mt, lbl, d in marks[-6:]:
            s = "  ".join(f"{k.split('-')[1]}:{v[0]:.0f}Hz/{v[1]:.2f}mm" for k, v in sorted(d.items()))
            out.append(f"    [{mt:6.1f}s] {lbl:12s} {s}")
    tap.write("\n".join(out) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
