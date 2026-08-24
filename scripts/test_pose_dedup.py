#!/usr/bin/env python
"""포즈 중복(dedup) 로직 검증 — 하드웨어/SteamVR 불필요.

실제 두 포즈 백엔드가 공유하는 SampleSeqTracker와 build_packet을 합성 스트림으로 구동한다:
  - 폴링 250Hz, 트래커 native 갱신 120Hz, 발행 200Hz
  - 추적손실 동결(eTrackingResult 201/300) = 같은 값 N연속

실행:
  python scripts/test_pose_dedup.py
"""
import bisect
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pika_win.pose_math import SampleSeqTracker  # noqa: E402
from scripts.umi_teleop_publish import build_packet  # noqa: E402

POLL_HZ, NATIVE_HZ, PUB_HZ, DUR = 250.0, 120.0, 200.0, 4.0
QUAT = (0.0, 0.0, 0.0, 1.0)


def test_sample_seq_oversampling():
    seq_tracker = SampleSeqTracker()
    poll_t, poll_seq = [], []
    fresh_polls = 0
    for k in range(int(POLL_HZ * DUR)):
        tp = k / POLL_HZ
        vidx = int(tp * NATIVE_HZ)            # 1/120s 마다 +1 되는 '진짜 샘플'
        pos = (vidx * 1e-3, 0.0, 0.0)         # 같은 bin 폴은 동일 좌표 → 중복
        seq, _sts, fresh = seq_tracker.update("LHR-TEST", pos, QUAT, tp)
        poll_t.append(tp)
        poll_seq.append(seq)
        fresh_polls += int(fresh)
    assert abs(fresh_polls / DUR - NATIVE_HZ) < 6      # 값변화 폴 ≈ native
    assert abs(len(set(poll_seq)) / DUR - NATIVE_HZ) < 2
    return poll_t, poll_seq


def test_publisher_dedup(poll_t, poll_seq):
    def seq_at(t):
        return poll_seq[max(0, bisect.bisect_right(poll_t, t) - 1)]
    last_sent, dup, sent, distinct = None, 0, 0, set()
    for k in range(int(PUB_HZ * DUR)):
        seq = seq_at(k / PUB_HZ)
        fresh = (last_sent is None) or (seq != last_sent)  # main 루프와 동일 식
        last_sent = seq
        sent += 1
        distinct.add(seq)
        if not fresh:
            dup += 1
    assert abs(dup / sent - (1 - NATIVE_HZ / PUB_HZ)) < 0.05   # 이론치 40%
    assert abs(len(distinct) / DUR - NATIVE_HZ) < 3            # 정보 손실 0
    return dup, sent


def test_build_packet_wire():
    pk = build_packet(1.0, {"left": {"pose": [0, 0, 0, 0, 0, 0, 1], "gripper": 0.2,
                                      "deadman": True, "pose_seq": 7, "pose_fresh": False}})
    assert pk["left"]["pose_fresh"] is False and pk["left"]["pose_seq"] == 7
    assert pk["left"]["deadman"] is True and pk["left"]["gripper"] == 0.2  # 중복이어도 유지
    pk3 = build_packet(1.0, {"left": {"pose": [0, 0, 0, 0, 0, 0, 1], "deadman": True}})
    assert "pose_fresh" not in pk3["left"] and "pose_seq" not in pk3["left"]  # 하위호환


def test_tracking_loss_freeze():
    seq_tracker = SampleSeqTracker()
    res = [seq_tracker.update("X", (1.23, 4.56, 7.89), QUAT, 10.0 + i * 0.005)
           for i in range(50)]
    freshes = [r[2] for r in res]
    assert freshes[0] is True and all(f is False for f in freshes[1:])  # 첫 폴만 fresh
    assert len(set(r[0] for r in res)) == 1                             # seq 완전 동결


if __name__ == "__main__":
    pt, psq = test_sample_seq_oversampling()
    dup, sent = test_publisher_dedup(pt, psq)
    test_build_packet_wire()
    test_tracking_loss_freeze()
    print(f"OK  oversampling dup={100 * dup / sent:.1f}%  (발행 {PUB_HZ:.0f}Hz / native {NATIVE_HZ:.0f}Hz)")
    print("ALL TESTS PASSED")
