#!/usr/bin/env python
"""libsurvive 포즈 스모크 테스트 — scripts/pose_test_openvr.py 의 libsurvive 판.

트래커 인식/시리얼 매핑/포즈 스트림 유효성을 SteamVR 없이 확인한다.
실행: .venv/bin/python scripts/pose_test_survive.py [--seconds 20] [--tip]
"""
import argparse
import json
import os
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from pika_win.libsurvive_config import default_exclude_ids, exclude_args  # noqa: E402
from pika_win.pose_survive import PoseSurvive  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--tip", action="store_true",
                    help="공식 트래커→그리퍼 팁 변환 적용 후 출력")
    ap.add_argument("--config", default=os.path.join(REPO_ROOT, "config", "arms.json"),
                    help="arms.json — tracker_sn 이 실제로 보이는지 대조")
    ap.add_argument("--survive-config",
                    default=os.path.join(REPO_ROOT, "config", "libsurvive_config.json"),
                    help="라이트하우스 해 저장 경로(실행 간 월드 프레임 고정)")
    ap.add_argument("--recalibrate", action="store_true",
                    help="저장된 라이트하우스 해를 버리고 새로 캘리브레이션. "
                         "실행 중 트래커를 작업 공간 전체에 걸쳐 천천히 움직여야 "
                         "scene solve 가 수렴한다(가만히 두면 발산)")
    ap.add_argument("--exclude-id", action="append", default=None, metavar="HEXID",
                    help="이 OOTX id 의 라이트하우스를 추적/해에서 제외(반복 가능). "
                         "공용 공간에서 다른 사용자의 스테이션이 켜져 있어도 우리 해에 "
                         "섞이지 않게 한다. 슬롯 번호는 매 실행 config 에서 다시 찾으므로 "
                         "재캘리브레이션으로 슬롯이 바뀌어도 안전하다. "
                         "**--recalibrate 와 같이 쓸 때 특히 중요** — 안 그러면 남의 "
                         "스테이션까지 같이 풀어서 해에 넣는다.")
    a = ap.parse_args()

    want = {}
    if a.config and os.path.exists(a.config):
        arms = json.load(open(a.config)).get("arms", {})
        want = {v.get("tracker_sn"): k for k, v in arms.items() if v.get("tracker_sn")}
        print(f"[cfg] arms.json 기대 트래커: {want}")

    ids = a.exclude_id if a.exclude_id is not None else default_exclude_ids()
    extra = exclude_args(ids, a.survive_config, log=print)
    pose = PoseSurvive(apply_gripper_offset=a.tip,
                       config_path=a.survive_config,
                       force_calibrate=a.recalibrate,
                       extra_args=extra,
                       # arms.json에 설정된 트래커가 모두 붙을 때까지 기다려야
                       # 한쪽이 늦게 올라오는 양팔 상태도 정확히 진단할 수 있다.
                       warmup_expect=list(want)).connect()
    if a.recalibrate:
        print("[cal] 재캘리브레이션 — 트래커를 작업 공간 전체에 걸쳐 천천히 움직이세요 "
              "(위치+자세 모두 바꿔가며, 베이스스테이션 시야 확보)")
    print(f"[pose] libsurvive 연결. {a.seconds:.0f}초 관찰 "
          f"(frame={'gripper_tip' if a.tip else 'tracker_raw'})")
    try:
        t0 = time.time()
        last = 0.0
        while time.time() - t0 < a.seconds:
            time.sleep(0.1)
            now = time.time()
            if now - last < 1.0:
                continue
            last = now
            devs = pose.get_devices()
            snap = pose.get_pose()
            # get_pose()는 살아있는 트래커가 하나면 pose dict 자체를, 둘 이상이면
            # serial -> pose dict를 반환한다. 단일 반환도 동일한 map 형태로 정규화한다.
            if isinstance(snap, dict) and "position" in snap:
                snap = {snap["device_name"]: snap}
            elif not isinstance(snap, dict):
                snap = {snap["device_name"]: snap} if snap else {}
            # 두 트래커 간 거리는 solve 품질의 가장 빠른 sanity check —
            # 실제 물리 거리와 맞고 움직여도 안정적이어야 한다.
            pts = [p["position"] for p in snap.values() if p.get("valid")]
            sep = ""
            if len(pts) == 2:
                d = sum((c - b) ** 2 for c, b in zip(pts[0], pts[1])) ** 0.5
                sep = f" sep={d:.3f}m"
            print(f"[{now - t0:5.1f}s] live={len(devs)} hz={pose.effective_hz:6.1f}{sep}")
            for sn, p in sorted(snap.items()):
                arm = want.get(sn, "?")
                x, y, z = p["position"]
                r = (x * x + y * y + z * z) ** 0.5
                print(f"    {sn:16s} arm={arm:5s} valid={p['valid']!s:5s} "
                      f"tr={p['tracking_result']} seq={p['sample_seq']:6d} "
                      f"pos=({x:8.4f},{y:8.4f},{z:8.4f}) |r|={r:6.2f}m")
        missing = [sn for sn in want if sn not in pose.get_devices()]
        print(f"\n[결과] 살아있는 트래커={pose.get_devices()}  누락={missing or '없음'}")
        return 1 if missing else 0
    finally:
        pose.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
