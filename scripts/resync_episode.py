"""수집된 에피소드에 카메라=마스터 타임싱크 보간을 (재)적용하고 오정렬 통계를 보고한다.

저장 시점에 writer 가 자동으로 pose_synced/gripper_synced 를 만들지만, 이 스크립트는
(1) 보간 로직이 개선됐을 때의 재실행, (2) "보간이 실제로 뭘 바꿨나"의 감사용이다.
원본 pose/gripper 는 절대 건드리지 않는다.

  .venv/bin/python scripts/resync_episode.py data/data_*/episode_*.hdf5
  .venv/bin/python scripts/resync_episode.py --report-only data/data_x/episode_000.hdf5
"""

import argparse
import glob
import os
import sys

import h5py
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pika_win.timesync import sync_arm  # noqa: E402


def process(path, report_only):
    with h5py.File(path, "r" if report_only else "r+") as f:
        for arm in list(f["observations"].keys()):
            g = f[f"observations/{arm}"]
            if "rs_ts" not in g:
                print(f"  {arm}: rs_ts 없음(구스키마 수집분) — 스킵")
                continue
            synced = sync_arm(g["pose"][:], g["pose_sample_ts"][:],
                              g["gripper"][:], g["gripper_ts"][:], g["rs_ts"][:])
            if "pose_synced" not in synced:
                print(f"  {arm}: 보간 불가(유효 샘플 부족) — 스킵")
                continue
            # 감사 지표: 보간이 원본 대비 얼마나 움직였나 = 샘플-홀드 오정렬의 실측치
            d = np.linalg.norm(synced["pose_synced"][:, :3] - g["pose"][:, :3], axis=1) * 1000
            print(f"  {arm}: 보간 이동량 p50 {np.percentile(d, 50):.2f}mm "
                  f"p95 {np.percentile(d, 95):.2f}mm max {d.max():.2f}mm")
            if not report_only:
                for k, v in synced.items():
                    if k in g:
                        del g[k]
                    g.create_dataset(k, data=v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--report-only", action="store_true",
                    help="파일을 수정하지 않고 오정렬 통계만 출력")
    a = ap.parse_args()
    files = sorted(p for pat in a.paths for p in glob.glob(pat))
    if not files:
        raise SystemExit("에피소드 파일 없음")
    for p in files:
        print(p)
        process(p, a.report_only)


if __name__ == "__main__":
    main()
