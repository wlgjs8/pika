#!/usr/bin/env python
"""rerun 기반 실시간 트래커 포즈 3D 시각화 (web / spawn / save 모드).

네이티브 창이 안 뜨는 환경(GPU/wgpu/원격 등)에서는 --mode web (브라우저 뷰어)를 쓰세요.
실행:
  conda run -n pika python scripts\\visualize_pose_rerun.py               # 기본: web(브라우저)
  conda run -n pika python scripts\\visualize_pose_rerun.py --mode spawn  # 네이티브 창
  conda run -n pika python scripts\\visualize_pose_rerun.py --mode save   # .rrd 파일로 저장 후 'rerun file.rrd'
종료: Ctrl-C
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rerun as rr  # noqa: E402
from pika_win.pose_steamvr import PoseSteamVR  # noqa: E402

TARGET_HZ = 120.0
TRAIL_MAX = 600


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["web", "spawn", "save"], default="web")
    ap.add_argument("--out", default="pose_log.rrd")
    args = ap.parse_args()

    rr.init("pika_tracker")
    if args.mode == "web":
        uri = rr.serve_grpc()
        rr.serve_web_viewer(connect_to=uri, open_browser=True)
        print(f"웹 뷰어 서빙 중 (gRPC: {uri}). 브라우저가 안 열리면 콘솔의 http URL로 접속하세요.")
    elif args.mode == "spawn":
        rr.spawn()
    else:
        rr.save(args.out)
        print(f"{args.out} 에 기록 중. 종료 후 'rerun {args.out}' 로 열기.")

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)
    reader = PoseSteamVR(target_hz=TARGET_HZ).connect()
    trail = []
    print("트래커를 움직여 보세요. Ctrl-C 종료.")
    try:
        while True:
            pose = reader.get_pose()
            if isinstance(pose, dict) and pose and "position" not in pose:
                pose = next(iter(pose.values()), None)
            if pose and pose.get("valid"):
                pos = pose["position"]
                quat = pose["rotation"]  # xyzw
                rr.log("world/tracker",
                       rr.Transform3D(translation=pos, quaternion=rr.Quaternion(xyzw=quat)))
                rr.log("world/tracker/axes",
                       rr.Arrows3D(origins=[[0, 0, 0]] * 3,
                                   vectors=[[0.15, 0, 0], [0, 0.15, 0], [0, 0, 0.15]],
                                   colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]))
                trail.append(pos)
                if len(trail) > TRAIL_MAX:
                    trail.pop(0)
                rr.log("world/trail", rr.LineStrips3D([trail]))
                rr.log("world/tracker/point", rr.Points3D([pos], radii=0.01))
            time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        pass
    finally:
        reader.disconnect()


if __name__ == "__main__":
    main()
