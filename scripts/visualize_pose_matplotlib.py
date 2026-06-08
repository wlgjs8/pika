#!/usr/bin/env python
"""matplotlib 기반 실시간 트래커 포즈 3D 시각화 — rerun 뷰어가 안 될 때의 확실한 대안.

네이티브 GUI 창(TkAgg)을 사용하므로 wgpu/그래픽 백엔드 문제 영향이 적다.
실행: conda run -n pika python scripts\\visualize_pose_matplotlib.py
종료: 창 닫기 또는 Ctrl-C
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
import matplotlib  # noqa: E402
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt  # noqa: E402

from pika_win.pose_steamvr import PoseSteamVR, quat_rotate_vec  # noqa: E402

AXES = [((0.15, 0, 0), "r"), ((0, 0.15, 0), "g"), ((0, 0, 0.15), "b")]  # X,Y,Z
TRAIL_MAX = 400
VIEW_RADIUS = 0.5  # m, 트래커 주변 표시 범위


def main():
    reader = PoseSteamVR(target_hz=120).connect()
    plt.ion()
    fig = plt.figure("PIKA tracker pose")
    ax = fig.add_subplot(111, projection="3d")
    trail = []
    print("창을 닫거나 Ctrl-C 로 종료.")
    try:
        while plt.fignum_exists(fig.number):
            pose = reader.get_pose()
            if isinstance(pose, dict) and pose and "position" not in pose:
                pose = next(iter(pose.values()), None)
            if pose and pose.get("valid"):
                p = np.array(pose["position"], dtype=float)
                q = pose["rotation"]
                trail.append(p)
                if len(trail) > TRAIL_MAX:
                    trail.pop(0)
                ax.cla()
                for vec, c in AXES:                       # 트래커 좌표축
                    d = np.array(quat_rotate_vec(q, vec))
                    ax.plot([p[0], p[0] + d[0]], [p[1], p[1] + d[1]],
                            [p[2], p[2] + d[2]], c + "-", lw=2)
                t = np.array(trail)                        # 궤적
                ax.plot(t[:, 0], t[:, 1], t[:, 2], "k-", lw=0.8, alpha=0.6)
                ax.scatter([p[0]], [p[1]], [p[2]], c="m", s=20)
                ax.set_xlim(p[0] - VIEW_RADIUS, p[0] + VIEW_RADIUS)
                ax.set_ylim(p[1] - VIEW_RADIUS, p[1] + VIEW_RADIUS)
                ax.set_zlim(p[2] - VIEW_RADIUS, p[2] + VIEW_RADIUS)
                ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
                ax.set_title(f"pos=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})  hz={reader.effective_hz:.0f}")
            plt.pause(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        reader.disconnect()


if __name__ == "__main__":
    main()
