#!/usr/bin/env python
"""수집 에피소드를 로컬 OpenCV 창에서 프레임 단위로 검수.

HDF5 에서 직접 디코드해 띄운다 — 예전 HTML 리뷰어처럼 JPEG 에셋을 디스크로 추출하거나
HTTP 서버를 띄우지 않는다(13 에피소드 빌드에 2분 이상 걸리던 단계가 사라진다).

기본 실행: **최신 세션의 첫 에피소드**에서 시작하고, 전 세션이 시간순으로 이어져 있어
이전 에피소드(a)로 계속 넘어가면 세션 경계를 넘어 직전 세션의 마지막 에피소드로 간다.
  .venv/bin/python scripts/review_episode.py

  --episode <path.hdf5>   에피소드 하나만
  --session <data_* dir>  그 세션으로 한정(세션 경계를 넘지 않음)
  --arms left,right       표시할 팔 (기본: 파일에 있는 전부)
  --streams color,fisheye,depth

조작키
  space      재생/일시정지          .  ,     다음/이전 프레임
  d  a       다음/이전 에피소드      + -     재생 속도
  g          첫 프레임으로           e       마지막 프레임으로
  s          현재 화면 PNG 저장      h       도움말 토글
  q / ESC    종료
  트랙바로 프레임 탐색 가능
"""
import argparse
import glob
import math
import os
import re
import sys
import time

import cv2
import h5py
import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_ROOT = os.path.join(REPO_ROOT, "data")

WINDOW = "pika episode review"

# (표시 이름, HDF5 데이터셋 키, 짧은 별칭)
STREAMS = (
    ("D405 Color", "realsense_color", "color"),
    ("Fisheye", "fisheye_color", "fisheye"),
    ("Depth 0-1000mm", "realsense_depth", "depth"),
)

FONT = cv2.FONT_HERSHEY_SIMPLEX


def _log(message):
    print(f"[review] {message}", flush=True)


def _episode_paths_for_session(session_dir):
    paths = sorted(glob.glob(os.path.join(session_dir, "episode_*.hdf5")))
    if not paths:
        raise FileNotFoundError(f"No episode_*.hdf5 files under {session_dir}")
    return paths


def _session_dirs():
    """에피소드가 있는 data_* 세션 전부, 오래된 것부터(이름이 data_YYYYMMDD_HHMMSS)."""
    dirs = [
        path for path in glob.glob(os.path.join(DATA_ROOT, "data_*"))
        if os.path.isdir(path) and glob.glob(os.path.join(path, "episode_*.hdf5"))
    ]
    if not dirs:
        raise FileNotFoundError(f"No data_* folders with episode_*.hdf5 under {DATA_ROOT}")
    return sorted(dirs)


def _all_episode_paths():
    """(전체 에피소드 경로, 최신 세션의 첫 에피소드 인덱스).

    전 세션을 시간순 한 줄로 이어 붙이되 **최신 세션의 첫 에피소드에서 시작**한다.
    그래서 이전 에피소드(a)로 계속 넘어가면 세션 경계를 넘어 직전 세션의 마지막
    에피소드로 이어진다.
    """
    paths, start = [], 0
    sessions = _session_dirs()
    for i, session in enumerate(sessions):
        if i == len(sessions) - 1:
            start = len(paths)          # 최신 세션이 시작점
        paths.extend(_episode_paths_for_session(session))
    return paths, start


def _attr_text(value):
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def _arm_names(h5):
    names = [x.strip() for x in _attr_text(h5.attrs.get("arm_names", "")).split(",") if x.strip()]
    if names:
        return names
    n_arms = int(h5.attrs.get("n_arms", 1))
    return ["arm"] if n_arms <= 1 else [f"arm{i}" for i in range(n_arms)]


def _arm_base(h5, arm):
    """양팔은 observations/<arm>, 단일팔은 평면 observations 레이아웃."""
    grouped = f"observations/{arm}"
    return grouped if grouped in h5 else "observations"


def _display_arm_order(names):
    ordered = [n for want in ("left", "right") for n in names if n.lower() == want]
    return ordered + [n for n in names if n not in ordered]


def _decode_image(buf, key):
    arr = np.asarray(buf, dtype=np.uint8)
    if arr.size == 0:
        return None
    flag = cv2.IMREAD_UNCHANGED if key.endswith("depth") else cv2.IMREAD_COLOR
    return cv2.imdecode(arr, flag)


def _depth_raw_to_mm(h5, base):
    """저장 depth 원시값 → mm 배율. camera_calib 의 depth_scale(m 단위)에서 얻는다."""
    calib = h5.get(f"{base}/camera_calib")
    if calib is None:
        return 1.0
    try:
        depth_scale = float(calib.attrs.get("depth_scale", 0.0))
    except (TypeError, ValueError):
        return 1.0
    return depth_scale * 1000.0 if depth_scale > 0 else 1.0


def _depth_lut(raw_to_mm):
    depth_mm = np.arange(65536, dtype=np.float32) * float(raw_to_mm)
    return (np.clip(depth_mm, 0.0, 1000.0) * (255.0 / 1000.0)).astype(np.uint8)


def _depth_visualization(depth, lut):
    if depth is None:
        return None
    vis = cv2.applyColorMap(lut[depth], cv2.COLORMAP_JET)
    vis[depth == 0] = 0   # 무효 픽셀은 검게
    return vis


def _fit(img, width, height):
    """비율 유지로 (width, height) 안에 맞추고 남는 곳은 검게 채운다."""
    canvas = np.zeros((height, width, 3), np.uint8)
    if img is None:
        cv2.putText(canvas, "no data", (10, height // 2), FONT, 0.5, (90, 90, 90), 1, cv2.LINE_AA)
        return canvas
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    h, w = img.shape[:2]
    scale = min(width / w, height / h)
    size = (max(1, int(w * scale)), max(1, int(h * scale)))
    resized = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
    y = (height - size[1]) // 2
    x = (width - size[0]) // 2
    canvas[y:y + size[1], x:x + size[0]] = resized
    return canvas


def _label(img, text, color=(230, 230, 230)):
    cv2.rectangle(img, (0, 0), (img.shape[1], 18), (0, 0, 0), -1)
    cv2.putText(img, text, (5, 13), FONT, 0.42, color, 1, cv2.LINE_AA)
    return img


class Episode:
    """에피소드 하나. 이미지는 요청 시점에 HDF5 에서 디코드한다(사전 추출 없음)."""

    def __init__(self, path, want_arms=None, want_streams=None):
        self.path = path
        self.name = os.path.basename(path)
        self.session = os.path.basename(os.path.dirname(path))
        self.h5 = h5py.File(path, "r")
        h5 = self.h5
        if "timestamp" not in h5:
            raise ValueError(f"Missing timestamp dataset: {path}")
        self.ts = np.asarray(h5["timestamp"][...], dtype=np.float64)
        if self.ts.size == 0:
            raise ValueError(f"Episode has no frames: {path}")
        self.attrs = {k: _attr_text(v) for k, v in h5.attrs.items()}

        names = _display_arm_order(_arm_names(h5))
        if want_arms:
            wanted = {a.lower() for a in want_arms}
            names = [n for n in names if n.lower() in wanted]
        self.arms = []
        for arm in names:
            base = _arm_base(h5, arm)
            images = h5.get(f"{base}/images")
            keys = [k for _, k, alias in STREAMS
                    if images is not None and k in images
                    and (not want_streams or alias in want_streams)]
            self.arms.append({
                "name": arm,
                "base": base,
                "keys": keys,
                "pose": np.asarray(h5[f"{base}/pose"][...], np.float64) if f"{base}/pose" in h5 else None,
                "gripper": np.asarray(h5[f"{base}/gripper"][...], np.float64) if f"{base}/gripper" in h5 else None,
                "lut": _depth_lut(_depth_raw_to_mm(h5, base)),
            })
        self.n = int(len(self.ts))

    @property
    def duration(self):
        return float(self.ts[-1] - self.ts[0]) if self.n > 1 else 0.0

    def frame_image(self, arm, key, idx):
        ds = self.h5.get(f"{arm['base']}/images/{key}")
        if ds is None or idx >= len(ds):
            return None
        img = _decode_image(ds[idx], key)
        if key.endswith("depth"):
            img = _depth_visualization(img, arm["lut"])
        return img

    def close(self):
        try:
            self.h5.close()
        except Exception:
            pass


def _stream_title(key):
    for title, k, _alias in STREAMS:
        if k == key:
            return title
    return key


def _compose(ep, idx, cell_w, cell_h, show_help):
    """팔=행, 스트림=열 격자 + 하단 상태줄."""
    rows = []
    for arm in ep.arms:
        cells = []
        for key in arm["keys"]:
            cell = _fit(ep.frame_image(arm, key, idx), cell_w, cell_h)
            cells.append(_label(cell, f"{arm['name']}  {_stream_title(key)}"))
        if not cells:
            cells = [_label(_fit(None, cell_w, cell_h), f"{arm['name']}  (이미지 없음)")]
        rows.append(np.hstack(cells))
    width = max(r.shape[1] for r in rows)
    rows = [np.pad(r, ((0, 0), (0, width - r.shape[1]), (0, 0))) for r in rows]
    grid = np.vstack(rows)

    # ---- 상태줄: 팔별 pose/gripper ----
    lines = [
        f"{ep.session}/{ep.name}   frame {idx + 1}/{ep.n}   "
        f"t={ep.ts[idx] - ep.ts[0]:6.2f}s / {ep.duration:.2f}s   "
        f"{ep.attrs.get('pose_frame', '?')} ({ep.attrs.get('pose_backend', 'backend?')})"
    ]
    for arm in ep.arms:
        pose, grip = arm["pose"], arm["gripper"]
        parts = [f"{arm['name']:>5s}"]
        if pose is not None and idx < len(pose) and pose.ndim == 2 and pose.shape[1] >= 7:
            p = pose[idx]
            ok = bool(np.all(np.isfinite(p)))
            parts.append(f"pos=({p[0]:7.4f},{p[1]:7.4f},{p[2]:7.4f})"
                         f" quat=({p[3]:6.3f},{p[4]:6.3f},{p[5]:6.3f},{p[6]:6.3f})"
                         if ok else "pose=INVALID")
        if grip is not None and idx < len(grip):
            g = np.atleast_1d(grip[idx])
            parts.append("grip=" + ",".join(f"{v:.3f}" for v in g))
        lines.append("   ".join(parts))
    if show_help:
        lines.append("space 재생/정지  . , 프레임  d a 에피소드  + - 속도  g e 처음/끝  s 저장  h 도움말  q 종료")

    bar = np.zeros((18 * len(lines) + 8, grid.shape[1], 3), np.uint8)
    for i, text in enumerate(lines):
        color = (255, 255, 255) if i == 0 else (185, 225, 185)
        cv2.putText(bar, text, (8, 16 + i * 18), FONT, 0.44, color, 1, cv2.LINE_AA)
    return np.vstack([grid, bar])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--episode", help="에피소드 HDF5 하나만 검수")
    ap.add_argument("--session",
                    help="이 data_* 폴더로 한정. 기본은 전 세션을 시간순으로 잇고 "
                         "최신 세션의 첫 에피소드에서 시작(a 로 이전 세션까지 거슬러 감)")
    ap.add_argument("--arms", help="표시할 팔(쉼표). 기본: 파일에 있는 전부")
    ap.add_argument("--streams", help="표시할 스트림(쉼표): color,fisheye,depth. 기본: 전부")
    ap.add_argument("--cell-width", type=int, default=420, help="셀 하나의 가로 픽셀")
    ap.add_argument("--cell-height", type=int, default=315, help="셀 하나의 세로 픽셀")
    ap.add_argument("--fps", type=float, default=30.0, help="재생 fps")
    ap.add_argument("--start-paused", action="store_true", default=True,
                    help="일시정지 상태로 시작(기본)")
    ap.add_argument("--play", dest="start_paused", action="store_false",
                    help="바로 재생 시작")
    a = ap.parse_args()

    if a.episode:
        paths, start = [a.episode], 0
    elif a.session:
        paths, start = _episode_paths_for_session(a.session), 0
    else:
        # 전 세션을 시간순으로 잇고 최신 세션의 첫 에피소드에서 시작한다.
        paths, start = _all_episode_paths()
    want_arms = [x.strip() for x in a.arms.split(",")] if a.arms else None
    want_streams = {x.strip() for x in a.streams.split(",")} if a.streams else None

    sessions = sorted({os.path.basename(os.path.dirname(p)) for p in paths})
    _log(f"{len(paths)} episode(s) across {len(sessions)} session(s): "
         f"{sessions[0]}{' … ' + sessions[-1] if len(sessions) > 1 else ''}")
    if start:
        _log(f"최신 세션의 첫 에피소드({start + 1}/{len(paths)})에서 시작 — "
             f"a 를 계속 누르면 이전 세션으로 거슬러 갑니다")

    ep_idx, ep, idx = start, None, 0
    playing = not a.start_paused
    speed = 1.0
    show_help = True
    seeking = {"user": False}

    def open_episode(i):
        nonlocal ep, ep_idx, idx
        if ep is not None:
            ep.close()
        ep_idx = i % len(paths)
        ep = Episode(paths[ep_idx], want_arms, want_streams)
        idx = 0
        _log(f"[{ep_idx + 1}/{len(paths)}] {ep.session}/{ep.name}  frames={ep.n}  "
             f"{ep.duration:.2f}s  arms={[x['name'] for x in ep.arms]}")
        cv2.setTrackbarMax("frame", WINDOW, max(1, ep.n - 1))
        cv2.setTrackbarPos("frame", WINDOW, 0)

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)

    def on_track(pos):
        nonlocal idx
        if not seeking["user"]:
            idx = max(0, min(pos, ep.n - 1)) if ep else 0

    cv2.createTrackbar("frame", WINDOW, 0, 1, on_track)
    open_episode(start)

    last = time.perf_counter()
    try:
        while True:
            canvas = _compose(ep, idx, a.cell_width, a.cell_height, show_help)
            cv2.imshow(WINDOW, canvas)

            period = 1.0 / max(1e-3, a.fps * speed)
            key = cv2.waitKey(max(1, int(period * 1000)) if playing else 20) & 0xFF

            if key in (ord("q"), 27):
                break
            elif key == ord(" "):
                playing = not playing
            elif key in (ord("."), 83):        # . 또는 →
                playing, idx = False, min(idx + 1, ep.n - 1)
            elif key in (ord(","), 81):        # , 또는 ←
                playing, idx = False, max(idx - 1, 0)
            elif key == ord("d"):
                open_episode(ep_idx + 1)
            elif key == ord("a"):
                open_episode(ep_idx - 1)
            elif key == ord("g"):
                idx = 0
            elif key == ord("e"):
                idx = ep.n - 1
            elif key in (ord("+"), ord("=")):
                speed = min(speed * 1.5, 16.0)
                _log(f"speed x{speed:.2f}")
            elif key == ord("-"):
                speed = max(speed / 1.5, 0.06)
                _log(f"speed x{speed:.2f}")
            elif key == ord("h"):
                show_help = not show_help
            elif key == ord("s"):
                out = os.path.join(os.path.dirname(ep.path),
                                   f"{os.path.splitext(ep.name)[0]}_f{idx:05d}.png")
                cv2.imwrite(out, canvas)
                _log(f"saved {out}")

            if playing:
                now = time.perf_counter()
                if now - last >= period:
                    last = now
                    if idx + 1 < ep.n:
                        idx += 1
                    elif ep_idx + 1 < len(paths):
                        open_episode(ep_idx + 1)   # 다음 에피소드로 이어서
                    else:
                        playing = False            # 마지막에서 정지

            seeking["user"] = True
            cv2.setTrackbarPos("frame", WINDOW, idx)
            seeking["user"] = False

            # 창을 닫으면 종료
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if ep is not None:
            ep.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
