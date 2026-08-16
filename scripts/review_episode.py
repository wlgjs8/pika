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
import os
import threading
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
                "ai": len(self.arms),
                "name": arm,
                "base": base,
                "keys": keys,
                "pose": np.asarray(h5[f"{base}/pose"][...], np.float64) if f"{base}/pose" in h5 else None,
                "gripper": np.asarray(h5[f"{base}/gripper"][...], np.float64) if f"{base}/gripper" in h5 else None,
                "lut": _depth_lut(_depth_raw_to_mm(h5, base)),
            })
        self.n = int(len(self.ts))

        # 프레임 하나를 그리려면 스트림 수만큼 PNG 를 디코드해야 한다(양팔 3스트림 = 6장,
        # 실측 ~34ms). 그냥 두면 재생 fps 가 디코드 시간에 묶여 30Hz 수집분이 절반 속도로
        # 재생된다 → 표시 중에 다음 프레임을 미리 디코드해 두는 워커를 둔다.
        self._cache = {}                     # (ai, key, idx) -> img
        self._lock = threading.Lock()
        self._want = None                    # 프리페치 목표 프레임
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._prefetch_loop, daemon=True)
        self._worker.start()

    @property
    def duration(self):
        return float(self.ts[-1] - self.ts[0]) if self.n > 1 else 0.0

    @property
    def rate(self):
        """타임스탬프에서 구한 실제 수집 레이트(Hz).

        에피소드 attrs 의 effective_hz 를 쓰지 않는다 — recorder 가 그것을
        `프레임수 / 구간` 으로 계산하는데 간격 개수는 `프레임수-1` 이라 0.4% 정도
        과대평가된다(277프레임/9.241s: attr 29.98 vs 실제 29.87).
        """
        return (self.n - 1) / self.duration if self.n > 1 and self.duration > 0 else 0.0

    def _decode(self, arm, key, idx):
        ds = self.h5.get(f"{arm['base']}/images/{key}")
        if ds is None or idx < 0 or idx >= len(ds):
            return None
        img = _decode_image(ds[idx], key)
        if key.endswith("depth"):
            img = _depth_visualization(img, arm["lut"])
        return img

    def frame_image(self, arm, key, idx):
        ckey = (arm["ai"], key, idx)
        with self._lock:
            if ckey in self._cache:
                return self._cache[ckey]
        img = self._decode(arm, key, idx)      # h5py 읽기는 락 밖에서(워커와 겹쳐도 GIL 하에 안전)
        with self._lock:
            self._cache[ckey] = img
        return img

    def prefetch(self, idx):
        """idx 프레임을 미리 디코드해 두라고 워커에 알린다."""
        with self._lock:
            self._want = idx
            # 현재 위치에서 멀어진 항목은 버려 메모리를 묶어 둔다(프레임 3개분).
            for k in [k for k in self._cache if abs(k[2] - idx) > 2]:
                del self._cache[k]

    def _prefetch_loop(self):
        while not self._stop.wait(0.002):
            with self._lock:
                idx, want = self._want, self._want
            if want is None:
                continue
            for arm in self.arms:
                for key in arm["keys"]:
                    if self._stop.is_set():
                        return
                    with self._lock:
                        if self._want != idx:      # 목표가 바뀌면 즉시 포기
                            break
                        done = (arm["ai"], key, idx) in self._cache
                    if not done:
                        self.frame_image(arm, key, idx)
            with self._lock:
                if self._want == idx:
                    self._want = None              # 이 프레임은 완료

    def close(self):
        self._stop.set()
        self._worker.join(timeout=1.0)
        try:
            self.h5.close()
        except Exception:
            pass


def _stream_title(key):
    for title, k, _alias in STREAMS:
        if k == key:
            return title
    return key


def _compose(ep, idx, cell_w, cell_h, show_help, achieved_fps=0.0,
             speed=1.0, playing=False, fixed_fps=None):
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
    rec_txt = (f"고정 {fixed_fps:.1f}fps 기준" if fixed_fps
               else f"수집 {ep.rate:.2f}Hz 기준")
    lines = [
        f"{ep.session}/{ep.name}   frame {idx + 1}/{ep.n}   "
        f"t={ep.ts[idx] - ep.ts[0]:6.2f}s / {ep.duration:.2f}s   "
        f"{ep.attrs.get('pose_frame', '?')} ({ep.attrs.get('pose_backend', 'backend?')})",
        # 실제 fps 는 재생 중에만 의미가 있다(정지 중 값은 UI 루프 속도일 뿐).
        f"{'[재생]' if playing else '[정지]'} 배속 x{speed:.2f}   {rec_txt}   "
        + (f"실제 {achieved_fps:4.1f}fps" if playing else "실제 --"),
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
    help_line = None
    if show_help:
        help_line = len(lines)
        lines.append("space 재생/정지  . , 프레임  d a 에피소드  + - 속도  g e 처음/끝  s 저장  h 도움말  q 종료")

    bar = np.zeros((18 * len(lines) + 8, grid.shape[1], 3), np.uint8)
    for i, text in enumerate(lines):
        if i == 0:
            color = (255, 255, 255)                                  # 파일/프레임
        elif i == 1:
            color = (110, 230, 255) if playing else (150, 190, 210)  # 재생 상태·배속(호박색/회색)
        elif i == help_line:
            color = (150, 150, 150)                                  # 도움말
        else:
            color = (185, 225, 185)                                  # 팔별 pose/gripper
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
    ap.add_argument("--fps", type=float, default=None,
                    help="재생 fps 를 이 값으로 고정. 기본은 파일의 타임스탬프를 그대로 "
                         "따라 수집 당시 속도로 재생(정확한 1배속)")
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

    def et(i):
        """프레임 i 의 에피소드 내 경과 시간(초).

        기본은 **파일에 기록된 타임스탬프**를 그대로 쓴다 → 수집 당시 속도로 정확히 1배속
        재생된다(이 리그 실측 프레임간격 33.46ms = 29.87Hz, 명목 30 과 미세하게 다르다).
        --fps 를 주면 그 값으로 균일 간격을 만들어 쓴다.
        """
        i = max(0, min(int(i), ep.n - 1))
        return i / a.fps if a.fps else float(ep.ts[i] - ep.ts[0])

    # 재생 기준점(앵커). 재생 시작·탐색·에피소드 전환·배속 변경마다 다시 잡는다.
    # 이걸 안 하면 일시정지 동안 흐른 시간이 그대로 '밀린 재생분'으로 남아, 재생을 누른
    # 순간 밀린 만큼 프레임을 몰아서 넘기며 고속 되감기처럼 튄다.
    anchor_wall = time.perf_counter()
    anchor_t = et(idx)

    def reanchor():
        nonlocal anchor_wall, anchor_t
        anchor_wall, anchor_t = time.perf_counter(), et(idx)

    shown = anchor_wall
    achieved = 0.0
    prev_idx, prev_playing, prev_speed = idx, playing, speed
    try:
        while True:
            # 프레임/배속/재생상태가 바깥에서 바뀌었으면 앵커를 다시 잡는다(트랙바 포함).
            if idx != prev_idx or playing != prev_playing or speed != prev_speed:
                reanchor()
                prev_idx, prev_playing, prev_speed = idx, playing, speed

            ep.prefetch(idx + 1 if playing else idx)   # 표시 중에 다음 프레임 디코드
            canvas = _compose(ep, idx, a.cell_width, a.cell_height, show_help,
                              achieved, speed, playing, a.fps)
            cv2.imshow(WINDOW, canvas)

            now = time.perf_counter()
            if playing:   # 정지 중 루프 속도가 재생 fps 로 새어들지 않게 재생 중에만 갱신
                inst = 1.0 / max(1e-6, now - shown)
                achieved = 0.8 * achieved + 0.2 * inst if achieved else inst
            shown = now
            # 다음 프레임이 나와야 할 벽시계 시각까지만 기다린다(렌더에 쓴 시간은 자동으로
            # 빠진다). 뒤처졌으면 음수가 되어 1ms 로 클램프 → 프레임을 건너뛰며 따라잡는다.
            if playing and idx + 1 < ep.n:
                due = anchor_wall + (et(idx + 1) - anchor_t) / speed
                wait_ms = int((due - now) * 1000)
            else:
                wait_ms = 20
            key = cv2.waitKey(max(1, wait_ms)) & 0xFF

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
            elif key in (ord("+"), ord("=")):     # 배속은 상태줄에 표시되므로 로그 불필요
                speed = min(speed * 1.5, 16.0)
            elif key == ord("-"):
                speed = max(speed / 1.5, 0.06)
            elif key == ord("h"):
                show_help = not show_help
            elif key == ord("s"):
                out = os.path.join(os.path.dirname(ep.path),
                                   f"{os.path.splitext(ep.name)[0]}_f{idx:05d}.png")
                cv2.imwrite(out, canvas)
                _log(f"saved {out}")

            if playing:
                # 앵커 기준 '지금 보여야 할' 에피소드 시각까지 프레임을 진행한다.
                # 렌더가 느려 뒤처지면 프레임을 건너뛸 뿐, 시간축은 늘어나지 않는다.
                target_t = anchor_t + (time.perf_counter() - anchor_wall) * speed
                nxt = idx
                while nxt + 1 < ep.n and et(nxt + 1) <= target_t:
                    nxt += 1
                if nxt != idx:
                    idx = prev_idx = nxt          # 여기서 진행한 것은 재앵커 대상이 아니다
                elif idx + 1 >= ep.n and target_t > et(ep.n - 1):
                    if ep_idx + 1 < len(paths):
                        open_episode(ep_idx + 1)  # 다음 에피소드로 이어서
                        prev_idx = idx
                        reanchor()
                    else:
                        playing = prev_playing = False   # 마지막에서 정지

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
