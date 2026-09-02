#!/usr/bin/env python
"""양팔 에피소드의 RealSense RGB 두 뷰를 원본 해상도로 좌우 재생.

왼쪽 카메라는 화면 왼쪽, 오른쪽 카메라는 화면 오른쪽에 고정한다.
RGB 프레임에는 리사이즈나 오버레이를 적용하지 않고, 에피소드/재생 정보는 영상
아래 상태줄에만 쓴다.

기본 실행: 최신 세션의 첫 에피소드에서 시작
  .venv/bin/python scripts/review_rgb_episode.py

  --episode <path.hdf5>   에피소드 하나만
  --session <data_* dir>  그 세션으로 한정
  --fps <hz>              저장 타임스탬프 대신 고정 fps로 재생
  --play                  일시정지하지 않고 바로 재생

조작키
  space      재생/일시정지          .  ,     다음/이전 프레임
  d  a       다음/이전 에피소드      + -     재생 속도
  g          첫 프레임으로           e       마지막 프레임으로
  s          현재 화면 PNG 저장      h       도움말 토글
  q / ESC    종료
  트랙바로 프레임 탐색 가능
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

from review_episode import Episode, _all_episode_paths, _episode_paths_for_session


WINDOW = "pika RGB episode review"
STREAM_KEY = "realsense_color"
ARM_ORDER = ("left", "right")
FONT = cv2.FONT_HERSHEY_SIMPLEX


def _log(message):
    print(f"[rgb-review] {message}", flush=True)


def _rgb_arms(ep):
    """Episode arm 목록을 화면 순서(left, right)로 반환하고 RGB 존재를 검증한다."""
    by_name = {arm["name"].lower(): arm for arm in ep.arms}
    missing_arms = [name for name in ARM_ORDER if name not in by_name]
    if missing_arms:
        raise ValueError(
            f"{ep.path}: RGB pair viewer needs arms {ARM_ORDER}; "
            f"missing arm(s): {', '.join(missing_arms)}"
        )

    ordered = [by_name[name] for name in ARM_ORDER]
    missing_rgb = [arm["name"] for arm in ordered if STREAM_KEY not in arm["keys"]]
    if missing_rgb:
        paths = ", ".join(
            f"{arm['base']}/images/{STREAM_KEY}"
            for arm in ordered
            if STREAM_KEY not in arm["keys"]
        )
        raise ValueError(f"{ep.path}: missing RGB dataset(s) for {missing_rgb}: {paths}")
    return ordered


def _native_rgb_pair(ep, idx):
    """두 RGB를 리사이즈 없이 left | right 순으로 합친다.

    높이가 다른 파일도 원본 픽셀을 유지하도록 작은 쪽의 위아래에 검은 여백만
    둔다. 반환하는 left_width는 하단 LEFT/RIGHT 라벨 정렬에 사용한다.
    """
    images = []
    for arm in _rgb_arms(ep):
        image = ep.frame_image(arm, STREAM_KEY, idx)
        if image is None:
            raise ValueError(
                f"{ep.path}: failed to decode {arm['name']} RGB at frame {idx} "
                f"({arm['base']}/images/{STREAM_KEY})"
            )
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(
                f"{ep.path}: expected 3-channel RGB at frame {idx}, "
                f"got shape={image.shape} for {arm['name']}"
            )
        images.append(image)

    height = max(image.shape[0] for image in images)
    widths = [image.shape[1] for image in images]
    pair = np.zeros((height, sum(widths), 3), dtype=np.uint8)
    x = 0
    for image in images:
        h, w = image.shape[:2]
        y = (height - h) // 2
        pair[y:y + h, x:x + w] = image
        x += w
    return pair, widths[0]


def _compose(ep, idx, show_help, achieved_fps=0.0, speed=1.0,
             playing=False, fixed_fps=None):
    """원본 RGB pair 아래에만 상태줄을 붙인다."""
    pair, left_width = _native_rgb_pair(ep, idx)
    line_height = 20
    lines = 3 if show_help else 2
    bar = np.zeros((line_height * lines + 6, pair.shape[1], 3), np.uint8)

    cv2.putText(bar, "LEFT", (8, 16), FONT, 0.48, (185, 225, 185), 1, cv2.LINE_AA)
    cv2.putText(bar, "RIGHT", (left_width + 8, 16), FONT, 0.48,
                (185, 225, 185), 1, cv2.LINE_AA)

    source_rate = f"fixed {fixed_fps:.1f} fps" if fixed_fps else f"source {ep.rate:.2f} Hz"
    actual = f"actual {achieved_fps:.1f} fps" if playing else "actual --"
    status = (
        f"{ep.session}/{ep.name}  frame {idx + 1}/{ep.n}  "
        f"t={ep.ts[idx] - ep.ts[0]:.2f}/{ep.duration:.2f}s  "
        f"{'PLAY' if playing else 'PAUSE'} x{speed:.2f}  {source_rate}  {actual}"
    )
    cv2.putText(bar, status, (8, 36), FONT, 0.43, (230, 230, 230), 1, cv2.LINE_AA)

    if show_help:
        help_text = (
            "space play/pause  ,/. frame  a/d episode  +/- speed  "
            "g/e first/last  s save  h help  q quit"
        )
        cv2.putText(bar, help_text, (8, 56), FONT, 0.40,
                    (150, 150, 150), 1, cv2.LINE_AA)
    return np.vstack([pair, bar])


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--episode", help="에피소드 HDF5 하나만 검수")
    target.add_argument(
        "--session",
        help="이 data_* 폴더로 한정. 기본은 전 세션을 시간순으로 잇고 "
             "최신 세션의 첫 에피소드에서 시작",
    )
    parser.add_argument(
        "--fps", type=float, default=None,
        help="재생 fps를 고정. 기본은 저장 타임스탬프를 따라 "
             "수집 당시 속도로 재생",
    )
    parser.add_argument("--play", action="store_true", help="바로 재생 시작(기본: 일시정지)")
    args = parser.parse_args(argv)
    if args.fps is not None and args.fps <= 0:
        parser.error("--fps must be greater than zero")
    return args


def main(argv=None):
    args = _parse_args(argv)
    if args.episode:
        paths, start = [args.episode], 0
    elif args.session:
        paths, start = _episode_paths_for_session(args.session), 0
    else:
        paths, start = _all_episode_paths()

    sessions = sorted({os.path.basename(os.path.dirname(path)) for path in paths})
    _log(
        f"{len(paths)} episode(s) across {len(sessions)} session(s): "
        f"{sessions[0]}{' ... ' + sessions[-1] if len(sessions) > 1 else ''}"
    )
    if start:
        _log(
            f"starting at first episode of latest session ({start + 1}/{len(paths)}); "
            "press a to review earlier sessions"
        )

    ep_idx, ep, idx = start, None, 0
    playing = args.play
    speed = 1.0
    show_help = True
    moving_trackbar = {"programmatic": False}

    def open_episode(i):
        nonlocal ep, ep_idx, idx
        next_idx = i % len(paths)
        new_ep = Episode(paths[next_idx], want_arms=list(ARM_ORDER), want_streams={"color"})
        try:
            _rgb_arms(new_ep)
            # 창을 띄우기 전에 첫 프레임까지 검증해 오류 경로를 분명하게 한다.
            _native_rgb_pair(new_ep, 0)
        except Exception:
            new_ep.close()
            raise
        if ep is not None:
            ep.close()
        ep = new_ep
        ep_idx = next_idx
        idx = 0
        _log(
            f"[{ep_idx + 1}/{len(paths)}] {ep.session}/{ep.name}  "
            f"frames={ep.n}  duration={ep.duration:.2f}s  layout=left|right native"
        )
        cv2.setTrackbarMax("frame", WINDOW, max(1, ep.n - 1))
        cv2.setTrackbarPos("frame", WINDOW, 0)

    # AUTOSIZE는 합성된 원본 픽셀 크기로 창의 영상 영역을 고정한다.
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)

    def on_track(position):
        nonlocal idx
        if not moving_trackbar["programmatic"]:
            idx = max(0, min(position, ep.n - 1)) if ep else 0

    cv2.createTrackbar("frame", WINDOW, 0, 1, on_track)
    open_episode(start)

    def episode_time(i):
        i = max(0, min(int(i), ep.n - 1))
        return i / args.fps if args.fps else float(ep.ts[i] - ep.ts[0])

    anchor_wall = time.perf_counter()
    anchor_t = episode_time(idx)

    def reanchor():
        nonlocal anchor_wall, anchor_t
        anchor_wall, anchor_t = time.perf_counter(), episode_time(idx)

    shown = anchor_wall
    achieved = 0.0
    prev_idx, prev_playing, prev_speed = idx, playing, speed
    try:
        while True:
            if idx != prev_idx or playing != prev_playing or speed != prev_speed:
                reanchor()
                prev_idx, prev_playing, prev_speed = idx, playing, speed

            ep.prefetch(idx + 1 if playing else idx)
            canvas = _compose(
                ep, idx, show_help, achieved, speed, playing, args.fps)
            cv2.imshow(WINDOW, canvas)

            now = time.perf_counter()
            if playing:
                instant = 1.0 / max(1e-6, now - shown)
                achieved = 0.8 * achieved + 0.2 * instant if achieved else instant
            shown = now
            if playing and idx + 1 < ep.n:
                due = anchor_wall + (episode_time(idx + 1) - anchor_t) / speed
                wait_ms = int((due - now) * 1000)
            else:
                wait_ms = 20
            key = cv2.waitKey(max(1, wait_ms)) & 0xFF

            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                playing = not playing
            elif key in (ord("."), 83):
                playing, idx = False, min(idx + 1, ep.n - 1)
            elif key in (ord(","), 81):
                playing, idx = False, max(idx - 1, 0)
            elif key == ord("d"):
                open_episode(ep_idx + 1)
                prev_idx = idx
                reanchor()
            elif key == ord("a"):
                open_episode(ep_idx - 1)
                prev_idx = idx
                reanchor()
            elif key == ord("g"):
                idx = 0
            elif key == ord("e"):
                idx = ep.n - 1
            elif key in (ord("+"), ord("=")):
                speed = min(speed * 1.5, 16.0)
            elif key == ord("-"):
                speed = max(speed / 1.5, 0.06)
            elif key == ord("h"):
                show_help = not show_help
            elif key == ord("s"):
                output = os.path.join(
                    os.path.dirname(ep.path),
                    f"{os.path.splitext(ep.name)[0]}_rgb_f{idx:05d}.png",
                )
                if not cv2.imwrite(output, canvas):
                    raise OSError(f"failed to save screenshot: {output}")
                _log(f"saved {output}")

            # waitKey/트랙바에서 바뀐 상태를 같은 루프의 재생 진행보다 먼저
            # 반영한다. 특히 긴 일시정지 뒤 space를 눌렀을 때 정지 시간이 밀린
            # 재생분으로 계산돼 프레임을 한꺼번에 건너뛰지 않게 한다.
            if idx != prev_idx or playing != prev_playing or speed != prev_speed:
                reanchor()
                prev_idx, prev_playing, prev_speed = idx, playing, speed

            if playing:
                target_t = anchor_t + (time.perf_counter() - anchor_wall) * speed
                next_frame = idx
                while next_frame + 1 < ep.n and episode_time(next_frame + 1) <= target_t:
                    next_frame += 1
                if next_frame != idx:
                    idx = prev_idx = next_frame
                elif idx + 1 >= ep.n and target_t > episode_time(ep.n - 1):
                    if ep_idx + 1 < len(paths):
                        open_episode(ep_idx + 1)
                        prev_idx = idx
                        reanchor()
                    else:
                        playing = prev_playing = False

            moving_trackbar["programmatic"] = True
            cv2.setTrackbarPos("frame", WINDOW, idx)
            moving_trackbar["programmatic"] = False

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
    try:
        raise SystemExit(main())
    except (FileNotFoundError, OSError, RuntimeError, ValueError) as exc:
        print(f"[rgb-review] ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
