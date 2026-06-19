#!/usr/bin/env python
"""Generate a frame-by-frame HTML review page for collected episodes.

Default usage:
  conda run --no-capture-output -n pika python scripts/review_episode.py

The script picks the newest data/data_* session folder, extracts color frames
for every episode in that folder, and serves an HTML viewer for checking image,
pose, and gripper values per episode and per frame.
"""
import argparse
import glob
import html
import json
import math
import os
import posixpath
import re
import shutil
import socket
import sys
import threading
import time
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import cv2
import h5py
import numpy as np


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_ROOT = os.path.join(REPO_ROOT, "data")


def _log(message):
    print(f"[review] {message}", flush=True)


def _latest_episode():
    paths = glob.glob(os.path.join(DATA_ROOT, "data_*", "episode_*.hdf5"))
    if not paths:
        raise FileNotFoundError(f"No episode_*.hdf5 files under {DATA_ROOT}")
    return max(paths, key=os.path.getmtime)


def _latest_session_dir():
    dirs = [
        path for path in glob.glob(os.path.join(DATA_ROOT, "data_*"))
        if os.path.isdir(path) and glob.glob(os.path.join(path, "episode_*.hdf5"))
    ]
    if not dirs:
        raise FileNotFoundError(f"No data_* folders with episode_*.hdf5 under {DATA_ROOT}")
    return sorted(dirs)[-1]


def _episode_paths_for_session(session_dir):
    paths = sorted(glob.glob(os.path.join(session_dir, "episode_*.hdf5")))
    if not paths:
        raise FileNotFoundError(f"No episode_*.hdf5 files under {session_dir}")
    return paths


def _safe_name(value):
    value = str(value or "arm")
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_") or "arm"


def _attr_text(value):
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def _arm_names(h5):
    raw = h5.attrs.get("arm_names", "")
    text = _attr_text(raw)
    names = [x.strip() for x in text.split(",") if x.strip()]
    if names:
        return names
    n_arms = int(h5.attrs.get("n_arms", 1))
    return ["arm"] if n_arms <= 1 else [f"arm{i}" for i in range(n_arms)]


def _arm_base(h5, arm):
    grouped = f"observations/{arm}"
    if grouped in h5:
        return grouped
    return "observations"


def _display_arm_order(names):
    ordered = []
    for wanted in ("left", "right"):
        for name in names:
            if name.lower() == wanted and name not in ordered:
                ordered.append(name)
    for name in names:
        if name not in ordered:
            ordered.append(name)
    return ordered


def _image_source_arm(display_arm, names, swap_arm_images):
    if not swap_arm_images:
        return display_arm
    lookup = {name.lower(): name for name in names}
    if "left" not in lookup or "right" not in lookup:
        return display_arm
    if display_arm.lower() == "left":
        return lookup["right"]
    if display_arm.lower() == "right":
        return lookup["left"]
    return display_arm


def _finite_or_none(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _matrix_to_jsonable(array):
    out = []
    for row in np.asarray(array):
        out.append([_finite_or_none(v) for v in row])
    return out


def _decode_image(buf, encoding):
    arr = np.asarray(buf, dtype=np.uint8)
    if arr.size == 0:
        return None
    flag = cv2.IMREAD_UNCHANGED if encoding == "png16" else cv2.IMREAD_COLOR
    return cv2.imdecode(arr, flag)


IMAGE_STREAMS = (
    ("realsenseColor", "realsense_color", "D405 Color"),
    ("fisheyeColor", "fisheye_color", "Fisheye"),
    ("realsenseDepth", "realsense_depth", "Depth 0-1000 mm"),
)

DEFAULT_PREVIEW_MAX_SIDE = 320
DEFAULT_PREVIEW_JPEG_QUALITY = 65
DEFAULT_PRELOAD_AHEAD = 1


def _depth_raw_to_mm(h5, source_base):
    calib = h5.get(f"{source_base}/camera_calib")
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
    height, width = depth.shape[:2]
    depth = cv2.resize(
        depth,
        (max(1, width // 2), max(1, height // 2)),
        interpolation=cv2.INTER_NEAREST,
    )
    valid = depth > 0
    gray = lut[depth]
    vis = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
    vis[~valid] = 0
    return vis


def _resize_preview(img, max_side):
    if img is None or max_side <= 0:
        return img
    height, width = img.shape[:2]
    longer = max(height, width)
    if longer <= max_side:
        return img
    scale = float(max_side) / float(longer)
    size = (max(1, int(round(width * scale))), max(1, int(round(height * scale))))
    return cv2.resize(img, size, interpolation=cv2.INTER_AREA)


def _preview_settings(max_side, jpeg_quality, preload_ahead):
    return {
        "format": "jpg",
        "maxSide": int(max_side),
        "jpegQuality": int(jpeg_quality),
        "preloadAhead": int(preload_ahead),
    }


def _write_image_assets(h5, display_arm, source_base, output_dir, asset_prefix, dataset_key,
                        preview_max_side, preview_jpeg_quality):
    img_group = h5.get(f"{source_base}/images")
    if img_group is None or dataset_key not in img_group:
        return None

    ds = img_group[dataset_key]
    encoding = _attr_text(ds.attrs.get("encoding", "jpeg")).lower()
    stream_name = _safe_name(dataset_key)
    arm_dir = os.path.join(output_dir, "assets", _safe_name(asset_prefix), _safe_name(display_arm), stream_name)
    os.makedirs(arm_dir, exist_ok=True)
    urls = []
    is_depth = dataset_key.endswith("depth")
    ext = ".jpg"
    raw_to_mm = _depth_raw_to_mm(h5, source_base) if is_depth else 1.0
    depth_lut = _depth_lut(raw_to_mm) if is_depth else None

    t0 = time.perf_counter()
    written = 0
    _log(f"extracting {asset_prefix}/{display_arm}/{stream_name}: {len(ds)} frames")
    for idx in range(len(ds)):
        buf = np.asarray(ds[idx], dtype=np.uint8)
        if buf.size == 0:
            urls.append(None)
            continue
        name = f"frame_{idx:06d}{ext}"
        path = os.path.join(arm_dir, name)
        img = _decode_image(buf, encoding)
        if img is None:
            urls.append(None)
            continue
        if is_depth:
            img = _depth_visualization(img, depth_lut)
            if img is None:
                urls.append(None)
                continue
        elif img.dtype == np.uint16:
            img = cv2.convertScaleAbs(img, alpha=0.03)
        img = _resize_preview(img, preview_max_side)
        ok = cv2.imwrite(path, img, [int(cv2.IMWRITE_JPEG_QUALITY), int(preview_jpeg_quality)])
        if not ok:
            urls.append(None)
            continue
        written += 1
        urls.append(posixpath.join("assets", _safe_name(asset_prefix), _safe_name(display_arm), stream_name, name))
    _log(f"wrote {asset_prefix}/{display_arm}/{stream_name}: {written}/{len(ds)} frames in {time.perf_counter() - t0:.1f}s")
    return urls


def _write_stream_assets(h5, display_arm, source_base, output_dir, asset_prefix="",
                         preview_max_side=DEFAULT_PREVIEW_MAX_SIDE,
                         preview_jpeg_quality=DEFAULT_PREVIEW_JPEG_QUALITY):
    streams = {}
    labels = {}
    for stream_id, dataset_key, label in IMAGE_STREAMS:
        urls = _write_image_assets(
            h5, display_arm, source_base, output_dir, asset_prefix, dataset_key,
            preview_max_side, preview_jpeg_quality,
        )
        if urls is not None:
            streams[stream_id] = urls
            labels[stream_id] = label
    return streams, labels


def _extract_episode(path, output_dir, swap_arm_images=False, clear_output=True, asset_prefix=None,
                     preview_max_side=DEFAULT_PREVIEW_MAX_SIDE,
                     preview_jpeg_quality=DEFAULT_PREVIEW_JPEG_QUALITY,
                     preload_ahead=DEFAULT_PRELOAD_AHEAD):
    if clear_output and os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(os.path.join(output_dir, "assets"), exist_ok=True)
    asset_prefix = asset_prefix if asset_prefix is not None else os.path.splitext(os.path.basename(path))[0]

    with h5py.File(path, "r") as h5:
        if "timestamp" not in h5:
            raise ValueError(f"Missing timestamp dataset: {path}")
        timestamps = np.asarray(h5["timestamp"][...], dtype=np.float64)
        if timestamps.size == 0:
            raise ValueError(f"Episode has no frames: {path}")

        arm_names = _arm_names(h5)
        arms = []
        for arm in _display_arm_order(arm_names):
            base = _arm_base(h5, arm)
            if f"{base}/pose" not in h5 or f"{base}/gripper" not in h5:
                arms.append({
                    "name": arm,
                    "present": False,
                    "images": [None] * len(timestamps),
                    "streams": {},
                    "streamLabels": {},
                    "pose": [],
                    "gripper": [],
                    "poseValid": 0,
                })
                continue
            pose = np.asarray(h5[f"{base}/pose"][...], dtype=np.float64)
            gripper = np.asarray(h5[f"{base}/gripper"][...], dtype=np.float64)
            image_source_arm = _image_source_arm(arm, arm_names, swap_arm_images)
            image_source_base = _arm_base(h5, image_source_arm)
            streams, stream_labels = _write_stream_assets(
                h5, arm, image_source_base, output_dir, asset_prefix=asset_prefix,
                preview_max_side=preview_max_side,
                preview_jpeg_quality=preview_jpeg_quality,
            )
            pose_valid = int(np.sum(np.isfinite(pose[:, 0]))) if pose.ndim == 2 and pose.shape[0] else 0
            arms.append({
                "name": arm,
                "present": True,
                "images": streams.get("realsenseColor", [None] * len(timestamps)),
                "streams": streams,
                "streamLabels": stream_labels,
                "pose": _matrix_to_jsonable(pose),
                "gripper": _matrix_to_jsonable(gripper),
                "poseValid": pose_valid,
                "imageSource": image_source_arm,
                "attrs": {
                    key: _attr_text(value)
                    for key, value in h5.get(base, {}).attrs.items()
                } if base in h5 else {},
            })

        metadata = {
            "episodePath": os.path.abspath(path),
            "episodeName": os.path.basename(path),
            "sessionName": os.path.basename(os.path.dirname(path)),
            "frameCount": int(len(timestamps)),
            "durationSec": float(timestamps[-1] - timestamps[0]) if len(timestamps) > 1 else 0.0,
            "elapsed": [float(t - timestamps[0]) for t in timestamps],
            "attrs": {key: _attr_text(value) for key, value in h5.attrs.items()},
            "preview": _preview_settings(preview_max_side, preview_jpeg_quality, preload_ahead),
            "arms": arms,
        }
    return metadata


def _session_metadata(paths, episodes, build_complete=True, build_error=None,
                      preview_max_side=DEFAULT_PREVIEW_MAX_SIDE,
                      preview_jpeg_quality=DEFAULT_PREVIEW_JPEG_QUALITY,
                      preload_ahead=DEFAULT_PRELOAD_AHEAD):
    session_dir = os.path.dirname(os.path.abspath(paths[0]))
    return {
        "sessionPath": session_dir,
        "sessionName": os.path.basename(session_dir),
        "episodeCount": len(paths),
        "readyCount": len(episodes),
        "buildComplete": bool(build_complete),
        "buildError": build_error,
        "preview": _preview_settings(preview_max_side, preview_jpeg_quality, preload_ahead),
        "episodes": episodes,
    }


def _write_metadata(metadata, output_dir):
    path = os.path.join(output_dir, "metadata.json")
    tmp_path = f"{path}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, ensure_ascii=False, allow_nan=False)
    os.replace(tmp_path, path)
    return path


def _extract_session_episodes(paths, output_dir, swap_arm_images=False, start_index=1, total_count=None,
                              preview_max_side=DEFAULT_PREVIEW_MAX_SIDE,
                              preview_jpeg_quality=DEFAULT_PREVIEW_JPEG_QUALITY,
                              preload_ahead=DEFAULT_PRELOAD_AHEAD):
    episodes = []
    total_count = total_count or (start_index + len(paths) - 1)
    for offset, path in enumerate(paths):
        idx = start_index + offset
        stem = os.path.splitext(os.path.basename(path))[0]
        _log(f"episode {idx}/{total_count}: {os.path.basename(path)}")
        episodes.append(_extract_episode(
            path,
            output_dir,
            swap_arm_images=swap_arm_images,
            clear_output=False,
            asset_prefix=stem,
            preview_max_side=preview_max_side,
            preview_jpeg_quality=preview_jpeg_quality,
            preload_ahead=preload_ahead,
        ))
    return episodes


def _extract_session(paths, output_dir, swap_arm_images=False,
                     preview_max_side=DEFAULT_PREVIEW_MAX_SIDE,
                     preview_jpeg_quality=DEFAULT_PREVIEW_JPEG_QUALITY,
                     preload_ahead=DEFAULT_PRELOAD_AHEAD):
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(os.path.join(output_dir, "assets"), exist_ok=True)
    episodes = _extract_session_episodes(
        paths,
        output_dir,
        swap_arm_images=swap_arm_images,
        total_count=len(paths),
        preview_max_side=preview_max_side,
        preview_jpeg_quality=preview_jpeg_quality,
        preload_ahead=preload_ahead,
    )
    return _session_metadata(
        paths, episodes, build_complete=True,
        preview_max_side=preview_max_side,
        preview_jpeg_quality=preview_jpeg_quality,
        preload_ahead=preload_ahead,
    )


def _extract_session_initial(paths, output_dir, swap_arm_images=False,
                             preview_max_side=DEFAULT_PREVIEW_MAX_SIDE,
                             preview_jpeg_quality=DEFAULT_PREVIEW_JPEG_QUALITY,
                             preload_ahead=DEFAULT_PRELOAD_AHEAD):
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(os.path.join(output_dir, "assets"), exist_ok=True)
    episodes = _extract_session_episodes(
        paths[:1],
        output_dir,
        swap_arm_images=swap_arm_images,
        total_count=len(paths),
        preview_max_side=preview_max_side,
        preview_jpeg_quality=preview_jpeg_quality,
        preload_ahead=preload_ahead,
    )
    return _session_metadata(
        paths, episodes, build_complete=len(paths) <= 1,
        preview_max_side=preview_max_side,
        preview_jpeg_quality=preview_jpeg_quality,
        preload_ahead=preload_ahead,
    )


def _build_session_remainder(paths, output_dir, metadata, swap_arm_images=False,
                             preview_max_side=DEFAULT_PREVIEW_MAX_SIDE,
                             preview_jpeg_quality=DEFAULT_PREVIEW_JPEG_QUALITY,
                             preload_ahead=DEFAULT_PRELOAD_AHEAD):
    try:
        for idx, path in enumerate(paths[1:], 2):
            episode = _extract_session_episodes(
                [path],
                output_dir,
                swap_arm_images=swap_arm_images,
                start_index=idx,
                total_count=len(paths),
                preview_max_side=preview_max_side,
                preview_jpeg_quality=preview_jpeg_quality,
                preload_ahead=preload_ahead,
            )[0]
            metadata["episodes"].append(episode)
            metadata["readyCount"] = len(metadata["episodes"])
            metadata["buildComplete"] = metadata["readyCount"] >= metadata["episodeCount"]
            _write_metadata(metadata, output_dir)
        metadata["buildComplete"] = True
        metadata["buildError"] = None
        _write_metadata(metadata, output_dir)
        _log("background build complete")
    except Exception as exc:
        metadata["buildError"] = f"{type(exc).__name__}: {exc}"
        metadata["buildComplete"] = False
        _write_metadata(metadata, output_dir)
        _log(f"background build failed: {metadata['buildError']}")


def _html_template(metadata):
    data = json.dumps(metadata, ensure_ascii=False, allow_nan=False)
    title_name = metadata.get("sessionName") or metadata.get("episodeName") or "Episode Review"
    title = html.escape(f"Episode Review - {title_name}")
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title}</title>
  <style>
    :root {{
      color-scheme: light;
      --bg: #f7f8fa;
      --panel: #ffffff;
      --ink: #18202a;
      --muted: #65717f;
      --line: #d8dee6;
      --accent: #0f766e;
      --warn: #b45309;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font: 14px/1.45 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: var(--bg);
      color: var(--ink);
    }}
    header {{
      padding: 14px 18px;
      border-bottom: 1px solid var(--line);
      background: var(--panel);
      position: sticky;
      top: 0;
      z-index: 5;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 18px;
      font-weight: 650;
      letter-spacing: 0;
    }}
    .meta {{
      display: flex;
      flex-wrap: wrap;
      gap: 10px 18px;
      color: var(--muted);
      font-size: 13px;
    }}
    main {{ padding: 16px 18px 24px; }}
    .controls {{
      display: grid;
      grid-template-columns: auto auto auto auto auto auto minmax(220px, 1fr) auto auto auto;
      gap: 10px;
      align-items: center;
      padding: 12px;
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      margin-bottom: 16px;
    }}
    button, input {{
      min-height: 34px;
      border: 1px solid var(--line);
      background: #fff;
      color: var(--ink);
      border-radius: 6px;
      padding: 0 10px;
      font: inherit;
    }}
    button {{ cursor: pointer; }}
    button.primary {{ background: var(--accent); border-color: var(--accent); color: #fff; }}
    input[type="range"] {{ width: 100%; padding: 0; }}
    input[type="number"] {{ width: 92px; }}
    .views {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(360px, 1fr));
      gap: 16px;
      align-items: start;
    }}
    .arm {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: hidden;
    }}
    .arm-head {{
      display: flex;
      justify-content: space-between;
      gap: 12px;
      align-items: baseline;
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
    }}
    .arm h2 {{ margin: 0; font-size: 15px; letter-spacing: 0; }}
    .badge {{ color: var(--muted); font-size: 12px; }}
    .image-wrap {{
      background: #111820;
      min-height: 240px;
      display: flex;
      align-items: center;
      justify-content: center;
    }}
    .stream-grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
      gap: 1px;
      background: var(--line);
    }}
    .stream {{
      min-width: 0;
      background: #111820;
    }}
    .stream-title {{
      padding: 7px 9px;
      color: #d6dee8;
      background: #18202a;
      font-size: 12px;
      font-weight: 650;
    }}
    .image-wrap img {{
      display: block;
      width: 100%;
      height: auto;
      max-height: 62vh;
      object-fit: contain;
    }}
    .image-wrap img.hidden {{ display: none; }}
    .empty {{ color: #cbd5e1; padding: 36px; text-align: center; }}
    .empty.hidden {{ display: none; }}
    .readout {{
      display: grid;
      grid-template-columns: 90px 1fr;
      gap: 6px 10px;
      padding: 12px;
      border-top: 1px solid var(--line);
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 12px;
      overflow-wrap: anywhere;
    }}
    .label {{ color: var(--muted); font-family: inherit; }}
    .warn {{ color: var(--warn); }}
    @media (max-width: 780px) {{
      .controls {{ grid-template-columns: repeat(4, auto); }}
      .controls input[type="range"] {{ grid-column: 1 / -1; }}
      .views {{ grid-template-columns: 1fr; }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>Episode Review</h1>
    <div class="meta">
      <span id="episode"></span>
      <span id="buildStatus"></span>
      <span id="frameSummary"></span>
      <span id="timeSummary"></span>
      <span id="path"></span>
    </div>
  </header>
  <main>
    <section class="controls" aria-label="Frame controls">
      <button id="prevEpisode" title="Previous episode">Prev Episode</button>
      <button id="nextEpisode" title="Next episode">Next Episode</button>
      <label for="episodeInput">Episode</label>
      <input id="episodeInput" type="number" min="1" value="1">
      <button id="prevFrame" title="Previous frame">Prev Frame</button>
      <button id="play" class="primary" title="Play or pause">Play</button>
      <input id="slider" type="range" min="0" value="0">
      <label for="frameInput">Frame</label>
      <input id="frameInput" type="number" min="0" value="0">
      <button id="nextFrame" title="Next frame">Next Frame</button>
    </section>
    <section id="views" class="views"></section>
  </main>
  <script>
    let DATA = {data};
    let EPISODES = DATA.episodes || [DATA];
    const state = {{ episode: 0, frame: 0, timer: null }};
    const views = document.getElementById("views");
    const slider = document.getElementById("slider");
    const episodeInput = document.getElementById("episodeInput");
    const frameInput = document.getElementById("frameInput");
    const playButton = document.getElementById("play");
    const preloadSeen = new Set();
    const preloadQueue = [];
    const maxPreloadRefs = 96;
    const previewSettings = DATA.preview || {{}};
    const preloadAhead = Math.max(0, Number.parseInt(previewSettings.preloadAhead, 10) || 0);
    const streamOrder = ["realsenseColor", "fisheyeColor", "realsenseDepth"];
    const defaultStreamLabels = {{
      realsenseColor: "D405 Color",
      fisheyeColor: "Fisheye",
      realsenseDepth: "Depth 0-1000 mm",
    }};

    function fmt(value, digits = 4) {{
      if (value === null || value === undefined) return "NaN";
      if (typeof value !== "number") return String(value);
      return Number.isFinite(value) ? value.toFixed(digits) : "NaN";
    }}

    function fmtArray(values, digits = 4) {{
      if (!values || !values.length) return "[]";
      return "[" + values.map(v => fmt(v, digits)).join(", ") + "]";
    }}

    function episodeTotal() {{
      return DATA.episodeCount || EPISODES.length;
    }}

    function updateBuildStatus() {{
      const el = document.getElementById("buildStatus");
      if (!el) return;
      if (DATA.buildError) {{
        el.textContent = `build error: ${{DATA.buildError}}`;
      }} else if (DATA.episodeCount) {{
        const ready = DATA.readyCount || EPISODES.length;
        el.textContent = DATA.buildComplete
          ? `ready ${{ready}} / ${{DATA.episodeCount}}`
          : `building ${{ready}} / ${{DATA.episodeCount}}`;
      }} else {{
        el.textContent = "";
      }}
    }}

    function clampFrame(value) {{
      const max = Math.max(0, currentEpisode().frameCount - 1);
      return Math.min(max, Math.max(0, Number.parseInt(value, 10) || 0));
    }}

    function clampEpisode(value) {{
      const max = Math.max(0, EPISODES.length - 1);
      return Math.min(max, Math.max(0, Number.parseInt(value, 10) || 0));
    }}

    function currentEpisode() {{
      return EPISODES[state.episode];
    }}

    function frameIntervalMs() {{
      const ep = currentEpisode();
      if (ep.elapsed && ep.elapsed.length > 1) {{
        const duration = ep.elapsed[ep.elapsed.length - 1] - ep.elapsed[0];
        if (Number.isFinite(duration) && duration > 0) {{
          return Math.max(16, Math.min(250, (duration / (ep.elapsed.length - 1)) * 1000));
        }}
      }}
      const hz = Number(ep.attrs && ep.attrs.record_hz);
      return Number.isFinite(hz) && hz > 0 ? Math.max(16, Math.min(250, 1000 / hz)) : 1000 / 30;
    }}

    function preloadUrl(url) {{
      if (!url || preloadSeen.has(url)) return;
      preloadSeen.add(url);
      const img = new Image();
      img.src = url;
      preloadQueue.push(img);
      if (preloadQueue.length > maxPreloadRefs) preloadQueue.shift();
    }}

    function preloadAround(frame) {{
      const ep = currentEpisode();
      for (let offset = 1; offset <= preloadAhead; offset += 1) {{
        const next = frame + offset;
        if (next >= ep.frameCount) break;
        ep.arms.forEach(arm => {{
          streamEntries(arm).forEach(stream => {{
            preloadUrl(stream.urls ? stream.urls[next] : null);
          }});
        }});
      }}
    }}

    function streamEntries(arm) {{
      if (arm.streams) {{
        return streamOrder
          .filter(key => arm.streams[key])
          .map(key => ({{
            key,
            label: (arm.streamLabels && arm.streamLabels[key]) || defaultStreamLabels[key] || key,
            urls: arm.streams[key],
          }}));
      }}
      return arm.images ? [{{ key: "realsenseColor", label: "D405 Color", urls: arm.images }}] : [];
    }}

    function renderImageSlot(img, empty, url, alt) {{
      if (url) {{
        if (img.dataset.currentSrc) {{
          empty.classList.add("hidden");
        }} else {{
          empty.textContent = "Loading image";
          empty.classList.remove("hidden");
        }}
        if (img.dataset.currentSrc === url) {{
          img.alt = alt;
          img.classList.remove("hidden");
          empty.classList.add("hidden");
        }} else {{
          img.dataset.pendingSrc = url;
          const loader = new Image();
          loader.onload = () => {{
            if (img.dataset.pendingSrc !== url) return;
            img.src = url;
            img.alt = alt;
            img.dataset.currentSrc = url;
            img.classList.remove("hidden");
            empty.classList.add("hidden");
          }};
          loader.onerror = () => {{
            if (img.dataset.pendingSrc !== url || img.dataset.currentSrc) return;
            img.classList.add("hidden");
            empty.textContent = "Image failed";
            empty.classList.remove("hidden");
          }};
          loader.src = url;
        }}
      }} else {{
        img.dataset.pendingSrc = "";
        img.dataset.currentSrc = "";
        img.removeAttribute("src");
        img.classList.add("hidden");
        empty.textContent = "No image";
        empty.classList.remove("hidden");
      }}
    }}

    function renderShell() {{
      const ep = currentEpisode();
      const sessionName = DATA.sessionName || ep.sessionName;
      document.getElementById("episode").textContent =
        `${{sessionName}} / ${{ep.episodeName}}`;
      updateBuildStatus();
      document.getElementById("path").textContent = ep.episodePath;
      episodeInput.max = EPISODES.length;
      episodeInput.value = state.episode + 1;
      document.getElementById("prevEpisode").disabled = state.episode <= 0;
      document.getElementById("nextEpisode").disabled = state.episode >= EPISODES.length - 1;
      slider.max = Math.max(0, ep.frameCount - 1);
      frameInput.max = Math.max(0, ep.frameCount - 1);
      views.innerHTML = ep.arms.map((arm, idx) => `
        <article class="arm">
          <div class="arm-head">
            <h2>${{arm.name}}</h2>
            <span class="badge">${{arm.present ? `${{arm.poseValid}}/${{ep.frameCount}} pose valid` : "not present"}}</span>
          </div>
          <div class="stream-grid">
            ${{streamEntries(arm).map(stream => `
              <div class="stream">
                <div class="stream-title">${{stream.label}}</div>
                <div class="image-wrap" id="imageWrap-${{idx}}-${{stream.key}}">
                  <img id="image-${{idx}}-${{stream.key}}" class="hidden" alt="">
                  <div id="empty-${{idx}}-${{stream.key}}" class="empty">No image</div>
                </div>
              </div>
            `).join("") || `
              <div class="stream">
                <div class="stream-title">Image</div>
                <div class="image-wrap">
                  <div class="empty">No image</div>
                </div>
              </div>
            `}}
          </div>
          <div class="readout">
            <div class="label">pose</div><div id="pose-${{idx}}">[]</div>
            <div class="label">gripper</div><div id="gripper-${{idx}}">[]</div>
            <div class="label">attrs</div><div id="attrs-${{idx}}"></div>
          </div>
        </article>`).join("");
      ep.arms.forEach((arm, idx) => {{
        const attrs = arm.attrs || {{}};
        document.getElementById(`attrs-${{idx}}`).textContent =
          Object.keys(attrs).length ? JSON.stringify(attrs) : "-";
      }});
    }}

    function renderFrame(frame) {{
      const ep = currentEpisode();
      state.frame = clampFrame(frame);
      slider.value = state.frame;
      frameInput.value = state.frame;
      const elapsed = ep.elapsed[state.frame] || 0;
      document.getElementById("frameSummary").textContent =
        `episode ${{state.episode + 1}} / ${{episodeTotal()}}   frame ${{state.frame + 1}} / ${{ep.frameCount}}`;
      document.getElementById("timeSummary").textContent =
        `t=${{fmt(elapsed, 3)}}s / ${{fmt(ep.durationSec, 3)}}s`;
      ep.arms.forEach((arm, idx) => {{
        streamEntries(arm).forEach(stream => {{
          const img = document.getElementById(`image-${{idx}}-${{stream.key}}`);
          const empty = document.getElementById(`empty-${{idx}}-${{stream.key}}`);
          if (!img || !empty) return;
          const url = stream.urls ? stream.urls[state.frame] : null;
          renderImageSlot(img, empty, url, `${{arm.name}} ${{stream.label}} frame ${{state.frame}}`);
        }});
        const pose = arm.pose ? arm.pose[state.frame] : [];
        const gripper = arm.gripper ? arm.gripper[state.frame] : [];
        const poseEl = document.getElementById(`pose-${{idx}}`);
        poseEl.textContent = fmtArray(pose, 5);
        poseEl.className = pose && pose[0] !== null ? "" : "warn";
        document.getElementById(`gripper-${{idx}}`).textContent = fmtArray(gripper, 3);
      }});
      preloadAround(state.frame);
    }}

    function step(delta) {{
      renderFrame(state.frame + delta);
    }}

    function renderEpisode(episodeIndex) {{
      stopPlayback();
      state.episode = clampEpisode(episodeIndex);
      state.frame = 0;
      renderShell();
      renderFrame(0);
    }}

    function stopPlayback() {{
      if (state.timer !== null) {{
        clearInterval(state.timer);
        state.timer = null;
      }}
      playButton.textContent = "Play";
    }}

    function togglePlayback() {{
      if (state.timer !== null) {{
        stopPlayback();
        return;
      }}
      playButton.textContent = "Pause";
      state.timer = setInterval(() => {{
        if (state.frame >= currentEpisode().frameCount - 1) {{
          stopPlayback();
          return;
        }}
        step(1);
      }}, frameIntervalMs());
    }}

    async function refreshMetadata() {{
      try {{
        const response = await fetch(`metadata.json?t=${{Date.now()}}`, {{ cache: "no-store" }});
        if (!response.ok) return;
        const nextData = await response.json();
        const previousCount = EPISODES.length;
        const previousEpisode = state.episode;
        DATA = nextData;
        EPISODES = DATA.episodes || [DATA];
        if (!EPISODES.length) return;
        state.episode = clampEpisode(state.episode);
        if (EPISODES.length !== previousCount || state.episode !== previousEpisode) {{
          renderShell();
          renderFrame(state.frame);
        }} else {{
          updateBuildStatus();
        }}
        if (metadataTimer !== null && (DATA.buildComplete || DATA.buildError)) {{
          clearInterval(metadataTimer);
          metadataTimer = null;
        }}
      }} catch (err) {{
        // The writer replaces metadata atomically, but transient read errors are harmless.
      }}
    }}

    document.getElementById("prevEpisode").addEventListener("click", () => renderEpisode(state.episode - 1));
    document.getElementById("nextEpisode").addEventListener("click", () => renderEpisode(state.episode + 1));
    episodeInput.addEventListener("change", event => renderEpisode(Number.parseInt(event.target.value, 10) - 1));
    document.getElementById("prevFrame").addEventListener("click", () => step(-1));
    document.getElementById("nextFrame").addEventListener("click", () => step(1));
    playButton.addEventListener("click", togglePlayback);
    slider.addEventListener("input", event => renderFrame(event.target.value));
    frameInput.addEventListener("change", event => renderFrame(event.target.value));
    window.addEventListener("keydown", event => {{
      if (event.key === "ArrowLeft") step(-1);
      if (event.key === "ArrowRight") step(1);
      if (event.key === " ") {{
        event.preventDefault();
        togglePlayback();
      }}
    }});

    renderShell();
    renderFrame(0);
    let metadataTimer = null;
    if (DATA.episodeCount && !DATA.buildComplete) {{
      metadataTimer = setInterval(refreshMetadata, 2000);
      refreshMetadata();
    }}
  </script>
</body>
</html>
"""


def _write_html(metadata, output_dir):
    path = os.path.join(output_dir, "index.html")
    with open(path, "w", encoding="utf-8") as f:
        f.write(_html_template(metadata))
    return path


def _free_port(preferred):
    for candidate in (preferred, 0):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.bind(("0.0.0.0", candidate))
            return sock.getsockname()[1]
        except OSError:
            continue
        finally:
            sock.close()
    return 0


def _lan_ips():
    ips = []

    def add(ip):
        if not ip or ip.startswith("127.") or ip.startswith("169.254."):
            return
        if ip not in ips:
            ips.append(ip)

    override = os.environ.get("PIKA_VIEW_LAN_IPS", "")
    for ip in [x.strip() for x in override.split(",") if x.strip()]:
        add(ip)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("1.1.1.1", 80))
        add(sock.getsockname()[0])
    except OSError:
        pass
    finally:
        sock.close()
    return ips


def _serve(output_dir, preferred_port, on_started=None):
    port = _free_port(preferred_port)
    handler = partial(SimpleHTTPRequestHandler, directory=output_dir)
    server = ThreadingHTTPServer(("0.0.0.0", port), handler)
    print(f"[review] local URL: http://127.0.0.1:{port}/")
    for ip in _lan_ips():
        print(f"[review] LAN URL:   http://{ip}:{port}/")
    print("[review] Ctrl-C to stop")
    if on_started is not None:
        on_started()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        server.server_close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episode", help="Review one episode HDF5 path")
    ap.add_argument("--session", help="Review all episode_*.hdf5 files under this data_* folder. Defaults to newest session")
    ap.add_argument("--out", help="Output directory. Defaults next to the reviewed episode/session")
    ap.add_argument("--port", type=int, default=8088, help="Preferred HTTP port")
    ap.add_argument("--no-serve", action="store_true", help="Only generate files; do not start HTTP server")
    ap.add_argument("--swap-arm-images", action="store_true",
                    help="Swap left/right camera images in the review UI for old reversed-camera data")
    ap.add_argument("--no-swap-arm-images", action="store_true",
                    help="Compatibility no-op; arm images are not swapped by default")
    ap.add_argument("--preview-max-side", type=int, default=DEFAULT_PREVIEW_MAX_SIDE,
                    help="Maximum long side for generated review JPEG previews; 0 skips final resize")
    ap.add_argument("--preview-jpeg-quality", type=int, default=DEFAULT_PREVIEW_JPEG_QUALITY,
                    help="JPEG quality for generated review previews")
    ap.add_argument("--preload-ahead", type=int, default=DEFAULT_PRELOAD_AHEAD,
                    help="Number of future frames to preload in the web viewer")
    args = ap.parse_args()

    if args.episode:
        episode = os.path.abspath(args.episode)
        if not os.path.exists(episode):
            sys.exit(f"Episode not found: {episode}")
        paths = [episode]
        default_output_dir = os.path.join(
            os.path.dirname(episode),
            f"review_{os.path.splitext(os.path.basename(episode))[0]}",
        )
    else:
        session_dir = os.path.abspath(args.session or _latest_session_dir())
        if not os.path.isdir(session_dir):
            sys.exit(f"Session folder not found: {session_dir}")
        paths = _episode_paths_for_session(session_dir)
        default_output_dir = os.path.join(session_dir, "review_session")

    output_dir = os.path.abspath(args.out) if args.out else default_output_dir
    swap_arm_images = bool(args.swap_arm_images)
    preview_max_side = max(0, int(args.preview_max_side))
    preview_jpeg_quality = min(100, max(1, int(args.preview_jpeg_quality)))
    preload_ahead = max(0, int(args.preload_ahead))

    t0 = time.perf_counter()
    worker = None
    if len(paths) == 1:
        metadata = _extract_episode(
            paths[0], output_dir, swap_arm_images=swap_arm_images,
            preview_max_side=preview_max_side,
            preview_jpeg_quality=preview_jpeg_quality,
            preload_ahead=preload_ahead,
        )
    elif args.no_serve:
        metadata = _extract_session(
            paths, output_dir, swap_arm_images=swap_arm_images,
            preview_max_side=preview_max_side,
            preview_jpeg_quality=preview_jpeg_quality,
            preload_ahead=preload_ahead,
        )
    else:
        metadata = _extract_session_initial(
            paths, output_dir, swap_arm_images=swap_arm_images,
            preview_max_side=preview_max_side,
            preview_jpeg_quality=preview_jpeg_quality,
            preload_ahead=preload_ahead,
        )
    html_path = _write_html(metadata, output_dir)
    _write_metadata(metadata, output_dir)
    elapsed = time.perf_counter() - t0
    if len(paths) == 1:
        print(f"[review] episode: {paths[0]}")
    else:
        print(f"[review] session: {os.path.dirname(paths[0])}")
        print(f"[review] episodes: {len(paths)}")
        print(f"[review] ready:   {metadata['readyCount']} / {metadata['episodeCount']}")
    print(f"[review] output:  {html_path}")
    first = metadata["episodes"][0] if "episodes" in metadata else metadata
    print(f"[review] frames:  {first['frameCount']}  arms: {[a['name'] for a in first['arms']]}")
    print(f"[review] built in {elapsed:.1f}s")

    if not args.no_serve:
        def start_worker():
            nonlocal worker
            worker = threading.Thread(
                target=_build_session_remainder,
                args=(paths, output_dir, metadata),
                kwargs={
                    "swap_arm_images": swap_arm_images,
                    "preview_max_side": preview_max_side,
                    "preview_jpeg_quality": preview_jpeg_quality,
                    "preload_ahead": preload_ahead,
                },
                daemon=True,
            )
            worker.start()

        on_started = None
        if len(paths) > 1 and not metadata.get("buildComplete"):
            on_started = start_worker
        _serve(output_dir, args.port, on_started=on_started)


if __name__ == "__main__":
    main()
