#!/usr/bin/env python
"""Summarize collected PIKA HDF5 episodes.

The analyzer intentionally avoids decoding image payloads. It reads timestamps,
metadata, pose/gripper/command arrays, and dataset shapes only, so it can be
used while monitoring a growing data directory.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import time
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

try:
    import h5py
    import numpy as np
except ImportError as e:
    missing = getattr(e, "name", "h5py/numpy")
    sys.exit(
        f"Missing dependency: {missing}. Run with the pika conda env, e.g.\n"
        "  conda run -n pika python scripts/analyze_data.py data\n"
    )


EPISODE_RE = re.compile(r"episode_(\d+)\.hdf5$")


@dataclass
class NumericStats:
    count: int = 0
    min: float | None = None
    mean: float | None = None
    max: float | None = None


@dataclass
class ArmInfo:
    name: str
    pose_frames: int = 0
    pose_valid: int = 0
    gripper_angle: NumericStats = field(default_factory=NumericStats)
    gripper_distance: NumericStats = field(default_factory=NumericStats)
    command_counts: dict[int, int] = field(default_factory=dict)
    image_streams: list[str] = field(default_factory=list)


@dataclass
class EpisodeInfo:
    path: str
    session: str
    index: int | None
    readable: bool
    frames: int = 0
    duration_s: float = 0.0
    effective_hz: float | None = None
    record_hz: float | None = None
    n_arms: int = 0
    arm_names: list[str] = field(default_factory=list)
    arms: list[ArmInfo] = field(default_factory=list)
    size_bytes: int = 0
    mtime: float = 0.0
    warnings: list[str] = field(default_factory=list)
    error: str | None = None


def decode_attr(value: Any) -> Any:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    if hasattr(value, "item"):
        try:
            return value.item()
        except Exception:
            return value
    return value


def attr_float(attrs: h5py.AttributeManager, key: str) -> float | None:
    if key not in attrs:
        return None
    try:
        value = float(decode_attr(attrs[key]))
    except Exception:
        return None
    return value if math.isfinite(value) else None


def parse_arm_names(value: Any) -> list[str]:
    if value is None:
        return []
    value = decode_attr(value)
    if isinstance(value, str):
        return [x.strip() for x in value.split(",") if x.strip()]
    if isinstance(value, (list, tuple, np.ndarray)):
        names = []
        for item in value:
            text = str(decode_attr(item)).strip()
            if text:
                names.append(text)
        return names
    text = str(value).strip()
    return [text] if text else []


def finite_stats(values: Any) -> NumericStats:
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return NumericStats()
    return NumericStats(
        count=int(arr.size),
        min=float(np.min(arr)),
        mean=float(np.mean(arr)),
        max=float(np.max(arr)),
    )


def dataset_len(ds: h5py.Dataset | None) -> int:
    if ds is None or not hasattr(ds, "shape") or len(ds.shape) == 0:
        return 0
    return int(ds.shape[0])


def episode_index(path: Path) -> int | None:
    match = EPISODE_RE.match(path.name)
    return int(match.group(1)) if match else None


def sort_key(path: Path) -> tuple[str, int, str]:
    idx = episode_index(path)
    return (path.parent.name, idx if idx is not None else 10**9, str(path))


def discover_episodes(root: Path, latest: bool) -> list[Path]:
    if root.is_file():
        return [root] if root.name.endswith(".hdf5") else []
    if not root.exists():
        return []

    scan_root = root
    if latest:
        sessions = [
            p for p in root.iterdir()
            if p.is_dir() and any(p.glob("episode_*.hdf5"))
        ]
        if sessions:
            scan_root = max(sessions, key=lambda p: (p.name, p.stat().st_mtime))

    return sorted(scan_root.rglob("episode_*.hdf5"), key=sort_key)


def detect_arm_groups(h5: h5py.File) -> list[tuple[str, h5py.Group]]:
    obs = h5.get("observations")
    if not isinstance(obs, h5py.Group):
        return []

    names = parse_arm_names(h5.attrs.get("arm_names"))
    if "pose" in obs or "gripper" in obs or "command" in obs:
        return [(names[0] if names else "arm", obs)]

    groups: list[tuple[str, h5py.Group]] = []
    seen = set()
    for name in names:
        obj = obs.get(name)
        if isinstance(obj, h5py.Group):
            groups.append((name, obj))
            seen.add(name)

    for name in sorted(obs.keys()):
        if name in seen:
            continue
        obj = obs.get(name)
        if isinstance(obj, h5py.Group) and (
            "pose" in obj or "gripper" in obj or "command" in obj
        ):
            groups.append((name, obj))
    return groups


def analyze_arm(name: str, group: h5py.Group) -> ArmInfo:
    arm = ArmInfo(name=name)

    pose = group.get("pose")
    if isinstance(pose, h5py.Dataset):
        arm.pose_frames = dataset_len(pose)
        if arm.pose_frames:
            pose_arr = pose[...]
            if pose_arr.ndim >= 2:
                arm.pose_valid = int(np.isfinite(pose_arr).all(axis=1).sum())
            else:
                arm.pose_valid = int(np.isfinite(pose_arr).sum())

    gripper = group.get("gripper")
    if isinstance(gripper, h5py.Dataset) and dataset_len(gripper):
        grip_arr = gripper[...]
        if grip_arr.ndim >= 2 and grip_arr.shape[1] >= 1:
            arm.gripper_angle = finite_stats(grip_arr[:, 0])
        if grip_arr.ndim >= 2 and grip_arr.shape[1] >= 2:
            arm.gripper_distance = finite_stats(grip_arr[:, 1])

    command = group.get("command")
    if isinstance(command, h5py.Dataset) and dataset_len(command):
        cmd_arr = np.asarray(command[...]).reshape(-1)
        arm.command_counts = {
            int(k): int(v)
            for k, v in Counter(int(x) for x in cmd_arr).items()
        }

    images = group.get("images")
    if isinstance(images, h5py.Group):
        streams = []
        for key in sorted(images.keys()):
            ds = images.get(key)
            if isinstance(ds, h5py.Dataset):
                encoding = decode_attr(ds.attrs.get("encoding", ""))
                suffix = f":{encoding}" if encoding else ""
                streams.append(f"{key}{suffix}[{dataset_len(ds)}]")
        arm.image_streams = streams

    return arm


def analyze_episode(path: Path) -> EpisodeInfo:
    stat = path.stat()
    info = EpisodeInfo(
        path=str(path),
        session=path.parent.name,
        index=episode_index(path),
        readable=False,
        size_bytes=int(stat.st_size),
        mtime=float(stat.st_mtime),
    )

    try:
        with h5py.File(path, "r") as h5:
            info.readable = True
            info.record_hz = attr_float(h5.attrs, "record_hz")
            info.effective_hz = attr_float(h5.attrs, "effective_hz")
            info.arm_names = parse_arm_names(h5.attrs.get("arm_names"))
            try:
                info.n_arms = int(decode_attr(h5.attrs.get("n_arms", 0)))
            except Exception:
                info.n_arms = 0

            ts = h5.get("timestamp")
            if isinstance(ts, h5py.Dataset) and dataset_len(ts):
                timestamps = np.asarray(ts[...], dtype=np.float64).reshape(-1)
                info.frames = int(timestamps.size)
                if timestamps.size > 1:
                    diffs = np.diff(timestamps)
                    if np.any(diffs < 0):
                        info.warnings.append("timestamp is not monotonic")
                    info.duration_s = float(timestamps[-1] - timestamps[0])
                    if info.effective_hz is None and info.duration_s > 0:
                        info.effective_hz = float(info.frames / info.duration_s)

            arm_groups = detect_arm_groups(h5)
            info.arms = [analyze_arm(name, group) for name, group in arm_groups]
            if not info.arm_names:
                info.arm_names = [arm.name for arm in info.arms]
            if info.n_arms <= 0:
                info.n_arms = len(info.arms)

            if info.frames == 0:
                lengths = [
                    arm.pose_frames for arm in info.arms
                    if arm.pose_frames > 0
                ]
                if lengths:
                    info.frames = max(lengths)
                    info.warnings.append("missing timestamp; frame count inferred")
            for arm in info.arms:
                if info.frames and arm.pose_frames and arm.pose_frames != info.frames:
                    info.warnings.append(
                        f"{arm.name} pose length {arm.pose_frames} != timestamp {info.frames}"
                    )
    except Exception as e:
        info.error = f"{type(e).__name__}: {e}"

    return info


def fmt_float(value: float | None, digits: int = 1) -> str:
    if value is None or not math.isfinite(value):
        return "-"
    return f"{value:.{digits}f}"


def fmt_int(value: int | None) -> str:
    return "-" if value is None else f"{value:,}"


def fmt_bytes(size: int) -> str:
    units = ["B", "KB", "MB", "GB", "TB"]
    value = float(size)
    for unit in units:
        if abs(value) < 1024.0 or unit == units[-1]:
            return f"{value:.1f}{unit}" if unit != "B" else f"{int(value)}B"
        value /= 1024.0
    return f"{size}B"


def fmt_duration(seconds: float | None) -> str:
    if seconds is None or not math.isfinite(seconds):
        return "-"
    if seconds < 60:
        return f"{seconds:.1f}s"
    minutes, sec = divmod(seconds, 60)
    if minutes < 60:
        return f"{int(minutes)}m{sec:04.1f}s"
    hours, minutes = divmod(minutes, 60)
    return f"{int(hours)}h{int(minutes):02d}m{sec:04.1f}s"


def summary_str(values: list[float], digits: int = 1) -> str:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return "-"
    return (
        f"{min(clean):.{digits}f}/"
        f"{sum(clean) / len(clean):.{digits}f}/"
        f"{max(clean):.{digits}f}"
    )


def stat_range(stats: NumericStats, digits: int = 1) -> str:
    if stats.count == 0:
        return "-"
    return (
        f"{stats.min:.{digits}f}/"
        f"{stats.mean:.{digits}f}/"
        f"{stats.max:.{digits}f}"
    )


def pose_summary(ep: EpisodeInfo) -> str:
    parts = []
    for arm in ep.arms:
        if not ep.frames:
            parts.append(f"{arm.name}:-")
        else:
            pct = 100.0 * arm.pose_valid / ep.frames
            parts.append(f"{arm.name}:{pct:.0f}%")
    return ",".join(parts) if parts else "-"


def gripper_summary(ep: EpisodeInfo) -> str:
    parts = []
    for arm in ep.arms:
        if arm.gripper_angle.count:
            parts.append(f"{arm.name}:{arm.gripper_angle.min:.1f}-{arm.gripper_angle.max:.1f}")
        else:
            parts.append(f"{arm.name}:-")
    return ",".join(parts) if parts else "-"


def print_table(headers: list[str], rows: list[list[Any]]) -> None:
    if not rows:
        print("  (none)")
        return
    text_rows = [[str(cell) for cell in row] for row in rows]
    widths = [
        max(len(headers[i]), *(len(row[i]) for row in text_rows))
        for i in range(len(headers))
    ]
    print("  " + "  ".join(headers[i].ljust(widths[i]) for i in range(len(headers))))
    print("  " + "  ".join("-" * widths[i] for i in range(len(headers))))
    for row in text_rows:
        print("  " + "  ".join(row[i].ljust(widths[i]) for i in range(len(headers))))


def to_jsonable(obj: Any) -> Any:
    if hasattr(obj, "__dataclass_fields__"):
        return {
            key: to_jsonable(getattr(obj, key))
            for key in obj.__dataclass_fields__.keys()
        }
    if isinstance(obj, dict):
        return {str(k): to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_jsonable(x) for x in obj]
    return obj


def session_rows(episodes: list[EpisodeInfo]) -> list[list[str]]:
    by_session: dict[str, list[EpisodeInfo]] = defaultdict(list)
    for ep in episodes:
        by_session[ep.session].append(ep)

    rows = []
    for session in sorted(by_session):
        eps = [ep for ep in by_session[session] if ep.readable]
        all_eps = by_session[session]
        frames = [ep.frames for ep in eps]
        durations = [ep.duration_s for ep in eps]
        hz_values = [
            ep.effective_hz for ep in eps
            if ep.effective_hz is not None and math.isfinite(ep.effective_hz)
        ]
        arms = Counter(ep.n_arms for ep in eps)
        arms_text = ",".join(f"{n}arm:{c}" for n, c in sorted(arms.items())) or "-"
        latest = max(all_eps, key=lambda ep: ep.mtime)
        rows.append([
            session,
            f"{len(eps)}/{len(all_eps)}",
            fmt_int(sum(frames)),
            fmt_duration(sum(durations)),
            summary_str([float(x) for x in frames], 1),
            summary_str(durations, 1),
            fmt_float(sum(hz_values) / len(hz_values), 1) if hz_values else "-",
            fmt_bytes(sum(ep.size_bytes for ep in all_eps)),
            arms_text,
            datetime.fromtimestamp(latest.mtime).strftime("%H:%M:%S"),
        ])
    return rows


def episode_rows(episodes: list[EpisodeInfo], max_rows: int) -> list[list[str]]:
    visible = sorted(episodes, key=lambda ep: (ep.mtime, ep.path), reverse=True)
    if max_rows > 0:
        visible = visible[:max_rows]
    visible = list(reversed(visible))

    rows = []
    for ep in visible:
        status = "ok" if ep.readable else "error"
        warn = f"warn:{len(ep.warnings)}" if ep.warnings else ""
        if ep.error:
            warn = ep.error[:48]
        rows.append([
            f"{ep.session}/{Path(ep.path).name}",
            status,
            fmt_int(ep.frames),
            fmt_duration(ep.duration_s),
            fmt_float(ep.effective_hz, 1),
            str(ep.n_arms or "-"),
            fmt_bytes(ep.size_bytes),
            pose_summary(ep),
            gripper_summary(ep),
            warn,
        ])
    return rows


def print_report(root: Path, episodes: list[EpisodeInfo], args: argparse.Namespace) -> None:
    readable = [ep for ep in episodes if ep.readable]
    errors = [ep for ep in episodes if not ep.readable]
    frames = [ep.frames for ep in readable]
    durations = [ep.duration_s for ep in readable]
    hz_values = [
        ep.effective_hz for ep in readable
        if ep.effective_hz is not None and math.isfinite(ep.effective_hz)
    ]
    warnings = [ep for ep in readable if ep.warnings]

    print("PIKA data episode summary")
    print(f"Generated : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Path      : {root}")
    print(f"Episodes  : {len(readable)} readable / {len(errors)} error / {len(episodes)} found")
    print(f"Sessions  : {len(set(ep.session for ep in episodes))}")
    print(f"Frames    : {fmt_int(sum(frames))}")
    print(f"Duration  : {fmt_duration(sum(durations))}")
    print(f"Length(fr): min/avg/max = {summary_str([float(x) for x in frames], 1)}")
    print(f"Length(s) : min/avg/max = {summary_str(durations, 1)}")
    print(f"Hz        : min/avg/max = {summary_str([float(x) for x in hz_values], 1)}")
    print(f"Disk      : {fmt_bytes(sum(ep.size_bytes for ep in episodes))}")
    if readable:
        latest = max(readable, key=lambda ep: ep.mtime)
        print(
            "Latest    : "
            f"{latest.path} "
            f"({datetime.fromtimestamp(latest.mtime).strftime('%Y-%m-%d %H:%M:%S')})"
        )
    print()

    print("Sessions")
    print_table(
        ["session", "eps", "frames", "duration", "fr min/avg/max", "s min/avg/max", "avg hz", "size", "arms", "latest"],
        session_rows(episodes),
    )

    if args.details:
        print()
        title = "Episodes"
        if args.max_episodes > 0:
            title += f" (latest {args.max_episodes})"
        print(title)
        print_table(
            ["episode", "status", "frames", "duration", "hz", "arms", "size", "pose", "grip angle", "note"],
            episode_rows(episodes, args.max_episodes),
        )

    if warnings or errors:
        print()
        print("Warnings")
        rows = []
        for ep in warnings:
            rows.append([f"{ep.session}/{Path(ep.path).name}", "; ".join(ep.warnings)])
        for ep in errors:
            rows.append([f"{ep.session}/{Path(ep.path).name}", ep.error or "unreadable"])
        print_table(["episode", "message"], rows)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze collected PIKA HDF5 episode lengths and sensor summaries."
    )
    parser.add_argument("path", nargs="?", default="data", help="data root, session dir, or one .hdf5 file")
    parser.add_argument("--latest", action="store_true", help="scan only the newest data_* session under path")
    parser.add_argument("--watch", type=float, default=0.0, help="refresh every N seconds; 0 runs once")
    parser.add_argument("--details", dest="details", action="store_true", default=True, help="show episode rows")
    parser.add_argument("--no-details", dest="details", action="store_false", help="hide episode rows")
    parser.add_argument("--max-episodes", type=int, default=30, help="episode rows to show; 0 means all")
    parser.add_argument("--json", action="store_true", help="emit JSON instead of a text report")
    return parser


def run_once(args: argparse.Namespace) -> int:
    root = Path(args.path)
    paths = discover_episodes(root, args.latest)
    episodes = [analyze_episode(path) for path in paths]

    if args.json:
        payload = {
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "path": str(root),
            "latest_only": bool(args.latest),
            "episodes": [to_jsonable(ep) for ep in episodes],
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    else:
        print_report(root, episodes, args)

    if not paths:
        return 1
    return 0 if all(ep.readable for ep in episodes) else 2


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.watch and args.json:
        sys.exit("--watch and --json cannot be used together")

    if args.watch <= 0:
        return run_once(args)

    interval = max(0.5, float(args.watch))
    try:
        while True:
            print("\033[2J\033[H", end="")
            code = run_once(args)
            print()
            print(f"Refreshing every {interval:.1f}s. Press Ctrl-C to stop.")
            sys.stdout.flush()
            time.sleep(interval)
    except KeyboardInterrupt:
        print()
        return code if "code" in locals() else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BrokenPipeError:
        try:
            sys.stdout.close()
        except Exception:
            pass
        raise SystemExit(0)
