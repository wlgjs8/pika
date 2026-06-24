#!/usr/bin/env python3
"""Measure SteamVR/Lighthouse spatial-distortion of the tracked volume — human-motion-INDEPENDENT.

Method (rigid two-tracker baseline):
  Bolt the LEFT and RIGHT Pika trackers onto ONE rigid bar/bracket so their TRUE relative
  pose is constant, then record a normal episode while slowly waving the bar through the
  WHOLE work volume (cover position AND orientation). Because the bar is rigid, any change in
  the MEASURED relative pose between the two trackers is pure reconstruction distortion — no
  assumption about a human "holding still" is needed.

Headline metrics (per swept episode):
  * baseline-length variation (mm)  -> METRIC distortion: a rigid bar of true length L should
       always measure L. If it measures L +/- delta depending on where it is, distances are
       not preserved (the reconstructed space is not metrically rigid).
  * relative-rotation variation (deg) -> ORIENTATION-FIELD distortion: how much the two
       trackers' relative orientation twists across the volume (the "local frame slowly
       rotates as you move" warp that leaks spurious rotation into ee_local action labels).

Use the SAME bar + SAME sweep BEFORE and AFTER a base-station re-calibration and compare:
    measure_tracker_distortion.py before.hdf5                       # baseline
    # ... SteamVR Room Setup / survive-cli --force-calibrate ...
    measure_tracker_distortion.py after.hdf5                        # re-measure
    measure_tracker_distortion.py before.hdf5 after.hdf5 --labels before after   # overlay

A good calibration drops both p95 numbers. Suggested PASS gates (tabletop UMI): baseline p95
<= ~3 mm and relative-rotation p95 <= ~0.5 deg over the work volume (tune to your accuracy need).
"""

from __future__ import annotations

import argparse
import json
import pathlib

import h5py
import numpy as np
from scipy.spatial.transform import Rotation as R


def _load_arm_poses(path: pathlib.Path) -> dict[str, np.ndarray]:
    """Return {'left': (T,7), 'right': (T,7)} in the recorded steamvr_world frame.

    Handles the BIMANUAL schema (observations/<arm>/pose). Raises on single-arm episodes
    (the rigid-baseline method needs two trackers)."""
    with h5py.File(path, "r") as f:
        obs = f["observations"]
        if not all(a in obs for a in ("left", "right")):
            raise SystemExit(
                f"{path}: needs a BIMANUAL episode with observations/left+right/pose "
                f"(found keys: {list(obs.keys())}). Record the two-tracker rigid-bar sweep dual-arm."
            )
        out = {s: np.asarray(obs[f"{s}/pose"], dtype=np.float64) for s in ("left", "right")}
        frame = f.attrs.get("pose_frame", b"")
        frame = frame.decode() if isinstance(frame, bytes) else str(frame)
    if frame and frame != "steamvr_world":
        print(f"  [warn] pose_frame={frame!r} (expected steamvr_world)")
    n = min(len(out["left"]), len(out["right"]))
    return {s: v[:n] for s, v in out.items()}


def _good_mask(*poses: np.ndarray) -> np.ndarray:
    """Drop frames with non-finite or zero-norm quaternions (tracking dropouts)."""
    m = np.ones(len(poses[0]), dtype=bool)
    for p in poses:
        m &= np.isfinite(p).all(axis=1) & (np.linalg.norm(p[:, 3:7], axis=1) > 1e-6)
    return m


def measure(left: np.ndarray, right: np.ndarray) -> dict:
    m = _good_mask(left, right)
    left, right = left[m], right[m]
    if len(left) < 10:
        raise SystemExit("too few valid frames after dropout filtering")
    Rl = R.from_quat(left[:, 3:7])
    Rr = R.from_quat(right[:, 3:7])
    pl, pr = left[:, :3], right[:, :3]

    # Relative pose T_rel = inv(T_left) . T_right, per frame (in LEFT's body frame).
    R_rel = Rl.inv() * Rr
    t_rel = Rl.inv().apply(pr - pl)                      # right origin in left frame (m)

    # --- METRIC distortion: measured rigid-bar length ---
    baseline_mm = np.linalg.norm(pr - pl, axis=1) * 1000.0
    base_ref = np.median(baseline_mm)
    base_dev = baseline_mm - base_ref

    # --- ORIENTATION-field distortion: relative-rotation deviation from a robust reference ---
    R_ref = R_rel.mean()                                 # chordal-L2 mean rotation
    rot_dev_deg = np.degrees((R_ref.inv() * R_rel).magnitude())

    # --- relative-translation-direction deviation (also reflects the orientation warp) ---
    t_ref = np.median(t_rel, axis=0)
    tdir_ref = t_ref / (np.linalg.norm(t_ref) + 1e-12)
    tdir = t_rel / (np.linalg.norm(t_rel, axis=1, keepdims=True) + 1e-12)
    tdir_dev_deg = np.degrees(np.arccos(np.clip(tdir @ tdir_ref, -1, 1)))

    # volume coverage of the bar centroid + correlation of distortion with position
    centroid = 0.5 * (pl + pr)
    cov_mm = (centroid.max(0) - centroid.min(0)) * 1000.0
    span = np.linalg.norm(centroid - centroid.mean(0), axis=1)
    corr_rot_pos = float(np.corrcoef(span, rot_dev_deg)[0, 1]) if span.std() > 1e-9 else float("nan")
    corr_base_pos = float(np.corrcoef(span, np.abs(base_dev))[0, 1]) if span.std() > 1e-9 else float("nan")

    def stats(a):
        return dict(median=float(np.median(a)), p95=float(np.percentile(a, 95)),
                    max=float(np.max(a)), std=float(np.std(a)))

    return {
        "n_frames": int(len(left)),
        "volume_coverage_mm_xyz": [round(float(c), 0) for c in cov_mm],
        "baseline_length_mm": {"reference": round(float(base_ref), 1), **stats(np.abs(base_dev))},
        "relative_rotation_deg": stats(rot_dev_deg),
        "relative_transdir_deg": stats(tdir_dev_deg),
        "distortion_correlates_with_position": {
            "rotation_vs_position_r": round(corr_rot_pos, 2),
            "baseline_vs_position_r": round(corr_base_pos, 2),
        },
        "_series": {  # for plotting / overlay (not printed)
            "span": span, "rot_dev": rot_dev_deg, "base_dev": base_dev, "baseline": baseline_mm,
        },
    }


def _verdict(rep: dict, rot_gate: float, base_gate: float) -> str:
    r95 = rep["relative_rotation_deg"]["p95"]
    b95 = rep["baseline_length_mm"]["p95"]
    ok = r95 <= rot_gate and b95 <= base_gate
    return (f"{'PASS' if ok else 'FAIL'}  (rot p95 {r95:.2f}deg vs gate {rot_gate}; "
            f"baseline p95 {b95:.2f}mm vs gate {base_gate})")


def _print(label: str, rep: dict, rot_gate: float, base_gate: float) -> None:
    print(f"\n=== {label} ===")
    print(f"  frames={rep['n_frames']}  volume covered (mm) x/y/z = {rep['volume_coverage_mm_xyz']}")
    b = rep["baseline_length_mm"]
    print(f"  rigid-bar measured length: ref {b['reference']} mm  |  deviation median {b['median']:.2f} "
          f"p95 {b['p95']:.2f} max {b['max']:.2f} mm   <- METRIC distortion")
    r = rep["relative_rotation_deg"]
    print(f"  relative-rotation deviation: median {r['median']:.2f} p95 {r['p95']:.2f} max {r['max']:.2f} deg"
          f"   <- ORIENTATION-field distortion")
    c = rep["distortion_correlates_with_position"]
    print(f"  correlates with position: rotation r={c['rotation_vs_position_r']}, baseline r={c['baseline_vs_position_r']}"
          f"   (high r => position-coupled warp, not random jitter)")
    print(f"  VERDICT: {_verdict(rep, rot_gate, base_gate)}")


def _plot(reps: list[tuple[str, dict]], out: pathlib.Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(13, 5))
    colors = ["#c0392b", "#2980b9", "#27ae60"]
    for i, (label, rep) in enumerate(reps):
        s = rep["_series"]; col = colors[i % len(colors)]
        ax[0].scatter(s["span"] * 1000, s["base_dev"], s=8, alpha=.4, color=col, label=f"{label} (p95 {rep['baseline_length_mm']['p95']:.2f}mm)")
        ax[1].scatter(s["span"] * 1000, s["rot_dev"], s=8, alpha=.4, color=col, label=f"{label} (p95 {rep['relative_rotation_deg']['p95']:.2f}°)")
    ax[0].set_xlabel("bar distance from volume center (mm)"); ax[0].set_ylabel("rigid-bar length deviation (mm)")
    ax[0].set_title("METRIC distortion (rigid-bar length should be flat at 0)"); ax[0].axhline(0, color='gray', lw=.6)
    ax[1].set_xlabel("bar distance from volume center (mm)"); ax[1].set_ylabel("relative-rotation deviation (deg)")
    ax[1].set_title("ORIENTATION-field distortion (should be flat at 0)")
    for a in ax: a.grid(alpha=.25); a.legend(fontsize=8)
    fig.suptitle("SteamVR tracked-volume distortion (rigid two-tracker baseline) — lower/flatter = more rigid", fontsize=11)
    fig.tight_layout(); fig.savefig(out, dpi=120, bbox_inches="tight")
    print(f"\nplot -> {out}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("episodes", nargs="+", type=pathlib.Path, help="rigid-bar sweep episode(s); pass 2 to compare before/after")
    ap.add_argument("--labels", nargs="*", default=None, help="labels matching the episodes (e.g. before after)")
    ap.add_argument("--rot-gate-deg", type=float, default=0.5, help="PASS threshold for relative-rotation p95 (deg)")
    ap.add_argument("--baseline-gate-mm", type=float, default=3.0, help="PASS threshold for baseline-length p95 (mm)")
    ap.add_argument("--plot", type=pathlib.Path, default=pathlib.Path("/tmp/tracker_distortion.png"))
    ap.add_argument("--json", type=pathlib.Path, default=None, help="write the report(s) as JSON")
    args = ap.parse_args()

    labels = args.labels or [e.parent.name + "/" + e.stem for e in args.episodes]
    if len(labels) != len(args.episodes):
        raise SystemExit("--labels count must match number of episodes")

    reps = []
    for ep, label in zip(args.episodes, labels):
        poses = _load_arm_poses(ep)
        rep = measure(poses["left"], poses["right"])
        _print(label, rep, args.rot_gate_deg, args.baseline_gate_mm)
        reps.append((label, rep))

    if len(reps) == 2:
        a, b = reps[0][1], reps[1][1]
        dr = a["relative_rotation_deg"]["p95"] - b["relative_rotation_deg"]["p95"]
        db = a["baseline_length_mm"]["p95"] - b["baseline_length_mm"]["p95"]
        print(f"\n=== {labels[0]} -> {labels[1]} ===")
        print(f"  relative-rotation p95: {a['relative_rotation_deg']['p95']:.2f} -> {b['relative_rotation_deg']['p95']:.2f} deg "
              f"({'improved' if dr > 0 else 'worse'} {abs(dr):.2f})")
        print(f"  baseline p95:          {a['baseline_length_mm']['p95']:.2f} -> {b['baseline_length_mm']['p95']:.2f} mm "
              f"({'improved' if db > 0 else 'worse'} {abs(db):.2f})")

    _plot(reps, args.plot)
    if args.json:
        args.json.write_text(json.dumps(
            [{"label": l, **{k: v for k, v in r.items() if k != "_series"}} for l, r in reps], indent=2))
        print(f"json  -> {args.json}")


if __name__ == "__main__":
    main()
