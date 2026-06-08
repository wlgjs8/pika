#!/usr/bin/env python
"""기록된 에피소드 HDF5 검수 — 구조/shape/attrs 출력 + 첫 프레임 디코드 미리보기 저장.

실행: conda run -n pika python scripts\\inspect_hdf5.py <path.hdf5>
"""
import os
import sys

import cv2
import h5py
import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else None
if not path or not os.path.exists(path):
    sys.exit("HDF5 경로를 넘기세요: inspect_hdf5.py <file.hdf5>")

out_dir = os.path.dirname(os.path.abspath(path))

with h5py.File(path, "r") as f:
    print("=== attrs ===")
    for k, v in f.attrs.items():
        print(f"  {k} = {v}")

    print("=== datasets ===")
    def show(name, obj):
        if isinstance(obj, h5py.Dataset):
            enc = obj.attrs.get("encoding", "")
            print(f"  {name:32s} shape={obj.shape} dtype={obj.dtype} {enc}")
    f.visititems(show)

    pose = f["observations/pose"][...]
    grip = f["observations/gripper"][...]
    cmd = f["observations/command"][...]
    ts = f["timestamp"][...]
    print("=== samples ===")
    print("  frames:", len(ts))
    if len(ts) > 1:
        print(f"  duration: {ts[-1]-ts[0]:.2f}s  mean dt: {np.mean(np.diff(ts))*1000:.1f}ms")
    print("  pose[0]   :", np.round(pose[0], 4))
    print("  pose[-1]  :", np.round(pose[-1], 4))
    print("  pose valid:", int(np.sum(~np.isnan(pose[:, 0]))), "/", len(pose))
    print("  gripper[0]:", np.round(grip[0], 3), "(angle_deg, dist_mm)")
    print("  command   : unique", np.unique(cmd))

    # 첫 프레임 디코드 미리보기
    img = f.get("observations/images")
    if img is not None:
        for key in img:
            ds = img[key]
            if len(ds) == 0 or ds[0].size == 0:
                print(f"  {key}: (빈 프레임)")
                continue
            buf = np.asarray(ds[0], np.uint8)
            flag = cv2.IMREAD_UNCHANGED if ds.attrs.get("encoding") == "png16" else cv2.IMREAD_COLOR
            im = cv2.imdecode(buf, flag)
            if im is None:
                print(f"  {key}: 디코드 실패")
                continue
            outp = os.path.join(out_dir, f"preview_{key}.png")
            if im.dtype == np.uint16:  # depth -> 8bit 시각화
                vis = cv2.convertScaleAbs(im, alpha=0.03)
                vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
                cv2.imwrite(outp, vis)
            else:
                cv2.imwrite(outp, im)
            print(f"  {key}: {im.shape} {im.dtype} -> {outp}")
print("검수 완료.")
