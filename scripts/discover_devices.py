#!/usr/bin/env python
"""연결된 장치 열거 — COM 포트(시리얼), 카메라 인덱스(DSHOW), RealSense.

PIKA Sense를 Windows USB에 연결한 뒤 실행해 포트/인덱스를 확인한다.
실행: conda run -n pika python scripts\\discover_devices.py
"""
print("=== COM ports (시리얼: PIKA Sense/Gripper 후보) ===")
try:
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        print("  (없음 — Sense가 연결됐는지 확인)")
    for p in ports:
        print(f"  {p.device}: {p.description}  [{p.hwid}]")
except Exception as e:
    print("  list_ports 실패:", e)

print("\n=== Cameras (DSHOW, index 0..5 probe) ===")
try:
    import cv2
    found = False
    for idx in range(6):
        try:
            cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
            if cap.isOpened():
                ok, frame = cap.read()
                h, w = (frame.shape[1], frame.shape[0]) if (ok and frame is not None) else (0, 0)
                fps = cap.get(cv2.CAP_PROP_FPS)
                print(f"  index {idx}: OPEN  {h}x{w}@{fps:.0f}  read_ok={ok}")
                found = True
            cap.release()
        except Exception as e:
            print(f"  index {idx}: error {e}")
    if not found:
        print("  (열린 카메라 없음 — 어안/RealSense RGB 연결 확인)")
except Exception as e:
    print("  opencv probe 실패:", e)

print("\n=== RealSense ===")
try:
    import pyrealsense2 as rs
    ctx = rs.context()
    devs = ctx.query_devices()
    if len(devs) == 0:
        print("  (없음)")
    for d in devs:
        name = d.get_info(rs.camera_info.name)
        sn = d.get_info(rs.camera_info.serial_number)
        print(f"  {name}  SN={sn}")
except Exception as e:
    print("  realsense query 실패:", e)

print("\n완료. PIKA Sense COM 포트와 어안 카메라 index를 위에서 확인하세요.")
