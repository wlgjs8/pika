"""PIKA Sense 유닛 = 하나의 물리 USB 체인 — 그 결속으로 장치를 찾는다.

왜 필요한가 (2026-09-04 실측): 유닛의 트래커·그리퍼 시리얼·카메라를 config 에
by-path 로 박아두면 USB 포트를 옮길 때마다 전부 깨진다. 더 나쁜 건 **조용한 교차
배선**이다 — 그때 arms.json 의 left 가 right 유닛의 그리퍼를 가리키고 있었는데
"[left] sense connected" 가 정상처럼 찍혔다. 왼팔 그리퍼 지령이 오른팔 유닛으로 갈
뻔했고, 로그만 봐서는 알 수 없었다.

ch341 어댑터 4개는 전부 시리얼 번호가 없어(1a86:7522) by-id 로도 구별할 수 없다.
믿을 수 있는 결속은 "트래커와 같은 물리 체인" 하나뿐이다.

USB3 장치(RealSense)는 같은 케이블이라도 **다른 버스**에 올라온다 — 하나의 허브가
USB2 트리와 USB3 트리로 따로 열거되기 때문이다. 커널이 그 둘을 `port/peer` 로 이어
두므로, USB2 체인 -> peer -> USB3 체인으로 건너가면 같은 유닛의 카메라를 찾을 수 있다.
"""
import os
import re

# ---------------------------------------------------------------------------
# PIKA Sense 유닛 = 하나의 USB 체인 (트래커 + 그리퍼 시리얼 + 카메라가 같은 허브 아래).
# 그래서 그리퍼 포트는 **트래커로부터 유도**할 수 있고, 그렇게 해야 안전하다:
#
#  * by-path 를 config 에 박아두면 USB 포트를 옮길 때마다 깨진다(2026-09-04 에 발판과
#    그리퍼가 같은 이유로 동시에 깨졌다).
#  * 더 나쁜 건 조용한 교차 배선이다 — 그때 arms.json 의 left 가 right 유닛의 그리퍼를
#    가리키고 있었고, "[left] sense connected" 가 정상처럼 찍혔다. 왼팔 그리퍼 지령이
#    오른팔 유닛으로 갈 뻔했다.
#  * ch341 어댑터 4개는 전부 시리얼 번호가 없어서(1a86:7522) by-id 로도 구별 불가다.
#    유일하게 믿을 수 있는 결속은 "같은 물리 체인"이다.
SENSE_USB_ID = ("1a86", "7522")     # CH341 USB-serial (Pika Sense 그리퍼 인코더)
TRACKER_USB_ID = ("28de", "2300")   # Valve LHR (Vive tracker)
_SYSFS_USB = "/sys/bus/usb/devices"


def _usb_attr(dev, name):
    try:
        with open(os.path.join(_SYSFS_USB, dev, name)) as fh:
            return fh.read().strip()
    except OSError:
        return None


def _chain_root(dev):
    """'3-4.1.3.1' -> '3-4' (그 유닛이 물린 호스트 포트)."""
    return dev.split(".")[0]


def tracker_chain(tracker_sn):
    """트래커 시리얼 -> 그 트래커가 물린 USB 체인 루트. 없으면 None."""
    for dev in os.listdir(_SYSFS_USB):
        if _usb_attr(dev, "idVendor") == TRACKER_USB_ID[0] and \
           _usb_attr(dev, "idProduct") == TRACKER_USB_ID[1] and \
           _usb_attr(dev, "serial") == tracker_sn:
            return _chain_root(dev)
    return None


def sense_port_for_tracker(tracker_sn):
    """트래커와 **같은 PIKA Sense 유닛**의 그리퍼 시리얼 포트(/dev/serial/by-path/...).

    찾지 못하면 None. 후보가 여러 개면(있어선 안 되는 상황) None 을 돌려주고
    호출부가 명시 설정을 요구하게 한다 — 추측해서 교차 배선하지 않는다.
    """
    chain = tracker_chain(tracker_sn)
    if not chain:
        return None
    hits = []
    for dev in os.listdir(_SYSFS_USB):
        if _chain_root(dev) != chain or dev == chain:
            continue
        if _usb_attr(dev, "idVendor") == SENSE_USB_ID[0] and \
           _usb_attr(dev, "idProduct") == SENSE_USB_ID[1]:
            hits.append(dev)
    if len(hits) != 1:
        return None
    # sysfs USB 경로 -> /dev/serial/by-path 심링크. 커널이 by-path 이름을 만드는 규칙을
    # 흉내내지 않고, 실제 심링크가 그 장치를 가리키는지 대조한다.
    want = os.path.realpath(os.path.join(_SYSFS_USB, hits[0]))
    by_path = "/dev/serial/by-path"
    try:
        links = os.listdir(by_path)
    except OSError:
        return None
    for link in links:
        full = os.path.join(by_path, link)
        tty = os.path.basename(os.path.realpath(full))
        sysfs_tty = f"/sys/class/tty/{tty}/device"
        try:
            if os.path.realpath(sysfs_tty).startswith(want):
                return full
        except OSError:
            continue
    return None


def _peer_chain(chain):
    """USB2 체인 루트 -> 같은 물리 허브의 USB3 체인 루트 (없으면 None).

    하나의 허브가 USB2 트리와 USB3 트리로 따로 열거되므로, 같은 케이블의 RealSense 는
    트래커와 **다른 버스**에 올라온다. 커널의 `port/peer` 가 SuperSpeed 포트와 그
    High-Speed 동반 포트를 이어주므로 이걸로 건너간다. 'usb4-port4' -> '4-4'.
    """
    link = os.path.join(_SYSFS_USB, chain, "port", "peer")
    try:
        name = os.path.basename(os.path.realpath(link))
    except OSError:
        return None
    m = re.match(r"usb(\d+)-port(\d+)$", name)
    return f"{m.group(1)}-{m.group(2)}" if m else None


def realsense_sn_for_tracker(tracker_sn):
    """트래커와 **같은 PIKA Sense 유닛**의 RealSense 시리얼. 못 찾으면 None.

    **USB 디스크립터의 serial 을 쓰면 안 된다** — librealsense 가 보고하는 시리얼과
    다른 체계다(실측: USB 334323070440 ↔ librealsense 419122270010). 설정·캘리브레이션은
    전부 librealsense 쪽 번호를 쓰므로 그것을 돌려줘야 한다.

    그래서 매칭은 librealsense 의 `physical_port`(sysfs 경로)로 한다: 트래커의 USB2
    체인 -> port/peer -> USB3 체인을 구하고, physical_port 가 그 체인 아래인 장치를 고른다.
    후보가 여러 개면 None — 다른 유닛의 카메라를 추측해서 붙이지 않는다.
    """
    chain = tracker_chain(tracker_sn)
    if not chain:
        return None
    peer = _peer_chain(chain)
    if not peer:
        return None
    try:
        import pyrealsense2 as rs
    except Exception:
        return None
    needle = f"/{peer}/"
    hits = []
    try:
        for dev in rs.context().devices:
            try:
                port = dev.get_info(rs.camera_info.physical_port)
                sn = dev.get_info(rs.camera_info.serial_number)
            except Exception:
                continue
            if needle in port:
                hits.append(sn)
    except Exception:
        return None
    return hits[0] if len(hits) == 1 else None
