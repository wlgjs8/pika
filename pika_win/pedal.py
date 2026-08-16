"""USB 발판(FootSwitch) evdev 공용 — 장치 탐지와 배타 점유(grab).

수집기(collect.py)와 teleop 발행자(umi_teleop_publish.py)가 각자 발판 리더를 갖고
있어 같은 함정에 두 번 빠졌다. 탐지·grab 규칙은 여기 한 곳에만 둔다.
"""
import fcntl
import glob
import logging
import os

log = logging.getLogger("pika.pedal")

# EVIOCGRAB = _IOW('E', 0x90, int) — evdev 배타 점유.
# 잡으면 그 장치의 입력이 X11/터미널을 포함한 다른 소비자에게 전달되지 않는다.
EVIOCGRAB = 0x40044590


def find_pedal_devices():
    """연결된 FootSwitch 키보드 evdev 경로 전부(by-path, 정렬).

    **by-id 를 쓰면 안 된다.** 발판 여러 개는 VID:PID(3553:b001)가 같고 시리얼이 없어서
    udev 가 이름 충돌을 피하려고 `/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd` 를
    먼저 잡힌 하나에만 만든다. 나머지 발판은 by-id 에 아예 나타나지 않아 구조적으로
    주소 지정이 불가능하고, 그 심링크는 꽂는 순서에 따라 다른 물리 발판을 가리킨다.
    by-path 는 물리 USB 포트 기반이라 고유하고 재부팅/재연결에도 안정적이다.

    Keyboard 노드만 돌려준다. 발판의 보조 HID 노드는 press 만 보내고 release 를 안 보낼
    수 있는데, momentary 클러치에서 그러면 눌림 상태가 True 로 고착된다(안전 문제).
    """
    by_path = {}
    for link in glob.glob("/dev/input/by-path/*-event-kbd"):
        try:
            by_path[os.path.realpath(link)] = link
        except OSError:
            continue
    kbd, other = [], []
    for d in sorted(glob.glob("/sys/class/input/event*")):
        try:
            with open(os.path.join(d, "device", "name"), encoding="utf-8") as f:
                name = f.read().strip()
        except OSError:
            continue
        low = name.lower()
        if "footswitch" not in low or "mouse" in low:
            continue
        node = "/dev/input/" + os.path.basename(d)
        (kbd if "keyboard" in low else other).append(by_path.get(node, node))
    return sorted(kbd or other)


def open_pedal(path, grab=True):
    """발판 evdev 를 논블로킹으로 열고(기본) 배타 점유한다. fd 반환.

    grab 하지 않으면 발판이 키보드로 인식되어 **포커스된 창/터미널에 키를 그대로 입력한다**
    (PCsensor 기본 설정은 a/b/c). 터미널이 'bbbb...' 로 도배되는 것은 물론, collect.py 는
    `b` 를 녹화 토글로 쓰기 때문에 evdev 로 한 번 + stdin 으로 한 번 = 즉시 시작·정지로
    상쇄되어 녹화가 안 걸린다.

    grab 실패는 대개 다른 프로세스가 이미 점유한 경우다(예: robotics_lab rb_gui). 그때는
    이 fd 로 이벤트가 **하나도 오지 않으므로** 조용히 넘기지 않고 경고한다.
    """
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    if not grab:
        return fd
    try:
        fcntl.ioctl(fd, EVIOCGRAB, 1)
    except OSError as e:
        log.warning("[pedal] %s 배타 점유(grab) 실패: %s — 다른 프로세스(rb_gui 등)가 "
                    "이미 잡고 있으면 이 발판 이벤트를 전혀 받지 못하고, 키 입력이 "
                    "터미널로 새어 나갑니다.", path, e)
    return fd
