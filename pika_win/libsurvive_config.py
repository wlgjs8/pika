"""libsurvive 설정 파일(라이트하우스 해) 읽기 도우미 — pysurvive 임포트 없음.

왜 별도 모듈인가: 발행자·캘리브 도구·계기판이 모두 "이 id 의 스테이션을 빼라" 를
필요로 하는데, 그러려면 config 를 파싱해야 하고, 그 파싱을 pysurvive 를 끌어오는
pose_survive 에 두면 백엔드 없이도 돌아야 하는 스크립트들이 못 쓴다.

파일 형식 주의: 확장자가 .json 이지만 **유효한 JSON 이 아니다** (구분자 콤마가
빠져 있다 — 실측 확인). libsurvive 자체 파서는 관대하지만 json.load 는 실패하므로
정규식으로 읽는다.
"""
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIG_PATH = os.path.join(REPO_ROOT, "config", "libsurvive_config.json")

_BLOCK = re.compile(r'"lighthouse(\d+)":\{(.*?)\n\}', re.S)

# 이 리그가 **쓰지 않는** 라이트하우스의 OOTX id — 공용 공간에서 다른 사용자의
# 베이스스테이션이다. 위치를 바꿀 수 없고, 전원이 올라가면 libsurvive 가 자동으로
# 해에 끌어들여 우리 캘리브레이션을 오염시킨다(실측: 우리 것 1sigma 가 mm 단위인데
# 그 대는 500~650 mm 로 들어와 있었다).
#
# 여기가 유일한 출처다 — 텔레옵·수집·캘리브·계기판이 모두 이 값을 기본으로 쓴다.
# 스테이션 구성이 바뀌면 여기만 고치면 된다. 임시로 덮어쓰려면 환경변수:
#   PIKA_LIGHTHOUSE_EXCLUDE_IDS="5a2c575b 다른id"   (빈 문자열이면 제외 없음)
#
# 2026-09-04 기준 우리 3대: ch2 4b4ffb83(기준) / ch0 e7acbce5 / ch1 d51ee7eb(천장)
EXCLUDED_IDS = ("5a2c575b",)


def default_exclude_ids():
    env = os.environ.get("PIKA_LIGHTHOUSE_EXCLUDE_IDS")
    if env is not None:
        return [x for x in env.split() if x]
    return list(EXCLUDED_IDS)


def lighthouses(path=None):
    """저장된 라이트하우스 목록 -> [{slot, channel, id_hex}] (파일 없으면 빈 리스트)."""
    try:
        text = open(path or DEFAULT_CONFIG_PATH).read()
    except OSError:
        return []
    out = []
    for m in _BLOCK.finditer(text):
        body = m.group(2)
        idx = re.search(r'"index":"(\d+)"', body)
        mode = re.search(r'"mode":"(\d+)"', body)
        oid = re.search(r'"id":"(\d+)"', body)
        out.append({
            "slot": int(idx.group(1)) if idx else int(m.group(1)),
            "channel": int(mode.group(1)) if mode else None,
            "id_hex": f"{int(oid.group(1)):08x}" if oid else None,
        })
    return out


def slot_for_id(hex_id, path=None):
    """OOTX id(hex 문자열) -> libsurvive 슬롯 인덱스. 못 찾으면 None."""
    want = str(hex_id).lower().replace("0x", "").lstrip("0") or "0"
    for lh in lighthouses(path):
        if lh["id_hex"] and lh["id_hex"].lstrip("0") == want:
            return lh["slot"]
    return None


def exclude_args(hex_ids, path=None, log=None):
    """제외할 id 목록 -> libsurvive 인자 리스트.

    슬롯 번호를 박지 않고 매번 config 에서 다시 찾는 이유: libsurvive 의
    `lighthouse-N-disable` 은 N 이 **채널이 아니라 슬롯 인덱스**이고, 슬롯은
    재캘리브레이션마다 달라질 수 있다(실측: 천장 대가 새로 잡히며 슬롯 3 을 받았다).
    번호를 고정하면 언젠가 엉뚱한 스테이션을 끄게 된다.
    """
    args = []
    for hex_id in hex_ids or []:
        slot = slot_for_id(hex_id, path)
        if slot is None:
            if log:
                log(f"[lh] 제외 실패: id={hex_id} 가 저장된 해에 없습니다 — "
                    f"이번 실행에서는 제외가 적용되지 않습니다 "
                    f"(그 스테이션이 한 번이라도 잡힌 뒤 다시 시도)")
            continue
        args += [f"--lighthouse-{slot}-disable", "1"]
        if log:
            log(f"[lh] 제외: id={hex_id} -> 슬롯 {slot}")
    return args
