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
# 2026-09-04 기준 실제로 쓰는 것은 2대: ch2 4b4ffb83(**기준 lighthouse**) / ch0 e7acbce5.
#
# 제외 2건:
#   5a2c575b (ch3) — 다른 사용자 것. 위치를 못 바꾼다.
#   d51ee7eb (ch1) — 우리가 천장에 올렸던 대. 각도를 고친 뒤에도 솔버 잔차가 다른 둘보다
#                    한 자릿수 나빴다(acc err 0.011~0.015 vs 0.0002~0.0006). 빼고 나니
#                    MPFIT up err 가 0.0090 -> 0.0003 으로 30배 좋아졌다. 즉 이 대는
#                    기여보다 오염이 컸다.
#
# 스테이션을 **빼는 데는 재캘리브레이션이 필요 없다** — 저장된 해는 각 대를 기준 대에
# 대해 기술하므로 남는 대들의 좌표가 그대로다. 반대로 **다시 켤 때는 반드시 재캘리브**
# 해야 한다: d51ee7eb 항목은 지금 1sigma [0,0,0] 인 미해결 상태로 남아 있어서, 그냥
# 켜면 그 나쁜 사전값이 해에 들어간다.
#
# **4b4ffb83 은 끄지 말 것.** 기준 lighthouse 라 빼면 libsurvive 가 다른 대를 기준으로
# 다시 잡고 월드 프레임이 통째로 바뀐다. 텔레옵은 몸체 상대 변위라 조작 방향은 유지되지만
# 그 전후 수집분은 좌표계가 달라진다.
EXCLUDED_IDS = ("5a2c575b", "d51ee7eb")


def default_exclude_ids():
    env = os.environ.get("PIKA_LIGHTHOUSE_EXCLUDE_IDS")
    if env is not None:
        return [x for x in env.split() if x]
    return list(EXCLUDED_IDS)


# 기준(reference) lighthouse 를 명시적으로 고정한다.
#
# libsurvive 의 기본 규칙은 src/poser.c 에 있다: `reference-basestation` 이 0(미설정)
# 이면 **해가 풀린 것 중 BaseStationID 가 가장 작은 대**를 기준으로 잡는다.
#   0x4b4ffb83 = 1263532931  ch2 우리   <- 넷 중 최소, 그래서 지금 기준이다
#   0x5a2c575b = 1512855387  ch3 타 사용자
#   0xd51ee7eb = 3575572459  ch1 천장
#   0xe7acbce5 = 3886857445  ch0 우리
#
# 즉 우리가 위에서 어느 대를 빼든 기준은 바뀌지 않는다 — 이게 "빼는 데는 재캘리브가
# 필요 없다" 의 근거다. 다만 그건 **우연히** 우리 대의 id 가 가장 작아서 성립한다.
# 다른 사용자가 더 작은 id 의 대를 켜면 기준이 조용히 넘어가고 월드 프레임이 통째로
# 돌아간다 — 로그 한 줄 말고는 티도 안 난다. 그래서 못 박는다.
#
# CONFIG_STRING 은 strtol(.., 0) 으로 읽히므로 "0x.." 표기가 그대로 통한다
# (libsurvive src/survive_config.c config_entry_as_uint32_t).
REFERENCE_ID = "4b4ffb83"


def reference_args(hex_id=None):
    """기준 lighthouse 고정 인자. 빈 값으로 끄면 libsurvive 기본(최소 id)로 돌아간다."""
    hex_id = REFERENCE_ID if hex_id is None else hex_id
    hex_id = os.environ.get("PIKA_LIGHTHOUSE_REFERENCE_ID", hex_id).strip()
    if not hex_id:
        return []
    return ["--reference-basestation", f"0x{hex_id.lower().replace('0x', '')}"]


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

