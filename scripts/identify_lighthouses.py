#!/usr/bin/env python
"""베이스스테이션을 하나씩 켜/꺼가며 "어느 채널이 어느 트래커를 보는가" 를 읽는다.

왜 필요한가: libsurvive 의 `lighthouse0/1/2` 는 슬롯 번호일 뿐 물리적 식별자가 아니다.
물리적으로 구분되는 것은 **채널(mode)** 과 **OOTX id** 이고, 그 둘은 로그에만 나온다.
그리고 "이 스테이션이 실제로 쓸모가 있는가" 는 해(pose)가 아니라 **트래커별 갱신률**로만
알 수 있다 — LH2 gen2 스테이션 하나가 트래커 하나에 약 123 Hz 를 준다. 즉

    트래커 갱신률 / 123 ~= 그 트래커가 보고 있는 스테이션 개수

2026-09-04 실측: 3개를 다 켜고 40초를 휘저어도 두 트래커가 125.3 / 246.3 Hz 로 고정,
소수점 둘째 자리까지 안 변했다. 스테이션 하나는 켜져 있고 해도 풀렸는데 어느 트래커에도
스윕을 주지 않고 있었다는 뜻이다. 그 상태는 해(pose)만 봐서는 절대 안 보인다.

사용법 — 스테이션을 원하는 조합으로 켜/꺼두고:

    scripts/identify_lighthouses.py --seconds 20 [--label "ch2 only"]

저장된 라이트하우스 해는 **건드리지 않는다**(임시 사본으로 실행). 켜둔 스테이션이 하나뿐이면
libsurvive 가 scene 을 못 풀 수 있는데, 그때도 수신 채널은 로그로 잡히므로 식별에는 충분하다.
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SMOKE = os.path.join(REPO_ROOT, "scripts", "pose_test_survive.py")
LIVE_CONFIG = os.path.join(REPO_ROOT, "config", "libsurvive_config.json")
# 스테이션 1대가 트래커 1대에 주는 갱신률은 상수로 가정하지 않는다. 2026-09-04 에
# 123 Hz/대로 가정했다가 틀렸다: 채널 0 하나만 켠 상태에서 left 트래커가 246 Hz 를
# 그대로 냈다(= 그 트래커의 246 Hz 는 처음부터 채널 0 단독에서 나온 것이었다).
# 그러므로 "몇 대가 보이는가" 는 나눗셈이 아니라 이 켜/끄기 실험 자체로만 확정된다.


def run_probe(seconds, python_exe, keep_config):
    tmp = tempfile.NamedTemporaryFile(suffix=".json", delete=False)
    tmp.close()
    if keep_config and os.path.exists(LIVE_CONFIG):
        shutil.copy(LIVE_CONFIG, tmp.name)   # 해를 실어야 정지 상태에서도 포즈가 유효하다
    cmd = [python_exe, SMOKE, "--seconds", str(seconds), "--survive-config", tmp.name]
    p = subprocess.run(cmd, capture_output=True, text=True,
                       timeout=seconds + 90, cwd=REPO_ROOT)
    os.unlink(tmp.name)
    return p.stdout + p.stderr


def parse(out):
    # 채널이 실제로 수신되고 있음을 나타내는 두 신호. libsurvive 는 설정에 저장된 OOTX 를
    # 복원하지 않으므로, 수신 중인 모든 채널에 대해 "OOTX not set" 이 한 번 뜬다.
    seen = {}
    for ch, dev in re.findall(r"OOTX not set for LH in channel (\d+); attaching ootx decoder using device (\S+)", out):
        seen.setdefault(int(ch), {})["ootx_via"] = dev.strip()
    for ch, idx in re.findall(r"Adding lighthouse ch (\d+) \(idx: (\d+)", out):
        seen.setdefault(int(ch), {})["slot"] = int(idx)
    for ch, oid in re.findall(r"Got OOTX packet (\d+) ([0-9a-fA-F]+)", out):
        seen.setdefault(int(ch), {})["id"] = oid.lower()
    ref = re.search(r"Using LH (\d+) \(([0-9a-fA-F]+)\) as reference lighthouse", out)
    # 트래커별 샘플 시퀀스: 첫/마지막 관측으로 레이트를 낸다.
    rows = re.findall(r"^\[\s*([\d.]+)s\]", out, re.M)
    per = {}
    for m in re.finditer(r"^\s+(LHR-\S+)\s+arm=(\S+)\s+valid=(\S+)\s+tr=(\d+)\s+seq=\s*(\d+)", out, re.M):
        sn, arm, valid, tr, seq = m.group(1), m.group(2), m.group(3), int(m.group(4)), int(m.group(5))
        per.setdefault(sn, {"arm": arm, "seq": [], "valid": [], "tr": []})
        per[sn]["seq"].append(seq); per[sn]["valid"].append(valid == "True"); per[sn]["tr"].append(tr)
    times = [float(x) for x in rows]
    missing = re.search(r"누락=(\[[^\]]*\]|없음)", out)
    return seen, ref, per, times, (missing.group(1) if missing else "?")


def watch(seconds, python_exe, keep_config):
    """켜/끄기를 하는 동안 실시간으로 채널과 트래커 레이트를 보여준다.

    한 번의 연결을 유지한 채 출력을 따라간다: 스테이션을 켜면 libsurvive 가
    "Adding lighthouse ch N" 을 새로 찍고, 끄면 해당 트래커 레이트가 떨어진다.
    매번 재연결(워밍업 10초)하지 않아도 되므로 조합 탐색이 훨씬 빠르다.
    """
    tmp = tempfile.NamedTemporaryFile(suffix=".json", delete=False)
    tmp.close()
    if keep_config and os.path.exists(LIVE_CONFIG):
        shutil.copy(LIVE_CONFIG, tmp.name)
    cmd = [python_exe, "-u", SMOKE, "--seconds", str(seconds),
           "--survive-config", tmp.name]
    print(f"실시간 관측 {seconds:.0f}초 — 스테이션을 켜고 끄면서 보세요 (Ctrl-C 로 종료)\n")
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1, cwd=REPO_ROOT)
    channels, prev = {}, {}
    try:
        for line in proc.stdout:
            m = re.search(r"Adding lighthouse ch (\d+) \(idx: (\d+)", line)
            if m:
                channels[int(m.group(1))] = int(m.group(2))
                print(f"  [+] 채널 {m.group(1)} 등장 (슬롯 lighthouse{m.group(2)})")
            m = re.search(r"Got OOTX packet (\d+) ([0-9a-fA-F]+)", line)
            if m:
                print(f"  [+] 채널 {m.group(1)} id={m.group(2).lower()}")
            m = re.search(r"OOTX not set for LH in channel (\d+)", line)
            if m and int(m.group(1)) not in channels:
                channels[int(m.group(1))] = None
                print(f"  [+] 채널 {m.group(1)} 수신 중")
            m = re.match(r"^\[\s*([\d.]+)s\]", line)
            if m:
                t = float(m.group(1))
                if prev:
                    dt = t - prev.get("_t", t)
                    if dt > 0:
                        parts = [f"{sn.split('-')[1]}={(v - prev[sn]) / dt:6.1f}Hz"
                                 for sn, v in prev.items() if sn != "_t" and sn in prev]
                prev["_t"] = t
            m = re.match(r"^\s+(LHR-\S+)\s+arm=(\S+)\s+valid=(\S+).*?seq=\s*(\d+)", line)
            if m:
                sn, arm, valid, seq = m.group(1), m.group(2), m.group(3), int(m.group(4))
                t = prev.get("_t", 0.0)
                last = prev.get(sn)
                if last is not None and t > last[0]:
                    hz = (seq - last[1]) / (t - last[0])
                    print(f"    [{t:5.1f}s] {sn} arm={arm:5s} {hz:6.1f} Hz"
                          + ("" if valid == "True" else "   valid=False"))
                prev[sn] = (t, seq)
    except KeyboardInterrupt:
        proc.terminate()
    finally:
        proc.wait(timeout=10)
        os.unlink(tmp.name)
    print(f"\n관측된 채널: {sorted(channels)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--label", default="", help="이번 조합 이름 (예: 'ch2 만 켬')")
    ap.add_argument("--python", default=os.path.join(REPO_ROOT, ".venv", "bin", "python"))
    ap.add_argument("--fresh-config", action="store_true",
                    help="레이트 측정도 해 없이 한 번만 실행(빠르지만 조합 간 비교 불가)")
    ap.add_argument("--raw", action="store_true", help="원본 출력도 함께 표시")
    ap.add_argument("--watch", action="store_true",
                    help="연결을 유지한 채 실시간 표시 — 켜/끄기를 하면서 바로 확인")
    a = ap.parse_args()

    if a.watch:
        watch(a.seconds, a.python, keep_config=not a.fresh_config)
        return 0

    print(f"관측 {a.seconds:.0f}초" + (f"  [{a.label}]" if a.label else ""))

    # 두 번 돌린다. 이유(2026-09-04 실측으로 확인):
    #  * 채널 식별은 **저장된 해 없이** 해야 한다. 해를 실으면 그 안에 적힌 채널에도
    #    OOTX 디코더가 붙어서, 전원이 내려간 스테이션이 "수신 중"처럼 보인다.
    #    실제로 채널 2 스테이션의 전원을 내린 상태에서 해를 실었더니 채널 2 가 떴고,
    #    같은 순간 해 없이 돌리니 채널 3 만 떴다.
    #  * 반대로 갱신률은 **해를 실어야** 의미가 있다. 해가 없으면 scene 을 못 풀어
    #    포즈 산출이 줄고(같은 하드웨어에서 246/125 -> 208/105), 조합 간 비교가 깨진다.
    ident = run_probe(a.seconds, a.python, keep_config=False)
    if a.raw:
        print(ident)
    seen, ref_i, _, _, _ = parse(ident)

    rate_out = ident if a.fresh_config else run_probe(a.seconds, a.python, keep_config=True)
    if a.raw and not a.fresh_config:
        print(rate_out)
    _, ref, per, times, missing = parse(rate_out)
    ref = ref or ref_i

    print("\n■ 실제로 수신되는 채널 (저장된 해 없이 확인 = 전원이 켜진 것만)")
    if not seen:
        print("   (없음) — 켜진 스테이션이 없거나 트래커 시야 밖입니다")
    for ch in sorted(seen):
        info = seen[ch]
        print(f"   채널 {ch}"
              + (f"  id={info['id']}" if "id" in info else "")
              + (f"  슬롯=lighthouse{info['slot']}" if "slot" in info else "")
              + (f"  OOTX수신={info['ootx_via']}" if "ootx_via" in info else ""))
    if ref:
        print(f"   기준(reference) = lighthouse{ref.group(1)} (id {ref.group(2)})")

    print("\n■ 트래커별 갱신률"
          + ("  (해 없이 측정 — 다른 조합과 비교하지 말 것)" if a.fresh_config
             else "  (저장된 해를 싣고 측정 = 조합 간 비교 가능)"))
    if not per or len(times) < 2:
        print("   포즈 없음 — 해를 못 풀었거나 트래커가 시야 밖입니다")
    else:
        span = times[-1] - times[0]
        for sn, d in sorted(per.items()):
            if len(d["seq"]) < 2 or span <= 0:
                continue
            hz = (d["seq"][-1] - d["seq"][0]) / span
            bad = sum(1 for v in d["valid"] if not v)
            print(f"   {sn}  arm={d['arm']:5s}  {hz:6.1f} Hz"
                  + (f"   (valid=False {bad}회)" if bad else ""))
    print(f"\n   누락 트래커(이 조합에서 전혀 안 보임): {missing}")
    print("\n조합을 바꿔가며 이 표를 채우면 '어느 채널이 어느 트래커를 보는가' 가 확정됩니다.")
    print("\n(저장된 해는 건드리지 않았습니다 — 임시 사본으로 실행)")


if __name__ == "__main__":
    raise SystemExit(main())
