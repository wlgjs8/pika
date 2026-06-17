"""rerun 기반 라이브 시각화 뷰어 (선택적, 뷰 전용, 단일/양팔).

수집/녹화 로직과 완전히 분리된다. 데이터의 source of truth는 data/.../episode_NNN.hdf5 이고,
이 뷰어는 현재 상태를 '보여주기만' 한다 — 더블-핀치/에피소드 저장과 무관.

엔티티 경로를 팔 이름으로 네임스페이스(world/<arm>/..., camera/<arm>/..., plots/<arm>/...)하여
양팔이면 양쪽 데이터가 동시에 보인다.

메모리: 뷰어 memory_limit 으로 라이브 스트림만 유지(상한 초과 시 오래된 데이터 폐기).
이미지는 인코딩 바이트(PNG)를 rr.EncodedImage 로 그대로 전송(디코딩/재인코딩 0),
img_every 프레임마다만 로깅.

mode='none' 이면 NullViewer(모든 호출 no-op, rerun import 조차 안 함).
"""
import logging
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from urllib.parse import quote

log = logging.getLogger("collect.viewer")


def _free_port(preferred):
    """rerun 서버와 동일 기준(SO_EXCLUSIVEADDRUSE)으로 빈 포트 선택.

    이전 실행의 잔여(죽은 PID의 TIME_WAIT/CloseWait 또는 좀비 rerun 서버) 소켓이
    preferred를 점유해도 엄격 검사로 걸러내고 OS 임의 포트(cand=0)로 폴백
    → 충돌(os error 10048) 회피.
    """
    for cand in (preferred, 0):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                s.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            s.bind(("0.0.0.0", cand))   # rerun이 wildcard(0.0.0.0)에 바인딩 → 동일 기준 검사
            return s.getsockname()[1]
        except OSError:
            continue
        finally:
            s.close()
    return 0


def _listening_tcp_ports_for_self():
    """Return TCP listen ports opened by this process on Linux."""
    inodes = set()
    fd_dir = "/proc/self/fd"
    try:
        for name in os.listdir(fd_dir):
            try:
                target = os.readlink(os.path.join(fd_dir, name))
            except OSError:
                continue
            if target.startswith("socket:[") and target.endswith("]"):
                inodes.add(target[8:-1])
    except OSError:
        return set()

    ports = set()
    try:
        with open("/proc/net/tcp", encoding="ascii") as f:
            next(f, None)
            for line in f:
                cols = line.split()
                if len(cols) < 10 or cols[3] != "0A" or cols[9] not in inodes:
                    continue
                _, port_hex = cols[1].split(":")
                ports.add(int(port_hex, 16))
    except OSError:
        pass
    return ports


def _pids_listening_on(port):
    """주어진 TCP 포트를 LISTEN(0A) 중인 프로세스 PID 목록(Linux, 같은 유저)."""
    target_inodes = set()
    for path in ("/proc/net/tcp", "/proc/net/tcp6"):
        try:
            with open(path, encoding="ascii") as f:
                next(f, None)
                for line in f:
                    cols = line.split()
                    if len(cols) < 10 or cols[3] != "0A":
                        continue
                    _, port_hex = cols[1].split(":")
                    if int(port_hex, 16) == port:
                        target_inodes.add(cols[9])
        except OSError:
            continue
    if not target_inodes:
        return []
    pids = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        fd_dir = f"/proc/{pid}/fd"
        try:
            for name in os.listdir(fd_dir):
                try:
                    target = os.readlink(os.path.join(fd_dir, name))
                except OSError:
                    continue
                if target.startswith("socket:[") and target[8:-1] in target_inodes:
                    pids.append(int(pid))
                    break
        except OSError:
            continue
    return pids


def _reap_stale_viewer_port(port):
    """시작 시 preferred 포트(예: 9876)를 잡고 있는 '이전 뷰어 좀비'를 정리.

    rerun serve_grpc 데드락 등으로 남은 collect.py/rerun/pika_view 프로세스만 대상.
    자기 자신/무관 프로세스는 건드리지 않는다. Linux 전용(그 외 no-op).
    """
    if not sys.platform.startswith("linux"):
        return
    me = os.getpid()
    markers = ("collect.py", "pika_view", "serve_grpc", "rerun")
    killed = False
    for pid in _pids_listening_on(port):
        if pid == me:
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\x00", b" ").decode("utf-8", "replace").strip()
        except OSError:
            continue
        if not any(mk in cmd for mk in markers):
            continue
        log.warning("[viewer] 포트 %d 점유 좀비 정리: pid=%d (%s)", port, pid, cmd[:80])
        killed = True
        try:
            os.kill(pid, signal.SIGTERM)
            time.sleep(1.0)
            os.kill(pid, signal.SIGKILL)  # 데드락 프로세스는 TERM 무시 → KILL
        except ProcessLookupError:
            pass
        except OSError as e:
            log.warning("[viewer] 좀비 정리 실패 pid=%d: %s", pid, e)
    if not killed:
        return
    # 커널이 LISTEN 소켓을 실제로 해제할 때까지 폴링(최대 ~2s) → 9876 재확보.
    deadline = time.time() + 2.0
    while time.time() < deadline:
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            probe.bind(("0.0.0.0", port))
            return
        except OSError:
            time.sleep(0.1)
        finally:
            probe.close()


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

    try:
        cp = subprocess.run(
            ["ip", "-o", "-4", "addr", "show", "scope", "global"],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        for line in cp.stdout.splitlines():
            cols = line.split()
            if len(cols) < 4:
                continue
            iface = cols[1]
            if iface == "lo" or iface.startswith(("docker", "br-", "veth", "virbr")):
                continue
            try:
                inet_idx = cols.index("inet")
            except ValueError:
                continue
            add(cols[inet_idx + 1].split("/", 1)[0])
    except OSError:
        pass

    try:
        host = socket.gethostname()
        for item in socket.getaddrinfo(host, None, socket.AF_INET, socket.SOCK_STREAM):
            add(item[4][0])
    except OSError:
        pass
    return ips

# 팔별 궤적/축 색(녹화 중이면 빨강으로 덮어씀)
_ARM_COLORS = {"right": [120, 120, 120], "left": [80, 160, 255]}
_DEFAULT_COL = [120, 120, 120]


def _ordered_arm_names(arm_names):
    names = [name for name in (arm_names or []) if name]
    ordered = [name for name in ("left", "right") if name in names]
    ordered.extend(name for name in names if name not in ordered)
    return ordered


def make_viewer(mode="none", **kw):
    if mode in (None, "none"):
        return NullViewer()
    return RerunViewer(mode=mode, **kw)


class NullViewer:
    enabled = False

    def state(self, *a, **k):
        pass

    def pose(self, *a, **k):
        pass

    def images(self, *a, **k):
        pass

    def close(self):
        pass


class RerunViewer:
    enabled = True

    def __init__(self, mode="spawn", memory_limit="2GB", img_every=3, trail_len=600,
                 session_dir=None, arm_names=None):
        import rerun as rr  # 지연 import: 뷰어 켤 때만 로드
        self.rr = rr
        self.img_every = max(1, int(img_every))
        self.trail_len = trail_len
        self._trails = {}   # arm -> [pos,...]
        self._fc = {}       # arm -> frame counter(이미지 스로틀)
        self.arm_names = _ordered_arm_names(arm_names)

        rr.init("pika_view")
        self._send_default_blueprint()
        if mode == "web":
            lan_ips = _lan_ips()
            _reap_stale_viewer_port(9876)
            gport = _free_port(9876)
            cors_allow_origin = [f"http://{ip}:*" for ip in lan_ips]
            log.info("[viewer] gRPC 서버 시작 중: port=%d mem<=%s", gport, memory_limit)
            uri = self._serve_grpc_with_watchdog(rr, gport, memory_limit, cors_allow_origin)
            log.info("[viewer] gRPC 서버 시작 완료: %s", uri)
            before = _listening_tcp_ports_for_self()
            wport = _free_port(9090)
            log.info("[viewer] web 서버 시작 중: port=%d", wport)
            rr.serve_web_viewer(web_port=wport, open_browser=False, connect_to=uri)
            log.info("[viewer] web 서버 시작 완료")
            time.sleep(0.2)
            after = _listening_tcp_ports_for_self()
            new_ports = sorted((after - before) - {gport})
            actual_wport = new_ports[-1] if new_ports else wport
            uri_text = str(uri)
            local_url = f"http://127.0.0.1:{actual_wport}/?url={quote(uri_text, safe='')}"
            lan_urls = []
            for ip in lan_ips:
                lan_uri = uri_text.replace("127.0.0.1", ip).replace("localhost", ip)
                lan_urls.append(f"http://{ip}:{actual_wport}/?url={quote(lan_uri, safe='')}")
            log.info("[viewer] web URL (host/Moonlight): %s", local_url)
            for lan_url in lan_urls:
                log.info("[viewer] web URL (host PC LAN browser): %s", lan_url)
            log.info("[viewer] gRPC URL: %s  (mem<=%s)", uri, memory_limit)
            log.info("[viewer] 컨트롤러 로컬 브라우저는 127.0.0.1 URL, host PC 브라우저는 LAN URL을 여세요.")
            if session_dir:
                path = os.path.join(session_dir, "viewer_url.txt")
                try:
                    with open(path, "w", encoding="utf-8") as f:
                        f.write(local_url + "\n")
                        for lan_url in lan_urls:
                            f.write(lan_url + "\n")
                        f.write(str(uri) + "\n")
                    log.info("[viewer] URL 파일: %s", path)
                except OSError as e:
                    log.warning("[viewer] URL 파일 저장 실패: %s", e)
        else:  # 'spawn' = 네이티브 창
            _reap_stale_viewer_port(9876)
            gport = _free_port(9876)
            rr.spawn(port=gport, memory_limit=memory_limit)
            log.info("[viewer] 네이티브 창(spawn) :%d (mem<=%s)", gport, memory_limit)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)

    @staticmethod
    def _serve_grpc_with_watchdog(rr, gport, memory_limit, cors_allow_origin, timeout=10.0):
        """rr.serve_grpc 를 워치독 스레드에서 호출.

        rerun 0.33 의 간헐적 serve_grpc 데드락(포트는 bind 되나 리턴 안 함) 대비.
        timeout 초 내 리턴이 없으면 무한 대기 대신 즉시 종료(os._exit)하여 재실행 유도.
        """
        result = {}

        def _run():
            try:
                result["uri"] = rr.serve_grpc(
                    grpc_port=gport,
                    server_memory_limit=memory_limit,
                    cors_allow_origin=cors_allow_origin,
                )
            except BaseException as e:  # noqa: BLE001  스레드 내 예외 보존
                result["err"] = e

        t = threading.Thread(target=_run, name="serve_grpc", daemon=True)
        t.start()
        t.join(timeout)
        if "uri" in result:
            return result["uri"]
        if "err" in result:
            log.error("[viewer] gRPC 서버 시작 실패: %s", result["err"])
        else:
            log.error("[viewer] rerun gRPC 데드락 감지 — 종료합니다. 다시 실행하세요")
        # 데드락 스레드는 native 호출에 묶여 정상 종료가 멈출 수 있어 즉시 종료.
        sys.stderr.flush()
        os._exit(1)

    def _send_default_blueprint(self):
        """Start with color/color/world visible; keep diagnostics available but hidden."""
        rr = self.rr
        try:
            import rerun.blueprint as rrb
        except Exception as e:
            log.warning("[viewer] 기본 blueprint 로드 실패: %s", e)
            return

        arms = self.arm_names or ["left", "right"]
        color_views = [
            rrb.Spatial2DView(
                origin=f"/camera/{name}/d405_color",
                name=f"{name}_color" if name in ("left", "right") else f"{name}_d405_color",
            )
            for name in arms[:2]
        ]
        while len(color_views) < 2:
            missing_name = next(
                (name for name in ("left", "right") if name not in arms),
                f"arm{len(color_views)}",
            )
            color_views.append(
                rrb.Spatial2DView(
                    origin=f"/camera/{missing_name}/d405_color",
                    name=f"{missing_name}_color",
                    visible=False,
                )
            )

        hidden_views = []
        for name in arms:
            hidden_views.append(
                rrb.Spatial2DView(
                    origin=f"/camera/{name}/d405_depth",
                    name=f"{name}_depth",
                    visible=False,
                )
            )
        hidden_views.extend([
            rrb.TimeSeriesView(origin="/plots", name="plots", visible=False),
            rrb.TextDocumentView(origin="/status", name="status", visible=False),
        ])

        blueprint = rrb.Blueprint(
            rrb.Horizontal(
                *color_views,
                rrb.Spatial3DView(origin="/world", name="world (pose)"),
                *hidden_views,
                column_shares=[1.0, 1.0, 1.0] + [0.0] * len(hidden_views),
                name="pika live",
            ),
            auto_layout=False,
            auto_views=False,
        )
        rr.send_blueprint(blueprint)

    # ---- 시계열/상태: per_arm = [(name, angle, closed), ...] ----
    def state(self, recording, ep, saved, rec_secs, per_arm):
        rr = self.rr
        rr.log("plots/recording", rr.Scalars(1.0 if recording else 0.0))
        for name, angle, closed in per_arm:
            if angle == angle:  # not NaN
                rr.log(f"plots/{name}/gripper_angle", rr.Scalars(float(angle)))
            rr.log(f"plots/{name}/gripper_closed", rr.Scalars(1.0 if closed else 0.0))
        head = f"● REC   {rec_secs:4.1f}s" if recording else "○ IDLE"
        body = "   ".join(f"{n}:{'CLOSE' if c else 'open '}{a:6.1f}" for n, a, c in per_arm)
        rr.log("status", rr.TextDocument(
            f"{head}   ep={ep:03d}  saved={saved}   |   {body}   (any double-pinch = toggle)"))

    # ---- 3D 트래커 포즈 + 궤적(팔별) ----
    def pose(self, name, pose, recording):
        rr = self.rr
        if pose is None or pose[0] != pose[0]:  # NaN
            return
        rr.log(f"world/{name}/tracker",
               rr.Transform3D(translation=pose[:3], quaternion=rr.Quaternion(xyzw=pose[3:7])))
        rr.log(f"world/{name}/tracker/axes",
               rr.Arrows3D(origins=[[0, 0, 0]] * 3,
                           vectors=[[0.15, 0, 0], [0, 0.15, 0], [0, 0, 0.15]],
                           colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]))
        tr = self._trails.setdefault(name, [])
        tr.append([float(pose[0]), float(pose[1]), float(pose[2])])
        if len(tr) > self.trail_len:
            tr.pop(0)
        col = [255, 40, 40] if recording else _ARM_COLORS.get(name, _DEFAULT_COL)
        rr.log(f"world/{name}/trail", rr.LineStrips3D([tr], colors=[col]))

    # ---- 카메라(인코딩 바이트 그대로, img_every 프레임마다) ----
    def images(self, name, arm):
        c = self._fc.get(name, 0) + 1
        self._fc[name] = c
        if c % self.img_every:
            return
        rr = self.rr
        rc = arm.get("realsense_color")
        if rc is not None and rc.size > 0:
            rr.log(f"camera/{name}/d405_color", rr.EncodedImage(contents=rc.tobytes(), media_type="image/png"))
        rd = arm.get("realsense_depth")
        if rd is not None and rd.size > 0:
            rr.log(f"camera/{name}/d405_depth", rr.EncodedImage(contents=rd.tobytes(), media_type="image/png"))
        fc = arm.get("fisheye_color")
        if fc is not None and fc.size > 0:
            rr.log(f"camera/{name}/fisheye_color", rr.EncodedImage(contents=fc.tobytes(), media_type="image/png"))

    def close(self):
        pass
