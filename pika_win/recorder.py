"""PIKA Sense 동기화 에피소드 레코더 (Windows/Linux, 단일/양팔 자동).

스트림(팔당): 포즈(SteamVR 트래커) / 그리퍼각도+command(Sense 시리얼) / RealSense(color+depth)
            / 어안(fisheye color, raw). 어안은 RealSense 와 같은 USB 허브에서 자동 매핑.
인식된 Vive 트래커 개수로 모드 자동 결정:
  - 1개 → SINGLE(한팔)  : 기존 평면 HDF5 레이아웃 그대로 저장(하위 호환).
  - 2개 → BIMANUAL(양팔): 팔별 그룹(observations/<arm>/...)으로 저장.

read_frame() 은 활성 팔 수만큼의 dict 리스트를 담은 프레임을 반환:
  {"ts": float, "arms": [ {pose[7], gripper[2], command, realsense_color, realsense_depth}, ... ]}

HDF5 레이아웃:
  공통 attrs: record_hz, effective_hz, pose_frame, pose_format, n_arms, arm_names
  /timestamp                              [T] f64 (epoch sec)
  [SINGLE] observations/pose [T,7] / gripper [T,2] / command [T] / images/{...} / (top)/action [T,8]
           + attrs realsense_sn
  [BIMANUAL] observations/<arm>/pose,gripper,command,images/{...},action  (팔마다)
           + 그룹 attrs realsense_sn, tracker_sn
  이미지: realsense_color=PNG, realsense_depth=PNG16, fisheye_color=PNG (vlen-u8)
  action [.,8] = pose(7) + gripper_distance(1)  (v1=관측 미러)
"""
import logging
import os
import time
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np

from .pose_steamvr import PoseSteamVR
from .realsense_win import RealSenseD4xx
from .fisheye import FisheyeCamera, resolve_fisheye_node

log = logging.getLogger("pika.recorder")


class ArmSpec:
    """한 팔의 하드웨어 바인딩(설정값)."""
    def __init__(self, name, com_port=None, realsense_sn=None, tracker_sn=None,
                 fisheye_dev=None):
        self.name = name
        self.com_port = com_port
        self.realsense_sn = realsense_sn
        self.tracker_sn = tracker_sn
        # 어안 카메라 디바이스 override(/dev/videoN, 인덱스, by-path).
        # None 이면 RealSense 와 같은 USB 허브에서 자동 매핑.
        self.fisheye_dev = fisheye_dev

    def __repr__(self):
        return (f"ArmSpec({self.name}, com={self.com_port}, "
                f"rs={self.realsense_sn}, tracker={self.tracker_sn}, "
                f"fisheye={self.fisheye_dev})")


class _ArmIO:
    """런타임 연결 묶음(Sense/RealSense) + 확정된 트래커 SN."""
    def __init__(self, spec):
        self.spec = spec
        self.sense = None
        self.rs = None
        self.fisheye = None
        self.fisheye_dev = None   # 런타임에 확정(자동 매핑 또는 spec override)
        self.tracker_sn = spec.tracker_sn   # 런타임에 확정될 수 있음


class EpisodeRecorder:
    # 저장 대상 이미지 스트림(인코딩/기록 순서 고정)
    IMAGE_KEYS = ("realsense_color", "realsense_depth", "fisheye_color")

    def __init__(self, out_dir, arms=None, record_hz=30, jpeg_quality=90,
                 use_pose=True, use_sense=True, use_realsense=True, use_fisheye=True,
                 settle=1.0, require_pose=False, require_all_trackers=False,
                 pose_valid_timeout=2.0, pose_tip_frame=False,
                 png_compression=1, png_depth_compression=None, encode_workers=None,
                 arm_bolt_colors="right=black,left=gray",
                 # ---- 레거시 단일-팔 호환 kwargs (arms 미지정 시 사용) ----
                 com_port="COM3", realsense_sn=None):
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        if arms is None:
            arms = [ArmSpec("arm", com_port=com_port, realsense_sn=realsense_sn)]
        self.arms_cfg = list(arms)
        self.record_hz = record_hz
        self.jpeg_quality = jpeg_quality
        self.png_compression = self._clamp_png_compression(png_compression)
        self.png_depth_compression = (
            None if png_depth_compression is None
            else self._clamp_png_compression(png_depth_compression)
        )
        if encode_workers is None or int(encode_workers) <= 0:
            self.encode_workers = max(1, min(8, (os.cpu_count() or 4)))
        else:
            self.encode_workers = max(1, int(encode_workers))
        self.flags = dict(pose=use_pose, sense=use_sense, realsense=use_realsense,
                          fisheye=use_fisheye)
        self.settle = settle
        self.require_pose = bool(require_pose)
        self.require_all_trackers = bool(require_all_trackers)
        self.pose_valid_timeout = float(pose_valid_timeout)
        # True 시 PIKA SDK 공식 트래커→그리퍼 팁 변환을 적용해 발행/기록
        # (pose_steamvr.apply_tip_transform 참조)
        self.pose_tip_frame = bool(pose_tip_frame)
        # 세션 단위 볼트-색 배정: 각 팔이 이번 세션에서 집는 볼트 색 (예: "right=black,left=gray"=normal,
        # "right=gray,left=black"=swap). 매 에피소드 HDF5 root attr `arm_bolt_colors`로 기록 →
        # 변환기가 색-grounded 프롬프트를 배정. 미지정/레거시 데이터는 normal(right=black,left=gray)로 간주.
        self.arm_bolt_colors = str(arm_bolt_colors)
        self.pose = None
        self.active = []   # list[_ArmIO] — 실제 활성 팔(1 또는 2)
        # 캡처 중 이미지 PNG 인코딩을 담당하는 스레드풀(cv2.imencode 는 GIL 해제).
        # 인코딩을 캡처 시간에 균등 분산 → 저장 시점 인코딩 버스트/원본 프레임 RAM 적재 제거.
        self._encode_pool = ThreadPoolExecutor(
            max_workers=self.encode_workers, thread_name_prefix="enc")

    @staticmethod
    def _clamp_png_compression(value):
        return max(0, min(9, int(value)))

    # ---------------- lifecycle ----------------
    def start(self):
        # 1) 포즈(SteamVR) 1개만 연결 — 모든 트래커가 공유
        if self.flags["pose"]:
            try:
                self.pose = PoseSteamVR(
                    target_hz=250, apply_gripper_offset=self.pose_tip_frame).connect()
                log.info("[pose] SteamVR 연결 (frame=%s)",
                         "gripper_tip" if self.pose_tip_frame else "tracker_raw")
            except Exception as e:
                log.error("[pose] SteamVR/OpenVR 연결 실패: %s", e)
                self.pose = None
                if self.require_pose:
                    raise RuntimeError("[pose] SteamVR/OpenVR 연결 실패 — SteamVR 실행 및 openvr 경로 확인") from e
        elif self.require_pose:
            raise RuntimeError("[pose] --require-pose 는 --no-pose 모드와 함께 사용할 수 없습니다.")

        # 2) 트래커 안정화 후 인식된 시리얼 수집 → 모드 결정
        detected = self._detect_trackers()
        log.info("[mode] 인식된 트래커 %d개: %s", len(detected), detected)
        if self.require_pose and not detected:
            self._raise_start_error("[pose] 유효한 Vive 트래커 pose가 없습니다 — tracker 전원/SteamVR tracking 확인")
        self._validate_required_trackers(detected)
        self.active = self._resolve_active_arms(detected)
        names = [io.spec.name for io in self.active]
        log.info("[mode] %s → arms=%s", "BIMANUAL(양팔)" if len(self.active) > 1 else "SINGLE(한팔)", names)
        if self.require_pose:
            self._validate_active_pose()

        # 3) 각 활성 팔의 Sense / RealSense 연결
        if self.flags["sense"]:
            from pika.sense import Sense
            for io in self.active:
                s = io.spec
                if not s.com_port:
                    raise RuntimeError(f"[{s.name}] Sense COM 포트 미지정 — config/arms.json 또는 --coms 로 지정하세요.")
                io.sense = Sense(port=s.com_port)
                log.info("[%s] sense %s connect -> %s", s.name, s.com_port, io.sense.connect())
        if self.flags["realsense"]:
            for io in self.active:
                s = io.spec
                io.rs = RealSenseD4xx(serial=(s.realsense_sn or None)).connect()
                log.info("[%s] realsense %s connected", s.name, s.realsense_sn or "(auto)")
        if self.flags["fisheye"]:
            for io in self.active:
                s = io.spec
                # 1) spec override 우선, 2) 없으면 RealSense 와 같은 USB 허브에서 자동 매핑
                dev = s.fisheye_dev
                if dev in (None, "", "auto"):
                    rs_port = getattr(io.rs, "physical_port", None) if io.rs else None
                    dev = resolve_fisheye_node(rs_port) if rs_port else None
                if not dev:
                    log.warning("[%s] 어안 카메라 미발견 — RealSense 허브 매핑 실패. "
                                "config fisheye_dev 또는 --fisheye-devs 로 지정하세요.", s.name)
                    continue
                try:
                    io.fisheye = FisheyeCamera(dev).connect()
                    io.fisheye_dev = dev
                    log.info("[%s] fisheye %s connected", s.name, dev)
                except Exception as e:
                    log.error("[%s] 어안 카메라 연결 실패(%s): %s", s.name, dev, e)
                    io.fisheye = None
        for io in self.active:
            log.info("[%s] tracker → %s", io.spec.name, io.tracker_sn or "(순서배정)")
        time.sleep(0.8)
        return self

    def _raise_start_error(self, message):
        self.stop()
        raise RuntimeError(message)

    def _configured_tracker_sns(self):
        return [s.tracker_sn for s in self.arms_cfg if s.tracker_sn]

    def _validate_required_trackers(self, detected):
        if not (self.require_pose and self.require_all_trackers):
            return
        expected = self._configured_tracker_sns()
        if not expected:
            self._raise_start_error("[pose] --require-all-trackers 사용 시 --tracker-sns 또는 config tracker_sn 이 필요합니다.")
        missing = [sn for sn in expected if sn not in detected]
        if missing:
            self._raise_start_error(
                "[pose] 설정된 tracker SN이 보이지 않습니다. "
                f"expected={expected} detected={detected} missing={missing}"
            )

    def _validate_active_pose(self):
        deadline = time.perf_counter() + max(0.1, self.pose_valid_timeout)
        missing = [io.tracker_sn or io.spec.name for io in self.active]
        while time.perf_counter() < deadline:
            missing = []
            for io in self.active:
                pose = self.pose.get_pose(io.tracker_sn) if (self.pose and io.tracker_sn) else None
                if not (pose and pose.get("valid")):
                    missing.append(io.tracker_sn or io.spec.name)
            if not missing:
                for io in self.active:
                    log.info("[%s] tracker pose valid", io.spec.name)
                return
            time.sleep(0.05)
        detected = self.pose.get_devices() if self.pose else []
        self._raise_start_error(
            "[pose] 활성 arm tracker pose가 유효하지 않습니다. "
            f"active={[io.tracker_sn for io in self.active]} detected={detected} invalid={missing}"
        )

    def _detect_trackers(self):
        """settle 동안 폴링해 안정적으로 보이는 트래커 시리얼 집합을 정렬 반환."""
        if self.pose is None:
            return []
        t0 = time.perf_counter()
        seen = set()
        while time.perf_counter() - t0 < self.settle:
            for sn in self.pose.get_devices():
                seen.add(sn)
            time.sleep(0.05)
        return sorted(seen)

    def _resolve_active_arms(self, detected):
        """트래커 개수/SN으로 활성 팔 목록 결정.

        - 모든 팔에 tracker_sn 설정 + 그 SN이 보이면 → SN 매핑(정확한 좌/우·Sense 짝).
        - 아니면 → 트래커 개수만큼 cfg 순서대로 활성 + 정렬 트래커를 순서 배정(경고).
        """
        ios = [_ArmIO(s) for s in self.arms_cfg]
        all_have_sn = bool(ios) and all(io.spec.tracker_sn for io in ios)
        if all_have_sn and detected:
            active = [io for io in ios if io.spec.tracker_sn in detected]
            if active:
                for io in active:
                    io.tracker_sn = io.spec.tracker_sn
                return active[:2]
            log.warning("[mode] 설정한 tracker_sn 이 하나도 안 보임 → 순서 배정으로 폴백")
        n = len(detected) if detected else 1
        n = max(1, min(n, len(ios)))
        active = ios[:n]
        for i, io in enumerate(active):
            io.tracker_sn = detected[i] if (detected and i < len(detected)) else io.spec.tracker_sn
        if detected and not all_have_sn and len(active) > 1:
            log.warning("[mode] 트래커 SN 미설정 → 정렬 순서로 배정. "
                        "좌/우·Sense 짝이 어긋나면 --tracker-sns 로 고정하세요.")
        return active

    def arm_names(self):
        return [io.spec.name for io in self.active]

    @property
    def n_arms(self):
        return len(self.active)

    # ---------------- encoding helpers ----------------
    def _png_color(self, frame):
        if frame is None:
            return np.zeros((0,), np.uint8)
        ok, buf = cv2.imencode(
            ".png", frame, [int(cv2.IMWRITE_PNG_COMPRESSION), self.png_compression])
        return buf.reshape(-1) if ok else np.zeros((0,), np.uint8)

    def _png16(self, depth):
        if depth is None:
            return np.zeros((0,), np.uint8)
        params = []
        if self.png_depth_compression is not None:
            params = [int(cv2.IMWRITE_PNG_COMPRESSION), self.png_depth_compression]
        ok, buf = cv2.imencode(".png", depth, params)
        return buf.reshape(-1) if ok else np.zeros((0,), np.uint8)

    def _encode_image(self, key, frame):
        return self._png16(frame) if key.endswith("depth") else self._png_color(frame)

    def read_gripper_angle(self, arm_idx=0):
        """특정 팔의 그리퍼 각도만 빠르게(캘리브/제스처용). 없으면 None."""
        if arm_idx >= len(self.active):
            return None
        io = self.active[arm_idx]
        if io.sense is None:
            return None
        try:
            return io.sense.get_encoder_data().get("angle")
        except Exception:
            return None

    def _read_arm(self, io):
        arm = {}
        # pose (이 팔의 트래커)
        p = [np.nan] * 7
        tr = None        # eTrackingResult (추적 품질) — 멈춤이 트래커 손실인지 판별용
        pose_ts = None   # pose source(폴링 스레드)가 이 트래커를 마지막 '폴링' 한 시각
        pose_seq = None  # 값 변화 시에만 증가 — 중복(같은 샘플 재독) 판별용
        pose_sample_ts = None  # pose 값이 마지막으로 '바뀐' 시각
        if self.pose is not None:
            pd = self.pose.get_pose(io.tracker_sn) if io.tracker_sn else self.pose.get_pose()
            if isinstance(pd, dict) and pd and "position" not in pd:
                pd = next(iter(pd.values()), None)
            if pd and pd.get("valid"):
                p = list(pd["position"]) + list(pd["rotation"])
                tr = pd.get("tracking_result")
                pose_ts = pd.get("timestamp")
                pose_seq = pd.get("sample_seq")
                pose_sample_ts = pd.get("sample_ts")
        arm["pose"] = p
        arm["tracking_result"] = tr
        arm["pose_ts"] = pose_ts
        arm["pose_seq"] = pose_seq
        arm["pose_sample_ts"] = pose_sample_ts
        # gripper + command
        ga = gd = np.nan
        cs = -1
        if io.sense is not None:
            try:
                ga = io.sense.get_encoder_data().get("angle", np.nan)
                gd = io.sense.get_gripper_distance()
                cs = io.sense.get_command_state()
            except Exception:
                pass
        arm["gripper"] = [ga, gd]
        arm["command"] = cs if isinstance(cs, (int, float)) else -1
        # camera (RealSense color+depth)
        if io.rs is not None:
            c, d, _ = io.rs.get_frames()
            arm["realsense_color"] = c
            arm["realsense_depth"] = d
        # 어안 카메라(raw fisheye → 저장 시 PNG)
        if io.fisheye is not None:
            fc, _ = io.fisheye.get_frame()
            arm["fisheye_color"] = fc
        return arm

    def read_frame(self):
        """활성 팔 전체의 현재 프레임을 반환. {"ts", "arms":[arm0, arm1?]}"""
        return {"ts": time.time(), "arms": [self._read_arm(io) for io in self.active]}

    # ---------------- 점진 인코딩 / payload (async writer 경로) ----------------
    def encode_frame(self, fr):
        """raw 프레임 → 이미지를 PNG 인코딩 Future 로 치환한 compact 프레임.

        캡처 루프가 매 녹화 프레임마다 호출한다. 인코딩은 백그라운드 풀에서 일어나므로
        호출은 사실상 논블로킹이고, 반환된 compact 프레임만 보관하면 원본 이미지(raw)는
        다음 tick 에 해제된다(에피소드 RAM = 인코딩 버퍼 합, 인코딩 버스트 없음).
        """
        arms_c = []
        for arm in fr["arms"]:
            enc = {}
            for key in self.IMAGE_KEYS:
                if key in arm:
                    enc[key] = self._encode_pool.submit(self._encode_image, key, arm[key])
            arms_c.append({
                "pose": arm["pose"],
                "gripper": arm["gripper"],
                "command": arm["command"],
                "enc": enc,
            })
        return {"ts": fr["ts"], "arms": arms_c}

    def build_payload(self, frames_c):
        """compact 프레임 리스트 → writer 프로세스로 보낼 picklable payload.

        Future 들을 resolve(대개 이미 인코딩 완료) 하여 PNG bytes 로 고정한다. cv2/하드웨어
        의존성이 없는 순수 데이터(dict/np.ndarray)만 담아 자식 프로세스로 넘긴다.
        """
        names = self.arm_names() or ["arm"]
        n = len(names)
        arms_meta = []
        for ai in range(n):
            io = self.active[ai] if ai < len(self.active) else None
            s = io.spec if io is not None else self.arms_cfg[ai]
            arms_meta.append({
                "realsense_sn": str((s.realsense_sn if s else None) or ""),
                "tracker_sn": str((getattr(io, "tracker_sn", None) if io else None) or ""),
                "fisheye_dev": str((getattr(io, "fisheye_dev", None) if io else None) or ""),
                "calib": (getattr(io.rs, "calib", None) if (io is not None and io.rs is not None) else None),
            })
        arms_data = []
        for ai in range(n):
            keys = list(frames_c[0]["arms"][ai]["enc"].keys()) if frames_c else []
            images = {
                key: [f["arms"][ai]["enc"][key].result() for f in frames_c]
                for key in keys
            }
            arms_data.append({
                "pose": [f["arms"][ai]["pose"] for f in frames_c],
                "gripper": [f["arms"][ai]["gripper"] for f in frames_c],
                "command": [f["arms"][ai]["command"] for f in frames_c],
                "images": images,
            })
        return {
            "record_hz": self.record_hz,
            "pose_tip_frame": self.pose_tip_frame,
            "arm_bolt_colors": self.arm_bolt_colors,
            "names": names,
            "ts": [f["ts"] for f in frames_c],
            "arms_meta": arms_meta,
            "arms_data": arms_data,
        }

    # ---------------- HDF5 저장 ----------------
    def _write_obs(self, grp, frames, ai, vlen):
        """grp 아래 pose/gripper/command/images 작성, action 배열 반환."""
        import numpy as _np
        pose = _np.asarray([f["arms"][ai]["pose"] for f in frames], _np.float32)
        grip = _np.asarray([f["arms"][ai]["gripper"] for f in frames], _np.float32)
        cmd = _np.asarray([f["arms"][ai]["command"] for f in frames], _np.int8)
        grp.create_dataset("pose", data=pose)
        grp.create_dataset("gripper", data=grip)
        grp.create_dataset("command", data=cmd)
        img = grp.create_group("images")

        def _vds(key):
            if key not in frames[0]["arms"][ai]:
                return
            values = [fr["arms"][ai].get(key) for fr in frames]
            with ThreadPoolExecutor(max_workers=self.encode_workers) as ex:
                encoded = list(ex.map(lambda frame: self._encode_image(key, frame), values))
            ds = img.create_dataset(key, (len(frames),), dtype=vlen)
            for i, buf in enumerate(encoded):
                ds[i] = buf
            ds.attrs["encoding"] = "png16" if key.endswith("depth") else "png"

        _vds("realsense_color")
        _vds("realsense_depth")
        _vds("fisheye_color")
        return _np.concatenate([pose, grip[:, 1:2]], axis=1).astype(_np.float32)

    def _write_camera_calib(self, grp, io):
        """RealSense 정적 캘리브(intrinsics/extrinsic)를 grp/camera_calib 에 기록."""
        import numpy as _np
        calib = getattr(io.rs, "calib", None) if (io is not None and io.rs is not None) else None
        if not calib:
            return
        cc = grp.create_group("camera_calib")
        for key in ("color_intrinsics", "depth_intrinsics"):
            intr = calib.get(key)
            if not intr:
                continue
            sub = cc.create_group(key)
            for k in ("width", "height", "fx", "fy", "ppx", "ppy", "model"):
                sub.attrs[k] = intr[k]
            sub.create_dataset("coeffs", data=_np.asarray(intr["coeffs"], _np.float64))
        # column-major 9 -> 실제 3x3 회전행렬 (p_color = R @ p_depth + t)
        R = _np.asarray(calib["depth_to_color_rotation"], _np.float64).reshape((3, 3), order="F")
        cc.create_dataset("depth_to_color_rotation", data=R)
        cc.create_dataset("depth_to_color_translation",
                          data=_np.asarray(calib["depth_to_color_translation"], _np.float64))
        cc.attrs["rotation_layout"] = "row_major_3x3; p_color = R @ p_depth + t"
        cc.attrs["translation_units"] = "meters"
        cc.attrs["depth_scale"] = calib["depth_scale"]
        if calib.get("stereo_baseline_mm") is not None:
            cc.attrs["stereo_baseline_mm"] = calib["stereo_baseline_mm"]
        cc.attrs["depth_aligned_to_color"] = calib["depth_aligned_to_color"]

    def write_episode(self, path, frames):
        """프레임 리스트 → HDF5. 활성 팔 수에 따라 평면(단일)/그룹(양팔) 레이아웃."""
        import h5py
        if not frames:
            print("[rec] 빈 에피소드 — 저장 생략")
            return None
        names = self.arm_names() or ["arm"]
        n = len(names)
        vlen = h5py.vlen_dtype(np.uint8)
        ts = np.asarray([f["ts"] for f in frames], np.float64)
        eff = len(frames) / max(ts[-1] - ts[0], 1e-6) if len(frames) > 1 else 0.0
        abs_path = os.path.abspath(path)
        os.makedirs(os.path.dirname(abs_path), exist_ok=True)
        tmp_path = abs_path + ".tmp"
        try:
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)
            with h5py.File(tmp_path, "w") as h:
                h.attrs["record_hz"] = self.record_hz
                h.attrs["effective_hz"] = eff
                # tip frame: 동일 world 에서 포즈 원점만 트래커→그리퍼 팁(공식 변환)으로 이동
                h.attrs["pose_frame"] = (
                    "steamvr_world_gripper_tip" if self.pose_tip_frame else "steamvr_world")
                h.attrs["pose_format"] = "x,y,z,qx,qy,qz,qw"
                h.attrs["n_arms"] = n
                h.attrs["arm_names"] = ",".join(names)
                h.attrs["arm_bolt_colors"] = self.arm_bolt_colors
                h.create_dataset("timestamp", data=ts)

                if n == 1:
                    # ---- 기존 평면 레이아웃(하위 호환) ----
                    s = self.active[0].spec if self.active else self.arms_cfg[0]
                    h.attrs["realsense_sn"] = str(s.realsense_sn or "")
                    h.attrs["fisheye_dev"] = str(self.active[0].fisheye_dev or "") if self.active else ""
                    obs = h.create_group("observations")
                    action = self._write_obs(obs, frames, 0, vlen)
                    h.create_dataset("action", data=action)
                    if self.active:
                        self._write_camera_calib(obs, self.active[0])
                else:
                    # ---- 양팔: 팔별 그룹 ----
                    for ai, name in enumerate(names):
                        g = h.create_group(f"observations/{name}")
                        s = self.active[ai].spec
                        g.attrs["realsense_sn"] = str(s.realsense_sn or "")
                        g.attrs["tracker_sn"] = str(self.active[ai].tracker_sn or "")
                        g.attrs["fisheye_dev"] = str(self.active[ai].fisheye_dev or "")
                        action = self._write_obs(g, frames, ai, vlen)
                        g.create_dataset("action", data=action)
                        self._write_camera_calib(g, self.active[ai])
            os.replace(tmp_path, abs_path)
        except BaseException:
            try:
                os.unlink(tmp_path)
            except FileNotFoundError:
                pass
            raise
        print(f"[rec] 저장 {path}  frames={len(frames)}  arms={n}  eff_hz={eff:.1f}")
        return path

    def record(self, duration=5.0, name="episode"):
        """고정 시간 녹화(단순 모드, 단일/양팔 공통)."""
        period = 1.0 / self.record_hz
        frames = []
        t0 = time.perf_counter()
        print(f"[rec] {duration}s @ {self.record_hz}Hz 시작...")
        while time.perf_counter() - t0 < duration:
            tick = time.perf_counter()
            frames.append(self.read_frame())
            rem = period - (time.perf_counter() - tick)
            if rem > 0:
                time.sleep(rem)
        return self.write_episode(os.path.join(self.out_dir, f"{name}.hdf5"), frames)

    def stop(self):
        if self._encode_pool is not None:
            try:
                self._encode_pool.shutdown(wait=True)
            except Exception:
                pass
            self._encode_pool = None
        for io in self.active:
            for c in (io.rs, io.sense, io.fisheye):
                try:
                    if c is not None:
                        c.disconnect()
                except Exception:
                    pass
        try:
            if self.pose is not None:
                self.pose.disconnect()
        except Exception:
            pass
        print("[rec] 모든 스트림 정리 완료")
