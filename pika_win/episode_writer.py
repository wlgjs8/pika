"""에피소드 HDF5 writer — 별도 프로세스에서 디스크 쓰기(캡처 루프 GIL 격리).

수집 루프(메인 프로세스)는 이미지 프레임을 캡처 즉시 PNG로 점진 인코딩하고
(recorder.encode_frame), 에피소드 종료 시 인코딩된 bytes + 스칼라 배열만 담은
picklable payload(recorder.build_payload)를 만들어 이 모듈의 writer 프로세스로 넘긴다.

writer 프로세스는 cv2/pyrealsense/openvr 등 무거운 의존성 없이 h5py + numpy 만으로
HDF5 를 조립·기록하므로:
  - 인코딩(PNG)은 메인 프로세스 스레드풀에서 캡처 시간 동안 균등 분산되고,
  - 대용량 HDF5 디스크 쓰기는 별도 프로세스(자체 GIL)에서 일어나
    수집 루프(순수 파이썬)가 GIL/디스크 경합으로 굶지 않는다.

레이아웃은 recorder.write_episode 와 동일(하위 호환): 평면(단일)/그룹(양팔).
"""
import multiprocessing as mp
import os
import traceback

import h5py
import numpy as np


def _write_payload_obs(grp, data, vlen):
    """payload 의 한 팔 데이터 → pose/gripper/command/images 작성, action 반환.

    이미지는 이미 인코딩된 PNG bytes 리스트이므로 여기서는 인코딩하지 않는다
    (recorder._write_obs 의 디스크측 미러, 인코딩 단계 제거 버전).
    """
    pose = np.asarray(data["pose"], np.float32)
    grip = np.asarray(data["gripper"], np.float32)
    cmd = np.asarray(data["command"], np.int8)
    grp.create_dataset("pose", data=pose)
    grp.create_dataset("gripper", data=grip)
    grp.create_dataset("command", data=cmd)
    # 스트림별 호스트 타임스탬프(time.time()). 카메라를 마스터로 pose/gripper 를 보간하는
    # 후처리 타임싱크의 재료 — 없던 레거시 payload 도 그대로 통과한다(하위호환).
    for ts_key in ("pose_sample_ts", "rs_ts", "gripper_ts"):
        if ts_key in data:
            grp.create_dataset(ts_key, data=np.asarray(data[ts_key], np.float64))
    # 카메라=마스터 보간을 저장 시점에 자동 수행 (pose_synced/gripper_synced).
    # 원본은 그대로 두는 파생 데이터라 실패해도 에피소드는 유효 -> fail-open.
    # 재실행/감사는 scripts/resync_episode.py.
    if "rs_ts" in data:
        try:
            from .timesync import sync_arm
            synced = sync_arm(pose, data.get("pose_sample_ts"), grip,
                              data.get("gripper_ts"), data["rs_ts"])
            for k, v in synced.items():
                grp.create_dataset(k, data=v)
        except Exception as e:  # noqa: BLE001
            print(f"[writer] timesync 스킵({e}) -- 원본 스트림은 저장됨", flush=True)
    img = grp.create_group("images")
    for key, buffers in data["images"].items():
        ds = img.create_dataset(key, (len(buffers),), dtype=vlen)
        for i, buf in enumerate(buffers):
            ds[i] = buf
        ds.attrs["encoding"] = "png16" if key.endswith("depth") else "png"
    return np.concatenate([pose, grip[:, 1:2]], axis=1).astype(np.float32)


def _write_payload_calib(grp, calib):
    """RealSense 정적 캘리브 dict → grp/camera_calib (recorder._write_camera_calib 미러)."""
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
        sub.create_dataset("coeffs", data=np.asarray(intr["coeffs"], np.float64))
    # depth 를 끈 수집분은 depth 관련 항목이 없다 → intrinsics 만 쓰고 빠진다.
    if "depth_to_color_rotation" not in calib:
        cc.attrs["depth_aligned_to_color"] = bool(calib.get("depth_aligned_to_color", False))
        return
    R = np.asarray(calib["depth_to_color_rotation"], np.float64).reshape((3, 3), order="F")
    cc.create_dataset("depth_to_color_rotation", data=R)
    cc.create_dataset("depth_to_color_translation",
                      data=np.asarray(calib["depth_to_color_translation"], np.float64))
    cc.attrs["rotation_layout"] = "row_major_3x3; p_color = R @ p_depth + t"
    cc.attrs["translation_units"] = "meters"
    cc.attrs["depth_scale"] = calib["depth_scale"]
    if calib.get("stereo_baseline_mm") is not None:
        cc.attrs["stereo_baseline_mm"] = calib["stereo_baseline_mm"]
    cc.attrs["depth_aligned_to_color"] = calib["depth_aligned_to_color"]


def _flush_and_drop_cache(path):
    """파일을 디스크로 즉시 flush(writeback)하고 page-cache 에서 제거.

    수집 호스트는 에피소드가 500MB+/7s 라 page-cache 가 dirty 페이지로 가득 차고,
    캡처 프로세스의 프레임당 할당이 direct-reclaim 에서 느린 SSD writeback 을 기다리며
    멈춘다(멀티초 frame gap 의 정체). 저장 직후 fsync(dirty 누적 차단) + fadvise DONTNEED
    (clean 페이지 캐시 반환)로 캡처 프로세스의 reclaim 압박을 줄인다. writer 프로세스에서만
    수행되므로 캡처 경로엔 영향 없음(저장은 7s 에피소드 대비 ~1s 로 여유 충분)."""
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(fd)
        if hasattr(os, "posix_fadvise"):
            os.posix_fadvise(fd, 0, 0, os.POSIX_FADV_DONTNEED)
    except OSError:
        pass
    finally:
        os.close(fd)


def write_episode_payload(path, payload):
    """미리 인코딩된 payload → HDF5 파일(.tmp 후 atomic replace). 경로 반환."""
    names = payload["names"]
    n = len(names)
    ts = np.asarray(payload["ts"], np.float64)
    eff = len(ts) / max(ts[-1] - ts[0], 1e-6) if len(ts) > 1 else 0.0
    vlen = h5py.vlen_dtype(np.uint8)
    abs_path = os.path.abspath(path)
    os.makedirs(os.path.dirname(abs_path), exist_ok=True)
    tmp_path = abs_path + ".tmp"
    try:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)
        with h5py.File(tmp_path, "w") as h:
            h.attrs["record_hz"] = payload["record_hz"]
            h.attrs["effective_hz"] = eff
            # pose_frame/pose_backend 는 recorder.pose_frame_attrs() 가 확정해 payload 에
            # 실어 보낸다(계산을 두 벌 두면 갈라진다). 이 키가 없는 payload 는 트래커
            # 백엔드가 없는 외부 생산자(robotics_lab collect_onrobot_teleop 등)뿐이고,
            # 그쪽은 저장 후 pose_frame 을 자기 값으로 덮어쓴다 → 레거시 이름만 남기고
            # pose_backend 는 아예 쓰지 않는다(모르는 값을 지어내지 않기 위해).
            h.attrs["pose_frame"] = payload.get("pose_frame") or (
                "steamvr_world_gripper_tip" if payload["pose_tip_frame"] else "steamvr_world")
            if payload.get("pose_backend"):
                h.attrs["pose_backend"] = payload["pose_backend"]
            h.attrs["pose_format"] = "x,y,z,qx,qy,qz,qw"
            h.attrs["n_arms"] = n
            h.attrs["arm_names"] = ",".join(names)
            # per-session bolt-color assignment (default normal if a legacy payload omits it)
            h.attrs["arm_bolt_colors"] = payload.get("arm_bolt_colors", "right=black,left=gray")
            h.create_dataset("timestamp", data=ts)
            if n == 1:
                meta = payload["arms_meta"][0]
                data = payload["arms_data"][0]
                h.attrs["realsense_sn"] = meta["realsense_sn"]
                h.attrs["fisheye_dev"] = meta["fisheye_dev"]
                obs = h.create_group("observations")
                action = _write_payload_obs(obs, data, vlen)
                h.create_dataset("action", data=action)
                _write_payload_calib(obs, meta["calib"])
            else:
                for ai, name in enumerate(names):
                    g = h.create_group(f"observations/{name}")
                    meta = payload["arms_meta"][ai]
                    data = payload["arms_data"][ai]
                    g.attrs["realsense_sn"] = meta["realsense_sn"]
                    g.attrs["tracker_sn"] = meta["tracker_sn"]
                    g.attrs["fisheye_dev"] = meta["fisheye_dev"]
                    action = _write_payload_obs(g, data, vlen)
                    g.create_dataset("action", data=action)
                    _write_payload_calib(g, meta["calib"])
        os.replace(tmp_path, abs_path)
    except BaseException:
        try:
            os.unlink(tmp_path)
        except FileNotFoundError:
            pass
        raise
    _flush_and_drop_cache(abs_path)
    return abs_path


def _writer_loop(in_q, out_q):
    """자식 프로세스 메인 루프: (path, payload) 수신 → 기록 → ("ok"/"err", info) 응답.

    fork 로 생성되면 부모의 atexit(예: 수집 락 해제)를 상속하므로, 종료 시 그것이 실행돼
    부모의 락 파일을 지우지 않도록 os._exit 로 즉시 종료한다.
    """
    while True:
        item = in_q.get()
        if item is None:
            break
        path, payload = item
        try:
            write_episode_payload(path, payload)
            out_q.put(("ok", path))
        except BaseException as exc:  # noqa: BLE001  부모로 에러 전달
            out_q.put(("err", f"{exc}\n{traceback.format_exc()}"))
    os._exit(0)


class EpisodeWriterProcess:
    """HDF5 디스크 쓰기를 전담하는 영속 자식 프로세스(spawn).

    write() 는 payload 를 자식에 보내고 완료까지 블록한다. 호출 스레드(저장 worker)는
    out_q.get() 대기 중 GIL 을 놓으므로, 그 사이 수집 루프가 30Hz 로 계속 돈다.
    """
    def __init__(self):
        # posix: fork(경량, 하드웨어 스레드 시작 전 단일 스레드 상태에서 생성하면 안전).
        # 그 외(Windows): spawn 폴백. 어느 쪽이든 payload 는 Queue 로 pickle 전달.
        try:
            ctx = mp.get_context("fork")
        except ValueError:
            ctx = mp.get_context("spawn")
        self._in = ctx.Queue()
        self._out = ctx.Queue()
        self._proc = ctx.Process(
            target=_writer_loop, args=(self._in, self._out),
            name="EpisodeWriter", daemon=True)
        self._proc.start()

    def write(self, path, payload):
        self._in.put((path, payload))
        status, info = self._out.get()
        if status != "ok":
            raise RuntimeError(f"[writer] episode write failed: {info}")
        return info

    def close(self):
        try:
            self._in.put(None)
        except Exception:
            pass
        if self._proc is not None:
            self._proc.join(timeout=10.0)
            if self._proc.is_alive():
                self._proc.terminate()
            self._proc = None
