# PIKA Sense 데이터 수집 — 셋업 & 진행 계획 (Windows 네이티브 / Ubuntu Python HDF5)

> 대상: AgileX **PIKA Sense** (어안 RGB + RealSense 뎁스 + Vive Tracker 6DoF + 그리퍼 각도 + IMU)
> 추적: **Vive/SteamVR Lighthouse 2.0** + Vive Tracker 3.0 (동글 무선)
> 실행 환경: **Windows 네이티브 또는 Ubuntu Linux + conda env `pika` (Python 3.10)**
> 포즈: **SteamVR + pyopenvr** (공식 SDK의 pysurvive/libsurvive 경로 대체)
> 수집 포맷: **HDF5 / ACT**

---

## Ubuntu 오프라인 수집 경로

`robotics_lab/policy_runner flow-train`과 바로 맞추는 1차 경로는 ROS가
아니라 이 디렉터리의 Python HDF5 수집기다. Ubuntu에서는 다음 순서로
검증한다.

1. Python 환경:
   ```bash
   conda create -y -n pika python=3.10
   conda run -n pika python -m pip install pyserial numpy opencv-python h5py pyrealsense2 openvr pillow
   conda run -n pika python -m pip install --no-deps agx-pypika
   ```
2. 장치 확인:
   ```bash
   conda run -n pika python scripts/detect_hardware.py
   ```
   Linux에서는 `config/arms.json`의 `com_port`에 `COM3` 대신
   `/dev/serial/by-id/...` 경로를 쓰는 것을 권장한다.
3. 좌/우 매핑 고정:
   ```bash
   conda run -n pika python scripts/identify_arms.py
   ```
4. 수집:
   ```bash
   conda run -n pika python scripts/collect.py --hz 30
   ```
   기존 Windows `config/arms.json`을 무시하고 CLI 포트를 쓰려면
   `--config '' --coms /dev/serial/by-id/<right>,/dev/serial/by-id/<left>`를
   지정한다.
5. `robotics_lab` flow 학습 smoke:
   ```bash
   cd /home/plaif/workspace/robotics_lab
   PYTHONPATH=policy_runner python3 -m policy_runner flow-train \
     --episodes-dir /home/plaif/workspace/pai_rectified_flow_matching/pika/data \
     --checkpoint outputs/flow_policy.pt \
     --epochs 1 --batch-size 1 --device cpu
   ```

현재 수집기는 OpenVR/SteamVR 포즈를 우선 사용한다. SteamVR이 Ubuntu에서
불안정하면 libsurvive 기반 포즈 백엔드는 별도 어댑터로 추가해야 한다.

## 왜 Windows 네이티브인가 (WSL2에서 전환)
WSL2/usbipd 경로는 두 하드웨어 전선에서 막혔다:
1. **libsurvive 추적 실패** — usbipd(USB/IP)의 타이밍 지터로 scene solve가 발산(42→141M), `seed runs 15/2262`, PoseUpdate 0개. libsurvive는 sub-ms USB 타이밍에 의존하는데 USB/IP가 이를 보장 못 함.
2. **RealSense 뎁스 불가** — WSL2 기본 커널에 UVC/V4L2 없음(커널 재빌드 필요).

→ Windows 네이티브는 둘 다 해결: SteamVR 추적은 안정적, RealSense는 네이티브 지원. 게다가 **공식 PIKA SDK가 깔끔히 분리돼 있어** 시리얼/센서 레이어를 그대로 재사용 가능.

## SDK 이식성 분석 결과 (핵심)
- `import pika` 는 pysurvive를 **안** 끌어옴 (`__init__`이 `sense/gripper/ego`만 로드; 카메라·트래커는 메서드 내 lazy import).
- `serial_comm.py`/`sense.py`/`gripper.py`/`ego.py` = pyserial 기반 **크로스플랫폼** (기본 포트 `/dev/ttyUSB0`만 `COM*`로).
- `camera/realsense.py` = pyrealsense2 lazy, **Windows 호환**.
- `camera/fisheye.py` = `cv2.CAP_V4L2` 하드코딩 → **Windows는 `cv2.CAP_DSHOW`/`CAP_MSMF`로 한 줄 패치 필요**.
- `tracker/vive_tracker.py` = pysurvive 의존 → **호출 안 함**, SteamVR+pyopenvr로 대체.

## 데이터 경로
| 스트림 | Windows 경로 | 소스 |
|---|---|---|
| 6DoF 포즈 (pos+quat xyzw) | SteamVR ← Lighthouse, `pyopenvr`로 읽기 | 신규 `PoseSteamVR` |
| 그리퍼 각도 / command | `pika.Sense` (시리얼) | 공식 SDK |
| IMU (accel/gyro/mag/quat) | `pika.Ego` (시리얼) | 공식 SDK |
| 어안 RGB | OpenCV (DSHOW 백엔드) | 공식 SDK(패치) |
| RealSense depth+color | `pyrealsense2` | 공식 SDK |

---

## 환경 (✅ 검증 완료 2026-06-01)
- conda env **`pika` (Python 3.10.20)** @ `C:\Users\vision\anaconda3\envs\pika`
- 설치/검증된 import: `pika 0.1.0`(`Sense`/`Gripper`), `pyserial 3.5`, `opencv 4.13`, `pyrealsense2 2.58.1`, `openvr`, `numpy`, `h5py`
- 설치 방법(재현): 
  ```powershell
  $conda="C:\Users\vision\anaconda3\Scripts\conda.exe"
  & $conda create -y -n pika python=3.10
  & $conda run -n pika python -m pip install pyserial numpy opencv-python openvr h5py pyrealsense2
  & $conda run -n pika python -m pip install --no-deps agx-pypika   # pysurvive/wxpython/Gooey 회피
  ```
- Steam + SteamVR **이미 설치됨** (`C:\Program Files (x86)\Steam\...`).

## 진행 단계

### Phase A — 포즈 검증 게이트 (← 현재)
1. **동글을 Windows로 복귀**: 본인 WSL 터미널에서 `usbipd detach --busid <X-Y>` 또는 `wsl --shutdown`.
2. **SteamVR 헤드리스 설정** (HMD 없이 트래커만): `C:\Program Files (x86)\Steam\steamapps\common\SteamVR\resources\settings\default.vrsettings`
   - `"requireHmd": false`, `"activateMultipleDrivers": true`
   - null 드라이버: `...\SteamVR\drivers\null\resources\settings\default.vrsettings` → `"enable": true`
3. **SteamVR 실행** → 트래커 페어링 확인(녹색). 미페어링 시 Devices ▸ Pair Controller.
4. **포즈 출력 확인**: `conda run -n pika python scripts\pose_test_openvr.py`
   - `Tracker <serial> pos=(...) quat=(...)` 가 안정적으로 스트림 + 트래커 움직이면 값 변화 → **게이트 통과**.

### Phase B — 스트림 리더 정리
- `PoseSteamVR` (pose_test_openvr.py 기반 모듈화)
- `Sense`(그리퍼/command) + `Ego`(IMU) 시리얼 — COM 포트 확인
- `FisheyeCamera` DSHOW 패치(서브클래스/몽키패치로 원본 보존)
- `RealSenseCamera`

### Phase C — 동기화 레코더 (HDF5/ACT)
- 고정 주파수(30/60Hz) 스레드/큐 → 타임스탬프 정렬 → 에피소드 HDF5
- ACT 스키마: `/observations/images/<cam>`, `/observations/qpos`(pose+gripper), `/action`, `/timestamps`

### Phase D — 캘리브레이션
- 어안 인트린식, 트래커↔카메라 익스트린식(hand-eye), depth-color 정렬
- ✅ RS color/depth intrinsics + depth↔color extrinsic + stereo baseline → 에피소드 `camera_calib` 그룹 자동 저장(2026-06-16, `recorder.py`/`realsense_win.py`)

### Phase E — 검증
- 포즈 안정성, 프레임 드롭, 타임스탬프 정합, 에피소드 재생/시각화

## 체크리스트
- [x] Windows conda env(`pika`) + 전 스택 import 검증
- [x] Phase A: SteamVR 헤드리스 + `pose_test_openvr.py` 포즈 스트림 확인 (트래커 LHR-40B32551)
- [x] Phase B: 스트림 리더 — 포즈(`PoseSteamVR`)/그리퍼(`Sense`@COM3, AS5047)/어안(`FisheyeCameraWin` idx2)/RealSense(D405 SN 260522277606). **Sense 시리얼엔 IMU 없음**(IMU=트래커 융합)
- [x] Phase C: 동기화 레코더 → HDF5 (`pika_win/recorder.py`; 118f@29.4Hz, pose 118/118 valid, 이미지 3종 디코드 OK)
- [x] Phase D(부분): RS color/depth intrinsics·depth↔color extrinsic·stereo baseline 에피소드 저장 (`camera_calib`, 2026-06-16)
- [ ] Phase D(잔여): 트래커↔카메라 extrinsics/hand-eye, 어안 intrinsics 저장
- [x] Phase C+: 에피소드 제어 — **그리퍼 더블-핀치 토글 + rerun 실시간 피드백** (`scripts/collect.py`, `pika_win/gesture.py`)
- [ ] Phase C++: `action` 정의 확정, `agilexrobotics/data_tools`/ACT 스키마 정합
- [ ] Phase E: 다중 에피소드 검증/재생

## 확정된 하드웨어/환경 값
- conda env: `pika` (Python 3.10) @ `C:\Users\vision\anaconda3`
- PIKA Sense 시리얼: **COM3** (CH340, 버전 2.0.0) — 그리퍼 엔코더(AS5047 angle/rad) + command
- 어안 카메라: DSHOW **index 2** / RealSense D405 RGB: index 0(미사용, pyrealsense2로 접근)
- RealSense D405 SN: **260522277606**
- Vive Tracker: **LHR-40B32551**, 베이스스테이션 2.0 ×2
- 데이터 출력: `C:\Work\umi\data\episode_NNN.hdf5` (스크립트 경로의 `data/` 폴더 기본값; `--out`으로 변경). 과거 테스트분은 `C:\Users\vision\pika_data`에 있음
