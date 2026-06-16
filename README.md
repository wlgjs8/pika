# PIKA 데이터 수집 도구

AgileX PIKA Sense 기반 데이터 수집을 위한 Python 도구입니다. Vive/SteamVR 트래커 포즈, PIKA Sense 그리퍼 값, RealSense 프레임을 동기화해서 에피소드 단위 HDF5 파일로 저장합니다.

현재 저장소는 `/home/plaif/workspace/pai_rectified_flow_matching/pika` 디렉터리를 독립 Git 저장소로 분리한 상태이며, 원격 저장소는 `https://github.com/wlgjs8/pika` 입니다.

## 주요 기능

- 단일 팔 또는 양팔 PIKA 데이터 수집
- SteamVR/OpenVR 기반 Vive Tracker 6DoF 포즈 수집
- PIKA Sense 시리얼 그리퍼 각도/명령 수집
- RealSense D4xx 컬러/뎁스 프레임 수집
- RealSense color/depth intrinsics·depth↔color extrinsic·stereo baseline을 에피소드에 저장
- 좌/우 팔 하드웨어 매핑을 `config/arms.json`에 저장
- `b` 키 또는 Linux FootSwitch 입력으로 에피소드 녹화 시작/정지
- 수집 데이터 분석, HDF5 구조 확인, 브라우저 기반 에피소드 리뷰

## 디렉터리 구조

```text
.
├── config/              # 좌/우 팔 하드웨어 매핑
├── data/                # 수집 결과 저장 위치, Git 제외
├── pika_win/            # 포즈, RealSense, recorder, viewer 모듈
├── scripts/             # 수집/검수/분석 실행 스크립트
├── Makefile             # 자주 쓰는 실행 명령
├── SETUP.md             # 하드웨어 셋업과 진행 기록
└── README.md
```

## 환경 준비

권장 Python 환경은 conda env `pika`, Python 3.10입니다.

```bash
conda create -y -n pika python=3.10
conda run -n pika python -m pip install pyserial numpy opencv-python h5py pyrealsense2 openvr pillow
conda run -n pika python -m pip install --no-deps agx-pypika
```

라이브 뷰어를 사용할 경우 `rerun-sdk`가 필요할 수 있습니다.

```bash
conda run -n pika python -m pip install rerun-sdk
```

## 하드웨어 전제

- PIKA Sense USB 시리얼 연결
- Vive Tracker와 Lighthouse 베이스스테이션 준비
- SteamVR 실행 및 트래커 포즈 유효 상태
- RealSense D4xx 연결
- Linux에서는 Sense 시리얼 포트에 `/dev/serial/by-id/...` 경로 사용 권장

현재 `config/arms.json`에는 다음 형태로 양팔 매핑이 저장됩니다.

```json
{
  "arms": {
    "right": {
      "tracker_sn": "LHR-...",
      "com_port": "/dev/ttyUSB1",
      "realsense_sn": "..."
    },
    "left": {
      "tracker_sn": "LHR-...",
      "com_port": "/dev/ttyUSB0",
      "realsense_sn": "..."
    }
  }
}
```

## 사용법

### 1. 하드웨어 인식 확인

```bash
conda run --no-capture-output -n pika python scripts/detect_hardware.py
```

Vive 트래커, RealSense, COM 포트 후보를 출력합니다. SteamVR이 실행 중이어야 트래커 포즈를 확인할 수 있습니다.

### 2. 좌/우 팔 매핑

```bash
make identify
```

또는 직접 실행합니다.

```bash
conda run --no-capture-output -n pika python scripts/identify_arms.py
```

마법사의 안내에 따라 오른손/왼손 트래커, 그리퍼, RealSense를 움직이면 `config/arms.json`이 갱신됩니다. 실행 전 `collect.py`, `make run`, `make view`는 종료해야 합니다.

### 3. 데이터 수집

헤드리스 수집:

```bash
make run
```

브라우저 라이브 뷰어와 함께 수집:

```bash
make view
```

직접 실행 예시:

```bash
conda run --no-capture-output -n pika python scripts/collect.py --hz 30
conda run --no-capture-output -n pika python scripts/collect.py --view web --hz 30
```

수집이 시작되면 먼저 각 팔의 그리퍼 캘리브레이션을 수행합니다. 안내가 나오면 그리퍼를 여러 번 끝까지 쥐었다 펴세요.

녹화 제어:

- `b`: 에피소드 녹화 시작/정지
- Linux FootSwitch: 연결되어 있고 권한이 있으면 녹화 시작/정지
- `Ctrl-C`: 종료, 녹화 중이면 현재 에피소드 저장

수집 결과는 기본적으로 다음 위치에 저장됩니다.

```text
data/data_YYYYMMDD_HHMMSS/
├── collect.log
└── episode_000.hdf5
```

출력 위치를 바꾸려면 `--out`을 사용합니다.

```bash
conda run --no-capture-output -n pika python scripts/collect.py --out /path/to/output
```

### 4. CLI로 하드웨어 직접 지정

`config/arms.json`이 있으면 기본적으로 그 설정이 우선입니다. 설정 파일을 무시하고 CLI 인자를 쓰려면 `--config ''`를 지정합니다.

```bash
conda run --no-capture-output -n pika python scripts/collect.py \
  --config '' \
  --coms /dev/serial/by-id/<right>,/dev/serial/by-id/<left> \
  --rs-sns <right_rs_sn>,<left_rs_sn> \
  --tracker-sns <right_tracker_sn>,<left_tracker_sn>
```

Windows 예시:

```powershell
conda run --no-capture-output -n pika python scripts\collect.py `
  --config '' `
  --coms COM3,COM4 `
  --rs-sns 260522277606,419122270010 `
  --tracker-sns LHR-RIGHT,LHR-LEFT
```

### 5. 수집 데이터 분석

최신 세션 요약:

```bash
conda run --no-capture-output -n pika python scripts/analyze_data.py data --latest
```

전체 데이터 요약:

```bash
conda run --no-capture-output -n pika python scripts/analyze_data.py data
```

JSON 출력:

```bash
conda run --no-capture-output -n pika python scripts/analyze_data.py data --latest --json
```

### 6. 에피소드 리뷰

최신 세션을 브라우저에서 리뷰:

```bash
conda run --no-capture-output -n pika python scripts/review_episode.py
```

특정 세션 또는 에피소드 지정:

```bash
conda run --no-capture-output -n pika python scripts/review_episode.py --session data/data_YYYYMMDD_HHMMSS
conda run --no-capture-output -n pika python scripts/review_episode.py --episode data/data_YYYYMMDD_HHMMSS/episode_000.hdf5
```

서버를 띄우지 않고 HTML 파일만 생성:

```bash
conda run --no-capture-output -n pika python scripts/review_episode.py --no-serve
```

### 7. HDF5 구조 확인

```bash
conda run --no-capture-output -n pika python scripts/inspect_hdf5.py data/data_YYYYMMDD_HHMMSS/episode_000.hdf5
```

데이터셋 shape, attrs, 샘플 값, 첫 프레임 미리보기를 확인합니다.

## 에피소드 HDF5 레이아웃

활성 팔 수에 따라 평면(단일)/팔별 그룹(양팔)으로 저장합니다.
공통 attrs: `record_hz`, `effective_hz`, `pose_frame`, `pose_format`, `n_arms`, `arm_names`.

- 단일팔: `observations/{pose,gripper,command,images/...}`, 최상위 `action`, `timestamp`
- 양팔: `observations/<arm>/{pose,gripper,command,images/...,action}`(팔마다), 최상위 `timestamp`
- 이미지(vlen-u8): `realsense_color`=JPEG, `realsense_depth`=PNG16, `fisheye_color`=JPEG

### 카메라 캘리브레이션 (`camera_calib`)

각 팔 관측 그룹 아래 RealSense 정적 캘리브를 에피소드당 1회 저장합니다
(단일팔 `observations/camera_calib`, 양팔 `observations/<arm>/camera_calib`).

```text
camera_calib
├── color_intrinsics/   attrs: width,height,fx,fy,ppx,ppy,model  + coeffs[5]
├── depth_intrinsics/   attrs: width,height,fx,fy,ppx,ppy,model  + coeffs[5]
├── depth_to_color_rotation     [3,3]   # row-major, p_color = R @ p_depth + t
├── depth_to_color_translation  [3]     # meters
└── attrs: depth_scale, stereo_baseline_mm, depth_aligned_to_color,
          rotation_layout, translation_units
```

- `depth_aligned_to_color=True`이므로 저장된 depth는 color 프레임 기준입니다.
  저장 depth를 deproject할 땐 `color_intrinsics`를 사용하세요(depth↔color extrinsic은 거의 identity).
- `stereo_baseline_mm`은 depth 스테레오 IR 이미저 간 baseline(mm)입니다.
- 트래커↔카메라(hand-eye) extrinsic은 별도이며 여기 포함되지 않습니다(미측정).

## Makefile 명령

```bash
make identify   # 좌/우 팔 하드웨어 매핑 생성
make run        # 헤드리스 데이터 수집
make view       # rerun 라이브 뷰어와 함께 수집
```

환경 이름이나 추가 인자는 다음처럼 바꿀 수 있습니다.

```bash
make run ENV=pika ARGS="--hz 30 --require-pose"
make view VIEW=web ARGS="--hz 30"
```

## 자주 쓰는 옵션

- `--hz 30`: 수집 주파수
- `--out data`: 출력 루트 디렉터리
- `--view web`: 브라우저 라이브 뷰어 사용
- `--no-realsense`: RealSense 없이 수집
- `--require-pose`: 유효한 포즈가 없으면 시작하지 않음
- `--require-all-trackers`: 설정된 모든 트래커가 보일 때만 시작
- `--no-pedal`: FootSwitch 입력 비활성화
- `--start-index N`: 에피소드 번호 시작값 지정

## 문제 해결

### `make view`(web 뷰어)가 `[viewer] gRPC 서버 시작 중`에서 멈춤

rerun 네이티브 `rr.serve_grpc()`가 드물게 시작 직후 리턴하지 않는 **일회성 데드락**입니다.
포트(9876)는 LISTEN 상태로 바인딩됐지만 `gRPC 서버 시작 완료` 로그가 안 찍힙니다.
메모리/디스크 문제가 아닙니다(동일 코드/포트로 재현되지 않는 transient race).

- 조치: `Ctrl-C`로 종료 후 재실행하면 대부분 풀립니다.
- 헤드리스 수집만 필요하면 `make run`(`--view` 없이)으로 우회할 수 있습니다.

## Git에 포함하지 않는 파일

다음 파일과 디렉터리는 `.gitignore`로 제외합니다.

- `.env`, `.env.*`
- `data/`
- `__pycache__/`
- HDF5, 모델 체크포인트, numpy dump 등 대용량 산출물
- 로컬 실행 로그와 출력 폴더

수집 데이터는 크기가 크고 장비별 로컬 산출물이므로 GitHub에 올리지 않습니다.

## 추가 문서

하드웨어 셋업 배경, Windows/Ubuntu 경로, 진행 단계 기록은 `SETUP.md`를 참고하세요.
