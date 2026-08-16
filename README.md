# PIKA 데이터 수집 도구

AgileX PIKA Sense 기반 데이터 수집을 위한 Python 도구입니다. Vive Tracker 포즈, PIKA Sense 그리퍼 값, 어안/RealSense 프레임을 동기화해서 에피소드 단위 HDF5 파일로 저장합니다.

포즈 백엔드는 **libsurvive가 기본**입니다(GUI·SteamVR 불필요, 헤드리스). SteamVR/OpenVR 경로도 `--pose-backend steamvr`로 그대로 쓸 수 있습니다. 두 백엔드는 **월드 원점이 다르므로** 한 데이터셋 안에서 섞지 마세요 — 에피소드 attrs의 `pose_frame`(`survive_world` / `steamvr_world`)과 `pose_backend`로 구분됩니다.

현재 저장소는 `/home/plaif/workspace/pai_rectified_flow_matching/pika` 디렉터리를 독립 Git 저장소로 분리한 상태이며, 원격 저장소는 `https://github.com/wlgjs8/pika` 입니다.

## 주요 기능

- 단일 팔 또는 양팔 PIKA 데이터 수집
- libsurvive(기본) 또는 SteamVR/OpenVR 기반 Vive Tracker 6DoF 포즈 수집
- PIKA Sense 시리얼 그리퍼 각도/명령 수집
- RealSense D4xx 컬러/뎁스 프레임 수집 (`--no-depth` 로 color 만, `--rs-fps 90` 지원)
- RealSense color/depth intrinsics·depth↔color extrinsic·stereo baseline을 에피소드에 저장
- 좌/우 팔 하드웨어 매핑을 `config/arms.json`에 저장
- `b` 키 또는 1구 FootSwitch(by-path 고정)로 에피소드 녹화 시작/정지
- 수집 데이터 분석, HDF5 구조 확인, 로컬 OpenCV 창 기반 에피소드 리뷰

## 디렉터리 구조

```text
.
├── config/              # 좌/우 팔 하드웨어 매핑
├── data/                # 수집 결과 저장 위치, Git 제외
├── pika_win/            # 포즈, RealSense, 어안, recorder, writer 모듈
├── scripts/             # 수집/검수/분석 실행 스크립트
├── Makefile             # 자주 쓰는 실행 명령
├── SETUP.md             # 하드웨어 셋업과 진행 기록
└── README.md
```

## 환경 준비

저장소 로컬 venv(Python 3.10)를 씁니다. conda는 필요 없습니다.

```bash
uv venv --python 3.10 .venv
uv pip install --python .venv/bin/python pyserial numpy opencv-python h5py pyrealsense2 pillow
uv pip install --python .venv/bin/python --no-deps agx-pypika
```

SteamVR 백엔드를 쓰면 `openvr`을 추가로 설치합니다.

```bash
uv pip install --python .venv/bin/python openvr      # --pose-backend steamvr
```

libsurvive(기본 포즈 백엔드)는 소스 빌드입니다. apt 의존성·udev 규칙은 root가 필요합니다.

```bash
sudo apt-get install -y libusb-1.0-0-dev liblapacke-dev libopenblas-dev
scripts/setup_libsurvive.sh          # clone + patches/ 적용 + 빌드 + pysurvive 연결
sudo install -m 644 ~/workspace/libsurvive/useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

`patches/`의 로컬 패치는 필수입니다 — 업스트림 libsurvive는 USB 전송 에러 한 번에
프로세스 전체가 abort 합니다(use-after-free). 자세한 내용은 패치 파일 헤더 참고.

## 하드웨어 전제

- PIKA Sense USB 시리얼 연결
- Vive Tracker와 Lighthouse 베이스스테이션 준비(트래커는 Sense 케이블로 유선 연결)
- RealSense D4xx 연결
- libsurvive 라이트하우스 캘리브레이션 완료 (`make pose-test ARGS="--recalibrate --seconds 60"`
  실행 중 트래커를 작업 공간 전체로 움직일 것 — 정지 상태로는 scene solve가 발산합니다)
- SteamVR 백엔드를 쓸 때만: SteamVR 실행 및 트래커 포즈 유효 상태

### ⚠️ 장치 경로는 반드시 `by-path`로 (`by-id` 금지)

이 리그의 CH340 시리얼(4개)·DECXIN 어안(4개)·FootSwitch(2개)는 **같은 VID:PID에
시리얼 문자열이 없거나 전부 동일**합니다. udev는 이름 충돌을 피하려고 `/dev/serial/by-id/`,
`/dev/input/by-id/`에 **먼저 잡힌 하나에만** 심링크를 만들고 나머지에는 아예 만들지 않습니다.
따라서 by-id 경로는 꽂는 순서에 따라 다른 물리 장치를 가리키고, 두 번째 장치는 주소 지정
자체가 불가능합니다. `/dev/ttyUSBN`·`/dev/videoN`·`/dev/input/eventN` 번호도 재부팅마다 바뀝니다.

**`/dev/serial/by-path/...`, `/dev/v4l/by-path/...`, `/dev/input/by-path/...`** 만 안정적입니다.
어안 카메라는 `pika_win/fisheye.py`가 짝 RealSense와 **같은 PCI 컨트롤러+루트 포트**를 보고
자동 매핑하므로 별도 지정이 필요 없습니다(xHCI 컨트롤러가 둘 이상이면 PCI 주소까지 봐야
교차 매핑이 안 생깁니다).

현재 `config/arms.json`에는 다음 형태로 양팔 매핑이 저장됩니다.

```json
{
  "arms": {
    "right": {
      "tracker_sn": "LHR-...",
      "com_port": "/dev/serial/by-path/pci-0000:13:00.0-usb-0:4.1.4:1.0-port0",
      "realsense_sn": "419122270010"
    },
    "left": {
      "tracker_sn": "LHR-...",
      "com_port": "/dev/serial/by-path/pci-0000:79:00.4-usb-0:1.1.4:1.0-port0",
      "realsense_sn": "260522277606"
    }
  }
}
```

`realsense_sn`은 **librealsense 시리얼**입니다. USB 디스크립터의 iSerial(sysfs `serial`,
robotics_lab `cam-status`의 `asic ...`)과는 값이 다르니 섞지 마세요.

## 사용법

### 1. 하드웨어 인식 확인

```bash
.venv/bin/python scripts/detect_hardware.py
```

RealSense, 시리얼 포트 후보를 출력합니다(Windows DSHOW 전제라 Linux 카메라 탐지는 부정확).

포즈는 백엔드별 전용 스모크 테스트로 확인하는 편이 정확합니다.

```bash
make pose-test                                   # libsurvive: 트래커 인식/시리얼 매핑/포즈 유효성
make pose-test ARGS="--recalibrate --seconds 60" # 라이트하우스 재캘리브레이션(움직이면서)
.venv/bin/python scripts/pose_test_openvr.py     # SteamVR 백엔드
.venv/bin/python scripts/pedal_test.py           # 발판이 어느 evdev 노드인지 확인
```

`make pose-test` 출력의 `sep=`(두 트래커 간 거리)가 실제 물리 거리와 맞고 밀리미터 단위로
안정적이면 스케일·기하가 검증된 것입니다.

### 2. 좌/우 팔 매핑

```bash
make identify
```

또는 직접 실행합니다.

```bash
.venv/bin/python scripts/identify_arms.py
```

마법사의 안내에 따라 오른손/왼손 트래커, 그리퍼, RealSense를 움직이면 `config/arms.json`이 갱신됩니다. 실행 전 `collect.py`, `make run`, `run_collect_fast.sh`는 종료해야 합니다.

### 3. 데이터 수집

헤드리스 수집:

```bash
make run
```

90fps RGB-only 수집(어안·depth 비활성):

```bash
./scripts/run_collect_fast.sh
```

직접 실행 예시:

```bash
.venv/bin/python scripts/collect.py --hz 30
.venv/bin/python scripts/collect.py --hz 90 --rs-fps 90 --no-fisheye --no-depth \
  --rs-json /home/plaif/workspace/robotics_lab/camera_server/config/realsense_d405_90fps.json
```

수집이 시작되면 먼저 각 팔의 그리퍼 캘리브레이션을 수행합니다. 안내가 나오면 그리퍼를 여러 번 끝까지 쥐었다 펴세요.

녹화 제어:

- `b`: 에피소드 녹화 시작/정지
- FootSwitch(1구): `run_collect_fast.sh` 가 by-path 로 고정. 직접 실행 시 `--pedal-device` 지정
- `Ctrl-C`: 종료, 녹화 중이면 현재 에피소드 저장

수집 결과는 기본적으로 다음 위치에 저장됩니다.

```text
data/data_YYYYMMDD_HHMMSS/
├── collect.log
└── episode_000.hdf5
```

출력 위치를 바꾸려면 `--out`을 사용합니다.

```bash
.venv/bin/python scripts/collect.py --out /path/to/output
```

### 4. CLI로 하드웨어 직접 지정

`config/arms.json`이 있으면 기본적으로 그 설정이 우선입니다. 설정 파일을 무시하고 CLI 인자를 쓰려면 `--config ''`를 지정합니다.

```bash
.venv/bin/python scripts/collect.py \
  --config '' \
  --coms /dev/serial/by-path/<right>,/dev/serial/by-path/<left> \
  --rs-sns <right_rs_sn>,<left_rs_sn> \
  --tracker-sns <right_tracker_sn>,<left_tracker_sn>
```

Windows 예시:

```powershell
python scripts\collect.py `
  --config '' `
  --coms COM3,COM4 `
  --rs-sns 260522277606,419122270010 `
  --tracker-sns LHR-RIGHT,LHR-LEFT
```

### 5. 수집 데이터 분석

최신 세션 요약:

```bash
.venv/bin/python scripts/analyze_data.py data --latest
```

전체 데이터 요약:

```bash
.venv/bin/python scripts/analyze_data.py data
```

JSON 출력:

```bash
.venv/bin/python scripts/analyze_data.py data --latest --json
```

### 6. 에피소드 리뷰 (로컬 OpenCV 창)

```bash
./scripts/run_review_episode.sh
```

**최신 세션의 첫 에피소드에서 시작**하고, 전 세션이 시간순으로 한 줄로 이어져 있습니다.
`a`(이전)를 계속 누르면 세션 경계를 넘어 직전 세션의 마지막 에피소드로 거슬러 갑니다.
목록의 양 끝에서는 순환합니다(가장 오래된 것 ↔ 가장 최신).

특정 세션 또는 에피소드로 한정:

```bash
./scripts/run_review_episode.sh --session data/data_YYYYMMDD_HHMMSS   # 세션 경계 안 넘음
./scripts/run_review_episode.sh --episode data/data_YYYYMMDD_HHMMSS/episode_000.hdf5
./scripts/run_review_episode.sh --arms left --streams color,depth     # 일부만
```

팔=행, 스트림(D405 color / 어안 / depth)=열로 배치하고 하단에 프레임 번호·경과 시간·
`pose_frame`·팔별 pose/gripper 를 표시합니다. HDF5 에서 **직접 디코드**하므로 JPEG 에셋
추출이나 HTTP 서버가 없습니다(예전 HTML 리뷰어는 13 에피소드 빌드에 2분 이상 걸렸고
세션마다 수백 MB 의 `review_session/` 을 남겼습니다).

| 키 | 동작 | 키 | 동작 |
|---|---|---|---|
| `space` | 재생/일시정지 | `.` `,` | 다음/이전 프레임 |
| `d` `a` | 다음/이전 에피소드 | `+` `-` | 재생 속도 |
| `g` `e` | 첫/마지막 프레임 | `s` | 현재 화면 PNG 저장 |
| `h` | 도움말 토글 | `q` / `ESC` | 종료 |

트랙바로도 탐색할 수 있습니다. 재생 중 에피소드 끝에 닿으면 다음 에피소드로 이어집니다.
로컬 모니터가 있는 세션에서 실행해야 하며, `DISPLAY` 가 없으면 안내 후 종료합니다.

재생은 **파일에 기록된 타임스탬프를 그대로 따라가므로 정확히 1배속**입니다(고정 fps 가정이
아니라서 수집 지터나 프레임 드롭이 있어도 시간축이 어긋나지 않습니다). 상태줄 둘째 줄에
재생 상태를 모두 표시하므로 터미널을 볼 필요가 없습니다.

```
[재생] 배속 x1.00   수집 29.87Hz 기준   실제 29.9fps    ← 재생 중(호박색)
[정지] 배속 x2.25   수집 29.87Hz 기준   실제 --         ← 정지 중(회색)
[재생] 배속 x1.00   고정 30.0fps 기준   실제 29.9fps    ← --fps 30 지정 시
```

`배속`은 `+`/`-` 로 바꾼 배수, `수집 …Hz`는 타임스탬프에서 구한 실제 수집 레이트,
`실제`는 지금 달성 중인 재생 fps 입니다(정지 중에는 의미가 없어 `--`).

> 에피소드 attrs 의 `effective_hz` 는 `프레임수 / 구간` 으로 계산돼 있어 간격 개수(`프레임수-1`)
> 기준보다 0.4% 정도 높게 나옵니다(277프레임/9.241초: attr 29.98 vs 실제 29.87).
> 뷰어는 타임스탬프에서 직접 계산한 값을 씁니다.

배속은 프레임을 건너뛰어 맞추므로 x2 에서도 실제 시간축이 정확히 2배가 됩니다
(실측 x1 오차 0.00%, x0.5 오차 0.00%, x2 오차 +0.6%).
양팔 3스트림이면 프레임마다 PNG 6장을 디코드해야 하는데(실측 ~33ms), 표시 중에 다음
프레임을 미리 디코드하는 워커가 있어 실시간이 유지됩니다(캐시 적중 시 합성 3.9ms).

### 7. HDF5 구조 확인

```bash
.venv/bin/python scripts/inspect_hdf5.py data/data_YYYYMMDD_HHMMSS/episode_000.hdf5
```

데이터셋 shape, attrs, 샘플 값, 첫 프레임 미리보기를 확인합니다.

## 에피소드 HDF5 레이아웃

활성 팔 수에 따라 평면(단일)/팔별 그룹(양팔)으로 저장합니다.
공통 attrs: `record_hz`, `effective_hz`, `pose_frame`, `pose_backend`, `pose_format`,
`n_arms`, `arm_names`, `arm_bolt_colors`.

`pose_frame`은 `survive_world` / `steamvr_world`(+ tip 변환 시 `_gripper_tip` 접미)이고
`pose_backend`는 `libsurvive` / `steamvr_openvr`입니다. 두 백엔드는 월드 원점이 다르므로
**변환·학습 시 섞으면 안 됩니다.** robotics_lab `umi-convert`는 retarget 설정의
`source_pose_frame`과 에피소드 `pose_frame`이 다르면 거부합니다
(libsurvive용: `calibration/umi_retarget_eelocal_survive.yaml`).

- 단일팔: `observations/{pose,gripper,command,images/...}`, 최상위 `action`, `timestamp`
- 양팔: `observations/<arm>/{pose,gripper,command,images/...,action}`(팔마다), 최상위 `timestamp`
- 이미지(vlen-u8): `realsense_color`=PNG, `realsense_depth`=PNG16, `fisheye_color`=PNG

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
make pose-test  # 포즈 백엔드 스모크 테스트(libsurvive)
```

인터프리터나 추가 인자는 다음처럼 바꿀 수 있습니다.

```bash
make run ARGS="--hz 30 --require-pose --require-all-trackers"
make run PY="conda run --no-capture-output -n pika python"   # conda 환경을 쓸 때
```

## 래퍼 스크립트

`scripts/*.sh` 는 모두 `scripts/_venv.sh` 를 source 해서 `.venv/bin/python` 을 씁니다
(conda 불필요). 다른 인터프리터는 `PY` 로 덮어씁니다.

```bash
./scripts/run_collect_fast.sh     # 90fps RGB-only 수집 (어안·depth 비활성)
./scripts/run_review_episode.sh   # 에피소드 리뷰 (로컬 OpenCV 창)
./scripts/analyze_data.sh data --latest
PY="conda run --no-capture-output -n pika python" ./scripts/run_collect_fast.sh
```

`run_collect_fast.sh` 는 다음을 기본으로 붙입니다:
`--hz 90 --rs-fps 90 --rs-json <robotics_lab 90fps json> --no-fisheye --no-depth`

RealSense advanced-mode JSON 은 **robotics_lab 것을 경로로 직접 참조**합니다(사본 없음 →
그 파일 하나만 고치면 수집·추론이 함께 따라갑니다). 파일이 없으면 **즉시 실패**합니다 —
그냥 두면 `realsense_win` 이 "advanced json 없음, 건너뜀" 으로 조용히 넘어가 카메라에 남아
있던 직전 설정(예: AE off)으로 수집돼 버리기 때문입니다.

| env | 기본 | 용도 |
|---|---|---|
| `RS_JSON` | robotics_lab `realsense_d405_90fps.json` | advanced-mode 프리셋 경로 |
| `PIKA_HZ` / `PIKA_RS_FPS` | 90 / 90 | 기록 주파수 / 스트림 fps |
| `PIKA_FISHEYE` / `PIKA_DEPTH` | 0 / 0 | `1` 로 두면 어안/depth 를 다시 켬 |
| `PIKA_ENCODE_WORKERS` | 4 | PNG 인코딩 worker 수 |
| `PEDAL_DEVICE` | 1구 발판 by-path (`scripts/_pedal.sh`) | 녹화 토글 발판 |

### 발판은 `scripts/_pedal.sh` 한 곳에서 정한다

이 저장소가 쓰는 발판은 **1구**입니다. 수집(녹화 토글)과 teleop(클러치)이 같은 발판을
공유하며, 둘을 동시에 돌리지 않으므로 충돌하지 않습니다. 3구 발판은 robotics_lab
`rb_gui`(InitMotion) 전용입니다.

`run_collect_fast.sh` 와 `run_umi_teleop_publish.sh` 가 모두 이 프렐류드를 source 하므로
발판을 다른 USB 포트로 옮겼을 때 **한 곳만 고치면** 됩니다(경로 확인:
`.venv/bin/python scripts/pedal_test.py`).

고정하지 않으면 `collect.py` 는 발판이 여러 개일 때 자동 선택을 거부하고 evdev 리더를
끕니다. 그래도 녹화는 "되는 것처럼" 보이는데, 발판이 보내는 `b` 키가 **stdin 으로 새어
키보드 토글을 대신 때리기** 때문입니다(로그에 `by=keyboard:b` 로 남습니다). 이 경로는
터미널 포커스가 있어야만 동작하고 화면에 `b` 가 찍힙니다. 고정하면 evdev 로 들어가고
장치를 배타 점유하므로 키 누수도 사라집니다.

수집 래퍼는 하드웨어를 **`config/arms.json` 에서만** 읽습니다. 예전에는 `--config ''` 로
arms.json 을 무시하고 `--coms /dev/ttyUSB0,/dev/ttyUSB1` 을 강제했는데, 그 두 포트는
지금 **로봇팔 그리퍼**라 엉뚱한 장치를 열거나 robotics_lab 과 충돌합니다. 임시로 다른
하드웨어를 쓰려면 `PIKA_CONFIG` / `PIKA_COMS` / `PIKA_RS_SNS` / `PIKA_TRACKER_SNS` 를
명시할 때만 CLI 인자가 붙습니다.

## teleop 발행 (robotics_lab 연동)

```bash
./scripts/run_umi_teleop_publish.sh                       # 같은 PC의 robotics_lab 으로
TARGET_HOST=172.28.61.3 ./scripts/run_umi_teleop_publish.sh   # 별도 로봇 PC로
```

포즈를 UDP `50380`(좌)/`50381`(우), 그리퍼 브리지 `50382`로 발행합니다.
클러치 발판은 `PEDAL_DEVICE`로 **by-path 고정**되어 있습니다 — 이 리그에는 발판이 2개이고
(1구=teleop 클러치, 3구=robotics_lab rb_gui InitMotion) 소프트웨어로는 구별할 수 없습니다.
발판을 다른 포트로 옮겼다면 `scripts/pedal_test.py`로 경로를 다시 확인하세요.

수신측 `stack_real.yaml`은 발행자의 기본값 `--pose-frame tip`과 짝지어져 있습니다
(`gripper_offset: [0,0,0]` + tip→TCP `r_align`). 한쪽만 바꾸면 안 됩니다.

## 자주 쓰는 옵션

- `--hz 30`: 수집 주파수
- `--out data`: 출력 루트 디렉터리
- `--no-realsense`: RealSense 없이 수집
- `--no-fisheye` / `--no-depth`: 어안 / depth 스트림 비활성
- `--rs-fps 90` / `--rs-json <path>`: RealSense 스트림 fps / advanced-mode 프리셋
- `--require-pose`: 유효한 포즈가 없으면 시작하지 않음
- `--require-all-trackers`: 설정된 모든 트래커가 보일 때만 시작
- `--no-pedal`: FootSwitch 입력 비활성화
- `--start-index N`: 에피소드 번호 시작값 지정
- `--pose-backend {survive,steamvr}`: 포즈 백엔드(기본 `survive`)

## 문제 해결

### 트래커를 못 찾거나 한쪽만 잡힘

libsurvive는 프로세스마다 콜드 스타트라 라이트하우스 획득에 ~2초가 걸리고, 트래커마다
획득 시각이 다릅니다. `PoseSurvive.connect()`가 설정된 트래커가 전부 붙을 때까지 기다리지만
(최대 10초), 계속 누락되면 그 트래커가 가려졌거나 베이스스테이션 시야 밖입니다.
`make pose-test`로 어느 쪽이 안 잡히는지 확인하세요.

### 포즈 위치가 수 미터~수십 미터로 나옴

라이트하우스 scene solve가 수렴하지 못한 상태입니다(정지 상태로 캘리브레이션하면 발산).
`make pose-test ARGS="--recalibrate --seconds 60"` 을 실행하고 그동안 트래커를 작업 공간
전체에 걸쳐 위치·자세를 바꿔가며 움직이세요. `config/libsurvive_config.json`의 라이트하우스
`variance`가 0에 가까우면 수렴한 것입니다.

**이 파일을 지우면 월드 프레임이 바뀝니다** — 기존 수집분과 좌표계가 어긋나므로 재캘리브레이션은
꼭 필요할 때만 하세요.

### 발판을 밟으면 터미널에 `bbbb...` 가 찍힘

발판을 **배타 점유(EVIOCGRAB)** 하지 않은 상태입니다. PCsensor 발판은 키보드로 인식되어
포커스된 창에 키(기본 a/b/c)를 그대로 입력합니다. 수집기·발행자 모두 기본으로 점유하므로
정상이라면 안 새어 나옵니다. `--no-pedal-grab` 을 켰거나, 다른 프로세스(rb_gui 등)가 먼저
점유해서 grab 에 실패한 경우입니다 — 후자면 그 발판 이벤트를 **아예 받지 못합니다**.

`collect.py` 에서는 이게 단순 미관 문제가 아닙니다. `b` 가 녹화 토글 키라서 발판 한 번에
evdev 로 한 번 + stdin 으로 한 번이 겹쳐 **즉시 시작·정지로 상쇄**됩니다.

글자가 수백 개씩 쏟아지는 건 **X11 키 오토리피트**입니다(밟고 있는 동안 초당 ~30자).
momentary 클러치는 계속 밟고 있으므로 한 번 밟을 때마다 길게 도배됩니다.

**클러치로 쓰지 않는 다른 발판이 원인일 수 있습니다.** 이 리그의 3구 발판은
robotics_lab `rb_gui` 전용이라 평소엔 rb_gui 가 점유해 문제가 없지만, 발행자만 단독으로
실행하면 아무도 점유하지 않아 그 발판이 그대로 샙니다. 두 가지 중 하나로 해결합니다.

`run_umi_teleop_publish.sh` 는 `--mute-other-pedals` 를 **기본으로 켭니다** — 나머지 발판을
점유만 해서(이벤트는 읽지 않음) 누수를 막습니다. `make run` 을 먼저 띄운 경우엔 rb_gui 가
이미 3구를 점유하고 있어 이 grab 이 실패하는데, 이미 막혀 있다는 뜻이라 무해합니다.

**순서만 주의하세요.** 발행자를 먼저 띄우면 나중에 뜨는 rb_gui 가 자기 발판을 못 잡아
InitMotion 발판이 비활성됩니다(rb_gui 가 그 이유를 로그에 명시합니다). 그 순서로 써야 하면:

```bash
MUTE_OTHER_PEDALS=0 ./scripts/run_umi_teleop_publish.sh
```

### 발판을 밟아도 반응이 없음

발판이 2개면 by-id로는 하나만 주소 지정이 되고, 나머지는 구조적으로 인식 불가입니다
(위 by-path 경고 참고). `scripts/pedal_test.py`로 밟은 발판의 by-path를 확인해
`--pedal-device`(또는 `PEDAL_DEVICE`)로 지정하세요. robotics_lab `rb_gui`가 실행 중이면
자기 발판을 **배타적으로 grab** 하므로 그쪽에 잡힌 발판은 여기서 반응하지 않습니다.

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
