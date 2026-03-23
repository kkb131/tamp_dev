# Manus Quantum Metagloves 설치 및 테스트 가이드

Manus Quantum Metagloves에서 손 관절 데이터를 수신하기 위한 환경 설정 가이드.

## 아키텍처

```
Operator PC (Manus Glove)    ──UDP(port 9872)──>    Robot PC (AGX Orin)
├─ manus_reader.py                                  ├─ ManusNetworkInput (추후)
│  └─ libManusSDK.so (ctypes)                       └─ Tesollo DG 5F M 제어
├─ manus_sender.py
│  └─ KeyboardState (pynput)
└─ 60 Hz polling loop
```

## 데이터 구조

```
HandData:
  joint_angles[20]    # 손가락 관절 각도 (rad)
  │                   # [Thumb(4), Index(4), Middle(4), Ring(4), Pinky(4)]
  │                   # 각 손가락: [Spread, Flexion, PIP/MCP, DIP/IP]
  finger_spread[5]    # 손가락 벌림 각도
  wrist_pos[3]        # 손목 위치 (m)
  wrist_quat[4]       # 손목 방향 (wxyz)
```

---

## 환경 설정 (권장: Conda)

호스트 PC 전체 세팅 가이드는 `docs/host_setup_guide.md`를 참조하세요.

```bash
# 1. Conda 환경 생성 (최초 1회)
cd ~/tamp_ws/src/tamp_dev
conda env create -f environment.yaml

# 2. 활성화
conda activate tamp_sender

# 3. LD_LIBRARY_PATH 자동 설정 (최초 1회)
mkdir -p $CONDA_PREFIX/etc/conda/activate.d
cat > $CONDA_PREFIX/etc/conda/activate.d/manus_env.sh << 'EOF'
#!/bin/bash
export MANUS_SDK_PATH=~/tamp_ws/src/tamp_dev/manus/sdk
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MANUS_SDK_PATH
EOF

# 재활성화하면 자동 적용
conda deactivate && conda activate tamp_sender
```

> Conda를 사용하지 않는 경우 아래 Step 0의 수동 설치를 따르세요.

---

## Step 0: 시스템 의존성 설치 (수동)

Conda 환경 없이 직접 설치하는 경우:

```bash
# 시스템 패키지
sudo apt update
sudo apt install -y build-essential libusb-1.0-0-dev libudev-dev \
    libncurses5-dev zlib1g-dev pkg-config

# Python 패키지
pip install numpy pyyaml pynput
```

**테스트:**
```bash
cd /path/to/tamp_ws/src/tamp_dev
python3 -m manus.tests.test_step0_deps
```

---

## SDK 모드

Manus SDK Linux는 두 가지 모드를 제공합니다:
- **Integrated Mode** (`libManusSDK_Integrated.so`): Linux PC에 USB 동글 직접 연결. 추가 의존성 불필요.
- **Remote Mode** (`libManusSDK.so`): Windows PC의 MANUS Core에 네트워크 연결. gRPC + Protobuf 필요.

우리 프로젝트는 **Integrated Mode**를 사용합니다. Remote Mode가 필요한 경우 `manus/sdk/README.md` 참조.

---

## Step 1: Manus SDK 설치 + USB 동글

### 1.1 SDK 다운로드

1. Manus 개발자 포털에서 Linux SDK 다운로드:
   - https://docs.manus-meta.com/3.1.0/Plugins/SDK/Linux/
2. 압축 해제하여 `manus/sdk/` 디렉토리에 배치:
   ```bash
   # 예시 (실제 파일명은 다를 수 있음)
   tar xzf ManusSDK_Linux_*.tar.gz -C manus/sdk/
   ```
3. 핵심 파일 확인:
   ```bash
   ls -la manus/sdk/libManusSDK.so
   ```

### 1.2 udev 규칙 설치

```bash
# udev 규칙 복사
sudo cp manus/udev/70-manus-hid.rules /etc/udev/rules.d/

# 규칙 리로드
sudo udevadm control --reload-rules
sudo udevadm trigger
```

> **NOTE**: `70-manus-hid.rules`의 vendor/product ID가 placeholder입니다.
> USB 동글을 꽂은 후 `lsusb | grep -i manus` 로 실제 ID를 확인하고
> 파일을 업데이트하세요.

### 1.3 USB 동글 연결

1. Manus 라이선스 USB 동글을 PC에 꽂기
2. 동글 인식 확인:
   ```bash
   lsusb | grep -i manus
   dmesg | tail -20
   ```

**테스트:**
```bash
python3 -m manus.tests.test_step1_sdk
# 또는 SDK 경로 지정:
python3 -m manus.tests.test_step1_sdk --sdk-path manus/sdk/libManusSDK.so
```

---

## Step 2: 글러브 연결

1. Manus Quantum Metagloves 전원 ON
2. BLE 범위 내에 위치 (동글 근처)
3. 글러브 LED가 연결됨을 나타낼 때까지 대기

**테스트:**
```bash
python3 -m manus.tests.test_step2_connection --hand right
# 양손 테스트:
python3 -m manus.tests.test_step2_connection --hand both
```

---

## Step 3: 데이터 스트리밍 확인

글러브를 착용하고 손가락을 움직여 데이터 수신 확인:

```bash
# 5초간 60Hz로 스트리밍 (기본값)
python3 -m manus.tests.test_step3_stream

# 설정 변경
python3 -m manus.tests.test_step3_stream --duration 10 --hz 120 --hand left
```

출력 예시:
```
  Per-finger joint ranges (min / max / range):
  Finger   Joint           Min      Max    Range
  ------------------------------------------------
  Thumb    CMC_Spread   -0.123   +0.456    0.579
  Thumb    CMC_Flex     +0.012   +1.234    1.222
  ...
```

**단독 reader 테스트:**
```bash
python3 -m manus.manus_reader --hand right --hz 30
```

---

## Step 4: UDP 네트워크 전송 테스트

### Mock 데이터로 테스트 (글러브 없이):
```bash
python3 -m manus.tests.test_step4_udp
```

### 실제 글러브로 테스트:
```bash
python3 -m manus.tests.test_step4_udp --real --sdk-path manus/sdk/libManusSDK.so
```

---

## Step 5: 실제 사용 — 로봇 PC로 데이터 전송

```bash
# 기본 설정 (config/default.yaml 참조)
python3 -m manus.manus_sender --target-ip <ROBOT_PC_IP>

# 옵션 지정
python3 -m manus.manus_sender \
    --target-ip 192.168.0.10 \
    --port 9872 \
    --hz 60 \
    --hand right

# YAML 설정 파일 사용
python3 -m manus.manus_sender --config manus/config/default.yaml
```

키보드 단축키:
- `Space` = E-Stop
- `R` = Reset
- `Q` / `Esc` = Quit
- `+`/`-` = Speed up/down

---

## 캘리브레이션 (선택)

사용자별 손가락 가동 범위를 기록하여 [0, 1]로 정규화:

```bash
python3 -m manus.calibrate --hand right --output manus/calibration_right.json
```

절차:
1. 손을 완전히 펴기 → Enter
2. 주먹 쥐기 → Enter
3. `calibration_right.json` 저장됨

설정에 적용:
```yaml
# config/default.yaml
joint_mapping:
  calibration_file: "manus/calibration_right.json"
```

---

## UDP 패킷 포맷

```json
{
    "type": "manus",
    "hand": "right",
    "joint_angles": [0.1, 0.2, ...],
    "finger_spread": [0.05, ...],
    "wrist_pos": [0.0, 0.0, 0.0],
    "wrist_quat": [1.0, 0.0, 0.0, 0.0],
    "tracking": true,
    "buttons": {
        "estop": false,
        "reset": false,
        "quit": false
    },
    "timestamp": 1234567890.123
}
```

- `joint_angles`: 20개 float (5 fingers x 4 joints)
- `finger_spread`: 5개 float (각 손가락 벌림)
- `wrist_quat`: wxyz 순서
- `tracking`: 글러브 데이터 수신 여부
- Port: **9872** (Vive는 9871)

---

## 트러블슈팅

### SDK 로드 실패
```bash
# 의존 라이브러리 확인
ldd manus/sdk/libManusSDK.so

# 누락된 라이브러리 설치
sudo apt install <missing-lib>
```

### USB 동글 미감지
```bash
# USB 장치 목록
lsusb

# 커널 로그 확인
dmesg | tail -30

# udev 규칙 확인
cat /etc/udev/rules.d/70-manus-hid.rules

# 권한 확인
ls -la /dev/hidraw*
```

### 글러브 미연결
- 글러브 배터리 확인 (충전 필요할 수 있음)
- 동글과 글러브의 BLE 범위 확인 (5m 이내)
- 글러브 전원 끄고 다시 켜기
- 동글 USB 재연결

### ctypes 매핑 오류
SDK 버전이 다른 경우 ctypes struct 레이아웃이 다를 수 있습니다:
1. `manus/sdk/ManusSDKTypes.h` 확인
2. `manus/manus_reader.py`의 ctypes 정의 업데이트
3. 함수 시그니처: `nm -D manus/sdk/libManusSDK.so | grep CoreSdk`

---

## 파일 구조

```
manus/
├── __init__.py
├── README.md                  ← 이 파일
├── requirements.txt
├── manus_config.py            # YAML 설정 로더
├── manus_reader.py            # SDK ctypes 래퍼 (핵심)
├── manus_sender.py            # UDP 데이터 전송
├── calibrate.py               # 손가락 ROM 캘리브레이션
├── config/
│   └── default.yaml           # 기본 설정
├── sdk/
│   ├── README.md              # SDK 다운로드 안내
│   ├── .gitkeep
│   └── (libManusSDK.so)       # SDK 파일 (별도 다운로드)
├── udev/
│   └── 70-manus-hid.rules    # USB 동글 udev 규칙
└── tests/
    ├── test_step0_deps.py     # 시스템 의존성
    ├── test_step1_sdk.py      # SDK 로드 + 동글
    ├── test_step2_connection.py  # 글러브 연결
    ├── test_step3_stream.py   # 연속 스트리밍
    └── test_step4_udp.py      # UDP 송수신
```
