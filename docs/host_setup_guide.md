# 조종 PC 호스트 환경 세팅 가이드

조종 PC(RTX 5090)에서 Vive Tracker, Manus 글러브, 조이스틱 등 하드웨어 sender를
Docker 없이 호스트에서 직접 실행하기 위한 환경 구성 가이드.

## 전체 구조

```
조종 PC (RTX 5090, Ubuntu 22.04)
┌──────────────────────────────────────────────────────┐
│ 호스트 OS                                             │
│  ├── Steam + SteamVR              (시스템 설치)       │
│  ├── Manus Core SDK               (별도 설치)        │
│  ├── conda env: tamp_sender       (Python 3.10)      │
│  │    └── openvr, pynput, numpy, pyyaml, pygame      │
│  │                                                    │
│  └── Sender 실행:                                    │
│       ├── vive_sender.py      ──UDP:9871──┐          │
│       ├── manus_sender.py     ──UDP:9872──┼──> 로봇 PC
│       └── joystick_sender.py  ──UDP:9870──┘          │
│                                                      │
│ Docker 컨테이너 (개발 환경, 선택)                      │
│  ├── 코드 편집 / 빌드 / 테스트                         │
│  └── volume mount로 코드 공유                         │
└──────────────────────────────────────────────────────┘
```

**원칙**: 하드웨어 접근이 필요한 sender는 호스트에서 실행, 로봇 제어 로직은 로봇 PC Docker에서 실행.

---

## 1. Miniconda 설치

이미 설치되어 있다면 건너뛰세요.

```bash
# Miniconda 설치 (Linux x86_64)
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -p $HOME/miniconda3

# Shell 초기화
~/miniconda3/bin/conda init bash
source ~/.bashrc

# 설치 확인
conda --version
```

---

## 2. Conda 환경 생성

### 2.1 코드 가져오기

두 가지 방법 중 선택:

**방법 A: Git clone (권장)**
```bash
cd ~
git clone <REPO_URL> tamp_ws
cd tamp_ws/src/tamp_dev
```

**방법 B: Docker 컨테이너와 코드 공유**
```bash
# Docker 실행 시 호스트 디렉토리를 마운트
docker run ... -v ~/tamp_ws:/workspaces/tamp_ws ...

# 호스트에서 같은 경로 사용
cd ~/tamp_ws/src/tamp_dev
```

### 2.2 환경 생성

```bash
cd ~/tamp_ws/src/tamp_dev

# environment.yaml로 conda 환경 생성
conda env create -f environment.yaml

# 활성화
conda activate tamp_sender

# 설치 확인
python3 -c "import openvr; print('openvr OK')"
python3 -c "import pynput; print('pynput OK')"
python3 -c "import pygame; print('pygame OK')"
python3 -c "import numpy; print('numpy OK')"
python3 -c "import yaml; print('pyyaml OK')"
```

모두 `OK`가 출력되면 환경 준비 완료.

---

## 3. SteamVR 설치 (Vive Tracker용)

### 3.1 Steam 설치

```bash
sudo dpkg --add-architecture i386
sudo apt update
sudo apt install -y steam
```

### 3.2 SteamVR 설치

1. Steam 실행 → 로그인
2. Library → 검색: "SteamVR" → Install

### 3.3 헤드셋 없이 트래커만 사용 (Null Driver)

```bash
# SteamVR 설정 파일 편집
nano ~/.steam/steam/config/steamvr.vrsettings
```

다음 내용 추가 (또는 기존 내용에 병합):
```json
{
   "steamvr": {
      "requireHmd": false,
      "forcedDriver": "null",
      "activateMultipleDrivers": true
   }
}
```

### 3.4 Vive Tracker 페어링

1. SteamVR 실행
2. USB 동글 연결
3. 트래커 전원 버튼 길게 누르기 (LED 파란색 점멸)
4. SteamVR → Devices → Pair Controller
5. 녹색 아이콘 = 트래킹 정상

---

## 4. Manus 글러브 설치

> 상세 가이드: `manus/README.md` 참조

### 4.1 Manus SDK 다운로드

1. Manus Developer Portal에서 Linux SDK 다운로드:
   - https://docs.manus-meta.com/2.4.0/Plugins/SDK/Linux/
   - "Download SDK" → Linux 버전 선택

2. SDK 파일 배치:
```bash
# 다운로드한 아카이브 압축 해제
tar xzf ManusSDK_Linux_*.tar.gz -C ~/tamp_ws/src/tamp_dev/manus/sdk/

# 핵심 파일 확인
ls -la ~/tamp_ws/src/tamp_dev/manus/sdk/libManusSDK.so
```

### 4.2 LD_LIBRARY_PATH 자동 설정 (conda activate.d)

`conda activate tamp_sender` 시 자동으로 `LD_LIBRARY_PATH`가 설정되도록 구성:

```bash
# activate.d 디렉토리 생성
mkdir -p $CONDA_PREFIX/etc/conda/activate.d
mkdir -p $CONDA_PREFIX/etc/conda/deactivate.d

# activate 스크립트 생성
cat > $CONDA_PREFIX/etc/conda/activate.d/manus_env.sh << 'EOF'
#!/bin/bash
export MANUS_SDK_PATH=~/tamp_ws/src/tamp_dev/manus/sdk
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MANUS_SDK_PATH
EOF

# deactivate 스크립트 생성 (정리)
cat > $CONDA_PREFIX/etc/conda/deactivate.d/manus_env.sh << 'EOF'
#!/bin/bash
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed "s|:$MANUS_SDK_PATH||g")
unset MANUS_SDK_PATH
EOF
```

설정 후 재활성화하면 자동 적용:
```bash
conda deactivate && conda activate tamp_sender
echo $MANUS_SDK_PATH  # ~/tamp_ws/src/tamp_dev/manus/sdk
```

> **대안**: `~/.bashrc`에 직접 추가해도 됩니다:
> ```bash
> echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/tamp_ws/src/tamp_dev/manus/sdk' >> ~/.bashrc
> ```

### 4.3 USB 권한 (udev rules)

```bash
# udev 규칙 설치
sudo cp ~/tamp_ws/src/tamp_dev/manus/udev/70-manus-hid.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# USB 동글 꽂은 후 확인
lsusb | grep -i manus
ls -la /dev/hidraw*
```

> **NOTE**: `70-manus-hid.rules`의 vendor/product ID가 placeholder입니다.
> USB 동글을 꽂은 후 `lsusb`로 실제 ID를 확인하고 파일을 업데이트하세요.

### 4.4 Manus USB 라이선스 동글

Manus Quantum Metagloves 사용에는 라이선스 USB 동글이 필요합니다.
동글을 꽂은 상태에서 글러브 전원을 켜면 BLE로 자동 연결됩니다.

### 4.5 단계별 검증

```bash
conda activate tamp_sender
cd ~/tamp_ws/src/tamp_dev

# Step 0: 시스템 의존성 (하드웨어 불필요)
python3 -m manus.tests.test_step0_deps

# Step 1: SDK 로드 + USB 동글 감지
python3 -m manus.tests.test_step1_sdk

# Step 2: 글러브 연결 (글러브 전원 ON 필요)
python3 -m manus.tests.test_step2_connection --hand right

# Step 3: 데이터 스트리밍 (글러브 착용)
python3 -m manus.tests.test_step3_stream --duration 5 --hz 60

# Step 4: UDP 송수신 (mock 데이터, 글러브 불필요)
python3 -m manus.tests.test_step4_udp
```

### 4.6 캘리브레이션 (선택)

사용자별 손가락 가동 범위를 기록하여 관절 각도를 [0, 1]로 정규화:

```bash
python3 -m manus.calibrate --hand right --output manus/calibration_right.json
```

절차: 손 완전히 펴기 → Enter → 주먹 쥐기 → Enter → JSON 저장

---

## 5. Sender 실행

모든 sender는 **통합 텔레옵 프로토콜**을 사용하여 로봇 base_link 프레임의 절대 목표 포즈를 전송합니다.
상세 프로토콜 설명: [`docs/unified_teleop_guide.md`](unified_teleop_guide.md)

```bash
conda activate tamp_sender
cd ~/tamp_ws/src/tamp_dev
```

### 5.1 Vive Tracker Sender

```bash
# 트래커 확인
python3 -m vive.vive_tracker --list

# Sender 실행 (통합 프로토콜)
python3 -m vive.vive_sender --target-ip <ROBOT_PC_IP>

# 캘리브레이션 적용
python3 -m vive.vive_sender --target-ip <ROBOT_PC_IP> --calibration vive/calibration.json

# Config 파일 사용
python3 -m vive.vive_sender --config vive/config/default.yaml --target-ip <ROBOT_PC_IP>
```

키보드 단축키: `Space`=E-Stop, `R`=Reset, `Q`/`Esc`=Quit, `+`/`-`=Speed

### 5.2 키보드 Sender

```bash
python3 -m vive.keyboard_sender --target-ip <ROBOT_PC_IP>
```

키 매핑: `W/S`=Y, `A/D`=X, `Q/E`=Z, `U/O`=Roll, `I/K`=Pitch, `J/L`=Yaw, `+/-`=Speed

### 5.3 조이스틱 Sender (Xbox/Logitech)

```bash
# 조이스틱 연결 확인
python3 -c "import pygame; pygame.init(); pygame.joystick.init(); print(f'{pygame.joystick.get_count()} joystick(s)')"

# Sender 실행 (통합 프로토콜)
python3 -m vive.joystick_sender --target-ip <ROBOT_PC_IP>
```

### 5.4 Manus 글러브 Sender

```bash
# SDK 로드 확인
python3 -m manus.tests.test_step1_sdk

# Sender 실행 (별도 프로토콜, 포트 9872)
python3 -m manus.manus_sender --target-ip <ROBOT_PC_IP> --port 9872
```

### 5.5 로봇 PC 수신부

로봇 PC에서 `--input unified`로 실행하면 위의 모든 sender(Vive/키보드/조이스틱)를 동일하게 수신:

```bash
# 로봇 PC (Docker 컨테이너 내)
python3 -m standalone.teleop_admittance.main --mode rtde --input unified --robot-ip 192.168.0.2
python3 -m standalone.teleop_impedance.main --mode rtde --input unified --robot-ip 192.168.0.2

# Sim 모드 테스트
python3 -m standalone.teleop_admittance.main --mode sim --input unified
```

---

## 6. 실행 전 체크리스트

### Vive Tracker

```bash
conda activate tamp_sender
cd ~/tamp_ws/src/tamp_dev

# Step 1: OpenVR 연결
python3 -m vive.tests.test_step1_openvr

# Step 2: 트래커 포즈
python3 -m vive.tests.test_step2_pose --duration 3

# Step 3: UDP 통신 (Vive 불필요)
python3 -m vive.tests.test_step3_udp --port 9873
```

### Manus 글러브

```bash
# Step 0: 시스템 의존성
python3 -m manus.tests.test_step0_deps

# Step 1: SDK 로드
python3 -m manus.tests.test_step1_sdk

# Step 2: 글러브 연결
python3 -m manus.tests.test_step2_connection
```

### 네트워크 연결

```bash
# 로봇 PC 연결 확인
ping <ROBOT_PC_IP>

# UDP 포트 열기 (필요 시)
sudo ufw allow 9870/udp
sudo ufw allow 9871/udp
sudo ufw allow 9872/udp
```

---

## 7. 빠른 시작 (요약)

```bash
# 1. 환경 생성 (최초 1회)
conda env create -f environment.yaml

# 2. 환경 활성화
conda activate tamp_sender

# 3. 작업 디렉토리 이동
cd ~/tamp_ws/src/tamp_dev

# 4. Sender 실행 (필요한 것만, 통합 프로토콜)
python3 -m vive.vive_sender --target-ip 192.168.0.10
python3 -m vive.keyboard_sender --target-ip 192.168.0.10
python3 -m vive.joystick_sender --target-ip 192.168.0.10
python3 -m manus.manus_sender --target-ip 192.168.0.10 --port 9872
```

---

## 8. 트러블슈팅

### `conda activate` 실패
```bash
# conda init 재실행
conda init bash
source ~/.bashrc
```

### `ModuleNotFoundError: No module named 'openvr'`
```bash
# 환경 활성화 확인
which python3
# → ~/miniconda3/envs/tamp_sender/bin/python3 이어야 함

# 재설치
pip install openvr
```

### `openvr.error.OpenVRError: VRInitError_Init_HmdNotFoundPresenceFailed`
- SteamVR이 실행 중인지 확인
- null driver 설정 적용 여부 확인 (섹션 3.3)
- `~/.steam/steam/config/steamvr.vrsettings` 확인

### Manus `libManusSDK.so` 로드 실패
```bash
# 파일 존재 확인
ls -la ~/tamp_ws/src/tamp_dev/manus/sdk/libManusSDK.so

# LD_LIBRARY_PATH 확인
echo $LD_LIBRARY_PATH

# 직접 로드 테스트
python3 -c "import ctypes; ctypes.cdll.LoadLibrary('~/tamp_ws/src/tamp_dev/manus/sdk/libManusSDK.so')"
```

### USB 장치 권한 문제
```bash
# 현재 사용자를 plugdev 그룹에 추가
sudo usermod -aG plugdev $USER

# 로그아웃 후 재로그인 필요
```

### 조이스틱 인식 안 됨
```bash
# 장치 확인
ls /dev/input/js*

# pygame에서 확인
python3 -c "
import pygame
pygame.init()
pygame.joystick.init()
n = pygame.joystick.get_count()
print(f'Found {n} joystick(s)')
for i in range(n):
    js = pygame.joystick.Joystick(i)
    js.init()
    print(f'  [{i}] {js.get_name()}')
"
```

---

## 9. 환경 업데이트/삭제

```bash
# 패키지 추가 설치
conda activate tamp_sender
pip install <new_package>

# environment.yaml 업데이트 후 재생성
conda env update -f environment.yaml --prune

# 환경 삭제
conda deactivate
conda env remove -n tamp_sender
```
