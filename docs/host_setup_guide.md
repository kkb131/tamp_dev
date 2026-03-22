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

## 4. Manus 글러브 설치 (선택)

### 4.1 Manus Core SDK

Manus Developer Portal에서 SDK 다운로드 후:

```bash
# libManusSDK.so 배치
cp /path/to/libManusSDK.so ~/tamp_ws/src/tamp_dev/manus/sdk/

# 라이브러리 경로 등록
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/tamp_ws/src/tamp_dev/manus/sdk
```

`LD_LIBRARY_PATH`를 매번 설정하지 않으려면 `~/.bashrc`에 추가:
```bash
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/tamp_ws/src/tamp_dev/manus/sdk' >> ~/.bashrc
```

### 4.2 USB 권한 (udev rules)

```bash
# udev 규칙 설치
sudo cp ~/tamp_ws/src/tamp_dev/manus/udev/70-manus-hid.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# USB 동글 재연결 후 확인
ls -la /dev/hidraw*
```

---

## 5. Sender 실행

모든 sender는 `src/tamp_dev/` 디렉토리에서 실행합니다.

```bash
conda activate tamp_sender
cd ~/tamp_ws/src/tamp_dev
```

### 5.1 Vive Tracker Sender

```bash
# 트래커 확인
python3 -m vive.vive_tracker --list

# Sender 실행
python3 -m vive.vive_sender --target-ip <ROBOT_PC_IP> --port 9871

# Config 파일 사용
python3 -m vive.vive_sender --config vive/config/default.yaml --target-ip <ROBOT_PC_IP>
```

키보드 단축키: `Space`=E-Stop, `R`=Reset, `Q`/`Esc`=Quit, `+`/`-`=Speed

### 5.2 Manus 글러브 Sender

```bash
# SDK 로드 확인
python3 -m manus.tests.test_step1_sdk

# Sender 실행
python3 -m manus.manus_sender --target-ip <ROBOT_PC_IP> --port 9872
```

### 5.3 Joystick Sender

```bash
# 조이스틱 연결 확인
python3 -c "import pygame; pygame.init(); pygame.joystick.init(); print(f'{pygame.joystick.get_count()} joystick(s)')"

# Sender 실행
python3 standalone/core/joystick_sender.py --target-ip <ROBOT_PC_IP> --port 9870
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

# 4. Sender 실행 (필요한 것만)
python3 -m vive.vive_sender --target-ip 192.168.0.10 --port 9871
python3 -m manus.manus_sender --target-ip 192.168.0.10 --port 9872
python3 standalone/core/joystick_sender.py --target-ip 192.168.0.10 --port 9870
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
