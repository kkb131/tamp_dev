# Manus + Tesollo 테스트 가이드

테스트 PC에서 Manus Quantum Metagloves → Tesollo DG 5F M 핸드 파이프라인을 단계별로 검증하는 가이드.

---

## 전체 구조

```
Operator PC (이 PC)                  Robot PC (AGX Orin)
┌──────────────┐    UDP:9872         ┌─────────────────────┐
│ Manus 글러브  │ ─────────────────> │ tesollo/receiver.py │
│ manus_sender │                     │   → retarget.py     │
│  (60Hz)      │                     │   → dg5f_client.py  │
└──────────────┘                     │   → DG5F Hand       │
                                     └─────────────────────┘
```

테스트는 4개 Phase로 나뉘며, Phase 1은 하드웨어 없이 실행 가능합니다.

---

## 0. 사전 준비

### 0.1 Miniconda 설치 (최초 1회)

```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
# 설치 후 터미널 재시작
```

### 0.2 코드 준비

```bash
cd ~/tamp_ws/src/tamp_dev
git pull   # 최신 코드 받기
```

### 0.3 Conda 환경 생성

```bash
cd ~/tamp_ws/src/tamp_dev
conda env create -f environment.yaml    # 최초 1회
# 또는 업데이트:
conda env update -f environment.yaml --prune
```

### 0.4 환경 활성화

```bash
conda activate tamp_sender
```

### 0.5 Manus SDK 설치 (Phase 2부터 필요)

1. https://docs.manus-meta.com/3.1.0/Plugins/SDK/Linux/ 에서 다운로드
2. 압축 해제:
   ```bash
   tar xzf ManusSDK_Linux_*.tar.gz -C ~/tamp_ws/src/tamp_dev/manus/sdk/
   ls -la manus/sdk/libManusSDK*.so    # 파일 확인 (libManusSDK.so 또는 libManusSDK_Integrated.so)
   ```

### 0.6 LD_LIBRARY_PATH 자동 설정 (최초 1회)

```bash
mkdir -p $CONDA_PREFIX/etc/conda/activate.d
cat > $CONDA_PREFIX/etc/conda/activate.d/manus_env.sh << 'EOF'
#!/bin/bash
export MANUS_SDK_PATH=~/tamp_ws/src/tamp_dev/manus/sdk
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MANUS_SDK_PATH
EOF

# 재활성화
conda deactivate && conda activate tamp_sender
```

---

## 올인원 테스트 러너 사용법

```bash
cd ~/tamp_ws/src/tamp_dev
conda activate tamp_sender

# Phase 1만 (하드웨어 불필요, 가장 먼저 실행)
bash run_tests.sh

# Phase 2 (Manus 글러브)
bash run_tests.sh --phase 2

# Phase 3 (Tesollo DG5F 핸드)
bash run_tests.sh --phase 3 --hand-ip 169.254.186.72

# 전체 실행
bash run_tests.sh --all --hand-ip 169.254.186.72

# 왼손 테스트
bash run_tests.sh --all --hand left
```

테스트 결과는 `test_logs/test_results_YYYYMMDD_HHMMSS.log`에 저장됩니다.

---

## Phase 1: 소프트웨어 검증 (하드웨어 불필요)

```bash
bash run_tests.sh --phase 1
```

### 포함 테스트

| ID | 테스트 | 설명 |
|----|--------|------|
| M0 | `manus.tests.test_step0_deps` | gcc, libusb, Python 3.10+, numpy, pyyaml, pynput |
| T1 | `tesollo.tests.test_retarget` | Manus→DG5F 리타겟 로직 (10개 단위 테스트) |
| T2 | `tesollo.tests.test_e2e` | E2E mock 파이프라인 (UDP → 리타겟 → 검증) |
| M4 | `manus.tests.test_step4_udp` | UDP 송수신 (mock 데이터) |

### 예상 성공 출력

```
[M0] 시스템 의존성 체크
  Result: PASS

[T1] Manus→DG5F 리타겟 로직
  Results: 10/10 passed, 0/10 failed
  Result: PASS

[T2] E2E mock 파이프라인
  Results: 8/8 passed, 0/8 failed
  Result: PASS

[M4] UDP 송수신
  Results: 7/7 passed, 0/7 failed
  Result: PASS
```

### 실패 시 대처

- **M0 FAIL**: `sudo apt install build-essential libusb-1.0-0-dev zlib1g-dev` 실행
- **M0 pynput FAIL**: `pip install pynput` 실행
- **T1/T2 FAIL**: 코드 문제 — `git pull`로 최신 코드 확인

---

## Phase 2: Manus 글러브 (SDK + USB 동글 + 글러브)

```bash
bash run_tests.sh --phase 2
```

### 사전 조건

- [x] Manus SDK (`libManusSDK.so` 또는 `libManusSDK_Integrated.so`)가 `manus/sdk/`에 설치됨
- [x] USB 라이선스 동글이 PC에 연결됨
- [x] Manus Quantum Metagloves 전원 ON

### 포함 테스트

| ID | 테스트 | 설명 | 하드웨어 |
|----|--------|------|----------|
| M1 | `manus.tests.test_step1_sdk` | SDK ctypes 로드 + 동글 감지 | USB 동글 |
| M2 | `manus.tests.test_step2_connection` | 글러브 BLE 연결 | 글러브 전원 ON |
| M3 | `manus.tests.test_step3_stream` | 5초 데이터 스트리밍 | 글러브 착용 |

### 수동 실행 (개별)

```bash
# SDK만 테스트
python3 -m manus.tests.test_step1_sdk  # libManusSDK.so 또는 libManusSDK_Integrated.so 자동 탐색

# 글러브 연결 (글러브 전원 켜고 대기)
python3 -m manus.tests.test_step2_connection --hand right

# 스트리밍 (글러브 착용 후 손가락 움직이기)
python3 -m manus.tests.test_step3_stream --duration 10 --hz 60 --hand right
```

### 예상 성공 출력 (M3)

```
Per-finger joint ranges (min / max / range):
Finger   Joint           Min      Max    Range
------------------------------------------------
Thumb    CMC_Spread   -0.123   +0.456    0.579
Thumb    CMC_Flex     +0.012   +1.234    1.222
...
Data availability: 98.5% (591/600 frames)
```

### 실패 시 대처

- **M1 SDK 로드 실패**: `ldd manus/sdk/libManusSDK*.so`로 누락 라이브러리 확인
- **M1 동글 미감지**: `lsusb | grep -i manus`로 동글 확인, udev 규칙 설치
- **M2 글러브 미연결**: 글러브 충전 확인, 전원 끄고 다시 켜기, 동글 근처(5m 이내)
- **M3 데이터 없음**: 글러브 착용 상태 확인

---

## Phase 3: Tesollo DG5F 핸드 (이더넷 연결)

```bash
bash run_tests.sh --phase 3 --hand-ip 169.254.186.72
```

### 사전 조건

- [x] DG5F 핸드에 전원 연결
- [x] 이더넷 케이블로 PC와 DG5F 직접 연결
- [x] PC 네트워크 인터페이스 IP를 같은 서브넷으로 설정

### 네트워크 설정

```bash
# PC의 이더넷 인터페이스 확인
ip addr show

# IP 설정 (인터페이스 이름은 다를 수 있음)
sudo ip addr add 169.254.186.1/24 dev eth0
# 또는 NetworkManager 사용:
# sudo nmcli con mod "유선 연결" ipv4.addresses "169.254.186.1/24" ipv4.method manual

# 핑 확인
ping 169.254.186.72
```

### 포함 테스트

| ID | 테스트 | 설명 |
|----|--------|------|
| T3 | `tesollo.tests.test_modbus` | Modbus TCP 연결, 위치/전류/속도 읽기, 시스템 시작/정지 |

### 수동 실행

```bash
# 읽기만 테스트
python3 -m tesollo.tests.test_modbus --ip 169.254.186.72

# 쓰기 포함 (주의: 모터가 0 위치로 이동)
python3 -m tesollo.tests.test_modbus --ip 169.254.186.72 --write
```

### 예상 성공 출력

```
[TEST] Modbus TCP connection... [PASS]
[TEST] Read current positions... [PASS] 20 joints read
       rj_dg_1_1: +0.0123 rad (+0.7 deg)
       rj_dg_1_2: -0.2345 rad (-13.4 deg)
       ...
[TEST] Read motor currents... [PASS] max |current| = 0.0234 A
[TEST] System start... [PASS]
[TEST] System stop... [PASS]
```

### 실패 시 대처

- **Ping 실패**: IP 설정 확인 (`ip addr show`), 케이블 교체
- **Modbus 연결 실패**: DG5F 전원 확인, 포트 502 방화벽 확인 (`sudo ufw allow 502`)
- **레지스터 읽기 실패**: DG5F 펌웨어 버전 확인

---

## Phase 4: 통합 테스트 (두 대의 PC)

```bash
bash run_tests.sh --phase 4
```

이 Phase는 안내 메시지만 출력합니다. 수동으로 진행:

### 4.1 Dry-run (DG5F 없이)

같은 PC에서 두 터미널을 열고:

```bash
# Terminal 1: UDP 수신 + 리타겟 (DG5F 미연결)
python3 -m tesollo.receiver --dry-run

# Terminal 2: mock 데이터 전송
python3 -m manus.tests.test_step4_udp
```

Terminal 1에서 리타겟 결과가 실시간으로 출력되면 성공.

### 4.2 실제 통합

```
┌─ Operator PC ────────────────────────────────┐
│  conda activate tamp_sender                   │
│  cd ~/tamp_ws/src/tamp_dev                    │
│                                               │
│  python3 -m manus.manus_sender \              │
│      --target-ip <ROBOT_PC_IP> \              │
│      --hand right                             │
│                                               │
│  키보드: Space=E-Stop, R=Reset, Q=Quit        │
└───────────────────────────────────────────────┘
                    │ UDP:9872
                    ▼
┌─ Robot PC ────────────────────────────────────┐
│  conda activate tamp_sender                   │
│  cd ~/tamp_ws/src/tamp_dev                    │
│                                               │
│  python3 -m tesollo.receiver \                │
│      --hand-ip 169.254.186.72 \               │
│      --hand right                             │
│                                               │
│  Ctrl+C로 종료                                 │
└───────────────────────────────────────────────┘
```

### 확인 사항

- [ ] Operator PC에서 글러브 데이터 수신 확인 (manus_sender 출력)
- [ ] Robot PC에서 UDP 패킷 수신 확인 (receiver 출력의 Pkts 카운터)
- [ ] 리타겟 결과 출력 확인 (Manus 각도 → DG5F 각도)
- [ ] DG5F 핸드가 글러브 움직임에 따라 동작

---

## 비주얼라이저

실시간 손 스켈레톤 시각화:

```bash
# Mock 데이터 (하드웨어 없이 테스트)
python3 -m manus.hand_visualizer

# 실제 글러브 데이터
python3 -m manus.hand_visualizer --sdk

# UDP 수신 (다른 PC에서 manus_sender 실행 중)
python3 -m manus.hand_visualizer --udp --port 9872

# 왼손
python3 -m manus.hand_visualizer --hand left
```

기능:
- 2D 손 스켈레톤 (관절 각도 반영)
- 20개 관절 바 차트 (실시간)
- 손목 위치/방향 표시
- FPS + 데이터 수신 Hz

종료: `ESC` 또는 `Q`

---

## 캘리브레이션 (선택)

사용자별 손가락 ROM (가동 범위) 기록:

```bash
python3 -m manus.calibrate --hand right --output manus/calibration_right.json
```

절차:
1. 손 완전히 펴기 → Enter
2. 주먹 쥐기 → Enter
3. `calibration_right.json` 저장됨

설정에 적용:
```yaml
# manus/config/default.yaml
joint_mapping:
  calibration_file: "manus/calibration_right.json"
```

---

## 트러블슈팅

### conda activate 실패

```bash
conda init bash
# 터미널 재시작 후
conda activate tamp_sender
```

### pynput 에러: "Xlib.error.DisplayConnectionError"

pynput은 X11 디스플레이가 필요합니다. SSH 환경에서:
```bash
export DISPLAY=:0
# 또는 headless 모드 (키보드 입력 비활성화):
export PYNPUT_BACKEND=dummy
```

### Manus SDK 의존 라이브러리 누락

```bash
ldd manus/sdk/libManusSDK*.so
# "not found" 항목 확인 후:
sudo apt install <missing-lib>
```

### DG5F IP 변경

기본 IP `169.254.186.72`가 아닌 경우:
```bash
# 네트워크 스캔
sudo nmap -sP 169.254.186.0/24

# 또는 Wireshark에서 Modbus TCP 패킷 확인
```

### UDP 패킷 미수신

```bash
# 방화벽 확인
sudo ufw status
sudo ufw allow 9872/udp

# 포트 리스닝 확인
ss -ulnp | grep 9872
```

### pymodbus 미설치

```bash
pip install pymodbus>=3.6
```

---

## 파일 구조 요약

```
src/tamp_dev/
├── run_tests.sh              ← 올인원 테스트 러너
├── docs/
│   └── testing_guide.md      ← 이 파일
├── manus/
│   ├── manus_reader.py       # SDK ctypes 래퍼
│   ├── manus_sender.py       # UDP 데이터 전송
│   ├── hand_visualizer.py    # 실시간 시각화
│   ├── calibrate.py          # ROM 캘리브레이션
│   ├── config/default.yaml
│   ├── sdk/                  # libManusSDK*.so (별도 다운로드)
│   └── tests/
│       ├── test_step0_deps.py
│       ├── test_step1_sdk.py
│       ├── test_step2_connection.py
│       ├── test_step3_stream.py
│       └── test_step4_udp.py
├── tesollo/
│   ├── dg5f_client.py        # Modbus TCP 드라이버
│   ├── retarget.py           # Manus→DG5F 리타겟
│   ├── receiver.py           # UDP 수신 + 제어 루프
│   ├── tesollo_config.py
│   ├── config/default.yaml
│   └── tests/
│       ├── test_retarget.py  # 리타겟 단위 테스트
│       ├── test_modbus.py    # Modbus 연결 테스트
│       └── test_e2e.py       # E2E mock 테스트
└── environment.yaml          # Conda 환경 정의
```
