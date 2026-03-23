# 원격조작 통합 가이드 (Unified Teleop Protocol)

모든 입력장치(Vive Tracker, 키보드, 조이스틱)가 **동일한 프로토콜**로 로봇을 원격조작하는 통합 시스템 사용 가이드.

---

## 전체 구조

```
조종 PC (Operator PC)                              로봇 PC (Robot PC)
┌──────────────────────────────────┐    UDP 9871    ┌─────────────────────────┐
│                                  │    (JSON)      │                         │
│  Vive Tracker ──→ vive_sender   ─┼──────────────→│                         │
│  키보드      ──→ keyboard_sender ┤  통합 패킷     │  UnifiedNetworkInput    │
│  조이스틱    ──→ joystick_sender ┤  (absolute     │    ↓                    │
│                                  │   pose in      │  ExpFilter (스무딩)      │
│  좌표변환 + 속도→포즈 적분 처리   │   base_link)  │    ↓                    │
│  (두뇌)                          │               │  Workspace Clamp (안전) │
│                                  │               │    ↓                    │
│                                  │  ← query_pose │  Pink IK → q_target    │
│  시작 시 로봇 현재 포즈 수신 ←───┼───────────────│    ↓                    │
│  (초기 가상 포즈)                │               │  SafetyMonitor          │
│                                  │               │    ↓                    │
└──────────────────────────────────┘               │  servoJ (실행)          │
                                                   │  (팔)                   │
                                                   └─────────────────────────┘
```

**핵심 원칙**:
- **조종 PC** = 센서 읽기 + 좌표변환 + 포즈 적분 (두뇌)
- **로봇 PC** = 수신 → 필터 → IK → 안전검사 → 실행 (팔)
- 모든 입력장치가 **동일한 패킷 형식**으로 로봇 base_link 프레임의 **절대 목표 포즈**를 전송

---

## 1. 통합 프로토콜 패킷

### 1.1 텔레옵 명령 패킷 (Operator → Robot)

```json
{
    "v": 1,
    "type": "teleop_pose",
    "pos": [0.1, -0.4, 0.3],
    "quat": [1.0, 0.0, 0.0, 0.0],
    "buttons": {
        "estop": false,
        "reset": false,
        "quit": false,
        "speed_up": false,
        "speed_down": false,
        "ft_zero": false,
        "admittance_toggle": false,
        "admittance_preset": "",
        "admittance_cycle": false,
        "impedance_preset": "",
        "gain_scale_up": false,
        "gain_scale_down": false
    },
    "gripper": 0.0,
    "timestamp": 1711180800.123
}
```

| 필드 | 타입 | 설명 |
|------|------|------|
| `v` | int | 프로토콜 버전 (현재 1) |
| `type` | str | `"teleop_pose"` 고정 |
| `pos` | float[3] | 목표 위치 [x, y, z] (m), robot base_link 기준 |
| `quat` | float[4] | 목표 방향 [w, x, y, z] (wxyz 규약) |
| `buttons` | dict | 버튼/명령 플래그 (edge-triggered) |
| `gripper` | float | 그리퍼 개폐 0.0~1.0 (향후 확장) |
| `timestamp` | float | `time.time()` (레이턴시 모니터링용) |

### 1.2 초기 포즈 질의 (Operator → Robot, 시작 시 1회)

```json
{"type": "query_pose"}
```

로봇 PC가 현재 TCP 포즈를 응답:

```json
{"type": "pose_response", "pos": [0.1, -0.4, 0.3], "quat": [1.0, 0.0, 0.0, 0.0]}
```

**동작 흐름**:
1. Sender 시작 → `query_pose` 전송
2. 로봇 PC의 `UnifiedNetworkInput`이 수신 → FK로 현재 TCP 계산 → 응답
3. Sender가 이 포즈를 **가상 목표점(virtual pose)** 초기값으로 설정
4. 이후 입력 델타를 적분하여 절대 포즈로 전송

> 질의 실패 시 (로봇 PC 미실행 등) 기본 home pose `[0.0, -0.4, 0.4]`로 fallback

---

## 2. 빠른 시작

### 2.1 로봇 PC (수신 + 실행)

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# Sim 모드 (mock hardware, 테스트용)
python3 -m standalone.teleop_admittance.main --mode sim --input unified

# 실제 로봇 (RTDE)
python3 -m standalone.teleop_admittance.main --mode rtde --input unified --robot-ip 192.168.0.2

# 임피던스 모드
python3 -m standalone.teleop_impedance.main --mode sim --input unified
```

### 2.2 조종 PC (입력 + 전송)

```bash
conda activate tamp_sender
cd ~/tamp_ws/src/tamp_dev

# 키보드로 조종
python3 -m vive.keyboard_sender --target-ip <ROBOT_PC_IP>

# 조이스틱(로지텍/Xbox)으로 조종
python3 -m vive.joystick_sender --target-ip <ROBOT_PC_IP>

# Vive Tracker로 조종 (SteamVR + 트래커 필요)
python3 -m vive.vive_sender --target-ip <ROBOT_PC_IP>
```

> **같은 PC에서 테스트**: `--target-ip 127.0.0.1` 사용 (터미널 2개)

---

## 3. 입력장치별 상세

### 3.1 키보드 (keyboard_sender)

```bash
python3 -m vive.keyboard_sender --target-ip <IP> [옵션]
```

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--port` | 9871 | UDP 포트 |
| `--hz` | 50 | 전송 주파수 |
| `--cart-step` | 0.005 | 틱당 이동량 (m) |
| `--rot-step` | 0.05 | 틱당 회전량 (rad) |

**키 매핑**:

| 키 | 동작 | 키 | 동작 |
|----|------|----|------|
| W / S | Y 전진/후진 | U / O | Roll ± |
| A / D | X 좌/우 | I / K | Pitch ± |
| Q / E | Z 상/하 | J / L | Yaw ± |
| Space | E-Stop | R | Reset (포즈 재동기화) |
| X / Esc | 종료 | + / - | 속도 증가/감소 |

**속도 프리셋**: 0.1x → 0.2x → **0.3x** (기본) → 0.5x → 0.8x → 1.0x

### 3.2 조이스틱 (joystick_sender)

Xbox, Logitech F710/F310 등 표준 게임패드와 호환.

```bash
python3 -m vive.joystick_sender --target-ip <IP> [옵션]
```

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--port` | 9871 | UDP 포트 |
| `--hz` | 50 | 전송 주파수 |
| `--linear-scale` | 0.01 | 선형 속도 스케일 (m/tick) |
| `--angular-scale` | 0.05 | 각속도 스케일 (rad/tick) |
| `--deadzone` | 0.1 | 축 데드존 |

**컨트롤러 매핑**:

| 입력 | 동작 |
|------|------|
| L-스틱 X/Y | X/Y 이동 |
| LT / RT (트리거) | Z 하강 / 상승 |
| R-스틱 X/Y | Yaw / Pitch |
| LB / RB (범퍼) | Roll -/+ |
| A / B | 속도 감소 / 증가 |
| X | F/T 센서 영점 |
| Y | 어드미턴스 프리셋 순환 |
| Back | Reset |
| Start | 종료 |
| L-스틱 클릭 | E-Stop |

### 3.3 Vive Tracker (vive_sender)

SteamVR + Vive Tracker 3.0 필요. Relative 매핑: 트래커의 **움직임 변화량(delta)**을 로봇 포즈에 적용.

```bash
python3 -m vive.vive_sender --target-ip <IP> [옵션]
```

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--config` | `vive/config/default.yaml` | YAML 설정 파일 |
| `--port` | 9871 | UDP 포트 |
| `--hz` | 50 | 전송 주파수 |
| `--tracker-serial` | 자동 | 특정 트래커 지정 (LHR-XXXXXXXX) |
| `--calibration` | null | 캘리브레이션 JSON 파일 |
| `--list-trackers` | - | 트래커 목록 출력 후 종료 |

**키보드 단축키** (pynput):

| 키 | 동작 |
|----|------|
| Space | E-Stop |
| R | Reset (로봇 포즈 재동기화) |
| Q / Esc | 종료 |
| + / - | 속도 증가/감소 |

**Relative 매핑 동작 원리**:
1. 시작 시 로봇 현재 TCP 포즈를 수신 → 가상 포즈 초기화
2. 매 프레임: 트래커 이전/현재 포즈 비교 → delta 계산
3. delta를 캘리브레이션 변환 (SteamVR → robot frame) 적용
4. 가상 포즈에 delta 누적 → 절대 포즈로 전송

```
Tracker(t-1) → Tracker(t)  →  delta = T(t) - T(t-1)
                                ↓ calibration
                            delta_robot = R @ delta_vive
                                ↓
                            virtual_pos += delta_robot
                                ↓
                            Send(virtual_pos, virtual_quat)
```

> **캘리브레이션 없이 사용**: 기본 SteamVR Y-up → UR Z-up 축 매핑이 적용됨.
> 정밀한 매핑은 `python3 -m vive.calibrate` 으로 3점 캘리브레이션 수행.

---

## 4. Reset 동작

**Reset** 버튼 (키보드 R, 조이스틱 Back, Vive R키)을 누르면:

1. Sender 측: 로봇 PC에 `reset` 플래그가 켜진 패킷 전송
2. 로봇 PC: IK/필터/타겟을 현재 로봇 상태로 재동기화
3. Sender: `query_pose`를 다시 보내서 로봇의 현재 TCP 포즈 수신
4. Sender의 가상 포즈가 로봇 현재 포즈로 리셋

**용도**: 트래커 트래킹 복구 후, 로봇이 안전 정지한 후, 포즈 드리프트 보정 시

---

## 5. 로봇 PC 설정

### 5.1 `--input unified` 옵션

기존 `--input keyboard`, `--input xbox`, `--input vive`와 별도로 `--input unified` 추가.

기존 입력(`keyboard`, `xbox`)은 **로컬** 입력으로 로봇 PC에서 직접 조작 시 사용.
`unified`는 **원격** 입력으로 조종 PC에서 오는 절대 포즈를 수신.

```bash
# 기존 (로컬 입력, 로봇 PC에서 직접 키보드/Xbox 조작)
python3 -m standalone.teleop_admittance.main --mode sim --input keyboard

# 통합 (원격 입력, 조종 PC에서 sender를 통한 원격 조작)
python3 -m standalone.teleop_admittance.main --mode sim --input unified
```

### 5.2 어드미턴스 / 임피던스 모드

통합 프로토콜은 두 제어 모드 모두 지원:

| 모드 | 명령 | 특징 |
|------|------|------|
| **어드미턴스** | `teleop_admittance.main --input unified` | Pink QP IK + F/T 힘제어 |
| **임피던스** | `teleop_impedance.main --input unified` | PD 토크 제어 (URScript) |

임피던스 모드 실행 시 `--mode rtde`가 필요하며, sim 모드에서는 위치제어로 fallback.

---

## 6. 모니터링

### 6.1 실시간 UDP 모니터

조종 PC 또는 로봇 PC에서 실행하여 수신 데이터를 시각화:

```bash
# 통합 프로토콜 패킷 모니터링
python3 -m vive.monitor --port 9871
```

curses 기반 터미널 대시보드로 위치, 방향, 속도, 네트워크 품질 등을 실시간 표시.
SSH 환경에서도 동작.

### 6.2 Sender 로그

Sender는 250 패킷(5초)마다 현재 가상 포즈를 출력:

```
[Sender] #250  pos=[0.102, -0.398, 0.305]
[Sender] #500  pos=[0.115, -0.390, 0.310]
```

---

## 7. 실전 시나리오

### 7.1 같은 PC에서 테스트 (개발용)

```bash
# Terminal 1: 로봇 PC (sim)
cd /workspaces/tamp_ws/src/tamp_dev
python3 -m standalone.teleop_admittance.main --mode sim --input unified

# Terminal 2: 키보드 sender (같은 PC)
cd /workspaces/tamp_ws/src/tamp_dev
python3 -m vive.keyboard_sender --target-ip 127.0.0.1
```

### 7.2 조종 PC → 로봇 PC (실전)

```
조종 PC (192.168.0.5)                로봇 PC (192.168.0.10, Docker)
┌──────────────────────┐             ┌───────────────────────────┐
│ conda activate       │             │ cd /workspaces/tamp_ws/   │
│   tamp_sender        │             │   src/tamp_dev            │
│                      │    UDP      │                           │
│ vive_sender          ├────9871────→│ teleop_admittance.main    │
│   --target-ip        │             │   --mode rtde             │
│     192.168.0.10     │             │   --input unified         │
│                      │             │   --robot-ip 192.168.0.2  │
└──────────────────────┘             └───────────────────────────┘
                                                   │
                                                   ↓ servoJ
                                              UR10e (192.168.0.2)
```

```bash
# 조종 PC
conda activate tamp_sender
cd ~/tamp_ws/src/tamp_dev
python3 -m vive.vive_sender --target-ip 192.168.0.10 --calibration vive/calibration.json

# 로봇 PC (Docker 컨테이너 내)
python3 -m standalone.teleop_admittance.main \
    --mode rtde --input unified --robot-ip 192.168.0.2
```

### 7.3 복수 입력장치 전환

같은 포트(9871)를 사용하므로 sender만 교체하면 로봇 PC 재시작 없이 입력 전환 가능:

```bash
# 처음: 키보드로 조작
python3 -m vive.keyboard_sender --target-ip 192.168.0.10
# Ctrl+C로 종료

# 전환: 조이스틱으로 조작
python3 -m vive.joystick_sender --target-ip 192.168.0.10
# Ctrl+C로 종료

# 전환: Vive Tracker로 조작
python3 -m vive.vive_sender --target-ip 192.168.0.10
```

> 로봇 PC의 `--input unified` 프로세스는 계속 실행 중. sender 교체 시 자동으로 새 sender의 패킷을 처리.
> 단, sender 전환 시 **R 키(Reset)**를 눌러 가상 포즈를 동기화하는 것을 권장.

---

## 8. 네트워크 설정

### 8.1 방화벽

```bash
# 로봇 PC에서 UDP 9871 포트 열기
sudo ufw allow 9871/udp
```

### 8.2 지연시간 확인

```bash
# 조종 PC → 로봇 PC 핑 테스트
ping <ROBOT_PC_IP>
# 1ms 이하가 이상적 (유선 연결 권장)
```

### 8.3 포트 사용 규약

| 포트 | 용도 | Sender |
|------|------|--------|
| 9871 | **통합 텔레옵** (모든 입력장치) | vive/keyboard/joystick_sender |
| 9872 | Manus 글러브 (별도 프로토콜) | manus_sender |

---

## 9. 트러블슈팅

### Sender 시작 시 "Pose query failed" 경고

```
[Sender] WARNING: Pose query failed. Using default home pose.
```

**원인**: 로봇 PC의 `--input unified` 프로세스가 아직 실행되지 않았거나 네트워크 문제.

**해결**:
1. 로봇 PC에서 먼저 teleop 프로세스 시작
2. 네트워크 연결 확인 (`ping`)
3. 방화벽 확인 (`ufw allow 9871/udp`)
4. 기본 home pose로 시작해도 동작은 가능 — R키로 Reset하면 재동기화

### 로봇이 움직이지 않음

1. Sender에 `[Sender] #250 pos=...` 로그가 출력되는지 확인 → 패킷 전송 중
2. 로봇 PC에서 `python3 -m vive.monitor --port 9871`로 수신 확인
3. `--input unified`로 실행했는지 확인 (기존 `--input vive`와 다름)
4. 로봇 PC의 safety timeout (200ms) — 패킷이 도착하지 않으면 자동 정지

### Vive Tracker 트래킹 끊김

트래킹이 끊기면 delta가 0이 되어 가상 포즈가 유지됨 (안전).
트래킹 복구 후 급격한 점프 없이 현재 위치에서 다시 시작.
심하게 끊기면 R키로 Reset 후 재시작.

### 조이스틱이 인식되지 않음

```bash
# pygame에서 조이스틱 확인
python3 -c "
import pygame; pygame.init(); pygame.joystick.init()
n = pygame.joystick.get_count()
print(f'{n} joystick(s) found')
for i in range(n):
    js = pygame.joystick.Joystick(i)
    js.init()
    print(f'  [{i}] {js.get_name()} (axes={js.get_numaxes()}, buttons={js.get_numbuttons()})')
"
```

---

## 10. 프로토콜 소스 파일

| 파일 | 위치 | 설명 |
|------|------|------|
| `standalone/core/teleop_protocol.py` | 로봇 PC | 패킷 정의 (원본) |
| `vive/teleop_protocol.py` | 조종 PC | 패킷 정의 (복사본, standalone 의존 없음) |
| `vive/teleop_sender.py` | 조종 PC | Sender ABC (UDP, 포즈 질의, 적분) |
| `vive/vive_sender.py` | 조종 PC | Vive Tracker sender |
| `vive/keyboard_sender.py` | 조종 PC | 키보드 sender |
| `vive/joystick_sender.py` | 조종 PC | 조이스틱 sender |
| `standalone/core/input_handler.py` | 로봇 PC | `UnifiedNetworkInput` 수신부 |
