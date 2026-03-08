# UR10e Teleop Servo System 사용자 가이드

## 개요

키보드 또는 Xbox 컨트롤러를 통한 UR10e 로봇 원격 조종 시스템.
무선 통신 환경에서의 Jitter/지연에 대비한 안전 시스템 내장.

**파이프라인:**
```
Input (Keyboard/Xbox) → Exponential Filter → Pink IK → Safety Monitor → Robot
```

**두 가지 백엔드:**
- **sim** — Isaac Sim 또는 ROS2 mock hardware (`/joint_states`, `/joint_command` 토픽)
- **rtde** — 실제 UR10e 로봇 (`ur_rtde` 라이브러리, servoJ 125Hz)


## 실행 방법

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# Sim 모드 (Isaac Sim 또는 mock hardware)
python3 -m standalone.teleop.main --mode sim --input keyboard

# 실제 로봇 (RTDE)
python3 -m standalone.teleop.main --mode rtde --input keyboard --robot-ip 192.168.0.2

# Xbox 컨트롤러 + CSV 로깅
python3 -m standalone.teleop.main --mode rtde --input xbox --robot-ip 192.168.0.2 --log

# 커스텀 설정 파일
python3 -m standalone.teleop.main --config path/to/config.yaml
```

### CLI 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `--mode` | config 파일 | `sim` 또는 `rtde` (백엔드 선택) |
| `--input` | config 파일 | `keyboard` 또는 `xbox` (입력 장치) |
| `--robot-ip` | `192.168.0.2` | RTDE 모드 로봇 IP |
| `--config` | `config/default.yaml` | 설정 파일 경로 |
| `--log` | off | CSV 로깅 활성화 (`teleop_log_YYYYMMDD_HHMMSS.csv`) |


## 키보드 조작

### 이동 (Cartesian)

| 키 | 동작 | 키 | 동작 |
|----|------|----|------|
| W / S | X축 +/- | U / O | Roll +/- |
| A / D | Y축 +/- | I / K | Pitch +/- |
| Q / E | Z축 +/- | J / L | Yaw +/- |

### 시스템 제어

| 키 | 동작 |
|----|------|
| +/= | 속도 증가 |
| - | 속도 감소 |
| Space | E-Stop 발동 |
| R | E-Stop 해제 |
| ESC / X | 종료 |

### 속도 스케일

5단계 속도 조절: **0.5x → 1.0x → 2.0x → 4.0x → 8.0x**

기본 스텝 크기에 배율을 곱합니다:
- 위치: `cartesian_step × speed_scale` (기본 0.01m × 1.0 = 10mm/press)
- 회전: `rotation_step × speed_scale` (기본 0.05rad × 1.0 = 2.86°/press)

키를 **누르고 있으면** 타겟이 계속 누적되어 연속 이동합니다.


## Xbox 컨트롤러 조작

| 입력 | 동작 |
|------|------|
| 왼쪽 스틱 X/Y | 선형 X/Y 속도 |
| 오른쪽 스틱 X/Y | 각속도 Roll/Pitch |
| LT / RT 트리거 | 선형 Z 속도 (RT - LT) |
| LB / RB 버튼 | 각속도 Yaw |
| A 버튼 | E-Stop 해제 |
| B 버튼 | E-Stop 발동 |
| Start 버튼 | 종료 |

데드존: 0.1 (이하 입력 무시)


## 안전 시스템 (4단계)

우선순위 높은 순서대로:

| 레벨 | 이름 | 트리거 | 동작 | 상태 표시 |
|-------|------|--------|------|-----------|
| 4 | **E-Stop** | Space/B 버튼 | 즉시 정지, 모든 명령 거부 | `ESTOP` |
| 3 | **Workspace Clamping** | EE 위치가 workspace 경계 초과 | 위치를 경계로 클램핑 (거부 아님) | `WS_CLAMP` |
| 2 | **Velocity Limiting** | 관절 속도 초과 (`max_joint_vel`) | 비례 스케일링으로 속도 제한 | `VEL_LIMIT` |
| 1 | **Packet Timeout** | 입력 없음 (`packet_timeout_ms` 초과) | `speed_stop()` 호출, 현재 위치 유지 | `TIMEOUT` |

### E-Stop 동작
1. Space키 → `backend.emergency_stop()` 호출 → 모든 명령 거부
2. R키 → E-Stop 해제 → IK/필터/타겟을 현재 로봇 위치로 재동기화
3. **수동 해제만 가능** — 자동 해제 없음

### Workspace 경계
기본 workspace (base_link 기준):
```
X: [-0.8, 0.8] m
Y: [-0.8, 0.8] m
Z: [0.05, 1.2] m
```
경계에 도달하면 명령이 거부되지 않고 클램핑되어 경계 위에서 움직입니다.


## 설정 파일 (`config/default.yaml`)

```yaml
robot:
  ip: "192.168.0.2"
  mode: "sim"                 # "sim" (ROS2 토픽) | "rtde" (실제 로봇)

control:
  frequency_sim: 50           # Hz (sim 모드)
  frequency_rtde: 125         # Hz (rtde 모드, servoJ 주기)

input:
  type: "keyboard"            # "keyboard" | "xbox"
  cartesian_step: 0.01        # m/press (1x 속도 기준)
  rotation_step: 0.05         # rad/press (1x 속도 기준)
  xbox_linear_scale: 0.03     # Xbox 스틱 → 선형 속도 스케일
  xbox_angular_scale: 0.08    # Xbox 스틱 → 각속도 스케일

filter:
  alpha_position: 0.85        # EMA alpha (0~1, 높을수록 반응 빠름)
  alpha_orientation: 0.85     # slerp alpha

ik:
  position_cost: 1.0          # Pink FrameTask 위치 비용
  orientation_cost: 0.5       # Pink FrameTask 자세 비용
  posture_cost: 1.0e-3        # PostureTask 비용 (관절 중심화)
  damping: 1.0e-12            # QP 솔버 댐핑

safety:
  packet_timeout_ms: 200      # 입력 없으면 speed_stop (ms)
  max_joint_vel: 0.5          # rad/s (보수적 설정)
  max_ee_velocity: 0.1        # m/s
  workspace:
    x: [-0.8, 0.8]            # base_link 기준 (m)
    y: [-0.8, 0.8]
    z: [0.05, 1.2]
```

### 주요 파라미터 튜닝 가이드

| 파라미터 | 효과 | 권장 |
|----------|------|------|
| `cartesian_step` | 키 1회당 이동량 (m) | 0.005~0.02 |
| `alpha_position` | 필터 반응성 (높을수록 빠름) | 0.7~0.95 |
| `max_joint_vel` | 관절 속도 상한 (rad/s) | 테스트: 0.5, 숙련: 1.0 |
| `packet_timeout_ms` | 입력 없음 허용 시간 | 무선: 500~1000, 로컬: 200 |
| `damping` | IK 안정성 (특이점 근처) | 1e-12 ~ 1e-6 |


## 제어 루프 상세

매 사이클 (sim: 20ms, rtde: 8ms):

```
1. 입력 읽기 (timeout 1ms)
2. 타겟 누적 (target += velocity_delta)
3. Exponential 필터 적용 (EMA + slerp)
4. Workspace 클램핑 (Level 3)
5. Pink IK 풀기 (QP solver, proxqp)
6. Safety 검사 (Level 1, 2, 4)
7. EE 속도 계산 (디스플레이용)
8. 관절 명령 전송 (안전 시 only)
9. 터미널 상태 표시 + CSV 로깅
10. 타이밍 보정 (sleep)
```


## 모듈 구조

```
standalone/teleop/
├── __init__.py           # 패키지 초기화
├── main.py               # TeleopController — 메인 제어 루프
├── teleop_config.py      # TeleopConfig — YAML 설정 로더
├── input_handler.py      # KeyboardInput / XboxInput — 입력 추상화
├── exp_filter.py         # ExpFilter — EMA 위치 + slerp 자세 필터
├── pink_ik.py            # PinkIK — QP 기반 task-level IK
├── safety_monitor.py     # SafetyMonitor — 4단계 안전 시스템
├── config/
│   └── default.yaml      # 기본 설정
└── docs/
    └── user_guide.md     # 이 문서
```

### 외부 의존 모듈

| 모듈 | 위치 | 용도 |
|------|------|------|
| `standalone.config` | `standalone/config.py` | URDF_PATH, JOINT_NAMES, 상수 |
| `standalone.robot_backend` | `standalone/robot_backend.py` | RobotBackend ABC + `create_backend()` |
| `standalone.sim_robot` | `standalone/sim_robot.py` | SimBackend (ROS2 토픽) |
| `standalone.ur_robot` | `standalone/ur_robot.py` | RTDEBackend (ur_rtde servoJ) |
| `standalone.servo.controller_utils` | `standalone/servo/` | ControllerSwitcher (mock hw 전용) |


## 의존성 설치

```bash
# Pink IK (주의: pip install pink은 코드 포매터 — 잘못된 패키지!)
pip install pin-pink proxsuite

# numpy 1.x 필수 (ROS Humble pinocchio 호환)
pip install "numpy<2"

# Xbox 컨트롤러 사용 시
pip install pygame
```


## 터미널 상태 표시

실행 중 7줄 고정 상태 블록이 0.1초 간격으로 갱신됩니다:

```
  EE Pos : x= 0.7295  y=-0.5837  z= 0.5130 m
  EE RPY : R= -90.0  P=  -0.0  Y= -49.9 deg
  Joints : [ -49.9  -65.4   74.3    0.0   86.1    0.0] deg
  Vel    : 0.0450 m/s  |  Speed: 2.0x
  Safety : VEL_LIMIT  vel scaled 0.60x
  E-Stop : off (Space: trigger)
  Input  : 15ms ago  |  sim 50Hz
```


## CSV 로그 형식

`--log` 플래그 사용 시 `teleop_log_YYYYMMDD_HHMMSS.csv` 파일 생성:

```csv
timestamp,ee_x,ee_y,ee_z,ee_roll,ee_pitch,ee_yaw,j1,j2,j3,j4,j5,j6,ee_vel,safety_status
1772934822.286684,0.729509,-0.583755,0.512759,-1.570796,-0.000012,-0.871000,...,0.045000,VEL_LIMIT
```


## 트러블슈팅

### 로봇이 안 움직임
- `damping`, `posture_cost` 등 과학적 표기법(`1e-12`)이 YAML에서 문자열로 파싱될 수 있음
  → `1.0e-12` (소수점 포함) 형식 사용
- IK 솔버 에러 확인: `solve()` 반환값이 `None`이면 IK 실패

### controller_manager 에러 (sim 모드)
- Isaac Sim 사용 시 정상 — `controller_manager`가 없으므로 자동 스킵됨
- mock hardware 사용 시: `ros2 launch ur_robot_driver ur10e.launch.py use_fake_hardware:=true` 먼저 실행

### TIMEOUT 상태 지속
- 키보드 입력이 터미널 포커스를 잃으면 발생
- `packet_timeout_ms` 값을 늘려서 완화 가능 (무선 환경: 500~1000ms)

### numpy 호환 에러
- `_ARRAY_API not found` → `pip install "numpy<2"` 실행
- ROS Humble의 pinocchio는 numpy 1.x로 컴파일됨
