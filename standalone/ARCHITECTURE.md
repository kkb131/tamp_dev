# standalone/ 패키지 구조

UR10e 로봇 제어를 위한 MoveIt 독립 Python 패키지.
모든 모듈은 `python3 -m standalone.<module>` 형태로 실행 가능.

---

## 디렉토리 구조

```
standalone/
├── config.py                          # 공유 설정 (경로, 관절, 안전 한계, 포즈)
├── Manual.md                          # 학습 매뉴얼 (왜/무엇/어떻게)
├── ARCHITECTURE.md                    # 이 파일
├── core/                              # 공유 인프라 (2개+ 기능 모듈이 사용)
│   ├── robot_backend.py               # RobotBackend ABC + create_backend() 팩토리
│   ├── ur_robot.py                    # RTDEBackend (실제 로봇, ur_rtde)
│   ├── sim_robot.py                   # SimBackend (Isaac Sim / mock hw, ROS2 토픽)
│   ├── trajectory_executor.py         # 궤적 리샘플링 + 실시간 스트리밍
│   ├── controller_utils.py            # ControllerSwitcher (ros2_control 전환)
│   ├── kinematics.py                  # PinocchioIK (FK/Jacobian/DLS IK)
│   ├── pink_ik.py                     # PinkIK (QP 기반 IK, proxqp)
│   ├── input_handler.py               # Keyboard/Xbox 입력 추상화
│   ├── exp_filter.py                  # 지수 필터 (EMA + slerp)
│   ├── ft_source.py                   # F/T 센서 추상화 (RTDE / Null)
│   └── compliant_control.py           # 어드미턴스 제어 (M-D-K 동역학)
├── cumotion/                          # 기능: GPU 모션 플래닝
│   ├── planner.py                     # StandaloneMotionPlanner (curobo)
│   ├── test_standalone.py             # 단일 목표 플래닝 테스트
│   ├── test_multi_goal.py             # 다중 목표 순차 테스트
│   └── docs/user_guide.md
├── servo/                             # 기능: 간단 teleop 스크립트 (Pinocchio DLS)
│   ├── keyboard_forward.py            # Joint-space 키보드 제어
│   ├── keyboard_cartesian.py          # Cartesian 키보드 제어
│   ├── keyboard_servo_admittance.py   # Cartesian + F/T 어드미턴스
│   ├── joystick_cartesian.py          # Xbox Cartesian 제어
│   └── docs/user_guide.md
├── teleop_admittance/                 # 기능: 어드미턴스 텔레옵 (Pink IK + F/T)
│   ├── main.py                        # 엔트리포인트, 메인 제어 루프
│   ├── admittance_layer.py            # 어드미턴스 제어 레이어
│   ├── safety_monitor.py              # 4단계 안전 시스템
│   ├── teleop_config.py               # YAML 설정 로더 (dataclass)
│   ├── config/default.yaml            # 기본 설정
│   └── docs/user_guide.md
└── teleop_impedance/                  # 기능: 임피던스 텔레옵 (URScript PD 토크)
    ├── main.py                        # 엔트리포인트, 듀얼 루프
    ├── urscript_manager.py            # URScript 업로드 + RTDE 레지스터 I/O
    ├── impedance_gains.py             # PD 게인 프리셋 (STIFF/MEDIUM/SOFT)
    ├── torque_safety.py               # 토크 모드 안전 검사
    ├── impedance_config.py            # YAML 설정 로더 (dataclass)
    ├── scripts/impedance_pd.script    # URScript PD 루프 (500Hz)
    ├── config/default.yaml            # 기본 설정
    └── docs/user_guide.md
```

---

## 계층 구조

```
┌──────────────────────────────────────────────────────────────────┐
│                         config.py                                │  공유 상수
├──────────────────────────────────────────────────────────────────┤
│                          core/                                   │  공유 인프라
│  robot_backend · ur_robot · sim_robot · trajectory_executor      │
│  kinematics · pink_ik · controller_utils                         │
│  input_handler · exp_filter · ft_source · compliant_control      │
├───────────┬───────────┬──────────────────┬───────────────────────┤
│  servo/   │ cumotion/ │teleop_admittance/│ teleop_impedance/     │  기능 모듈
│ DLS 텔레옵│ GPU 플래닝 │ F/T 어드미턴스    │ URScript PD 토크      │
└───────────┴───────────┴──────────────────┴───────────────────────┘
```

**의존 규칙**: 기능 모듈은 `config.py`와 `core/`만 import한다. 기능 모듈 간 교차 import 금지.

---

## config.py — 공유 설정

모든 모듈이 사용하는 상수를 한곳에서 관리.

| 카테고리 | 상수 예시 |
|---------|---------|
| 경로 | `XRDF_PATH`, `URDF_PATH` |
| 로봇 | `JOINT_NAMES`, `DEFAULT_ROBOT_IP`, `RTDE_FREQUENCY` |
| 안전 | `MAX_JOINT_VEL_RAD_S`, `MAX_JOINT_ACCEL_RAD_S2`, `UR10E_MAX_TORQUES` |
| servoJ | `SERVOJ_DT`, `SERVOJ_LOOKAHEAD`, `SERVOJ_GAIN` |
| 토크 제어 | `DIRECT_TORQUE_FREQUENCY` (500Hz), `UR_SECONDARY_PORT` |
| cuMotion | `INTERPOLATION_DT`, `NUM_GRAPH_SEEDS`, `TRAJOPT_TSTEPS` |
| 제어 | `SERVO_RATE_HZ`, 컨트롤러 이름 (`TRAJECTORY_CONTROLLER` 등) |
| 포즈 | `HOME_JOINTS`, `UP_JOINTS`, `NEAR_HOME_WAYPOINTS` |

---

## core/ — 공유 인프라

2개 이상의 기능 모듈이 사용하는 코드. 기능 모듈의 기반.

### robot_backend.py
- `RobotBackend` ABC: 모든 로봇 통신의 추상 인터페이스
  - `connect()`, `disconnect()`, `get_joint_positions()`, `send_joint_command()` 등
- `create_backend(mode)`: 팩토리 함수 (`"rtde"` → RTDEBackend, `"sim"` → SimBackend)
- Context manager 지원: `with create_backend("sim") as robot:`

### ur_robot.py — RTDEBackend
- 실제 UR10e 로봇과 ur_rtde 라이브러리로 통신
- servoJ 명령으로 125Hz 실시간 제어
- F/T 센서 읽기: `get_tcp_force()`

### sim_robot.py — SimBackend
- Mock hardware / Isaac Sim과 ROS2 토픽으로 통신
- `/joint_states` 구독, `/joint_command` 발행
- 내부 ROS2 노드를 백그라운드 스레드에서 spin

### trajectory_executor.py
- `resample_trajectory()`: cuMotion 궤적을 servoJ 주기(125Hz)로 리샘플링
- `validate_trajectory()`: 속도/가속도 안전 한계 검증
- `execute_trajectory()`: 리샘플링 후 실시간 스트리밍 실행 (busy-wait)

### controller_utils.py — ControllerSwitcher
- ROS2 `controller_manager` 서비스를 통한 컨트롤러 전환
- `activate_forward_position()`: joint_trajectory → forward_position 전환
- `restore_original()`: 원래 컨트롤러 복구

### kinematics.py — PinocchioIK
- Pinocchio 라이브러리 기반 FK/Jacobian/DLS IK
- `get_ee_pose(q)`: 순운동학 (position + rotation)
- `compute_joint_delta(q, twist, dt)`: Damped Least Squares 미분 역운동학
- `clamp_positions(q)`: 관절 한계 클램핑

### pink_ik.py — PinkIK
- Pink + Pinocchio 기반 QP IK 솔버
- `FrameTask` (EE 추적) + `PostureTask` (관절 중심화)
- 관절 한계를 QP 제약 조건으로 보장 (DLS보다 안전)
- `solve(target_pos, target_quat, dt)` → 관절 위치 또는 None

### input_handler.py
- `TeleopCommand` 데이터클래스: 6-DOF velocity, E-Stop, 속도 배율, 어드미턴스/임피던스 명령
- `KeyboardInput`: termios 기반 논블로킹 키보드 (WASD/QEUOIKJL)
- `XboxInput`: pygame 기반 Xbox 컨트롤러 (아날로그 스틱/트리거)
- `create_input()`: 팩토리 함수

### exp_filter.py — ExpFilter
- 위치: 선형 EMA (Exponential Moving Average)
- 자세: 쿼터니언 slerp (Spherical Linear Interpolation)
- 텔레옵 입력의 노이즈/불연속성을 부드럽게 처리

### ft_source.py
- `FTSource` Protocol: F/T 센서 인터페이스 (`get_wrench()`, `zero_sensor()`)
- `RTDEFTSource`: 실제 UR10e F/T (바이어스 보정 포함)
- `NullFTSource`: 시뮬레이션용 (항상 영벡터)

### compliant_control.py
- `ComplianceParams`: M(질량)/D(감쇠)/K(강성) 6-DOF 파라미터
- `COMPLIANCE_PRESETS`: STIFF, MEDIUM, SOFT 프리셋
- `AdmittanceController`: M·ẍ + D·ẋ + K·x = F_ext (오일러 적분, 변위 클램핑)

---

## cumotion/ — GPU 모션 플래닝

NVIDIA cuRobo 기반 GPU 가속 모션 플래닝. MoveIt 없이 독립 실행.

### planner.py — StandaloneMotionPlanner
- cuRobo `MotionGen` 래퍼
- `plan_joint(start, goal)`: 관절 공간 플래닝
- `plan_cartesian(start, position, quaternion)`: 카르테시안 플래닝
- `get_ee_pose(joint_positions)`: FK로 EE 포즈 계산
- `add_cuboid()`, `clear_world()`: 장애물 관리
- **GPU 필수** (`nvidia-smi` 정상 출력 필요)

### test_standalone.py / test_multi_goal.py
- 단일/다중 목표 플래닝 테스트 스크립트
- `--plan-only`: GPU만 있으면 실행 가능 (로봇 연결 불필요)
- `--execute`: 플래닝 후 로봇으로 실행

```bash
python3 -m standalone.cumotion.test_standalone --plan-only
python3 -m standalone.cumotion.test_multi_goal --plan-only --rounds 3
```

---

## servo/ — 간단 Teleop 스크립트

Pinocchio DLS IK 기반의 독립 실행 스크립트. 각 파일이 하나의 완전한 제어 루프.

| 스크립트 | 입력 | 제어 방식 |
|---------|------|----------|
| `keyboard_forward.py` | 키보드 | Joint-space 직접 제어 |
| `keyboard_cartesian.py` | 키보드 | Cartesian (DLS IK) |
| `joystick_cartesian.py` | Xbox 컨트롤러 | Cartesian (DLS IK) |
| `keyboard_servo_admittance.py` | 키보드 + F/T | Cartesian + 어드미턴스 |

```bash
python3 -m standalone.servo.keyboard_cartesian --mode sim
python3 -m standalone.servo.keyboard_forward --mode rtde --robot-ip 192.168.0.2
```

---

## teleop_admittance/ — 파이프라인 텔레옵 (어드미턴스)

Pink IK (QP 기반) 솔버를 사용하는 통합 teleop 파이프라인. F/T 센서 기반 어드미턴스 제어.

### 파이프라인 흐름
```
Input(keyboard/xbox) → ExpFilter → Workspace Clamp → Admittance(F/T) → Pink IK → SafetyMonitor → Robot
```

| 파일 | 역할 |
|------|------|
| `main.py` | 엔트리포인트, 메인 루프 |
| `admittance_layer.py` | 어드미턴스 제어 레이어 (F/T→변위) |
| `safety_monitor.py` | 4단계 안전 검사 (E-Stop, 워크스페이스, 속도, 타임아웃) |
| `teleop_config.py` | YAML 설정 로더 (dataclass 기반) |
| `config/default.yaml` | 기본 설정 파일 |

```bash
python3 -m standalone.teleop_admittance.main --mode sim --input keyboard
python3 -m standalone.teleop_admittance.main --mode rtde --input xbox --robot-ip 192.168.0.2
```

---

## teleop_impedance/ — 임피던스 텔레옵 (URScript PD 토크)

URScript PD 토크 제어 기반 임피던스 텔레옵. Python 125Hz + URScript 500Hz 듀얼 루프.

### 듀얼 루프 구조
```
Python (125Hz): Input → Filter → IK → q_desired → RTDE registers
URScript (500Hz): q_desired + Kp/Kd → τ = Kp*(q_d-q) - Kd*q̇ + C(q,q̇) → direct_torque()
```

| 파일 | 역할 |
|------|------|
| `main.py` | 엔트리포인트, 듀얼 루프 디스패치 |
| `urscript_manager.py` | URScript 업로드 + RTDE 레지스터 I/O |
| `impedance_gains.py` | PD 게인 프리셋 (STIFF/MEDIUM/SOFT) + 런타임 스케일링 |
| `torque_safety.py` | 토크 모드 안전 검사 (위치 편차, 속도, 타임아웃) |
| `impedance_config.py` | YAML 설정 로더 (dataclass 기반) |
| `scripts/impedance_pd.script` | URScript PD 토크 루프 (500Hz) |
| `config/default.yaml` | 기본 설정 파일 |

```bash
python3 -m standalone.teleop_impedance.main --mode sim --input keyboard
python3 -m standalone.teleop_impedance.main --mode rtde --input keyboard --robot-ip 192.168.0.2
```

**요구사항**: PolyScope 5.23.0+ (`direct_torque()`, `get_coriolis_and_centrifugal_torques()`)

---

## 새 기능 추가 가이드

### 새 기능 모듈 만들기

1. `standalone/` 아래에 새 폴더 생성 (예: `standalone/collision/`)
2. `__init__.py` 생성
3. `core/`에서 필요한 인프라 import:
   ```python
   from standalone.core.robot_backend import RobotBackend, create_backend
   from standalone.core.kinematics import PinocchioIK
   ```
4. `config.py`에서 필요한 상수 import:
   ```python
   from standalone.config import JOINT_NAMES, SERVO_RATE_HZ
   ```

### 기존 유틸리티를 core/로 승격하기

어떤 유틸리티가 2개 이상 기능 모듈에서 사용되면 `core/`로 이동.

예시: 새 모듈에서 `exp_filter`가 필요하다면 이미 `core/exp_filter.py`에 있으므로 바로 사용.

### config.py 확장

새 기능에 필요한 상수는 `config.py`에 추가. 파일이 ~150줄을 넘으면
`config/` 패키지로 분리 고려 (`__init__.py`에서 re-export하여 하위 호환 유지).

---

## 의존성

| 패키지 | 출처 | 용도 |
|--------|------|------|
| numpy (<2.0) | pip | 수치 연산 (pinocchio ABI 호환 필수) |
| pinocchio | apt (`ros-humble-pinocchio`) | FK/Jacobian (core/kinematics.py, core/pink_ik.py) |
| pin-pink | pip | QP 기반 IK (core/pink_ik.py) |
| proxsuite | pip | QP solver backend |
| curobo | apt (`ros-humble-curobo-core`) | GPU 모션 플래닝 (cumotion/planner.py) |
| ur-rtde | pip | 실제 로봇 통신 (core/ur_robot.py, teleop_impedance/) |
| pygame | pip (optional) | Xbox 컨트롤러 입력 |
| PyYAML | pip | YAML 설정 파싱 |
| rclpy | ROS2 | SimBackend, ControllerSwitcher |
