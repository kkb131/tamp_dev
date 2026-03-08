# standalone/ 패키지 구조

UR10e 로봇 제어를 위한 MoveIt 독립 Python 패키지.
모든 모듈은 `python3 -m standalone.<module>` 형태로 실행 가능.

---

## 디렉토리 구조

```
standalone/
├── config.py                          # 공유 설정
├── core/                              # 공유 인프라
│   ├── robot_backend.py
│   ├── ur_robot.py
│   ├── sim_robot.py
│   ├── trajectory_executor.py
│   ├── controller_utils.py
│   └── kinematics.py
├── cumotion/                          # 기능: GPU 모션 플래닝
│   ├── planner.py
│   ├── test_standalone.py
│   ├── test_multi_goal.py
│   └── docs/user_guide.md
├── servo/                             # 기능: 간단 teleop 스크립트
│   ├── keyboard_cartesian.py
│   ├── keyboard_forward.py
│   ├── keyboard_servo_admittance.py
│   ├── joystick_cartesian.py
│   └── docs/user_guide.md
└── teleop/                            # 기능: 파이프라인 teleop
    ├── main.py
    ├── input_handler.py
    ├── pink_ik.py
    ├── exp_filter.py
    ├── safety_monitor.py
    ├── teleop_config.py
    ├── config/default.yaml
    └── docs/user_guide.md
```

---

## 계층 구조

```
┌─────────────────────────────────────────────────────┐
│                    config.py                        │  공유 상수
├─────────────────────────────────────────────────────┤
│                     core/                           │  공유 인프라
│  robot_backend · ur_robot · sim_robot               │
│  trajectory_executor · controller_utils · kinematics│
├──────────┬──────────┬───────────┬───────────────────┤
│ cumotion │  servo   │  teleop   │  (향후 기능...)    │  기능 모듈
│ GPU 플래닝│ DLS 스크립트│ Pink IK   │ impedance 등     │
└──────────┴──────────┴───────────┴───────────────────┘
```

**의존 규칙**: 기능 모듈은 `config.py`와 `core/`만 import한다. 기능 모듈 간 교차 import 금지.

---

## config.py — 공유 설정

모든 모듈이 사용하는 상수를 한곳에서 관리.

| 카테고리 | 상수 예시 |
|---------|---------|
| 경로 | `XRDF_PATH`, `URDF_PATH` |
| 로봇 | `JOINT_NAMES`, `DEFAULT_ROBOT_IP`, `RTDE_FREQUENCY` |
| 안전 | `MAX_JOINT_VEL_RAD_S`, `MAX_JOINT_ACCEL_RAD_S2` |
| servoJ | `SERVOJ_DT`, `SERVOJ_LOOKAHEAD`, `SERVOJ_GAIN` |
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
- `execute_trajectory()`: 리샘플링 후 실시간 스트리밍 실행

### controller_utils.py — ControllerSwitcher
- ROS2 `controller_manager` 서비스를 통한 컨트롤러 전환
- `activate_forward_position()`: joint_trajectory → forward_position 전환
- `restore_original()`: 원래 컨트롤러 복구

### kinematics.py — PinocchioIK
- Pinocchio 라이브러리 기반 FK/Jacobian/DLS IK
- `get_ee_pose(q)`: 순운동학 (position + rotation)
- `compute_joint_delta(q, twist, dt)`: Damped Least Squares 미분 역운동학
- `clamp_positions(q)`: 관절 한계 클램핑

---

## cumotion/ — GPU 모션 플래닝

NVIDIA cuRobo 기반 GPU 가속 모션 플래닝. MoveIt 없이 독립 실행.

### planner.py — StandaloneMotionPlanner
- cuRobo `MotionGen` 래퍼
- `plan_joint(start, goal)`: 관절 공간 플래닝
- `plan_cartesian(start, position, quaternion)`: 카르테시안 플래닝
- `get_ee_pose(joint_positions)`: FK로 EE 포즈 계산
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
| `keyboard_cartesian.py` | 키보드 | Cartesian (DLS IK) |
| `keyboard_forward.py` | 키보드 | Joint-space 직접 제어 |
| `keyboard_servo_admittance.py` | 키보드 + F/T | Cartesian + 어드미턴스 |
| `joystick_cartesian.py` | Xbox 컨트롤러 | Cartesian (DLS IK) |

```bash
python3 -m standalone.servo.keyboard_cartesian --mode sim
python3 -m standalone.servo.keyboard_forward --mode rtde --robot-ip 192.168.0.2
```

---

## teleop/ — 파이프라인 Teleop

Pink IK (QP 기반) 솔버를 사용하는 통합 teleop 파이프라인.

### 파이프라인 흐름
```
Input(keyboard/xbox) → ExpFilter → Pink IK → SafetyMonitor → Robot
```

| 파일 | 역할 |
|------|------|
| `main.py` | 엔트리포인트, 메인 루프 |
| `input_handler.py` | 키보드/Xbox 입력 추상화 |
| `exp_filter.py` | 지수 필터 (노이즈 제거) |
| `pink_ik.py` | Pink IK 솔버 (QP 기반, proxqp) |
| `safety_monitor.py` | 4단계 안전 검사 (관절/EE 속도, 워크스페이스, 패킷 타임아웃) |
| `teleop_config.py` | YAML 설정 로더 (dataclass 기반) |
| `config/default.yaml` | 기본 설정 파일 |

```bash
python3 -m standalone.teleop.main --mode sim --input keyboard
python3 -m standalone.teleop.main --mode rtde --input xbox --robot-ip 192.168.0.2
```

---

## 새 기능 추가 가이드

### 새 기능 모듈 만들기

1. `standalone/` 아래에 새 폴더 생성 (예: `standalone/impedance/`)
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

예시: `teleop/exp_filter.py`가 새 `impedance/` 모듈에서도 필요하다면:
1. `teleop/exp_filter.py` → `core/exp_filter.py`로 이동
2. `core/__init__.py`에 re-export 추가
3. 기존 import 경로 업데이트

### config.py 확장

새 기능에 필요한 상수는 `config.py`에 추가. 파일이 ~150줄을 넘으면
`config/` 패키지로 분리 고려 (`__init__.py`에서 re-export하여 하위 호환 유지).

---

## 의존성

| 패키지 | 출처 | 용도 |
|--------|------|------|
| numpy (<2.0) | pip | 수치 연산 (pinocchio ABI 호환 필수) |
| pinocchio | apt (`ros-humble-pinocchio`) | FK/Jacobian (core/kinematics.py) |
| pin-pink | pip | QP 기반 IK (teleop/pink_ik.py) |
| proxsuite | pip | QP solver backend |
| curobo | apt (`ros-humble-curobo-core`) | GPU 모션 플래닝 (cumotion/planner.py) |
| ur-rtde | pip | 실제 로봇 통신 (core/ur_robot.py) |
| pygame | pip (optional) | Xbox 컨트롤러 입력 |
| PyYAML | pip | YAML 설정 파싱 |
| rclpy | ROS2 | SimBackend, ControllerSwitcher |
