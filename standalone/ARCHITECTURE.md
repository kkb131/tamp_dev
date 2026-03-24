# standalone/ 패키지 구조

UR10e 로봇의 GPU 모션 플래닝을 위한 MoveIt 독립 Python 패키지.
모든 모듈은 `python3 -m standalone.<module>` 형태로 실행 가능.

> 원격조종(teleop) 코드는 `bak/`에 백업되어 있다. 필요 시 복원 가능.

---

## 디렉토리 구조

```
standalone/
├── config.py                          # 공유 설정 (경로, 관절, 안전 한계, 포즈)
├── Manual.md                          # 학습 매뉴얼 (왜/무엇/어떻게)
├── ARCHITECTURE.md                    # 이 파일
├── core/                              # 공유 인프라 (로봇 통신 + 궤적 실행)
│   ├── robot_backend.py               # RobotBackend ABC + create_backend() 팩토리
│   ├── ur_robot.py                    # RTDEBackend (실제 로봇, ur_rtde)
│   ├── sim_robot.py                   # SimBackend (Isaac Sim / mock hw, ROS2 토픽)
│   └── trajectory_executor.py         # 궤적 리샘플링 + 실시간 스트리밍
└── cumotion/                          # 기능: GPU 모션 플래닝
    ├── planner.py                     # StandaloneMotionPlanner (curobo)
    ├── test_standalone.py             # 단일 목표 플래닝 테스트
    ├── test_multi_goal.py             # 다중 목표 순차 테스트
    ├── DEPENDENCIES.md                # 의존성 분석 문서
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
├──────────────────────────────────────────────────────────────────┤
│                        cumotion/                                 │  기능 모듈
│                      GPU 모션 플래닝                               │
└──────────────────────────────────────────────────────────────────┘
```

**의존 규칙**: 기능 모듈은 `config.py`와 `core/`만 import한다.

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
| 포즈 | `HOME_JOINTS`, `UP_JOINTS`, `NEAR_HOME_WAYPOINTS` |

---

## core/ — 공유 인프라

로봇 통신 및 궤적 실행을 담당하는 공통 코드.

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

---

## cumotion/ — GPU 모션 플래닝

NVIDIA cuRobo 기반 GPU 가속 모션 플래닝. MoveIt 없이 독립 실행.
→ 의존성 상세: [cumotion/DEPENDENCIES.md](cumotion/DEPENDENCIES.md)

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

## 새 기능 추가 가이드

### 새 기능 모듈 만들기

1. `standalone/` 아래에 새 폴더 생성 (예: `standalone/grasp/`)
2. `__init__.py` 생성
3. `core/`에서 필요한 인프라 import:
   ```python
   from standalone.core.robot_backend import RobotBackend, create_backend
   from standalone.core.trajectory_executor import execute_trajectory
   ```
4. `config.py`에서 필요한 상수 import:
   ```python
   from standalone.config import JOINT_NAMES, XRDF_PATH
   ```

### config.py 확장

새 기능에 필요한 상수는 `config.py`에 추가. 파일이 ~150줄을 넘으면
`config/` 패키지로 분리 고려 (`__init__.py`에서 re-export하여 하위 호환 유지).

---

## 의존성

| 패키지 | 출처 | 용도 |
|--------|------|------|
| numpy (<2.0) | pip | 수치 연산 (curobo/torch ABI 호환 필수) |
| curobo | apt (`ros-humble-curobo-core`) | GPU 모션 플래닝 (cumotion/planner.py) |
| ur-rtde | pip | 실제 로봇 통신 (core/ur_robot.py) |
| rclpy | ROS2 | SimBackend (core/sim_robot.py) |
| torch | apt (curobo-core pip-shim) | GPU 텐서 연산 |
