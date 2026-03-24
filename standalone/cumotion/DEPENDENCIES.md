# cumotion/ 의존성 분석

`standalone/cumotion/`은 GPU 모션 플래닝의 **최상위 모듈**이다.
NVIDIA cuRobo `MotionGen`을 래핑하며, ROS2 없이 GPU만 있으면 독립 실행 가능하다.
**teleop 모듈(servo/, teleop_admittance/, teleop_impedance/)과 교차 import 없음** — 완전 분리 상태.

---

## 의존성 계층도

```
Layer 0 (계획만):   planner.py ──→ config.py + curobo + torch + numpy
Layer 1 (+ 실행):   test_*.py  ──→ Layer 0 + core/robot_backend + core/trajectory_executor
Layer 2a (+ 실물):  Layer 1    ──→ core/ur_robot   (ur-rtde)
Layer 2b (+ 시뮬):  Layer 1    ──→ core/sim_robot   (rclpy, sensor_msgs)
```

- **Layer 0**만 있으면 `--plan-only` 모드 실행 가능 (로봇/시뮬 연결 불필요)
- **Layer 1**은 lazy import로 `--execute` 플래그 사용 시에만 로드됨
- **Layer 2**는 `--mode rtde|sim` 선택에 따라 하나만 로드됨

---

## 파일별 import 상세

### planner.py — 핵심 플래너 (ROS2 무관)

| Import | 타입 | 용도 |
|--------|------|------|
| `standalone.config` | local | `INTERPOLATION_DT`, `JOINT_NAMES`, `MAX_ATTEMPTS`, `NUM_GRAPH_SEEDS`, `NUM_TRAJOPT_SEEDS`, `TRAJOPT_TSTEPS` |
| `numpy` | third-party | 궤적 배열 변환 |
| `torch` | third-party | GPU 텐서 연산 (CUDA 필수) |
| `curobo.wrap.reacher.motion_gen` | third-party | `MotionGen`, `MotionGenConfig`, `MotionGenPlanConfig` |
| `curobo.types.math` | third-party | `Pose` |
| `curobo.types.state` | third-party | `JointState` (as `CuJointState`) |
| `curobo.geom.types` | third-party | `Cuboid`, `WorldConfig` |
| `curobo.geom.sdf.world` | third-party | `CollisionCheckerType` |
| `curobo.types.base` | third-party | `TensorDeviceType` |
| `curobo.types.file_path` | third-party | `ContentPath` |
| `curobo.cuda_robot_model.util` | third-party | `load_robot_yaml` |

### test_standalone.py — 단일 목표 테스트

| Import | 타입 | 로드 시점 | 용도 |
|--------|------|----------|------|
| `standalone.config` | local | top-level | `DEFAULT_MODE`, `DEFAULT_ROBOT_IP`, `DEFAULT_VELOCITY_SCALE`, `HOME_JOINTS`, `SERVOJ_DT`, `UP_JOINTS`, `XRDF_PATH`, `URDF_PATH` |
| `standalone.cumotion.planner` | local | top-level | `StandaloneMotionPlanner` |
| `numpy` | third-party | top-level | 목표 관절값 배열 |
| `standalone.core.robot_backend` | local | **lazy** (`run_with_backend()`) | `create_backend` |
| `standalone.core.trajectory_executor` | local | **lazy** (`run_with_backend()`) | `check_start_match`, `execute_trajectory`, `validate_trajectory` |

### test_multi_goal.py — 다중 목표 순차 테스트

| Import | 타입 | 로드 시점 | 용도 |
|--------|------|----------|------|
| `standalone.config` | local | top-level | `DEFAULT_MODE`, `DEFAULT_ROBOT_IP`, `DEFAULT_VELOCITY_SCALE`, `HOME_JOINTS`, `NEAR_HOME_WAYPOINTS`, `SERVOJ_DT`, `XRDF_PATH`, `URDF_PATH` |
| `standalone.cumotion.planner` | local | top-level | `StandaloneMotionPlanner` |
| `numpy` | third-party | top-level | |
| `standalone.core.robot_backend` | local | **lazy** (`run_with_backend()`) | `create_backend` |
| `standalone.core.trajectory_executor` | local | **lazy** (`run_with_backend()`) | `check_start_match`, `execute_trajectory`, `validate_trajectory` |

---

## 전이 의존성 (core/ 모듈)

cumotion이 사용하는 core/ 모듈과 그 의존:

| 모듈 | local 의존 | third-party | ROS2 필요? |
|------|-----------|-------------|-----------|
| `core/robot_backend.py` | ABC만 정의. lazy import: `ur_robot`, `sim_robot` | — | No |
| `core/trajectory_executor.py` | `config` (`MAX_JOINT_VEL_RAD_S`, `MAX_JOINT_ACCEL_RAD_S2`, `SERVOJ_DT`), `robot_backend` (타입 힌트) | `numpy` | No |
| `core/ur_robot.py` | `config` (`RTDE_FREQUENCY`, `SERVOJ_DT`, `SERVOJ_GAIN`, `SERVOJ_LOOKAHEAD`), `robot_backend` | `rtde_control`, `rtde_receive` (try/except) | No |
| `core/sim_robot.py` | `config` (`JOINT_NAMES`), `robot_backend` | `rclpy`, `sensor_msgs` | **Yes** |

---

## config.py 사용 상수 분류

### 계획 전용 (planner.py만 사용)

| 상수 | 값 | 용도 |
|------|---|------|
| `INTERPOLATION_DT` | 0.025 | 궤적 보간 시간 간격 (40Hz) |
| `JOINT_NAMES` | 6개 UR 관절명 | cuRobo 관절 매핑 |
| `MAX_ATTEMPTS` | 10 | 플래닝 재시도 횟수 |
| `NUM_GRAPH_SEEDS` | 6 | RRT-Connect 시드 수 |
| `NUM_TRAJOPT_SEEDS` | 6 | 궤적 최적화 시드 수 |
| `TRAJOPT_TSTEPS` | 32 | 궤적 최적화 타임스텝 |

### 실행/테스트 (test_*.py에서 사용, 다른 모듈과 공유)

| 상수 | 용도 |
|------|------|
| `DEFAULT_MODE` | 기본 실행 모드 ("sim") |
| `DEFAULT_ROBOT_IP` | 기본 로봇 IP |
| `DEFAULT_VELOCITY_SCALE` | 기본 속도 스케일 |
| `HOME_JOINTS` | 홈 포즈 관절값 |
| `UP_JOINTS` | 위쪽 포즈 관절값 (test_standalone) |
| `NEAR_HOME_WAYPOINTS` | 근접 홈 웨이포인트 (test_multi_goal) |
| `SERVOJ_DT` | servoJ 주기 (0.008s = 125Hz) |
| `XRDF_PATH` | cuRobo XRDF 파일 경로 |
| `URDF_PATH` | URDF 파일 경로 |

---

## cumotion이 사용하지 않는 core/ 모듈

다음 core/ 모듈은 cumotion에서 **전혀 import하지 않으며**, teleop 모듈 전용:

| 모듈 | 주요 사용처 | 용도 |
|------|-----------|------|
| `core/kinematics.py` | servo/ | Pinocchio DLS IK |
| `core/pink_ik.py` | teleop_admittance/, teleop_impedance/ | QP 기반 IK (Pink) |
| `core/controller_utils.py` | servo/ | ros2_control 컨트롤러 전환 |
| `core/input_handler.py` | servo/, teleop_admittance/, teleop_impedance/ | 키보드/Xbox 입력 |
| `core/exp_filter.py` | teleop_admittance/, teleop_impedance/ | EMA+slerp 필터 |
| `core/ft_source.py` | teleop_admittance/ | F/T 센서 추상화 |
| `core/compliant_control.py` | teleop_admittance/ | 어드미턴스 제어 (M-D-K) |
| `core/teleop_protocol.py` | teleop 모듈 | 텔레옵 프로토콜 |
| `core/joystick_sender.py` | teleop 모듈 | 조이스틱 데이터 전송 |

---

## 패키지 분리 로드맵

cumotion과 teleop을 분리할 때의 경계선:

### Motion Planning 패키지 (cumotion)
- **필수**: `cumotion/planner.py` + config.py의 계획 전용 상수 6개
- **third-party**: numpy, torch, curobo
- **ROS2**: 불필요
- **선택**: `core/robot_backend.py`, `core/trajectory_executor.py` (실행까지 필요 시)
- **선택**: `core/ur_robot.py` (실물 로봇) 또는 `core/sim_robot.py` (시뮬레이션)

### Teleop 패키지 (servo + teleop_admittance + teleop_impedance)
- 나머지 core/ 모듈 전체 (9개)
- config.py의 실행/제어/안전 상수
- pinocchio, pin-pink, proxsuite, ur-rtde, pygame 등

### 공유 인프라 (양쪽 모두 사용)
- `config.py` → 분리 시 각 패키지에 필요한 상수만 추출하거나 생성자 파라미터로 전환
- `core/robot_backend.py` + `core/trajectory_executor.py` → cumotion 실행 + servo 모두 사용
- `core/ur_robot.py`, `core/sim_robot.py` → 양쪽 실행 백엔드
