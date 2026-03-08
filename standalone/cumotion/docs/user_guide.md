# Standalone cuMotion 사용자 가이드 (듀얼 모드: RTDE / Isaac Sim)

## 개요

ROS2, MoveIt, UR Driver 없이 **단일 Python 스크립트**로 UR10e 모션 플래닝 및 실행이 가능한 경량 시스템.
`--mode rtde|sim` 옵션으로 **실제 로봇(ur_rtde)**과 **Isaac Sim(ROS2 토픽)** 간 전환이 가능합니다.

### 아키텍처 비교

**기존 (ROS2 Full Stack)**
```
test_motion_plan_real.py
  → MoveGroup Action (ROS2)
  → MoveIt2 move_group node
  → cuMotion Planner Node (ROS2)
  → scaled_joint_trajectory_controller (ROS2)
  → UR Driver (ROS2) + URCap
  → UR10e Robot
```
- 필요: ROS2 노드 3개 + URCap 설치 + 티치펜던트 External Control 실행
- 장점: RViz 시각화, 표준 ROS2 생태계 호환

**Standalone (듀얼 모드)**
```
standalone.cumotion.test_standalone --mode rtde|sim
    │
    ├── StandaloneMotionPlanner (curobo GPU 직접 호출, 공통)
    │
    ├── RobotBackend (공통 인터페이스, ABC)
    │     ├── RTDEBackend  (ur_rtde → RTDE 프로토콜 → 실제 로봇)
    │     └── SimBackend   (rclpy → ROS2 토픽 → Isaac Sim)
    │
    └── execute_trajectory() (공통 실행기)
          - resample + 타이밍 루프
          - robot.send_joint_command(q) 호출 (backend별 구현)
```
- 필요: Python + CUDA GPU + (RTDE 모드: ur_rtde / Sim 모드: rclpy + Isaac Sim)
- 장점: 단순, 빠른 시작, 동일 코드로 시뮬레이션/실제 전환

---

## 사전 요구사항

### 필수
- NVIDIA GPU (CUDA 지원)
- `nvidia-smi` 정상 출력 확인
- curobo 설치 완료 (컨테이너에 기본 포함)

### RTDE 모드 (실제 로봇)
```bash
pip install ur-rtde
```

### Sim 모드 (Isaac Sim)
- rclpy (ROS2 Python 클라이언트, 컨테이너에 기본 포함)
- Isaac Sim 실행 중이며 `/joint_states`, `/joint_command` 토픽 발행

> Plan-only 모드 (로봇/시뮬레이터 연결 없이 경로 계획만)는 ur_rtde, rclpy 없이도 동작합니다.

---

## 파일 구조

```
standalone/
├── config.py                    # 공유 설정 (경로, 상수, 컨트롤러, 모드)
├── robot_backend.py             # RobotBackend ABC + create_backend() 팩토리
├── ur_robot.py                  # RTDEBackend — ur_rtde 기반 실제 로봇 통신
├── sim_robot.py                 # SimBackend — rclpy 기반 Isaac Sim 통신
├── trajectory_executor.py       # trajectory → 125Hz 스트리밍 (backend 공통)
│
├── cumotion/                    # GPU 모션 플래닝 서브패키지
│   ├── planner.py               # curobo MotionGen standalone wrapper
│   ├── test_standalone.py       # 단일 목표 테스트 (--mode rtde|sim)
│   ├── test_multi_goal.py       # 다중 목표 순차 이동 테스트
│   └── docs/user_guide.md      # 이 문서
│
└── servo/                       # 실시간 제어 서브패키지
    ├── controller_utils.py      # ControllerSwitcher (ros2_control)
    ├── pinocchio_utils.py       # PinocchioIK (FK/Jacobian/DLS)
    ├── keyboard_cartesian.py    # Pinocchio DLS 키보드 텔레옵
    ├── keyboard_forward.py      # Joint-space 키보드 제어
    ├── keyboard_servo.py        # MoveIt Servo 키보드 Cartesian
    ├── joystick_cartesian.py    # Xbox + Pinocchio DLS
    ├── joystick_servo.py        # Xbox + MoveIt Servo
    └── docs/servo_research.md   # Servo 아키텍처 리서치
```

---

## 빠른 시작

### 1. Plan-Only 테스트 (로봇/시뮬레이터 불필요)

GPU와 curobo만 있으면 경로 계획을 테스트할 수 있습니다.

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# Joint-space 플래닝 (HOME → UP)
python3 -m standalone.cumotion.test_standalone --plan-only

# Cartesian 플래닝
python3 -m standalone.cumotion.test_standalone --plan-only --goal-type cartesian

# 속도 스케일 조절
python3 -m standalone.cumotion.test_standalone --plan-only --velocity-scale 0.3

# 커스텀 목표 지정 (6개 관절 각도, radian)
python3 -m standalone.cumotion.test_standalone --plan-only \
    --goal-joints 0.0 -1.5708 0.0 -1.5708 0.0 0.0
```

출력 예시:
```
[cuMotion] Warming up GPU planner...
[cuMotion] Ready! (warmup: 2.7s)

[Test] Joint planning: start -> goal
[Test] Planning SUCCESS

==================================================
  Planning time  : 0.037s
  Waypoints      : 1109
  Time step (dt) : 0.0250s
  Total duration : 27.700s
  Start joints   : [2.2400, -1.2808, 2.1600, -0.8848, 2.2400, 0.0000]
  End joints     : [0.0000, -1.5708, 0.0000, -1.5708, 0.0000, 0.0000]
==================================================
```

### 2. 다중 목표 순차 이동 테스트

HOME 근처의 여러 waypoint를 순차적으로 이동하며 플래너 안정성을 검증합니다.

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# Plan-only (로봇/시뮬레이터 불필요)
python3 -m standalone.cumotion.test_multi_goal --plan-only

# 2라운드 (A→B→C→D→E→HOME 2회 반복)
python3 -m standalone.cumotion.test_multi_goal --plan-only --rounds 2

# HOME 복귀 생략
python3 -m standalone.cumotion.test_multi_goal --plan-only --no-return-home

# Isaac Sim 실행
python3 -m standalone.cumotion.test_multi_goal --mode sim --execute --velocity-scale 0.1

# 실제 로봇 실행
python3 -m standalone.cumotion.test_multi_goal --mode rtde --robot-ip 192.168.0.2 --execute
```

출력 예시:
```
[MultiGoal] Plan-only: 6 moves, 1 round(s)
[MultiGoal] Start: HOME [2.24, -1.2808, 2.16, -0.8848, 2.24, 0.0]

[1/6] → A  plan=0.029s  pts=173  dur=4.30s
[2/6] → B  plan=0.025s  pts=221  dur=5.50s
[3/6] → C  plan=0.024s  pts=198  dur=4.93s
[4/6] → D  plan=0.026s  pts=185  dur=4.60s
[5/6] → E  plan=0.023s  pts=167  dur=4.15s
[6/6] → HOME  plan=0.025s  pts=190  dur=4.73s

==================================================
  Results: 6/6 succeeded, 0 failed
  Total planning time: 0.152s
==================================================
```

#### test_multi_goal CLI 옵션

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--mode` | `sim` | 백엔드 모드: `rtde` 또는 `sim` |
| `--robot-ip` | `192.168.0.2` | UR10e IP (rtde 모드) |
| `--execute` | (미지정) | trajectory 실행 |
| `--plan-only` | (미지정) | 연결 없이 플래닝만 |
| `--velocity-scale` | `0.05` | 속도 스케일 0.0~1.0 |
| `--rounds` | `1` | waypoint 순회 반복 횟수 |
| `--no-return-home` | (미지정) | 라운드 간 HOME 복귀 생략 |
| `--no-confirm` | (미지정) | 실행 전 안전 확인 건너뛰기 |

### 3. Isaac Sim 모드 (기본)

Isaac Sim이 ROS2 토픽을 발행하고 있어야 합니다.

```bash
# 연결 + 플래닝만 (로봇 현재 위치 읽기 → 경로 계획)
python3 -m standalone.cumotion.test_standalone --mode sim

# 플래닝 + 실행
python3 -m standalone.cumotion.test_standalone --mode sim --execute --velocity-scale 0.1

# Cartesian 목표로 실행
python3 -m standalone.cumotion.test_standalone --mode sim --goal-type cartesian \
    --execute --velocity-scale 0.1
```

Isaac Sim 필수 토픽:
| 토픽 | 메시지 타입 | 방향 |
|------|-------------|------|
| `/joint_states` | `sensor_msgs/JointState` | Sim → Backend (subscribe) |
| `/joint_command` | `sensor_msgs/JointState` | Backend → Sim (publish) |

### 4. 실제 로봇 실행 (RTDE)

```bash
# 플래닝만 (로봇 상태 읽기 + 경로 계획, 실행 안 함)
python3 -m standalone.cumotion.test_standalone --mode rtde --robot-ip 192.168.0.2

# 플래닝 + 실행 (안전 확인 프롬프트 포함)
python3 -m standalone.cumotion.test_standalone --mode rtde --robot-ip 192.168.0.2 \
    --execute --velocity-scale 0.05

# Cartesian 목표로 실행
python3 -m standalone.cumotion.test_standalone --mode rtde --robot-ip 192.168.0.2 \
    --goal-type cartesian --execute --velocity-scale 0.05
```

> `--execute` 없이 실행하면 로봇 연결 + 현재 관절 읽기 + 플래닝만 수행합니다.

---

## CLI 옵션 (test_standalone)

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--mode` | `sim` | 백엔드 모드: `rtde` (실제 로봇) 또는 `sim` (Isaac Sim) |
| `--robot-ip` | `192.168.0.2` | UR10e IP 주소 (rtde 모드 전용) |
| `--goal-type` | `joint` | 목표 타입: `joint` 또는 `cartesian` |
| `--execute` | (미지정) | 로봇/시뮬레이터에 trajectory 실행 |
| `--plan-only` | (미지정) | 연결 없이 플래닝만 |
| `--velocity-scale` | `0.05` | 속도 스케일 0.0~1.0 (5% = 매우 느림) |
| `--start-joints` | HOME | 시작 관절 각도 6개 (radian) |
| `--goal-joints` | UP | 목표 관절 각도 6개 (radian) |
| `--no-confirm` | (미지정) | 실행 전 안전 확인 건너뛰기 |

---

## 모듈별 사용법

### StandaloneMotionPlanner

```python
from standalone.config import XRDF_PATH, URDF_PATH
from standalone.cumotion.planner import StandaloneMotionPlanner

# 초기화 (GPU warmup 포함, ~3초)
planner = StandaloneMotionPlanner(XRDF_PATH, URDF_PATH)

# Joint-space 플래닝
traj = planner.plan_joint(
    start_joints=[2.24, -1.28, 2.16, -0.88, 2.24, 0.0],
    goal_joints=[0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0],
    velocity_scale=0.1,
)
# traj["positions"]       → (N, 6) ndarray
# traj["velocities"]      → (N, 6) ndarray
# traj["accelerations"]   → (N, 6) ndarray
# traj["dt"]              → 0.025 (초)
# traj["timestamps"]      → (N,) ndarray
# traj["n_points"]        → int
# traj["total_time"]      → float (초)
# traj["plan_time"]       → float (초)

# Cartesian 플래닝 (quaternion: [qw, qx, qy, qz])
traj = planner.plan_cartesian(
    start_joints=[2.24, -1.28, 2.16, -0.88, 2.24, 0.0],
    goal_position=[0.5, 0.2, 0.3],
    goal_quaternion=[0.707, -0.707, 0.0, 0.0],
    velocity_scale=0.1,
)

# Forward Kinematics
ee = planner.get_ee_pose([0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0])
# ee["position"]    → [x, y, z]
# ee["quaternion"]  → [qw, qx, qy, qz]

# 장애물 추가 (pose: [x,y,z,qw,qx,qy,qz], dims: [dx,dy,dz])
planner.add_cuboid("box1", pose=[0.5, 0.0, 0.5, 1, 0, 0, 0], dims=[0.3, 0.3, 0.3])
planner.clear_world()
```

### RobotBackend (공통 인터페이스)

```python
from standalone.robot_backend import create_backend

# 팩토리로 백엔드 생성 (mode: "rtde" 또는 "sim")
with create_backend("sim") as robot:
    joints = robot.get_joint_positions()       # [6] radians
    velocities = robot.get_joint_velocities()  # [6] rad/s
    robot.send_joint_command([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])

with create_backend("rtde", robot_ip="192.168.0.2") as robot:
    joints = robot.get_joint_positions()
    # ... 동일한 인터페이스
```

### RTDEBackend (실제 로봇)

```python
from standalone.ur_robot import RTDEBackend

with RTDEBackend("192.168.0.2") as robot:
    joints = robot.get_joint_positions()    # [6] radians
    velocities = robot.get_joint_velocities()  # [6] rad/s
    tcp = robot.get_tcp_pose()              # [x,y,z,rx,ry,rz] (RTDE 전용)
```

### SimBackend (Isaac Sim)

```python
from standalone.sim_robot import SimBackend

with SimBackend() as robot:
    # 기본 토픽: /joint_states (sub), /joint_command (pub)
    joints = robot.get_joint_positions()
    velocities = robot.get_joint_velocities()

# 커스텀 토픽
with SimBackend(
    joint_states_topic="/my_robot/joint_states",
    joint_command_topic="/my_robot/joint_command",
) as robot:
    joints = robot.get_joint_positions()
```

### TrajectoryExecutor

```python
from standalone.trajectory_executor import (
    execute_trajectory,
    validate_trajectory,
    check_start_match,
)

# 안전 검증
warnings = validate_trajectory(traj)

# 시작 상태 확인
ok = check_start_match(traj["positions"][0], robot.get_joint_positions())

# 스트리밍 실행 (backend 무관 — RTDE든 Sim이든 동일)
execute_trajectory(robot, traj)
```

---

## 통신 방식 상세

### RTDE 모드 (실제 로봇)

ur_rtde 라이브러리가 UR 컨트롤러와 RTDE 프로토콜(포트 30004)로 통신합니다.

- **RTDEReceiveInterface**: 로봇 상태 읽기 (관절 위치/속도, TCP pose 등) — 125Hz
- **RTDEControlInterface**: servoJ/moveJ 등 실시간 제어 명령 전송

```
curobo trajectory (40Hz, dt=0.025s)
  → numpy linear interpolation
  → 125Hz (dt=0.008s) 리샘플링
  → servoJ 루프 (busy-wait 정밀 타이밍)
  → UR 컨트롤러 (RTDE)
```

#### servoJ 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `dt` | 0.008s | 제어 주기 (125Hz) |
| `lookahead_time` | 0.1s | 모션 스무딩 시간 (0.03~0.2) |
| `gain` | 300 | P 제어 게인 (100~2000) |

- `lookahead_time` ↑: 더 부드럽지만 응답 느림
- `gain` ↑: 더 정확하지만 진동 가능

### Sim 모드 (Isaac Sim)

rclpy를 사용하여 Isaac Sim의 ROS2 Bridge와 토픽으로 통신합니다.

- **Subscriber**: `/joint_states` — 시뮬레이션 로봇의 현재 관절 상태 수신
- **Publisher**: `/joint_command` — 관절 위치 명령 전송

```
curobo trajectory (40Hz, dt=0.025s)
  → numpy linear interpolation
  → 125Hz (dt=0.008s) 리샘플링
  → JointState publish 루프 (busy-wait 정밀 타이밍)
  → Isaac Sim (ROS2 Bridge)
```

#### Sim 모드 특징
- 연결 시 `/joint_states` 첫 메시지 수신 대기 (timeout: 10초)
- 관절 이름 순서 자동 정렬 (`JOINT_NAMES` 기준)
- `on_trajectory_done()`은 Sim 모드에서는 no-op (servoStop 불필요)

---

## velocity_scale 가이드

| 값 | 속도 | 용도 |
|----|------|------|
| 0.03 | 매우 느림 | 첫 테스트, 안전 확인 |
| 0.05 | 느림 | 일반 테스트 (기본값) |
| 0.1 | 보통 | 검증된 동작 반복 |
| 0.3 | 빠름 | 성능 테스트 |
| 0.5 | 매우 빠름 | 최대 속도에 가까움 |

> 실제 로봇 첫 테스트 시 반드시 0.03~0.05로 시작하세요.
> Isaac Sim에서는 0.1~0.3으로 시작해도 안전합니다.

---

## 사전 정의 포즈

| 이름 | 관절 각도 (radian) | 설명 |
|------|--------------------|------|
| HOME | `[2.24, -1.2808, 2.16, -0.8848, 2.24, 0.0]` | 홈 위치 |
| UP | `[0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]` | 수직 위치 |

### 근거리 Waypoint (NEAR_HOME_WAYPOINTS)

`test_multi_goal.py`에서 사용하는 HOME 근처 waypoint입니다. HOME에서 ±0.1~0.2 rad 범위의 작은 변위입니다.

| 라벨 | 관절 각도 (radian) | 특징 |
|------|---------------------|------|
| A | `[2.40, -1.18, 2.06, -0.88, 2.24, 0.15]` | shoulder+elbow shift |
| B | `[2.10, -1.40, 2.30, -0.75, 2.24, -0.15]` | 반대 방향 |
| C | `[2.24, -1.10, 2.00, -1.05, 2.40, 0.10]` | wrist 강조 |
| D | `[2.35, -1.35, 2.25, -0.80, 2.10, -0.10]` | 복합 변위 |
| E | `[2.15, -1.20, 2.10, -0.95, 2.30, 0.20]` | 소폭 변형 |

`config.py`에서 수정 가능합니다.

---

## 안전 기능

### 자동 검증
1. **Trajectory 검증**: 모든 waypoint의 관절 속도/가속도가 UR10e 한계 이내인지 확인
2. **시작 상태 확인**: 로봇의 현재 위치가 계획된 시작 위치와 일치하는지 확인 (tolerance: 0.05 rad)

### 실행 전 확인
`--execute` 사용 시 안전 체크리스트가 표시되며, "yes" 입력 시에만 실행됩니다.
- RTDE 모드: "REAL ROBOT EXECUTION" + 작업 영역/E-stop/위치 체크리스트
- Sim 모드: "ISAAC SIM EXECUTION" (간소화)

`--no-confirm` 옵션으로 건너뛸 수 있습니다.

### 비상 정지
- 프로그램 중단: `Ctrl+C`
- 실제 로봇 E-stop: 항상 접근 가능한 위치에 준비

---

## 트러블슈팅

### GPU 관련

**`RuntimeError: No CUDA GPUs are available`**
- `nvidia-smi`가 GPU를 인식하는지 확인
- Docker: `--runtime=nvidia` 또는 `--gpus all` 옵션 필요

**GPU warmup이 오래 걸림 (>10초)**
- 첫 실행 시 CUDA 커널 컴파일로 느릴 수 있음
- 이후 실행은 캐시 덕분에 2~3초

### ur_rtde 관련 (RTDE 모드)

**`ImportError: ur_rtde is not installed`**
```bash
pip install ur-rtde
```

**`Connection refused` / 연결 실패**
- 로봇 IP 주소 확인 (`ping <ROBOT_IP>`)
- 로봇이 켜져 있고 Remote Control 모드인지 확인
- 방화벽에서 포트 30004 허용 확인

**`servoJ` 실행 중 로봇이 멈춤**
- `lookahead_time` 값을 늘려보기 (0.1 → 0.15)
- `gain` 값을 줄여보기 (300 → 200)
- `velocity_scale`을 더 낮춰보기

### Isaac Sim 관련 (Sim 모드)

**`TimeoutError: No message received on /joint_states within 10s`**
- Isaac Sim이 실행 중이고 ROS2 Bridge가 활성화되어 있는지 확인
- `ros2 topic list`로 `/joint_states` 토픽이 있는지 확인
- `ros2 topic echo /joint_states`로 메시지가 발행되고 있는지 확인
- ROS2 도메인 ID가 일치하는지 확인 (`ROS_DOMAIN_ID`)

**관절 순서가 맞지 않음**
- SimBackend는 `JOINT_NAMES` 기준으로 자동 재정렬합니다
- Isaac Sim에서 발행하는 관절 이름이 UR10e 표준 이름과 일치하는지 확인
- `config.py`의 `JOINT_NAMES` 목록 확인

**Isaac Sim 로봇이 움직이지 않음**
- `/joint_command` 토픽을 Isaac Sim이 구독하고 있는지 확인
- `ros2 topic echo /joint_command`로 명령이 발행되는지 확인
- Isaac Sim의 Action Graph에서 ROS2 Subscribe 노드가 `/joint_command`를 구독하도록 설정되어 있는지 확인

### 플래닝 관련

**`Planning failed: IK_FAIL`**
- Cartesian 목표가 로봇의 작업 범위 밖일 수 있음
- 다른 orientation으로 시도

**`Planning failed: INVALID_START_STATE_JOINT_LIMITS`**
- 시작 관절 각도가 UR10e의 관절 한계를 벗어남

**Planning은 성공하지만 trajectory가 너무 김**
- `velocity_scale`을 높여서 속도 증가
- `config.py`의 `INTERPOLATION_DT`를 늘리면 waypoint 수 감소

---

## 기존 ROS2 시스템과의 차이점

| 항목 | ROS2 Full Stack | Standalone |
|------|-----------------|------------|
| 필요 노드 | 3개 (UR Driver, MoveIt, cuMotion) | 없음 |
| URCap | 필수 (설치 + 실행) | 불필요 |
| 시작 시간 | 30초+ (노드 순차 시작) | ~3초 (GPU warmup만) |
| 시각화 | RViz | 없음 (Isaac Sim 사용 가능) |
| 실시간 제어 | scaled_joint_trajectory_controller | servoJ 125Hz (RTDE) / JointState pub (Sim) |
| 로봇 상태 읽기 | `/joint_states` 토픽 | RTDE 직접 또는 `/joint_states` 토픽 |
| 장애물 회피 | nvblox ESDF + Voxel | Mesh 기반 primitive |
| 시뮬레이션 | Gazebo/Isaac Sim + UR Driver | Isaac Sim ROS2 토픽 직접 |
| 백엔드 전환 | launch 파일 변경 | `--mode rtde\|sim` 옵션 하나 |
