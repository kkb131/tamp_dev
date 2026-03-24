# Standalone 학습 매뉴얼

UR10e 로봇의 GPU 모션 플래닝을 MoveIt/ROS2 없이 독립적으로 수행하는 Python 패키지.
이 문서는 주니어 개발자가 각 모듈을 **왜 만들었는지**, **무엇을 하는지**, **핵심 코드가 무엇인지** 순서로 학습할 수 있도록 구성했다.

> 원격조종(teleop) 관련 코드는 `bak/`에 백업되어 있다. 해당 문서는 `bak/` 내 각 모듈의 `docs/` 참조.

---

## 목차

1. [프로젝트 개요](#1-프로젝트-개요)
2. [config.py — 공유 설정](#2-configpy--공유-설정)
3. [core/ — 공유 인프라](#3-core--공유-인프라)
4. [cumotion/ — GPU 경로 계획](#4-cumotion--gpu-경로-계획)
5. [학습 로드맵](#5-학습-로드맵)
6. [용어 사전](#6-용어-사전)

---

## 1. 프로젝트 개요

### 해결하는 문제

MoveIt + ROS2를 사용하면 모션 플래닝과 로봇 제어가 가능하지만:
- 시작이 복잡하다 (3개 터미널, 여러 노드 실행)
- GPU 플래닝(cuMotion)을 단독으로 테스트하기 어렵다

standalone 패키지는 이 문제를 해결한다:
- `python3 -m standalone.cumotion.test_standalone --plan-only` 한 줄로 GPU 플래닝 테스트
- 로봇 연결 없이 경로 계획 검증 가능

### 전체 아키텍처

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

### 3가지 설계 원칙

| 원칙 | 설명 | 예시 |
|------|------|------|
| **Backend ABC** | 실제 로봇과 시뮬레이션을 동일 인터페이스로 사용 | `create_backend("sim")` ↔ `create_backend("rtde")` |
| **Core 공유** | 2개 이상 모듈이 쓰는 코드는 core/로 승격 | robot_backend, trajectory_executor |
| **Feature 독립** | 기능 모듈은 core/와 config만 import | cumotion은 core/만 의존 |

---

## 2. config.py — 공유 설정

### 왜 필요한가

로봇 IP, 관절 이름, 안전 한계 등을 각 모듈에서 하드코딩하면 값이 달라질 위험이 있다.
`config.py` 하나에서 관리하여 **단일 진실 원천(single source of truth)** 을 보장한다.

### 주요 상수

| 카테고리 | 상수 | 값 | 용도 |
|---------|------|-----|------|
| 경로 | `URDF_PATH` | `.docker/assets/ur10e.urdf` | IK/FK에 로봇 모델 제공 |
| 경로 | `XRDF_PATH` | cuMotion XRDF 경로 | GPU 플래너 충돌 정보 |
| 로봇 | `JOINT_NAMES` | 6개 관절명 | 모든 모듈에서 관절 순서 통일 |
| 로봇 | `RTDE_FREQUENCY` | 125 Hz | servoJ 명령 주기 |
| 안전 | `MAX_JOINT_VEL_RAD_S` | 2.094 rad/s | 궤적 검증 기준 |
| 제어 | `SERVOJ_DT` | 0.008s (1/125) | servoJ 타이밍 |
| cuMotion | `INTERPOLATION_DT` | 0.025s | 궤적 보간 간격 (40Hz) |
| cuMotion | `NUM_GRAPH_SEEDS` | 6 | RRT-Connect 시드 수 |
| 포즈 | `HOME_JOINTS` | [2.24, -1.28, ...] rad | 기본 홈 위치 |

### 핵심 코드 (`config.py`)

```python
JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

MAX_JOINT_VEL_RAD_S = 2.094        # ~120 deg/s
HOME_JOINTS = [2.24, -1.2808, 2.16, -0.8848, 2.24, 0.0]
```

---

## 3. core/ — 공유 인프라

로봇 통신과 궤적 실행을 담당하는 공통 코드.
**이 섹션을 먼저 이해해야** cumotion 모듈이 이해된다.

### 3.1 로봇 통신 — robot_backend + ur_robot + sim_robot

#### 왜 필요한가

실제 로봇(ur_rtde 라이브러리)과 시뮬레이션(ROS2 토픽)의 통신 방식은 완전히 다르다.
하지만 상위 코드에서는 동일한 방법으로 로봇을 제어하고 싶다.
**ABC(Abstract Base Class) + Factory 패턴**으로 해결한다.

#### 구조

```
RobotBackend (ABC)             ← 인터페이스 정의
├── RTDEBackend (ur_robot.py)  ← 실제 로봇 구현 (ur_rtde servoJ)
└── SimBackend  (sim_robot.py) ← 시뮬레이션 구현 (ROS2 /joint_states, /joint_command)

create_backend("rtde"|"sim")   ← 팩토리 함수
```

#### 핵심 코드 (`core/robot_backend.py`)

```python
class RobotBackend(ABC):
    @abstractmethod
    def connect(self): ...
    @abstractmethod
    def get_joint_positions(self) -> List[float]: ...
    @abstractmethod
    def send_joint_command(self, positions: List[float]): ...

    def __enter__(self):          # Context manager 지원
        self.connect()
        return self

def create_backend(mode: str, **kwargs) -> RobotBackend:
    if mode == "rtde":
        from standalone.core.ur_robot import RTDEBackend
        return RTDEBackend(robot_ip=kwargs["robot_ip"])
    elif mode == "sim":
        from standalone.core.sim_robot import SimBackend
        return SimBackend(**{k: v for k, v in kwargs.items() if k != "robot_ip"})
```

#### 사용 패턴 (모든 기능 모듈에서 동일)

```python
with create_backend("sim") as robot:
    q = robot.get_joint_positions()     # 현재 관절 위치 읽기
    robot.send_joint_command([...])     # 관절 명령 전송
```

---

### 3.2 궤적 실행 — trajectory_executor

#### 왜 필요한가

cuMotion GPU 플래너는 0.025s(40Hz) 간격으로 궤적을 생성하지만, 로봇의 servoJ는 0.008s(125Hz)로 명령을 받아야 한다. **리샘플링** 후 **정밀 타이밍**으로 스트리밍해야 한다.

#### 핵심 코드 (`core/trajectory_executor.py`)

```python
def resample_trajectory(positions, source_dt, target_dt):
    """cuMotion 40Hz → servoJ 125Hz 선형 보간."""
    source_times = np.arange(len(positions)) * source_dt
    target_times = np.arange(0, source_times[-1] + target_dt * 0.5, target_dt)
    resampled = np.zeros((len(target_times), positions.shape[1]))
    for j in range(positions.shape[1]):
        resampled[:, j] = np.interp(target_times, source_times, positions[:, j])
    return resampled

def execute_trajectory(robot, trajectory, command_dt=SERVOJ_DT):
    """리샘플링 후 busy-wait로 정밀 스트리밍."""
    resampled = resample_trajectory(trajectory["positions"], trajectory["dt"], command_dt)
    t_start = time.perf_counter()
    for i in range(len(resampled)):
        robot.send_joint_command(resampled[i].tolist())
        t_target = t_start + (i + 1) * command_dt
        while time.perf_counter() < t_target:   # busy-wait (sleep보다 정밀)
            pass
    robot.on_trajectory_done()
```

---

## 4. cumotion/ — GPU 경로 계획

> **상세 사용법**: [cumotion/docs/user_guide.md](cumotion/docs/user_guide.md)
> **의존성 분석**: [cumotion/DEPENDENCIES.md](cumotion/DEPENDENCIES.md)

### 왜 필요한가

A에서 B로 가는 **최적 경로**를 계산하고 **장애물 회피**가 필요하다.
cuMotion은 NVIDIA GPU에서 수천 개의 경로를 **병렬 최적화**하여:
- 충돌 없는 경로를 찾고
- 속도/가속도 프로파일까지 생성한다

### Joint vs Cartesian 플래닝

| 방식 | 입력 | 사용 시나리오 |
|------|------|-------------|
| `plan_joint()` | 시작/목표 관절 각도 | 정확한 관절 위치가 알려진 경우 |
| `plan_cartesian()` | 시작 관절 + 목표 EE 포즈 | 직교 좌표로 목표를 지정할 때 |

### 데이터 흐름

```
plan_joint(start, goal)
  → curobo MotionGen (GPU 병렬 최적화)
  → interpolated trajectory (0.025s 간격)
  → {"positions": (N,6), "velocities": (N,6), "dt": 0.025, ...}

실행 시:
  trajectory → resample_trajectory(0.025s → 0.008s)
             → execute_trajectory(robot, 125Hz busy-wait)
```

### 핵심 코드 (`cumotion/planner.py`)

```python
class StandaloneMotionPlanner:
    def plan_joint(self, start_joints, goal_joints, velocity_scale=0.5):
        start_state = CuJointState.from_position(
            position=self.tensor_args.to_device(start_joints).unsqueeze(0),
            joint_names=self.joint_names,
        )
        goal_state = CuJointState.from_position(
            position=self.tensor_args.to_device(goal_joints).unsqueeze(0),
            joint_names=self.joint_names,
        )
        result = self.motion_gen.plan_single_js(start_state, goal_state, plan_config)
        if not result.success.item():
            return None
        return self._extract_trajectory(result, plan_time)
```

반환값:
```python
{
    "positions":     np.ndarray (N, 6),   # 각 시점의 관절 위치
    "velocities":    np.ndarray (N, 6),   # 각 시점의 관절 속도
    "accelerations": np.ndarray (N, 6),   # 각 시점의 관절 가속도
    "dt":            float,               # 시간 간격 (0.025s)
    "n_points":      int,                 # 궤적 점 수
    "total_time":    float,               # 전체 시간
    "plan_time":     float,               # 계획 소요 시간
}
```

### 실행

```bash
cd /workspaces/tamp_ws/src/tamp_dev
# GPU만 있으면 실행 가능 (로봇 연결 불필요)
python3 -m standalone.cumotion.test_standalone --plan-only

# 다중 목표 순차 테스트
python3 -m standalone.cumotion.test_multi_goal --plan-only --rounds 3

# 시뮬레이션에서 실행
python3 -m standalone.cumotion.test_standalone --mode sim --execute

# 실제 로봇에서 실행
python3 -m standalone.cumotion.test_standalone --mode rtde --robot-ip 192.168.0.2 --execute
```

---

## 5. 학습 로드맵

### 단계별 학습 계획

| 단계 | 모듈 | 학습 목표 | 이해해야 할 핵심 개념 |
|------|------|----------|---------------------|
| **0** | `config.py` | 프로젝트 구조 이해 | 상수 관리, 경로 설정 |
| **1** | `core/robot_backend.py` | 로봇 통신 추상화 | ABC, Factory, Context Manager |
| **2** | `core/trajectory_executor.py` | 궤적 개념 | 리샘플링, 실시간 스트리밍, busy-wait |
| **3** | `cumotion/planner.py` | GPU 모션 플래닝 | curobo MotionGen, 충돌 회피, Joint/Cartesian 플래닝 |
| **4** | `cumotion/test_standalone.py` | 통합 테스트 | plan-only vs execute, dual-mode (sim/rtde) |

### 선수 지식

- **Python**: 클래스, ABC, 데코레이터, numpy 기초
- **로봇 기초**: 관절(joint), 순운동학(FK), Jacobian 개념
- **ROS2 기초** (sim 모드만): 토픽(publish/subscribe)

---

## 6. 용어 사전

| 용어 | 설명 |
|------|------|
| **ABC** | Abstract Base Class — 인터페이스 정의용 추상 클래스 |
| **cuRobo** | NVIDIA의 GPU 가속 로봇 모션 플래닝 라이브러리 |
| **FK** | Forward Kinematics — 관절 각도 → 말단 위치 계산 |
| **IK** | Inverse Kinematics — 말단 위치 → 관절 각도 계산 |
| **Jacobian** | 관절 속도와 말단 속도의 관계 행렬 (6×N) |
| **MotionGen** | cuRobo의 모션 플래닝 엔진 (GPU 병렬 최적화) |
| **RTDE** | Real-Time Data Exchange — UR 로봇과의 125Hz 실시간 통신 프로토콜 |
| **servoJ** | UR 로봇의 실시간 관절 위치 명령 함수 |
| **XRDF** | cuMotion 로봇 충돌 모델 파일 형식 |
