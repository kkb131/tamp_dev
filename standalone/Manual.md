# Standalone 학습 매뉴얼

UR10e 로봇을 MoveIt/ROS2 없이 독립적으로 제어하는 Python 패키지.
이 문서는 주니어 개발자가 각 모듈을 **왜 만들었는지**, **무엇을 하는지**, **핵심 코드가 무엇인지** 순서로 학습할 수 있도록 구성했다.

---

## 목차

1. [프로젝트 개요](#1-프로젝트-개요)
2. [config.py — 공유 설정](#2-configpy--공유-설정)
3. [core/ — 공유 인프라](#3-core--공유-인프라)
4. [servo/ — 기본 텔레옵 (난이도 ★)](#4-servo--기본-텔레옵-난이도-)
5. [cumotion/ — GPU 경로 계획 (난이도 ★★)](#5-cumotion--gpu-경로-계획-난이도-)
6. [teleop_admittance/ — 어드미턴스 텔레옵 (난이도 ★★★)](#6-teleop_admittance--어드미턴스-텔레옵-난이도-)
7. [teleop_impedance/ — 임피던스 텔레옵 (난이도 ★★★★)](#7-teleop_impedance--임피던스-텔레옵-난이도-)
8. [학습 로드맵](#8-학습-로드맵)
9. [용어 사전](#9-용어-사전)

---

## 1. 프로젝트 개요

### 해결하는 문제

MoveIt + ROS2를 사용하면 모션 플래닝과 로봇 제어가 가능하지만:
- 시작이 복잡하다 (3개 터미널, 여러 노드 실행)
- 실시간 텔레옵에 적합하지 않다 (MoveIt의 plan-execute 패턴은 지연이 크다)
- GPU 플래닝(cuMotion)을 단독으로 테스트하기 어렵다

standalone 패키지는 이 문제들을 해결한다:
- `python3 -m standalone.<module>` 한 줄로 실행
- 실시간 텔레옵 (125Hz servoJ 루프)
- GPU 플래닝 단독 테스트 가능

### 전체 아키텍처

```
┌──────────────────────────────────────────────────────────────────┐
│                         config.py                                │  공유 상수
├──────────────────────────────────────────────────────────────────┤
│                          core/                                   │  공유 인프라
│  robot_backend · ur_robot · sim_robot · trajectory_executor      │
│  kinematics · pink_ik · input_handler · exp_filter               │
│  ft_source · compliant_control · controller_utils                │
├───────────┬───────────┬──────────────────┬───────────────────────┤
│  servo/   │ cumotion/ │ teleop_admittance│ teleop_impedance/     │  기능 모듈
│  DLS 텔레옵│ GPU 플래닝 │ F/T 어드미턴스    │ URScript PD 토크      │
│  ★        │ ★★       │ ★★★             │ ★★★★                 │
└───────────┴───────────┴──────────────────┴───────────────────────┘
```

### 3가지 설계 원칙

| 원칙 | 설명 | 예시 |
|------|------|------|
| **Backend ABC** | 실제 로봇과 시뮬레이션을 동일 인터페이스로 사용 | `create_backend("sim")` ↔ `create_backend("rtde")` |
| **Core 공유** | 2개 이상 모듈이 쓰는 코드는 core/로 승격 | input_handler, exp_filter, pink_ik |
| **Feature 독립** | 기능 모듈 간 교차 import 금지 | servo는 teleop_admittance를 import하지 않음 |

### 학습 순서 (추천)

```
servo(★) → cumotion(★★) → teleop_admittance(★★★) → teleop_impedance(★★★★)
 위치제어       경로계획        위치+힘제어            토크제어
 단순 루프      오프라인        Pink IK+Safety        URScript 500Hz
```

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
| 안전 | `UR10E_MAX_TORQUES` | [150,150,56,56,28,28] Nm | 관절별 토크 한계 |
| 제어 | `SERVOJ_DT` | 0.008s (1/125) | servoJ 타이밍 |
| 포즈 | `HOME_JOINTS` | [2.24, -1.28, ...] rad | 기본 홈 위치 |

### 핵심 코드 (`config.py`)

```python
JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

MAX_JOINT_VEL_RAD_S = 2.094        # ~120 deg/s
UR10E_MAX_TORQUES = [150.0, 150.0, 56.0, 56.0, 28.0, 28.0]  # Nm
HOME_JOINTS = [2.24, -1.2808, 2.16, -0.8848, 2.24, 0.0]
```

---

## 3. core/ — 공유 인프라

> **심화 학습**: [core/docs/manual.md](core/docs/manual.md) — 각 모듈의 설계 근거, 수식, 핵심 코드 상세 분석

모든 기능 모듈이 의존하는 공통 코드. **이 섹션을 먼저 이해해야** 이후 모듈이 이해된다.

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

### 3.3 역기구학 — DLS vs QP (두 가지 IK)

#### 왜 2개인가

| | Pinocchio DLS (`kinematics.py`) | Pink QP (`pink_ik.py`) |
|---|---|---|
| 방식 | Damped Least Squares | Quadratic Programming |
| 관절 한계 | 후처리 클램핑 (위반 가능) | QP 제약 조건 (위반 불가) |
| 복잡도 | 단순 (~10줄 핵심) | 복잡 (task + constraint) |
| 사용처 | servo/ (학습용, 단순) | teleop_admittance, teleop_impedance (실전) |

단순한 teleop 스크립트(servo/)에는 DLS로 충분하다.
하지만 안전이 중요한 파이프라인(teleop_*)에서는 **관절 한계를 수학적으로 보장**하는 QP가 필요하다.

#### 핵심 코드 — DLS (`core/kinematics.py`)

```python
def compute_joint_delta(self, q, twist, dt, damping=0.05):
    """DLS: dq = J^T @ inv(J@J^T + λ²I) @ twist * dt"""
    J = self.get_jacobian(q)
    JJt_damped = J @ J.T + (damping ** 2) * np.eye(6)
    dq = J.T @ np.linalg.solve(JJt_damped, twist * dt)
    return dq
```

#### 핵심 코드 — QP (`core/pink_ik.py`)

```python
def solve(self, target_pos, target_quat, dt):
    """QP: 관절 한계 제약 하에 EE 추적 최적화."""
    target_se3 = pin.SE3(pin.Quaternion(w, x, y, z).matrix(), target_pos)
    self._ee_task.set_target(target_se3)
    velocity = pink.solve_ik(self._config, self._tasks, dt,
                             solver="proxqp", damping=self._damping)
    self._config.integrate_inplace(velocity, dt)
    return self._config.q.copy()
```

---

### 3.4 입력 처리 — input_handler

#### 왜 필요한가

키보드, Xbox 컨트롤러 등 다양한 입력 장치를 지원해야 하지만, 상위 코드는 입력 장치를 몰라도 되어야 한다.
`TeleopCommand` 데이터클래스로 **입력을 통일**한다.

#### 핵심 구조

```python
@dataclass
class TeleopCommand:
    velocity: np.ndarray    # [vx, vy, vz, wx, wy, wz] — 6-DOF
    estop: bool             # 비상 정지
    reset: bool             # E-Stop 해제
    quit: bool              # 종료
    speed_scale: float      # 속도 배율 (0.5~8.0)
    admittance_toggle: bool # 어드미턴스 ON/OFF
    admittance_preset: str  # "STIFF"/"MEDIUM"/"SOFT"

class InputHandler(ABC):
    def get_command(self, timeout=0.02) -> TeleopCommand: ...

# 팩토리
create_input("keyboard")  → KeyboardInput
create_input("xbox")      → XboxInput
```

#### 키보드 매핑

```
이동: W/S(X) A/D(Y) Q/E(Z)    회전: U/O(Roll) I/K(Pitch) J/L(Yaw)
속도: +/- (5단계: 0.5x~8.0x)  E-Stop: Space   종료: ESC
```

---

### 3.5 필터링 — exp_filter

#### 왜 필요한가

키보드 입력은 불연속적(키를 누르면 즉시 값 변화)이다. 이를 그대로 로봇에 보내면 **급격한 움직임(jerk)** 이 발생한다.
**지수 이동 평균(EMA)** 으로 위치를, **구면 선형 보간(slerp)** 으로 자세를 부드럽게 만든다.

#### 핵심 코드 (`core/exp_filter.py`)

```python
class ExpFilter:
    def update(self, position, quaternion):
        # 위치: 선형 EMA
        filtered_pos = self._alpha_pos * position + (1 - self._alpha_pos) * self._prev_pos
        # 자세: 쿼터니언 slerp
        filtered_quat = self._prev_quat.slerp(self._alpha_ori, new_quat)
        return filtered_pos, filtered_quat
```

- `alpha = 1.0`: 필터 없음 (입력 그대로)
- `alpha = 0.1`: 매우 부드러움 (반응 느림)
- 기본값: `0.85` (적당한 반응성 + 부드러움)

---

### 3.6 힘/토크 센서 — ft_source

#### 왜 필요한가

어드미턴스 제어에는 외력 측정(F/T 센서)이 필요하지만, 시뮬레이션에서는 F/T 센서가 없다.
**Protocol 패턴**으로 실제 센서와 null 센서를 교체 가능하게 만든다.

#### 핵심 코드 (`core/ft_source.py`)

```python
class FTSource(Protocol):
    def get_wrench(self) -> np.ndarray: ...     # [fx,fy,fz,tx,ty,tz]
    def zero_sensor(self) -> None: ...          # 바이어스 보정

class RTDEFTSource:                              # 실제 로봇
    def get_wrench(self):
        return np.array(self._backend.get_tcp_force()) - self._bias

class NullFTSource:                              # 시뮬레이션
    def get_wrench(self): return np.zeros(6)
```

---

### 3.7 컴플라이언스 제어 — compliant_control

#### 왜 필요한가

로봇이 외력에 **유연하게 반응**하려면, 외력을 측정하고 그에 따라 위치를 보정해야 한다.
가상의 질량-댐퍼-스프링(Mass-Damper-Spring) 시스템으로 모델링한다.

#### 어드미턴스 동역학

```
M * ẍ + D * ẋ + K * x = F_ext

M: 가상 질량 (클수록 관성이 커서 천천히 반응)
D: 감쇠 (진동 억제)
K: 강성 (외력 제거 시 원위치 복원 속도)
F_ext: 외부 힘/토크
x: 변위 (로봇이 얼마나 밀리는지)
```

#### 프리셋

| 프리셋 | M (질량) | D (감쇠) | K (강성) | 특성 |
|--------|---------|---------|---------|------|
| STIFF | [10,10,10,...] | [200,...] | [500,...] | 단단함, 거의 안 밀림 |
| MEDIUM | [5,5,5,...] | [100,...] | [200,...] | 적당한 유연성 |
| SOFT | [2,2,2,...] | [40,...] | [50,...] | 부드러움, 쉽게 밀림 |

#### 핵심 코드 (`core/compliant_control.py`)

```python
class AdmittanceController:
    def update(self, f_ext, dt):
        # M*xddot + D*xdot + K*x = f_ext  →  오일러 적분
        xddot = (f_ext - self._params.D * self._xdot - self._params.K * self._x) / self._params.M
        self._xdot += xddot * dt
        self._x += self._xdot * dt
        return self._x.copy()    # 변위 [dx,dy,dz, drx,dry,drz]
```

---

### 3.8 컨트롤러 전환 — controller_utils

#### 왜 필요한가

ROS2 mock hardware에서 실시간 servoJ를 사용하려면, 기본 `joint_trajectory_controller`를 `forward_position_controller`로 전환해야 한다. (Isaac Sim에서는 불필요)

#### 핵심 코드

```python
class ControllerSwitcher:
    def activate_forward_position(self):
        self.switch_controller(
            start=[FORWARD_POSITION_CONTROLLER],
            stop=[TRAJECTORY_CONTROLLER, SCALED_TRAJECTORY_CONTROLLER]
        )
    def restore_original(self):    # 종료 시 원래 컨트롤러 복구
```

---

## 4. servo/ — 기본 텔레옵 (난이도 ★)

> **심화 학습**: [servo/docs/manual.md](servo/docs/manual.md) — 4개 스크립트 비교, 데이터 흐름, 진화 과정
> **상세 사용법**: [servo/docs/user_guide.md](servo/docs/user_guide.md)

### 왜 필요한가

가장 단순한 실시간 제어. 각 파일이 **하나의 완전한 제어 루프**이다.
core/ 모듈의 사용법을 익히는 **학습 시작점**으로 적합하다.

### 4개 스크립트 비교

| 스크립트 | 입력 | 제어 방식 | IK | 특징 |
|---------|------|----------|-----|------|
| `keyboard_forward.py` | 키보드 | Joint-space 직접 | 없음 | 가장 단순, 관절별 조작 |
| `keyboard_cartesian.py` | 키보드 | Cartesian | DLS | 직교 좌표 이동 |
| `joystick_cartesian.py` | Xbox | Cartesian | DLS | 아날로그 입력 |
| `keyboard_servo_admittance.py` | 키보드+F/T | Cartesian+어드미턴스 | DLS | 외력 반응 (RTDE만) |

### 데이터 흐름 (keyboard_cartesian 기준)

```
키보드 입력 → twist [vx,vy,vz,wx,wy,wz]
            → DLS IK (J^T @ inv(J@J^T + λ²I) @ twist)
            → dq (관절 변위)
            → q_new = q + dq
            → clamp(관절 한계)
            → send_joint_command(q_new)
            → 50Hz 루프 반복
```

### 핵심 코드 — keyboard_cartesian.py 메인 루프 (요약)

```python
kin = PinocchioIK()                      # FK/IK 초기화
with create_backend(mode) as robot:
    q = np.array(robot.get_joint_positions())
    while running:
        key = read_key()
        twist = KEY_MAP.get(key, zeros) * speed_scale
        dq = kin.compute_joint_delta(q, twist, dt=1/50)
        q = kin.clamp_positions(q + dq)
        robot.send_joint_command(q.tolist())
        time.sleep(1/50)
```

### 실행

```bash
cd /workspaces/tamp_ws/src/tamp_dev
python3 -m standalone.servo.keyboard_cartesian --mode sim
python3 -m standalone.servo.keyboard_forward --mode rtde --robot-ip 192.168.0.2
```

---

## 5. cumotion/ — GPU 경로 계획 (난이도 ★★)

> **심화 학습**: [cumotion/docs/manual.md](cumotion/docs/manual.md) — cuRobo 개념, planner.py 분석, 궤적 실행 파이프라인
> **상세 사용법**: [cumotion/docs/user_guide.md](cumotion/docs/user_guide.md)

### 왜 필요한가

servo/의 DLS IK는 **한 스텝씩** 이동하므로, A에서 B로 가는 **최적 경로**를 계산할 수 없다.
또한 **장애물 회피**가 불가능하다.

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
```

---

## 6. teleop_admittance/ — 어드미턴스 텔레옵 (난이도 ★★★)

> **심화 학습**: [teleop_admittance/docs/manual.md](teleop_admittance/docs/manual.md) — 10단계 제어 루프, AdmittanceLayer, SafetyMonitor 상세
> **상세 사용법**: [teleop_admittance/docs/user_guide.md](teleop_admittance/docs/user_guide.md)

### 왜 필요한가

servo/의 단순 텔레옵에 **3가지 한계**가 있다:

1. **안전 시스템 없음** — 관절 속도 초과, 워크스페이스 이탈 시 보호 장치가 없다
2. **외력 대응 불가** — 로봇이 물체에 부딪혀도 밀려나지 않는다 (위치 제어만)
3. **관절 한계 위반** — DLS IK는 클램핑만 하므로 한계 근처에서 불안정하다

teleop_admittance는 이를 해결한다:
- **Pink QP IK**: 관절 한계를 수학적으로 보장
- **4단계 Safety Monitor**: 속도/워크스페이스/타임아웃/E-Stop
- **Admittance Layer**: F/T 센서 기반 외력 대응

### 파이프라인

```
Input (keyboard/xbox)
  │
  ▼
ExpFilter (EMA + slerp → 노이즈 제거)
  │
  ▼
Workspace Clamp (3D 경계: x,y [-0.8,0.8] z [0.05,1.2])
  │
  ▼
Admittance Layer (F/T 센서 → M·ẍ+D·ẋ+K·x=F → 변위 보정)
  │
  ▼
Pink QP IK (Cartesian → Joint, 관절한계 제약)
  │
  ▼
Safety Monitor (속도 제한, 패킷 타임아웃, E-Stop)
  │
  ▼
Robot (send_joint_command @ 125Hz)
```

### servo/와의 핵심 차이

| | servo/ | teleop_admittance/ |
|---|---|---|
| IK | Pinocchio DLS (클램핑) | Pink QP (제약 보장) |
| 안전 | 없음 | 4단계 (E-Stop, 워크스페이스, 속도, 타임아웃) |
| 외력 대응 | 없음 | 어드미턴스 (F/T 센서) |
| 설정 | 코드 내 하드코딩 | YAML config (dataclass) |
| 필터 | 없음 | ExpFilter (EMA + slerp) |

### 4단계 안전 시스템

| 우선순위 | 레벨 | 동작 | 트리거 |
|---------|------|------|--------|
| 최고 | **E-Stop** | 즉시 정지, 수동 리셋 필요 | Space 키 / 외부 신호 |
| 높음 | **Workspace Clamp** | EE 위치를 경계 내로 강제 | 워크스페이스 이탈 |
| 중간 | **Velocity Limit** | 관절 속도 스케일링 | max_joint_vel 초과 |
| 낮음 | **Packet Timeout** | speed_stop() 호출 | 200ms 입력 없음 |

### 핵심 코드 — 제어 루프 (`teleop_admittance/main.py` 요약)

```python
while self.running:
    cmd = self.input_handler.get_command()                # 1. 입력 읽기
    target_pos += cmd.velocity[:3]                        # 2. 목표 누적
    filt_pos, filt_quat = self.exp_filter.update(...)     # 3. 필터링
    clamped_pos = self.safety.clamp_workspace(filt_pos)   # 4. 워크스페이스 클램핑
    adm_disp = self.admittance.compute_displacement(...)  # 4.5. 어드미턴스 보정
    compliant_pos = clamped_pos + adm_disp[:3]
    q_target = self.ik.solve(compliant_pos, filt_quat, dt) # 5. Pink IK
    result = self.safety.check_and_apply(q_target, ...)   # 6. 안전 검사
    if result.is_safe:
        self.backend.send_joint_command(result.q_safe)    # 7. 명령 전송
```

### 실행

```bash
cd /workspaces/tamp_ws/src/tamp_dev
python3 -m standalone.teleop_admittance.main --mode sim --input keyboard
python3 -m standalone.teleop_admittance.main --mode rtde --input xbox --robot-ip 192.168.0.2
```

---

## 7. teleop_impedance/ — 임피던스 텔레옵 (난이도 ★★★★)

> **심화 학습**: [teleop_impedance/docs/manual.md](teleop_impedance/docs/manual.md) — 듀얼 루프, RTDE 레지스터, URScript PD, 게인 설계 상세
> **상세 사용법**: [teleop_impedance/docs/user_guide.md](teleop_impedance/docs/user_guide.md)

### 왜 필요한가

teleop_admittance의 어드미턴스 제어는 **외력을 측정하고 위치를 보정**하는 방식이다.
하지만 근본적 한계가 있다:

- 위치 제어 기반이라 **접촉 순간 충격이 크다**
- 센서 노이즈에 민감하다
- 컴플라이언스가 소프트웨어적(가상)이다

임피던스 제어는 **직접 토크를 인가**하여 **물리적 컴플라이언스**를 구현한다:
- 접촉 시 자연스럽게 밀림 (스프링처럼)
- 센서 불필요 (PD 게인이 강성/감쇠를 결정)
- 500Hz URScript 루프로 빠른 반응

### 어드미턴스 vs 임피던스 비교

| | Admittance (teleop_admittance) | Impedance (teleop_impedance) |
|---|---|---|
| 제어 변수 | **위치** (position) | **토크** (torque) |
| 공식 | M·ẍ+D·ẋ+K·x = F_ext | τ = Kp·(q_d-q) - Kd·q̇ |
| F/T 센서 | **필요** (외력 측정) | **불필요** (게인이 강성 결정) |
| 컴플라이언스 | 소프트웨어적 (가상) | 물리적 (실제 토크) |
| 접촉 충격 | 큼 (위치 제어) | 작음 (토크 제어) |
| 제어 주기 | 125Hz (Python) | **500Hz** (URScript) |
| 요구사항 | ur_rtde | PolyScope 5.23.0+ (`direct_torque()`) |

### 듀얼 루프 아키텍처

```
Python 측 (125Hz)                    URScript 측 (500Hz)
─────────────────                    ────────────────────
Input → Filter → IK → q_desired ──→ RTDE input registers 0..5
                                     │
                                     ▼
                           q = get_actual_joint_positions()
                           qd = get_actual_joint_speeds()
                           tau = Kp*(q_d - q) - Kd*qd    ← PD 토크 계산
                           tau += C(q, qd)                ← 코리올리 보상
                           tau = clamp(tau, max_tau)       ← 토크 제한
                           direct_torque(tau)              ← 토크 인가 (중력 자동보상)
                                     │
                                     ▼
                           RTDE output registers 0..5 ──→ Python 모니터링
```

### RTDE 레지스터 맵

| 레지스터 | 인덱스 | 방향 | 용도 |
|---------|--------|------|------|
| Input 0-5 | q_desired | Python→UR | 목표 관절 위치 (rad) |
| Input 6-11 | Kp | Python→UR | PD 위치 강성 (Nm/rad) |
| Input 12-17 | Kd | Python→UR | PD 속도 감쇠 (Nm·s/rad) |
| Input 18 | mode | Python→UR | 0=대기, 1=활성, -1=정지 |
| Input 19 | coriolis | Python→UR | 코리올리 보상 ON/OFF |
| Output 0-5 | torque | UR→Python | 실제 인가 토크 (Nm) |
| Output 6 | heartbeat | UR→Python | URScript 생존 확인 |

### PD 게인 프리셋

관절마다 **관성이 다르므로** 게인도 관절별로 다르다 (어깨 > 손목):

| 프리셋 | Kp (Nm/rad) | Kd (Nm·s/rad) | 특성 |
|--------|-------------|---------------|------|
| STIFF | [800,800,400,200,100,50] | [40,40,20,10,5,2.5] | 정밀 추적 |
| MEDIUM | [400,400,200,100,50,25] | [20,20,10,5,2.5,1.25] | 균형 |
| **SOFT** (기본) | [100,100,50,25,12.5,6.25] | [10,10,5,2.5,1.25,0.625] | 안전 우선 |

런타임 게인 조절: `[`/`]` 키로 전체 게인 0.25배~2.0배 스케일링

### 핵심 코드 — URScript PD 루프 (`scripts/impedance_pd.script`)

```python
# 500Hz 루프 (URScript, 로봇 컨트롤러에서 실행)
while True:
    mode = read_input_float_register(18)
    if mode > 0.5:
        q_d = [read_input_float_register(i) for i in range(6)]  # 목표 위치
        kp  = [read_input_float_register(i) for i in range(6,12)]  # 강성
        kd  = [read_input_float_register(i) for i in range(12,18)] # 감쇠
        q   = get_actual_joint_positions()
        qd  = get_actual_joint_speeds()

        # PD 토크: τ = Kp*(q_d - q) - Kd*q̇
        tau[i] = kp[i] * (q_d[i] - q[i]) - kd[i] * qd[i]

        # 코리올리/원심력 보상 (선택)
        if use_coriolis:
            coriolis = get_coriolis_and_centrifugal_torques(q, qd)
            tau[i] += coriolis[i]

        # 토크 제한 후 인가 (중력은 자동 보상)
        direct_torque(clamp(tau, max_tau), friction_comp=True)
```

### 핵심 코드 — Python 제어 루프 (`teleop_impedance/main.py` 요약)

```python
mgr = URScriptManager(robot_ip)
mgr.connect()
mgr.set_gains(impedance.Kp.tolist(), impedance.Kd.tolist())
mgr.upload_and_start()         # URScript 업로드 → 500Hz PD 시작
mgr.set_mode(1.0)              # 활성화

while running:
    cmd = input_handler.get_command()
    target_pos += cmd.velocity[:3]
    filt_pos, filt_quat = exp_filter.update(...)
    clamped_pos = safety.clamp_workspace(filt_pos)
    q_desired = ik.solve(clamped_pos, filt_quat, dt)
    # URScript가 500Hz로 토크 계산 — Python은 목표만 전달
    mgr.set_desired_position(q_desired.tolist())
```

### 실행

```bash
cd /workspaces/tamp_ws/src/tamp_dev
# Sim (위치 제어 폴백, 토크 없음)
python3 -m standalone.teleop_impedance.main --mode sim --input keyboard

# 실제 로봇 (PolyScope 5.23.0+ 필수)
python3 -m standalone.teleop_impedance.main --mode rtde --input keyboard --robot-ip 192.168.0.2
```

---

## 8. 학습 로드맵

### 단계별 학습 계획

| 단계 | 모듈 | 학습 목표 | 이해해야 할 핵심 개념 |
|------|------|----------|---------------------|
| **0** | `config.py` + `core/robot_backend.py` | 프로젝트 구조 이해 | ABC, Factory, Context Manager |
| **1** | `servo/keyboard_forward.py` | 가장 단순한 제어 루프 | Joint-space 직접 제어 |
| **2** | `servo/keyboard_cartesian.py` | Cartesian 제어 이해 | FK, Jacobian, DLS IK |
| **3** | `core/trajectory_executor.py` | 궤적 개념 | 리샘플링, 실시간 스트리밍 |
| **4** | `cumotion/planner.py` | GPU 모션 플래닝 | curobo MotionGen, 충돌 회피 |
| **5** | `core/pink_ik.py` | QP 기반 IK | Task, Constraint, proxqp |
| **6** | `core/exp_filter.py` + `core/input_handler.py` | 신호 처리 | EMA, slerp, 입력 추상화 |
| **7** | `teleop_admittance/main.py` | 파이프라인 텔레옵 | Safety Monitor, Pipeline 패턴 |
| **8** | `core/compliant_control.py` + `core/ft_source.py` | 힘 제어 기초 | Admittance, M-D-K 모델 |
| **9** | `teleop_impedance/impedance_gains.py` | 임피던스 이론 | PD 토크, 관절별 게인 |
| **10** | `teleop_impedance/urscript_manager.py` + `impedance_pd.script` | 듀얼 루프 | RTDE 레지스터, URScript |

### 선수 지식

- **Python**: 클래스, ABC, 데코레이터, numpy 기초
- **로봇 기초**: 관절(joint), 순운동학(FK), 역운동학(IK), Jacobian 개념
- **ROS2 기초** (sim 모드만): 토픽(publish/subscribe), 서비스(service call)
- **제어 이론 기초** (teleop_impedance): PD 제어, 토크, 관성

---

## 9. 용어 사전

| 용어 | 설명 |
|------|------|
| **ABC** | Abstract Base Class — 인터페이스 정의용 추상 클래스 |
| **Admittance** | 힘 입력 → 위치 출력 제어 (F→x, 위치 제어 기반) |
| **DLS** | Damped Least Squares — 특이점 근처에서 안정적인 역기구학 |
| **EMA** | Exponential Moving Average — 지수 이동 평균 (노이즈 필터) |
| **F/T** | Force/Torque — 힘/토크 센서 |
| **FK** | Forward Kinematics — 관절 각도 → 말단 위치 계산 |
| **Impedance** | 위치 입력 → 토크 출력 제어 (x→τ, 토크 제어 기반) |
| **IK** | Inverse Kinematics — 말단 위치 → 관절 각도 계산 |
| **Jacobian** | 관절 속도와 말단 속도의 관계 행렬 (6×N) |
| **QP** | Quadratic Programming — 제약 조건 하 이차 최적화 |
| **RTDE** | Real-Time Data Exchange — UR 로봇과의 125Hz 실시간 통신 프로토콜 |
| **servoJ** | UR 로봇의 실시간 관절 위치 명령 함수 |
| **slerp** | Spherical Linear Interpolation — 쿼터니언(회전) 보간 |
| **URScript** | UR 로봇 컨트롤러에서 실행되는 스크립트 언어 |
| **XRDF** | cuMotion 로봇 충돌 모델 파일 형식 |
