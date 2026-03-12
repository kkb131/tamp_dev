# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 프로젝트 개요

UR10e 로봇 제어를 위한 standalone Python 패키지 + Isaac ROS cuMotion GPU 모션 플래닝 + UR ROS2 Driver.

- **Workspace**: `/workspaces/tamp_ws`
- **소스**: `/workspaces/tamp_ws/src/tamp_dev/`
- **ROS2**: Humble | **OS**: Ubuntu 22.04 (NVIDIA Isaac ROS base image)
- **빌드 아티팩트**: `.docker/build`, `.docker/install`, `.docker/log` (컨테이너 재시작 후에도 유지)

## 빌드

```bash
cd /workspaces/tamp_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 사용 시 Python 파일 수정이 rebuild 없이 즉시 반영됨.

## 아키텍처

### 패키지 구조

```
standalone/                        # 핵심 Python 패키지 (MoveIt 독립)
├── config.py                      # 공유 설정 (JOINT_NAMES, URDF_PATH, controller 상수)
├── core/                          # 공유 인프라 (2개+ 기능 모듈이 사용)
├── cumotion/                      # GPU 모션 플래닝 (curobo)
├── servo/                         # 간단 teleop (Pinocchio DLS IK)
├── teleop_admittance/             # 어드미턴스 텔레옵 (Pink QP IK + F/T)
└── teleop_impedance/              # 임피던스 텔레옵 (URScript PD 토크)
```

외부 의존:
- `cumotion/isaac_ros_cumotion/` — Isaac ROS cuMotion 소스 (release-3.2)
- `ur/` — Universal Robots ROS2 Driver (Humble branch)

### 모듈 의존 규칙

- 기능 모듈(servo, teleop_admittance, teleop_impedance, cumotion)은 `standalone.core.*`와 `standalone.config`에서만 import
- 기능 모듈 간 상호 import 금지
- 2개+ 기능이 공유하는 유틸은 `core/`로 승격

### 제어 파이프라인

**어드미턴스 텔레옵** (`teleop_admittance/main.py`):
```
Input → ExpFilter(EMA+slerp) → Workspace Clamp → Admittance(F/T) → Pink IK(QP) → SafetyMonitor → servoJ
```

**임피던스 텔레옵** (`teleop_impedance/main.py`):
```
Input → ExpFilter → Pink IK(QP) → q_desired → Python PD(Kp·Δq - Kd·qd + C) → RTDE registers → URScript(500Hz)
```

### 두 가지 IK 솔버

- **PinocchioIK** (`core/kinematics.py`): Damped Least Squares. 상태 없음(stateless). `servo/` 모듈에서 사용. 매 루프 q_actual에서 출발하므로 drift 없음.
- **PinkIK** (`core/pink_ik.py`): QP 기반 velocity IK. 내부 상태(`config.q`) 유지. `teleop_admittance/`, `teleop_impedance/`에서 사용. Cartesian 목표 pose 추적 가능하나 drift 가능성 있음 → `soft_sync()`로 보정.

### IK drift 방지 메커니즘

Pink IK는 내부 `config.q`를 누적 적분하므로 실제 로봇 상태와 괴리 발생 가능:
- `soft_sync(q_actual, alpha)`: 매 루프 `config.q`를 α%만큼 실제 상태로 블렌드 (기본 α=0.05). IK lead-ahead를 보존하면서 drift 보정.
- `sync_configuration(q)`: 하드 리셋. e-stop 해제 등 비상 시에만 사용. 매 루프 사용하면 safety velocity limiter와 충돌하여 오리엔테이션 모션 불가.
- `soft_sync_alpha`는 `config/default.yaml`의 `ik:` 섹션에서 조절 가능.

### YAML 설정 구조

각 teleop 모듈은 `config/default.yaml`을 가지며, 동일한 구조:
```yaml
ik:
  position_cost, orientation_cost, posture_cost  # Pink FrameTask/PostureTask 가중치
  damping                                         # QP solver damping
  soft_sync_alpha                                 # IK drift 보정 블렌드율
safety:
  packet_timeout_ms, max_joint_vel, workspace     # 안전 제한
```

## Standalone 실행

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# cuMotion standalone (GPU 필요)
python3 -m standalone.cumotion.test_standalone --plan-only

# Servo
python3 -m standalone.servo.keyboard_cartesian --mode sim

# Teleop 어드미턴스
python3 -m standalone.teleop_admittance.main --mode sim --input keyboard
python3 -m standalone.teleop_admittance.main --mode rtde --input xbox

# Teleop 임피던스 (PolyScope 5.23.0+)
python3 -m standalone.teleop_impedance.main --mode sim --input keyboard
python3 -m standalone.teleop_impedance.main --mode rtde --input keyboard --robot-ip 192.168.0.2
```

## cuMotion ROS2 스택 실행 (UR10e + Mock Hardware)

**3개 터미널 순서대로 실행:**

```bash
# Terminal 1 - UR10e Mock Hardware Driver
source /workspaces/tamp_ws/install/setup.bash
ros2 launch ur_robot_driver ur10e.launch.py use_fake_hardware:=true robot_ip:=0.0.0.0

# Terminal 2 - MoveIt2 + RViz (Terminal 1 시작 후)
source /workspaces/tamp_ws/install/setup.bash
ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e

# Terminal 3 - cuMotion Planner Node (move_group 시작 후)
source /workspaces/tamp_ws/install/setup.bash
XRDF=/workspaces/tamp_ws/install/isaac_ros_cumotion_robot_description/share/isaac_ros_cumotion_robot_description/xrdf/ur10e.xrdf
URDF=/workspaces/tamp_ws/src/tamp_dev/.docker/assets/ur10e.urdf
ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \
  cumotion_planner.robot:=${XRDF} cumotion_planner.urdf_path:=${URDF}
```

## Docker 이미지

```bash
cd /workspaces/tamp_ws/src/tamp_dev/docker
./build_image.sh               # amd64 자동 감지
./build_image.sh --arch arm64  # Jetson AGX Orin
./build_image.sh --no-cache    # 캐시 무시 재빌드
```

## 중요 사항 / Gotchas

**Pink IK 설치**: `pip install pin-pink proxsuite`. `pip install pink`은 코드 포맷터 — 절대 다른 패키지.

**numpy < 2 필수**: ROS Humble pinocchio가 numpy 1.x로 컴파일됨.

**RTDEControlInterface 사용 금지** (임피던스 모드): 우리 UR10e에서 연결 시 hang. 대신 RTDEIOInterface + TCP socket(port 30002) 조합 사용.

**cuMotion GPU 필수**: `nvidia-smi` 정상 출력 필요. NVIDIA Runtime 없으면 `RuntimeError: No CUDA GPUs are available`.

**ISAAC_ROS_WS 환경 변수**: `devcontainer.json`의 `containerEnv`에 설정됨. 직접 실행 시 `export ISAAC_ROS_WS=/workspaces/tamp_ws`.

**Mock hardware**: `use_fake_hardware:=true` 시 `joint_trajectory_controller` 자동 활성화. 수동 전환 불필요.

**Cartesian 플래닝**: `link_name="tool0"`, `PositionConstraint` + `OrientationConstraint` 둘 다 필수.

## 세션 영속화

`/root/.claude/` → `.docker/claude/`에 bind mount. 컨테이너 재시작 후에도 Claude Code 히스토리 유지.
