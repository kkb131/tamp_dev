# cuMotion + UR10e ROS2 스택 가이드

## 개요

NVIDIA Isaac ROS cuMotion을 UR10e 로봇과 함께 사용하기 위한 ROS2 스택 가이드입니다.
cuMotion은 CUDA GPU를 사용해 실시간 모션 플래닝을 수행하며, MoveIt2의 기본 플래닝 파이프라인으로 통합됩니다.

**버전**: isaac_ros_cumotion release-3.2
**ROS2**: Humble
**OS**: Ubuntu 22.04 (NVIDIA Isaac ROS base image)

> **참고**: MoveIt 독립 standalone 모듈도 제공됩니다. `standalone/cumotion/docs/user_guide.md` 참조.

---

## 목차

1. [사전 요구사항](#1-사전-요구사항)
2. [빌드](#2-빌드)
3. [실행](#3-실행)
4. [트러블슈팅](#4-트러블슈팅)
5. [알려진 버그 및 수정](#5-알려진-버그-및-수정)
6. [Teleop Admittance](#6-teleop-admittance)

---

## 1. 사전 요구사항

### 1.1 하드웨어

- **NVIDIA GPU 필수** (CUDA 지원) — cuMotion은 GPU 없이 동작하지 않습니다.
- 확인 방법:

```bash
nvidia-smi
```

`Unknown Error` 또는 디바이스 없음이 출력되면 컨테이너를 NVIDIA Runtime으로 재시작해야 합니다.

### 1.2 컨테이너 실행 요구사항

컨테이너는 반드시 NVIDIA GPU 접근 권한으로 실행되어야 합니다.

**VS Code devcontainer** (`.devcontainer/devcontainer.json`):
```json
"runArgs": ["--gpus=all", "--network=host", "--ipc=host"]
```

**직접 실행** (`docker/run_container.sh`):
```bash
./docker/run_container.sh  # --gpus=all 포함됨
```

### 1.3 필수 ROS2 패키지

Dockerfile에 포함되어 있으나, 이미지 재빌드 전이라면 수동 설치:

```bash
apt-get install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ur-msgs \
  ros-humble-moveit \
  ros-humble-curobo-core \
  ros-humble-isaac-ros-cumotion-interfaces \
  ros-humble-isaac-manipulator-ros-python-utils \
  ros-humble-nvblox-msgs
```

### 1.4 환경 변수

```bash
export ISAAC_ROS_WS=/workspaces/tamp_ws
```

> devcontainer.json에 설정되어 있어 컨테이너 시작 시 자동 적용됩니다.

---

## 2. 빌드

```bash
cd /workspaces/tamp_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> **참고**: `colcon build` 중 `LookupError: Could not find the resource 'isaac_ros_common'` 오류 또는 `curobo_core` 빌드 실패 시 [4.6 colcon build 오류](#46-colcon-build-오류) 참조.

---

## 3. 실행

3개의 터미널(또는 백그라운드 프로세스)이 필요합니다.

### Terminal 1 — UR10e 드라이버 (Mock Hardware)

```bash
source /workspaces/tamp_ws/install/setup.bash
ros2 launch ur_robot_driver ur10e.launch.py \
  use_fake_hardware:=true \
  robot_ip:=0.0.0.0
```

> 실제 로봇 사용 시 `robot_ip:=<ROBOT_IP>`로 변경하고 `use_fake_hardware:=true` 제거.

**확인**: `/joint_states` 토픽이 발행되는지 확인:
```bash
ros2 topic echo /joint_states --once
```

### Terminal 2 — MoveIt2 + RViz

```bash
source /workspaces/tamp_ws/install/setup.bash
ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e
```

**확인**: 로그에 `You can start planning now!` 출력.

> 기본 planning pipeline: `isaac_ros_cumotion`
> ```bash
> ros2 param get /move_group default_planning_pipeline
> # 출력: isaac_ros_cumotion
> ```

### Terminal 3 — cuMotion Planner Node

```bash
source /workspaces/tamp_ws/install/setup.bash
XRDF=$(ros2 pkg prefix isaac_ros_cumotion_robot_description)/share/isaac_ros_cumotion_robot_description/xrdf/ur10e.xrdf
URDF=/workspaces/tamp_ws/src/tamp_dev/.docker/assets/ur10e.urdf

ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \
  cumotion_planner.robot:=${XRDF} \
  cumotion_planner.urdf_path:=${URDF}
```

**확인**: 로그에 `cuMotion is ready for planning queries!` 출력 (약 5~10초 소요).

### Mock Hardware에서 실행(Execution) 설정

Mock hardware에서 실행이 정상 동작하려면 두 가지가 동시에 맞아야 합니다:

| 레이어 | 필요 상태 | 처리 방식 |
|--------|-----------|-----------|
| **ros2_control** | `joint_trajectory_controller` active | `ur_control.launch.py`가 `use_fake_hardware:=true` 시 자동 처리 |
| **MoveIt2** | `joint_trajectory_controller.default: true` | `moveit_controllers.yaml`에 이미 적용됨 |

두 설정 모두 이미 적용되어 있습니다. Terminal 1~3을 순서대로 기동하면 됩니다.

```bash
# 확인: ros2_control에서 joint_trajectory_controller가 active 상태인지 검증
ros2 control list_controllers | grep trajectory
# 예상 출력:
# joint_trajectory_controller[active]
# scaled_joint_trajectory_controller[inactive]
```

### Cartesian 플래닝 참고

`cumotion_planner.py`는 Joint-Space 목표와 Cartesian 목표 모두를 지원합니다.

| 목표 유형 | 인식 조건 | 내부 함수 |
|-----------|-----------|-----------|
| Joint-Space | `joint_constraints` 존재 | `plan_single_js()` |
| Cartesian | `position_constraints` + `orientation_constraints` 모두 존재 | `plan_single()` |

**필수 조건**:
- `PositionConstraint.link_name` = `"tool0"` (cuMotion XRDF의 ee_link)
- `OrientationConstraint.link_name` = `"tool0"`
- 두 constraint가 **모두** 있어야 Cartesian 경로로 인식됨

---

## 4. 트러블슈팅

### 4.1 `RuntimeError: No CUDA GPUs are available`

```
[ERROR] [cumotion_goal_set_planner_node-2]: process has died
RuntimeError: No CUDA GPUs are available
```

**원인**: 컨테이너가 NVIDIA Runtime 없이 시작됨.
**해결**:

```bash
# 확인
nvidia-smi  # 정상 출력되어야 함

# VS Code: Ctrl+Shift+P → "Dev Containers: Rebuild Container"
# 또는 직접 실행:
./docker/run_container.sh
```

### 4.2 `No module named 'curobo'` / `'nvblox_msgs'` 등

```
ModuleNotFoundError: No module named 'curobo'
```

**원인**: 필수 패키지 미설치 (이미지 재빌드 필요).
**해결**:
```bash
apt-get update && apt-get install -y \
  ros-humble-curobo-core \
  ros-humble-nvblox-msgs \
  ros-humble-isaac-ros-cumotion-interfaces \
  ros-humble-isaac-manipulator-ros-python-utils
```

### 4.3 `ISAAC_ROS_WS environment variable is not set`

```
RuntimeError: ISAAC_ROS_WS environment variable is not set
```

**해결**:
```bash
export ISAAC_ROS_WS=/workspaces/tamp_ws
```

> 영구 적용: `.devcontainer/devcontainer.json`의 `containerEnv`에 이미 설정되어 있습니다.

### 4.4 `No trajectory` (MoveItErrorCode=-1)

MoveIt2 로그에서:
```
No trajectory
Planner 'Generate minimum-jerk trajectories using NVIDIA Isaac ROS cuMotion' failed with error code PLANNING_FAILED
```

**원인**: rclpy 7.1.9의 action server 버그 — `goal_handle.succeed()`가 result 없이 호출되어 빈 결과 전송.
**해결**: `cumotion_planner.py`의 `execute_callback`에서 `goal_handle.succeed(result)` 패치 적용 (이미 수정됨).
→ [알려진 버그 및 수정](#5-알려진-버그-및-수정) 참조.

### 4.5 `CONTROL_FAILED` (MoveItErrorCode=-4)

```
Goal request rejected
Failed to send trajectory part 1 of 1 to controller scaled_joint_trajectory_controller
CONTROL_FAILED
```

**원인**: ros2_control과 MoveIt2의 컨트롤러 설정이 불일치할 때 발생. 두 레이어 모두 동시에 맞아야 함.

| 레이어 | 올바른 상태 (mock hw) | 확인 방법 |
|--------|----------------------|-----------|
| ros2_control | `joint_trajectory_controller` active | `ros2 control list_controllers` |
| MoveIt2 | `moveit_controllers.yaml`에서 `joint_trajectory_controller.default: true` | 파일 직접 확인 |

**해결 Step 1** — `moveit_controllers.yaml` 확인 및 수정:

파일 위치: `ur/Universal_Robots_ROS2_Driver/ur_moveit_config/config/moveit_controllers.yaml`

아래와 같이 설정되어 있어야 합니다 (mock hardware용):

```yaml
scaled_joint_trajectory_controller:
  default: false  # real robot: true / mock hardware: false

joint_trajectory_controller:
  default: true   # real robot: false / mock hardware: true
```

`scaled: true` / `joint: false`로 되어있다면 수정 후 **Terminal 2 재시작**.

**해결 Step 2** — yaml이 올바른데도 발생하면 Terminal 1 재시작:
```bash
ros2 launch ur_robot_driver ur10e.launch.py use_fake_hardware:=true robot_ip:=0.0.0.0
```

> **실제 로봇 전환 시**: `moveit_controllers.yaml`에서 `scaled: true`, `joint: false`로 변경 후 MoveIt 재시작 필요.

### 4.6 `colcon build` 오류

#### 4.6.1 `LookupError: Could not find the resource 'isaac_ros_common'`

```
LookupError: Could not find the resource 'isaac_ros_common' of type 'isaac_ros_common_scripts_path'
Summary: 2 packages failed: curobo_core isaac_ros_cumotion_python_utils
```

**원인**: `curobo_core`와 `isaac_ros_cumotion_python_utils`의 `setup.py`가 빌드 시 `isaac_ros_common` 패키지를 필요로 하지만, 이 devcontainer에는 설치되어 있지 않음 (Docker 이미지에서 사전 빌드됨).

**해결**: `/opt/ros/humble`에 stub 파일 생성:

```bash
# Python stub
mkdir -p /opt/ros/humble/share/ament_index/resource_index/isaac_ros_common_scripts_path
mkdir -p /opt/ros/humble/share/isaac_ros_common/scripts
echo -n "/opt/ros/humble/share/isaac_ros_common/scripts" \
  > /opt/ros/humble/share/ament_index/resource_index/isaac_ros_common_scripts_path/isaac_ros_common

cat > /opt/ros/humble/share/isaac_ros_common/scripts/isaac_ros_common-version-info.py << 'EOF'
from setuptools.command.build_py import build_py
class GenerateVersionInfoCommand(build_py):
    description = 'build Python files (stub: skips version info)'
    user_options = build_py.user_options
    def run(self): super().run()
EOF

# CMake stub
mkdir -p /opt/ros/humble/share/ament_index/resource_index/isaac_ros_common_cmake_path
mkdir -p /opt/ros/humble/share/isaac_ros_common/cmake
echo -n "/opt/ros/humble/share/isaac_ros_common/cmake" \
  > /opt/ros/humble/share/ament_index/resource_index/isaac_ros_common_cmake_path/isaac_ros_common

cat > /opt/ros/humble/share/isaac_ros_common/cmake/isaac_ros_common-version-info.cmake << 'EOF'
macro(generate_version_info package_name)
endmacro()
EOF
```

> **참고**: 이 stub 파일들은 devcontainer 재빌드 시 사라집니다. 재빌드 후 다시 생성 필요.

#### 4.6.2 `curobo_core` 빌드 실패 (`egg_base: 'curobo/src' does not exist`)

```
error: error in 'egg_base' option: 'curobo/src' does not exist or is not a directory
Summary: 1 package failed: curobo_core
```

**원인**: `curobo_core/curobo/src`가 비어있음. 실제 cuRobo 라이브러리는 `/opt/ros/humble`에 사전 설치되어 있으며 이 workspace에서 재빌드할 수 없음.

**해결**: `curobo_core` 디렉토리에 `COLCON_IGNORE` 파일 추가 (이미 적용됨):

```bash
# 이미 적용됨 — 확인만:
ls cumotion/isaac_ros_cumotion/curobo_core/COLCON_IGNORE
```

### 4.7 여러 개의 `/move_action` 서버 경고

```
Ignoring unexpected goal response. There may be more than one action server for the action '/move_action'
```

**원인**: MoveIt2 런치 프로세스가 중복 실행됨.
**해결**:
```bash
# 중복 move_group 프로세스 확인 후 종료
ps aux | grep move_group
kill <중복_PID>
```

---

## 5. 알려진 버그 및 수정

### 5.1 cuMotion planner `goal_handle.succeed()` 조기 호출 버그

**파일**: `cumotion/isaac_ros_cumotion/isaac_ros_cumotion/isaac_ros_cumotion/cumotion_planner.py`
**증상**: cuMotion이 내부적으로 계획 성공(`success: True`)을 반환하지만, MoveIt2가 `No trajectory`를 수신함.

**원인**:
rclpy 7.1.9에서 `ServerGoalHandle.succeed(response=None)` 호출 시:
1. goal 상태를 SUCCEEDED로 변경
2. `_set_result(None)`으로 **즉시 빈 result** future를 resolve
3. action client(MoveIt)가 빈 `MoveGroup.Result()`를 받음
4. 실제 trajectory가 담긴 return value는 이미 resolve된 future에 전달 불가

원래 코드 (버그):
```python
goal_handle.succeed()   # ← trajectory 계산 전에 호출됨 (line 754)

# ... trajectory 계산 (약 300~800ms 소요) ...

return result  # ← 이미 빈 결과가 전송된 후라 무시됨
```

**수정**:
```python
# 1. 조기 succeed() 제거
# 2. trajectory 계산 후 result를 직접 전달

motion_gen_result = self.motion_gen.plan_single(...)

result = MoveGroup.Result()
if motion_gen_result.success.item():
    result.error_code.val = MoveItErrorCodes.SUCCESS
    result.planned_trajectory = traj
    goal_handle.succeed(result)    # ← result 직접 전달
else:
    goal_handle.abort(result)

return result
```

오류 케이스(조기 return)에도 `goal_handle.abort()` 추가:
```python
if not world_update_status:
    goal_handle.abort()    # ← 추가
    return result

if self.__js_buffer is None:
    goal_handle.abort()    # ← 추가
    return result
```

**적용 파일**: `cumotion/isaac_ros_cumotion/isaac_ros_cumotion/isaac_ros_cumotion/cumotion_planner.py`

### 5.2 `moveit_controllers.yaml` 컨트롤러 기본값 불일치

**파일**: `ur/Universal_Robots_ROS2_Driver/ur_moveit_config/config/moveit_controllers.yaml`

**증상**: `MoveItErrorCode=-4 (CONTROL_FAILED)` — mock hardware에서 첫 번째 goal부터 실패

**원인**:
`ur_control.launch.py`는 `use_fake_hardware:=true` 시 자동으로 컨트롤러를 교체합니다:
- `scaled_joint_trajectory_controller` → **inactive** (mock hw에서 `speed_scaling_interface` 미지원)
- `joint_trajectory_controller` → **active**

그런데 `moveit_controllers.yaml`이 실제 로봇 기본값(`scaled: default: true`)으로 남아있으면 MoveIt2가 inactive 컨트롤러로 trajectory를 전송 → CONTROL_FAILED.

**수정** (mock hardware용):
```yaml
scaled_joint_trajectory_controller:
  default: false  # real robot: true / mock hardware: false

joint_trajectory_controller:
  default: true   # real robot: false / mock hardware: true
```

이후 Terminal 2 재시작 (Terminal 1, 3 유지).

> **실제 로봇 전환 시**: `scaled: true`, `joint: false`로 복원 후 Terminal 2 재시작.

---

## 6. Teleop Admittance

### 6.1 개요

`standalone/teleop_admittance/`는 F/T 센서 기반 컴플라이언트 Cartesian 텔레옵 모듈입니다.
키보드 또는 Xbox 컨트롤러로 엔드이펙터를 제어하며, 선택적으로 F/T 어드미턴스를 활성화하여 외력에 순응하는 동작이 가능합니다.

**제어 파이프라인**:
```
Input → ExpFilter → Workspace Clamp → Admittance(F/T) → Pink IK → SafetyMonitor → servoJ
```

| 모드 | 주파수 | 백엔드 | 용도 |
|------|--------|--------|------|
| `sim` | 50 Hz | ROS2 mock hardware | 시뮬레이션 테스트 |
| `rtde` | 125 Hz | ur_rtde `servoJ` | 실제 로봇 제어 |

### 6.2 사전 요구사항

**Python 의존성**:

```bash
pip install pin-pink proxsuite
```

> **주의**: `pip install pink`는 코드 포매터입니다. 반드시 `pin-pink`를 설치해야 합니다.

> **주의**: numpy 2.x와 호환되지 않습니다. `pip install "numpy<2"` 필요.

**URDF**: `.docker/assets/ur10e.urdf` (자동 참조됨)

**sim 모드**: Terminal 1 (UR mock hardware 드라이버)이 실행 중이어야 합니다. → [3. 실행](#3-실행) 참조.

### 6.3 설정

설정 파일: `standalone/teleop_admittance/config/default.yaml`

CLI 인자로 주요 설정을 오버라이드할 수 있으며, 전체 커스텀 YAML 파일도 지정 가능합니다.

| 섹션 | 주요 설정 | 기본값 |
|------|-----------|--------|
| `robot` | `mode`, `ip` | `sim`, `192.168.0.2` |
| `control` | `frequency_sim`, `frequency_rtde` | 50 Hz, 125 Hz |
| `input` | `type`, `cartesian_step`, `rotation_step` | `keyboard`, 0.01 m, 0.05 rad |
| `filter` | `alpha_position`, `alpha_orientation` | 0.85, 0.85 |
| `ik` | `position_cost`, `orientation_cost`, `posture_cost`, `damping` | 1.0, 0.5, 1e-3, 1e-12 |
| `safety` | `packet_timeout_ms`, `max_joint_vel` | 200 ms, 0.5 rad/s |
| `safety.workspace` | `x`, `y`, `z` 범위 | [-0.8, 0.8], [-0.8, 0.8], [0.05, 1.2] m |
| `admittance` | `enabled_by_default`, `default_preset` | `false`, `MEDIUM` |

### 6.4 실행

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# sim 모드 + 키보드 (기본)
python3 -m standalone.teleop_admittance.main --mode sim --input keyboard

# 실제 로봇 + Xbox 컨트롤러
python3 -m standalone.teleop_admittance.main --mode rtde --input xbox --robot-ip 192.168.0.2

# 커스텀 설정 + CSV 로깅
python3 -m standalone.teleop_admittance.main --config my_config.yaml --log
```

| 인자 | 설명 | 기본값 |
|------|------|--------|
| `--mode {sim\|rtde}` | 백엔드 모드 | config 파일 값 |
| `--input {keyboard\|xbox}` | 입력 장치 | config 파일 값 |
| `--robot-ip IP` | 로봇 IP (rtde 모드) | `192.168.0.2` |
| `--config PATH` | 커스텀 YAML 설정 파일 | `config/default.yaml` |
| `--log` | CSV 로깅 활성화 | 비활성 |

> **참고**: 안전 시스템 없이 테스트하려면 `teleop_nosafety.py`를 사용할 수 있습니다:
> ```bash
> python3 -m standalone.teleop_admittance.teleop_nosafety --mode sim --input keyboard
> ```

### 6.5 키보드 조작

**이동 (Cartesian)**:

| 키 | 동작 | 키 | 동작 |
|----|------|----|------|
| `W` / `S` | 앞/뒤 (±Y) | `Q` / `E` | 위/아래 (±Z) |
| `A` / `D` | 좌/우 (±X) | | |

**회전**:

| 키 | 동작 | 키 | 동작 |
|----|------|----|------|
| `U` / `O` | Roll ± | `I` / `K` | Pitch ± |
| `J` / `L` | Yaw ± | | |

**기타**:

| 키 | 동작 |
|----|------|
| `+` / `-` | 속도 스케일 조절 (0.5x ~ 8.0x) |
| `Space` | E-Stop 발동 |
| `R` | E-Stop 해제 |
| `T` | 어드미턴스 ON/OFF 토글 |
| `Z` | F/T 센서 영점 보정 |
| `1` / `2` / `3` / `4` | 어드미턴스 프리셋 (STIFF / MEDIUM / SOFT / FREE) |

### 6.6 안전 시스템

`SafetyMonitor`는 4단계 안전 검사를 수행합니다. 우선순위: Level 4 > 1 > 2 > 3.

| Level | 이름 | 트리거 조건 | 동작 | 복구 |
|-------|------|-------------|------|------|
| **1** | Packet Timeout | 입력 없음 > 200 ms | 현재 위치 유지 (hold) | 입력 재개 시 자동 복구 |
| **2** | Velocity Limit | 관절 속도 > 0.5 rad/s | 속도 스케일링 (비례 축소) | 자동 (축소 적용) |
| **3** | Workspace Clamp | EE 위치가 x/y/z 범위 초과 | 범위 내로 클램핑 | 자동 (범위 내 복귀) |
| **4** | E-Stop | `Space` 키 입력 | `emergency_stop()` 호출 | `R` 키로 수동 해제 |

터미널 상태 표시에서 현재 안전 상태를 실시간으로 확인할 수 있습니다.

### 6.7 어드미턴스 제어

F/T 센서에서 측정된 외력을 기반으로 EE에 변위를 추가하는 컴플라이언스 제어입니다.

**동역학 모델**: `M·ẍ + D·ẋ + K·x = f_ext` (가상 질량-댐퍼-스프링)

- `T` 키로 런타임 ON/OFF 토글 (rtde 모드에서만 실제 F/T 데이터 사용)
- `Z` 키로 F/T 센서 바이어스 보정 (zero calibration)
- `1`/`2`/`3` 키로 프리셋 전환:

| 프리셋 | 특성 | 용도 |
|--------|------|------|
| `STIFF` | 높은 강성, 낮은 순응성 | 정밀 위치 유지 |
| `MEDIUM` | 중간 강성 | 일반 작업 (기본값) |
| `SOFT` | 낮은 강성, 높은 순응성 | 섬세한 접촉 작업 |
| `FREE` | 강성 없음, 최대 순응성 | 자유 컴플라이언스 (핸드 가이딩) |

**주요 파라미터** (`admittance` 섹션):

| 파라미터 | 설명 | 기본값 |
|----------|------|--------|
| `max_displacement_trans` | 최대 병진 변위 | 0.05 m |
| `max_displacement_rot` | 최대 회전 변위 | 0.15 rad (~8.6°) |
| `force_deadzone` | 힘/토크 데드존 [N, Nm] | [3.0, 3.0, 3.0, 0.3, 0.3, 0.3] |
| `force_saturation` | 힘 포화 한계 (초과 시 리셋) | 100.0 N |
| `torque_saturation` | 토크 포화 한계 (초과 시 리셋) | 10.0 Nm |

> **참고**: sim 모드에서는 `NullFTSource`가 사용되어 어드미턴스를 활성화해도 변위가 0입니다. 실제 F/T 테스트는 rtde 모드에서 수행하세요.

### 6.8 트러블슈팅

#### `ModuleNotFoundError: No module named 'pink'`

**원인**: `pip install pink` (코드 포매터)를 설치한 경우.
**해결**:
```bash
pip uninstall pink
pip install pin-pink proxsuite
```

#### numpy 버전 충돌 (`ValueError` / segfault)

**원인**: pinocchio가 numpy 1.x로 컴파일되어 numpy 2.x와 ABI 비호환.
**해결**:
```bash
pip install "numpy<2"
```

#### 어드미턴스 활성화해도 반응 없음

**원인**: sim 모드에서는 F/T 센서가 `NullFTSource`(항상 0)로 대체됨.
**해결**: 정상 동작입니다. 실제 F/T 테스트는 `--mode rtde`로 실행하세요.

#### 컨트롤러 전환 실패 (sim 모드)

```
[WARN] controller_manager not available, skipping controller switch
```

**원인**: Terminal 1 (UR mock hardware 드라이버)이 실행되지 않았거나 아직 초기화 중.
**해결**: Terminal 1을 먼저 실행하고 `/joint_states` 토픽이 발행되는지 확인 후 재시도.

#### 순수 어드미턴스 테스트 (키보드 입력 없이)

외력만으로 로봇을 밀어 컴플라이언스를 테스트하려면:
```bash
python3 -m standalone.teleop_admittance.test_admittance --robot-ip 192.168.0.2
```
키: `1`/`2`/`3` (프리셋), `z` (영점+리셋), `q` (종료)

---

## 주요 파일 경로

| 항목 | 경로 |
|------|------|
| XRDF (ur10e) | `install/isaac_ros_cumotion_robot_description/share/.../xrdf/ur10e.xrdf` |
| URDF (ur10e) | `.docker/assets/ur10e.urdf` |
| cuMotion params | `cumotion/isaac_ros_cumotion/isaac_ros_cumotion/params/isaac_ros_cumotion_params.yaml` |
| UR launch | `install/ur_robot_driver/share/ur_robot_driver/launch/ur10e.launch.py` |
| cuMotion examples launch | `install/isaac_ros_cumotion_examples/share/.../launch/ur.launch.py` |
| cuMotion launch | `install/isaac_ros_cumotion/share/isaac_ros_cumotion/launch/isaac_ros_cumotion.launch.py` |
| cuMotion planner (수정됨) | `cumotion/isaac_ros_cumotion/isaac_ros_cumotion/isaac_ros_cumotion/cumotion_planner.py` |
| moveit_controllers.yaml | `ur/Universal_Robots_ROS2_Driver/ur_moveit_config/config/moveit_controllers.yaml` |
| teleop_admittance main | `standalone/teleop_admittance/main.py` |
| teleop_admittance 설정 | `standalone/teleop_admittance/config/default.yaml` |
| teleop_admittance 안전 | `standalone/teleop_admittance/safety_monitor.py` |

---

## ROS2 Action 인터페이스

| Action | 타입 | 설명 |
|--------|------|------|
| `/move_action` | `moveit_msgs/action/MoveGroup` | MoveIt2 메인 인터페이스 |
| `/cumotion/move_group` | `moveit_msgs/action/MoveGroup` | cuMotion 직접 인터페이스 (MoveIt2 → cuMotion) |
| `/cumotion/motion_plan` | `isaac_manipulator_msgs/action/MotionPlan` | cuMotion GoalSet 인터페이스 (Isaac Manipulator용) |
| `/cumotion/ik` | `isaac_ros_cumotion_interfaces/action/IKSolution` | IK 솔버 직접 인터페이스 |
