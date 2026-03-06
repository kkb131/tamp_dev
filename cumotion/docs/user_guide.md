# cuMotion + UR10e 사용자 가이드

## 개요

NVIDIA Isaac ROS cuMotion을 UR10e 로봇과 함께 사용하기 위한 가이드입니다.
cuMotion은 CUDA GPU를 사용해 실시간 모션 플래닝을 수행하며, MoveIt2의 기본 플래닝 파이프라인으로 통합됩니다.

**버전**: isaac_ros_cumotion release-3.2
**ROS2**: Humble
**OS**: Ubuntu 22.04 (NVIDIA Isaac ROS base image)

---

## 목차

1. [사전 요구사항](#1-사전-요구사항)
2. [빌드](#2-빌드)
3. [실행](#3-실행)
4. [테스트 (Joint-Space)](#4-테스트-joint-space)
5. [Cartesian 플래닝 테스트](#5-cartesian-플래닝-테스트)
6. [트러블슈팅](#6-트러블슈팅)
7. [알려진 버그 및 수정](#7-알려진-버그-및-수정)

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

> **참고**: `colcon build` 중 `LookupError: Could not find the resource 'isaac_ros_common'` 오류 또는 `curobo_core` 빌드 실패 시 [6.7 colcon build 오류](#67-colcon-build-오류) 참조.

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

---

## 4. 테스트

모든 노드가 기동된 후 실행합니다.

### 4.0 초기 위치 이동

테스트 전 로봇을 **초기 위치(home)**로 이동시킵니다. 모든 테스트 스크립트는 이 위치를 `home` 기준점으로 사용합니다.

```bash
source /workspaces/tamp_ws/install/setup.bash

# 계획만 검증 (기본값, 실행 없음):
python3 /workspaces/tamp_ws/src/tamp_dev/go_to_init_pose_mock.py

# 실행 포함:
python3 /workspaces/tamp_ws/src/tamp_dev/go_to_init_pose_mock.py --execute

# 속도 조정 (기본 50%):
python3 /workspaces/tamp_ws/src/tamp_dev/go_to_init_pose_mock.py --execute --velocity-scale 0.3
```

**초기 위치 (home)**:

| 관절 | 값 (rad) | 값 (°) |
|------|----------|--------|
| shoulder_pan | +2.2400 | +128.35° |
| shoulder_lift | −1.2808 | −73.37° |
| elbow | +2.1600 | +123.76° |
| wrist_1 | −0.8848 | −50.68° |
| wrist_2 | +2.2400 | +128.35° |
| wrist_3 | 0.0000 | 0.00° |

**예상 출력**:
```
[PLAN ONLY] 초기 위치: [+128.35°, -73.37°, +123.76°, -50.68°, +128.35°, +0.00°]
초기 위치 플래닝 완료.
```

> Mock hardware에서는 `--execute` 없이도 계획 검증이 가능합니다.
> 실행 시 안전 확인 프롬프트 없이 바로 동작합니다 (mock 전용).

---

### 4.1 Stage 1 — 경로 이동 테스트

장애물 없는 환경에서 `home → up → test_configuration → home` 경로를 계획합니다.

```bash
source /workspaces/tamp_ws/install/setup.bash

# 플래닝만 검증 (실행 안 함)
python3 /workspaces/tamp_ws/src/tamp_dev/test_motion_plan.py --plan-only

# 플래닝 + 실행 (controller 설정 필요, 아래 참조)
python3 /workspaces/tamp_ws/src/tamp_dev/test_motion_plan.py
```

**예상 출력**:
```
✓ OK    home
✓ OK    up
✓ OK    test_configuration
✓ OK    home
```

### 4.2 Stage 2 — 장애물 회피 테스트

장애물을 MoveIt 플래닝 씬에 추가한 후 cuMotion이 회피 경로를 계획하는지 검증합니다.

```bash
source /workspaces/tamp_ws/install/setup.bash

# 장애물 추가
python3 /workspaces/tamp_ws/src/tamp_dev/test_collision_objects.py

# 장애물 회피 경로 테스트 (plan-only)
python3 /workspaces/tamp_ws/src/tamp_dev/test_motion_plan.py --obstacle-test --plan-only

# 테스트 완료 후 장애물 제거
python3 /workspaces/tamp_ws/src/tamp_dev/test_collision_objects.py --clear
```

**장애물 배치**:
| 이름 | 위치 (m) | 크기 (m) |
|------|----------|----------|
| `table` | (0.0, 0.0, −0.025) | 2.0 × 2.0 × 0.05 |
| `obstacle_front` | (0.7, 0.0, 0.5) | 0.2 × 0.2 × 0.6 |
| `obstacle_right` | (0.3, −0.6, 0.4) | 0.15 × 0.15 × 0.4 |

**예상 출력**:
```
✓ OK    home
✓ OK    obstacle_test
✓ OK    home
```

### 4.3 Mock Hardware에서 실행(Execution) 활성화

Mock hardware에서 실행이 정상 동작하려면 두 가지가 동시에 맞아야 합니다:

| 레이어 | 필요 상태 | 처리 방식 |
|--------|-----------|-----------|
| **ros2_control** | `joint_trajectory_controller` active | `ur_control.launch.py`가 `use_fake_hardware:=true` 시 자동 처리 |
| **MoveIt2** | `joint_trajectory_controller.default: true` | `moveit_controllers.yaml`에 이미 적용됨 |

두 설정 모두 이미 적용되어 있습니다. Section 3의 Terminal 1~3을 순서대로 기동하면 됩니다.

> **배경**: MoveIt2의 `simple_controller_manager`는 `ros2 control` 활성 상태를 무시하고
> `moveit_controllers.yaml`의 `default: true` 컨트롤러를 사용합니다. 따라서 ros2_control과
> MoveIt 양쪽 모두 `joint_trajectory_controller`를 가리켜야 합니다.

```bash
# 확인: ros2_control에서 joint_trajectory_controller가 active 상태인지 검증
ros2 control list_controllers | grep trajectory
# 예상 출력:
# joint_trajectory_controller[active]
# scaled_joint_trajectory_controller[inactive]
```

**실행**:
```bash
python3 /workspaces/tamp_ws/src/tamp_dev/test_motion_plan.py
```

> **실제 로봇 전환 시**: `moveit_controllers.yaml`에서 `scaled_joint_trajectory_controller.default: true`,
> `joint_trajectory_controller.default: false`로 변경 후 MoveIt 재시작 필요.
> (실제 로봇은 `ur_control.launch.py`가 `scaled_joint_trajectory_controller`를 자동 활성화함)

---

## 5. Cartesian 플래닝 테스트

### 5.1 개요

`cumotion_planner.py`는 Joint-Space 목표와 **Cartesian 목표** 모두를 지원합니다.

| 목표 유형 | 인식 조건 | 내부 함수 |
|-----------|-----------|-----------|
| Joint-Space | `joint_constraints` 존재 | `plan_single_js()` |
| Cartesian | `position_constraints` + `orientation_constraints` 모두 존재 | `plan_single()` |

**필수 조건**:
- `PositionConstraint.link_name` = `"tool0"` (cuMotion XRDF의 ee_link)
- `OrientationConstraint.link_name` = `"tool0"`
- 두 constraint가 **모두** 있어야 Cartesian 경로로 인식됨

### 5.2 실행

모든 노드(Terminal 1~3)가 기동된 후 실행합니다.

```bash
source /workspaces/tamp_ws/install/setup.bash

# Joint 목표만 테스트 (기존 test_motion_plan.py와 동일한 시퀀스)
python3 /workspaces/tamp_ws/src/tamp_dev/test_motion_plan_cartesian.py --goal-type joint

# Cartesian 목표만 테스트 (plan-only)
python3 /workspaces/tamp_ws/src/tamp_dev/test_motion_plan_cartesian.py --goal-type cartesian

# Joint + Cartesian 모두 테스트
python3 /workspaces/tamp_ws/src/tamp_dev/test_motion_plan_cartesian.py --goal-type both

# 실행 포함 (controller 전환 필요, 4.3 참조)
python3 /workspaces/tamp_ws/src/tamp_dev/test_motion_plan_cartesian.py --goal-type cartesian --execute
```

**주요 옵션**:
| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--goal-type` | `cartesian` | `joint` / `cartesian` / `both` |
| `--execute` | 미지정(plan-only) | 지정 시 실행까지 수행 |
| `--delta-cm` | `5.0` | Cartesian 이동 거리 (cm) |
| `--velocity-scale` | `0.1` | 속도 스케일 (0.0~1.0) |

### 5.3 Cartesian 테스트 시퀀스

```
home (joint) → TF2 lookup → +5cm Z (Cartesian) → +5cm X (Cartesian) → home (joint)
```

1. `home` 위치로 Joint-Space 이동
2. TF2로 현재 `tool0` 포즈(`base_link` 기준) 조회
3. Z 방향 +5cm Cartesian 이동
4. X 방향 +5cm Cartesian 이동 (Z는 home 높이 유지)
5. `home`으로 복귀 (Joint-Space)

**예상 출력**:
```
[Joint Goals]
✓ OK    home
...

[Cartesian Goals]
✓ OK    home+5cm_z (Cartesian)
✓ OK    home+5cm_x (Cartesian)
✓ OK    home (return)
```

**cuMotion 로그 확인** (Terminal 3):
```
[INFO] Using goal from Pose  ← Cartesian 경로로 인식됨
```

### 5.4 트러블슈팅: `INVALID_LINK_NAME`

```
PLANNING_FAILED: orientation constraint link 'tool0' does not match cuMotion ee_link
```

**원인**: `link_name`이 `tool0`이 아닌 다른 이름(예: `ee_link`, `wrist_3_link`) 사용.
**해결**: 반드시 `link_name = "tool0"` 사용.

### 5.5 트러블슈팅: `TF2 lookup failed`

```
[WARN] TF2 lookup failed after 5.0s timeout
```

**원인**: robot_state_publisher 또는 joint_state_publisher가 TF를 발행하지 않음.
**해결**:
```bash
# TF 발행 확인
ros2 topic echo /tf --once
ros2 run tf2_tools view_frames
```

---

## 6. 트러블슈팅

### 6.1 `RuntimeError: No CUDA GPUs are available`

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

### 6.2 `No module named 'curobo'` / `'nvblox_msgs'` 등

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

### 6.3 `ISAAC_ROS_WS environment variable is not set`

```
RuntimeError: ISAAC_ROS_WS environment variable is not set
```

**해결**:
```bash
export ISAAC_ROS_WS=/workspaces/tamp_ws
```

> 영구 적용: `.devcontainer/devcontainer.json`의 `containerEnv`에 이미 설정되어 있습니다.

### 6.4 `No trajectory` (MoveItErrorCode=-1)

MoveIt2 로그에서:
```
No trajectory
Planner 'Generate minimum-jerk trajectories using NVIDIA Isaac ROS cuMotion' failed with error code PLANNING_FAILED
```

**원인**: rclpy 7.1.9의 action server 버그 — `goal_handle.succeed()`가 result 없이 호출되어 빈 결과 전송.
**해결**: `cumotion_planner.py`의 `execute_callback`에서 `goal_handle.succeed(result)` 패치 적용 (이미 수정됨).
→ [알려진 버그 및 수정](#7-알려진-버그-및-수정) 참조.

### 6.5 `CONTROL_FAILED` (MoveItErrorCode=-4)

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

파일 위치: `src/tamp_dev/ur/Universal_Robots_ROS2_Driver/ur_moveit_config/config/moveit_controllers.yaml`

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

상세 내용은 [4.3 Mock Hardware에서 실행 활성화](#43-mock-hardware에서-실행execution-활성화) 참조.

### 6.6 `PLANNING_FAILED` (MoveItErrorCode=-2) — 플래닝 파이프라인 불일치

```
FAILED 'target'. MoveItErrorCode=-2
```

**원인**: `default_planning_pipeline`이 `ompl`로 설정되어 있어 cuMotion 대신 OMPL이 사용됨. Servo 사용 후 복귀하거나, `ur.launch.py` 수정 전 빌드를 사용한 경우 발생.

**확인**:
```bash
ros2 param get /move_group default_planning_pipeline
# 예상: isaac_ros_cumotion
# 문제: ompl
```

**해결 방법 1** — 런타임 전환 스크립트:
```bash
# cuMotion으로 전환
bash switch_to_cumotion.sh

# 현재 상태 확인
bash switch_to_cumotion.sh status

# OMPL로 복원
bash switch_to_cumotion.sh ompl
```

**해결 방법 2** — `ur.launch.py`가 이미 `default_planning_pipeline: isaac_ros_cumotion`으로 설정되어 있으므로 Terminal 2 재시작:
```bash
ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e
```

> **참고**: 모든 테스트 스크립트(`test_motion_plan*.py`, `go_to_init_pose*.py`)는 `pipeline_id = 'isaac_ros_cumotion'`을 명시적으로 설정하므로, `default_planning_pipeline` 값과 무관하게 cuMotion을 사용합니다. RViz의 MoveIt MotionPlanning 패널에서 직접 플래닝할 때만 default 값이 영향을 줍니다.

### 6.7 여러 개의 `/move_action` 서버 경고

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

### 6.7 `colcon build` 오류

#### 6.7.1 `LookupError: Could not find the resource 'isaac_ros_common'`

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

#### 6.7.2 `curobo_core` 빌드 실패 (`egg_base: 'curobo/src' does not exist`)

```
error: error in 'egg_base' option: 'curobo/src' does not exist or is not a directory
Summary: 1 package failed: curobo_core
```

**원인**: `curobo_core/curobo/src`가 비어있음. 실제 cuRobo 라이브러리는 `/opt/ros/humble`에 사전 설치되어 있으며 이 workspace에서 재빌드할 수 없음.

**해결**: `curobo_core` 디렉토리에 `COLCON_IGNORE` 파일 추가 (이미 적용됨):

```bash
# 이미 적용됨 — 확인만:
ls src/tamp_dev/cumotion/isaac_ros_cumotion/curobo_core/COLCON_IGNORE
```

---

## 7. 알려진 버그 및 수정

### 7.1 cuMotion planner `goal_handle.succeed()` 조기 호출 버그

**파일**: `isaac_ros_cumotion/isaac_ros_cumotion/cumotion_planner.py`
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

### 7.2 `moveit_controllers.yaml` 컨트롤러 기본값 불일치

**파일**: `src/tamp_dev/ur/Universal_Robots_ROS2_Driver/ur_moveit_config/config/moveit_controllers.yaml`

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
| Stage 1 테스트 | `test_motion_plan.py` |
| Stage 2 장애물 | `test_collision_objects.py` |
| Cartesian 테스트 (mock) | `test_motion_plan_cartesian.py` |
| Cartesian 테스트 (실제 로봇) | `test_motion_plan_real_cartesian.py` |
| 런치 가이드 스크립트 | `launch_cumotion_test.sh` |
| 플래닝 파이프라인 전환 | `switch_to_cumotion.sh` |

---

## ROS2 Action 인터페이스

| Action | 타입 | 설명 |
|--------|------|------|
| `/move_action` | `moveit_msgs/action/MoveGroup` | MoveIt2 메인 인터페이스 (테스트 스크립트 사용) |
| `/cumotion/move_group` | `moveit_msgs/action/MoveGroup` | cuMotion 직접 인터페이스 (MoveIt2 → cuMotion) |
| `/cumotion/motion_plan` | `isaac_manipulator_msgs/action/MotionPlan` | cuMotion GoalSet 인터페이스 (Isaac Manipulator용) |
| `/cumotion/ik` | `isaac_ros_cumotion_interfaces/action/IKSolution` | IK 솔버 직접 인터페이스 |
