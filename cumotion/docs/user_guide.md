# cuMotion + UR10e 사용자 가이드

## 개요

NVIDIA Isaac ROS cuMotion을 UR10e 로봇과 함께 사용하기 위한 가이드입니다.
cuMotion은 CUDA GPU를 사용해 실시간 모션 플래닝을 수행하며, MoveIt2의 기본 플래닝 파이프라인으로 통합됩니다.

**버전**: isaac_ros_cumotion release-4.2
**ROS2**: Jazzy
**OS**: Ubuntu 24.04 (NVIDIA Isaac ROS base image)

---

## 목차

1. [사전 요구사항](#1-사전-요구사항)
2. [빌드](#2-빌드)
3. [실행](#3-실행)
4. [테스트](#4-테스트)
5. [트러블슈팅](#5-트러블슈팅)
6. [알려진 버그 및 수정](#6-알려진-버그-및-수정)

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
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ur-msgs \
  ros-jazzy-moveit \
  ros-jazzy-curobo-core \
  ros-jazzy-isaac-ros-cumotion-interfaces \
  ros-jazzy-isaac-manipulator-ros-python-utils \
  ros-jazzy-nvblox-msgs
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
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 3. 실행

3개의 터미널(또는 백그라운드 프로세스)이 필요합니다.

### Terminal 1 — UR10e 드라이버 (Mock Hardware)

```bash
source /workspaces/tamp_ws/install/setup.bash
ros2 launch ur_robot_driver ur10e.launch.py \
  use_mock_hardware:=true \
  robot_ip:=0.0.0.0
```

> 실제 로봇 사용 시 `robot_ip:=<ROBOT_IP>`로 변경하고 `use_mock_hardware:=true` 제거.

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

기본 설정에서 `scaled_joint_trajectory_controller`는 실제 UR 로봇 연결을 요구합니다.
Mock hardware에서 실행 테스트를 하려면 `joint_trajectory_controller`로 전환해야 합니다.

```bash
source /workspaces/tamp_ws/install/setup.bash

ros2 control switch_controllers \
  --deactivate scaled_joint_trajectory_controller \
  --activate joint_trajectory_controller
```

이후 `--plan-only` 플래그 없이 실행:
```bash
python3 /workspaces/tamp_ws/src/tamp_dev/test_motion_plan.py
```

---

## 5. 트러블슈팅

### 5.1 `RuntimeError: No CUDA GPUs are available`

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

### 5.2 `No module named 'curobo'` / `'nvblox_msgs'` 등

```
ModuleNotFoundError: No module named 'curobo'
```

**원인**: 필수 패키지 미설치 (이미지 재빌드 필요).
**해결**:
```bash
apt-get update && apt-get install -y \
  ros-jazzy-curobo-core \
  ros-jazzy-nvblox-msgs \
  ros-jazzy-isaac-ros-cumotion-interfaces \
  ros-jazzy-isaac-manipulator-ros-python-utils
```

### 5.3 `ISAAC_ROS_WS environment variable is not set`

```
RuntimeError: ISAAC_ROS_WS environment variable is not set
```

**해결**:
```bash
export ISAAC_ROS_WS=/workspaces/tamp_ws
```

> 영구 적용: `.devcontainer/devcontainer.json`의 `containerEnv`에 이미 설정되어 있습니다.

### 5.4 `No trajectory` (MoveItErrorCode=-1)

MoveIt2 로그에서:
```
No trajectory
Planner 'Generate minimum-jerk trajectories using NVIDIA Isaac ROS cuMotion' failed with error code PLANNING_FAILED
```

**원인**: rclpy 7.1.9의 action server 버그 — `goal_handle.succeed()`가 result 없이 호출되어 빈 결과 전송.
**해결**: `cumotion_planner.py`의 `execute_callback`에서 `goal_handle.succeed(result)` 패치 적용 (이미 수정됨).
→ [알려진 버그 및 수정](#6-알려진-버그-및-수정) 참조.

### 5.5 `CONTROL_FAILED` (MoveItErrorCode=-4)

```
Goal request rejected
Failed to send trajectory part 1 of 1 to controller scaled_joint_trajectory_controller
CONTROL_FAILED
```

**원인**: Mock hardware에서 `scaled_joint_trajectory_controller`가 goal 거부.
**해결**: [4.3 Mock Hardware에서 실행 활성화](#43-mock-hardware에서-실행execution-활성화) 참조.

### 5.6 여러 개의 `/move_action` 서버 경고

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

## 6. 알려진 버그 및 수정

### 6.1 cuMotion planner `goal_handle.succeed()` 조기 호출 버그

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
| 런치 가이드 스크립트 | `launch_cumotion_test.sh` |

---

## ROS2 Action 인터페이스

| Action | 타입 | 설명 |
|--------|------|------|
| `/move_action` | `moveit_msgs/action/MoveGroup` | MoveIt2 메인 인터페이스 (테스트 스크립트 사용) |
| `/cumotion/move_group` | `moveit_msgs/action/MoveGroup` | cuMotion 직접 인터페이스 (MoveIt2 → cuMotion) |
| `/cumotion/motion_plan` | `isaac_manipulator_msgs/action/MotionPlan` | cuMotion GoalSet 인터페이스 (Isaac Manipulator용) |
| `/cumotion/ik` | `isaac_ros_cumotion_interfaces/action/IKSolution` | IK 솔버 직접 인터페이스 |
