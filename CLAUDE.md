# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 프로젝트 개요

NVIDIA Isaac ROS cuMotion + UR10e + ROS2 Jazzy 기반 모션 플래닝 개발 환경.

- **Workspace**: `/workspaces/tamp_ws`
- **소스**: `/workspaces/tamp_ws/src/tamp_dev/`
- **ROS2**: Jazzy | **OS**: Ubuntu 24.04 (NVIDIA Isaac ROS base image)
- **빌드 아티팩트**: `.docker/build`, `.docker/install`, `.docker/log` (컨테이너 재시작 후에도 유지)

## 빌드

```bash
cd /workspaces/tamp_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 사용 시 소스 파일 수정이 rebuild 없이 즉시 반영됨 (Python 파일).

## cuMotion 테스트 실행 (UR10e + Mock Hardware)

```bash
# 실행 지침 출력 (권장)
bash /workspaces/tamp_ws/src/tamp_dev/launch_cumotion_test.sh

# tmux로 자동 실행
bash /workspaces/tamp_ws/src/tamp_dev/launch_cumotion_test.sh --tmux
```

**3개 터미널 순서대로 실행:**

```bash
# Terminal 1 - UR10e Mock Hardware Driver
source /workspaces/tamp_ws/install/setup.bash
ros2 launch ur_robot_driver ur10e.launch.py use_mock_hardware:=true robot_ip:=0.0.0.0

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

**테스트:**
```bash
source /workspaces/tamp_ws/install/setup.bash

# Stage 1: 경로 이동 (home → up → test_configuration → home)
python3 /workspaces/tamp_ws/src/tamp_dev/test_motion_plan.py --plan-only

# Stage 2: 장애물 회피
python3 /workspaces/tamp_ws/src/tamp_dev/test_collision_objects.py          # 장애물 추가
python3 /workspaces/tamp_ws/src/tamp_dev/test_motion_plan.py --obstacle-test --plan-only
python3 /workspaces/tamp_ws/src/tamp_dev/test_collision_objects.py --clear  # 정리

# 실행(execution) 테스트: ur_control.launch.py가 use_mock_hardware:=true 시 자동으로
# joint_trajectory_controller를 활성화하므로 수동 전환 불필요 (이미 적용됨)
python3 /workspaces/tamp_ws/src/tamp_dev/test_motion_plan.py  # --plan-only 없이

# Cartesian 목표 테스트 (mock hardware)
python3 /workspaces/tamp_ws/src/tamp_dev/test_motion_plan_cartesian.py --goal-type joint
python3 /workspaces/tamp_ws/src/tamp_dev/test_motion_plan_cartesian.py --goal-type cartesian
python3 /workspaces/tamp_ws/src/tamp_dev/test_motion_plan_cartesian.py --goal-type both --execute
```

## 아키텍처

```
tamp_dev/
├── cumotion/isaac_ros_cumotion/   # Isaac ROS cuMotion 소스 (release-4.2)
│   ├── isaac_ros_cumotion/        # cuMotion planner ROS2 노드 (핵심)
│   ├── isaac_ros_cumotion_examples/  # ur.launch.py (MoveIt2 + cuMotion 통합)
│   ├── isaac_ros_cumotion_moveit/    # MoveItPlannerManager plugin
│   ├── isaac_ros_cumotion_robot_description/  # XRDF (robot geometry for curobo)
│   └── curobo_core/               # CUDA 모션 계획 라이브러리 (apt 설치, 소스 아님)
├── ur/                            # Universal Robots ROS2 Driver (Jazzy branch)
│   ├── Universal_Robots_ROS2_Driver/  # ur_robot_driver
│   └── Universal_Robots_ROS2_Description/  # URDF/xacro
├── docker/
│   ├── Dockerfile                 # NVIDIA Isaac ROS base → tamp_dev image
│   ├── build_image.sh             # 이미지 빌드 (amd64/arm64 자동 감지)
│   └── run_container.sh           # 컨테이너 실행 (GPU, X11, volume 자동 설정)
├── .devcontainer/devcontainer.json  # VS Code devcontainer 설정
├── .docker/assets/ur10e.urdf      # 캐시된 UR10e URDF (cuMotion planner 필요)
├── launch_cumotion_test.sh        # 테스트 런치 가이드 / tmux 자동화
├── test_motion_plan.py            # Stage 1 & 2 모션 플래닝 테스트
├── test_collision_objects.py      # Stage 2 장애물 추가/제거
└── cumotion/docs/user_guide.md    # 상세 사용자 가이드 (트러블슈팅 포함)
```

**플래닝 파이프라인 흐름:**
```
test_motion_plan.py
  → /move_action (MoveGroup action)
  → move_group (default_planning_pipeline: isaac_ros_cumotion)
  → /cumotion/move_group (MoveGroup action)
  → cumotion_planner.py (CUDA GPU 모션 계획)
```

**핵심 파일:**
- `cumotion/isaac_ros_cumotion/isaac_ros_cumotion/isaac_ros_cumotion/cumotion_planner.py` — cuMotion planner 노드 (rclpy 7.1.9 action 버그 수정 적용됨)
- `cumotion/isaac_ros_cumotion/isaac_ros_cumotion/params/isaac_ros_cumotion_params.yaml` — planner 파라미터
- `.docker/assets/ur10e.urdf` — cuMotion planner 시작 시 필요 (없으면 xacro로 생성)

## Docker 이미지

```bash
cd /workspaces/tamp_ws/src/tamp_dev/docker
./build_image.sh               # amd64 자동 감지
./build_image.sh --arch arm64  # Jetson AGX Orin
./build_image.sh --no-cache    # 캐시 무시 재빌드
```

Base image: `nvcr.io/nvidia/isaac/ros:isaac_ros_740c8500df2685ab1f4a4e53852601df-{amd64|arm64-jetpack}`

## ⚠️ 중요 사항

**cuMotion GPU 필수**: `nvidia-smi`가 정상 출력되어야 함. 컨테이너가 NVIDIA Runtime 없이 시작되면 `RuntimeError: No CUDA GPUs are available` 발생.

**ISAAC_ROS_WS 환경 변수**: `devcontainer.json`의 `containerEnv`에 설정됨. 직접 실행 시 `export ISAAC_ROS_WS=/workspaces/tamp_ws` 필요.

**알려진 버그 (수정됨)**: `cumotion_planner.py`의 rclpy 7.1.9 action server 버그 — `goal_handle.succeed()`가 result 없이 호출되어 빈 결과 전송. `goal_handle.succeed(result)` 형태로 수정 적용됨. 상세 내용: `cumotion/docs/user_guide.md`.

**Mock hardware 실행**: `ur_control.launch.py`가 `use_mock_hardware:=true` 시 자동으로 `joint_trajectory_controller`를 활성화하고 `moveit_controllers.yaml`도 이미 수정됨. 수동 controller 전환 불필요.

**Cartesian 플래닝**: `cumotion_planner.py`는 Joint-Space(`plan_single_js`)와 Cartesian(`plan_single`) 목표 모두 지원. Cartesian 사용 시 `link_name="tool0"`, `PositionConstraint+OrientationConstraint` 둘 다 필수.

## 세션 영속화

`/root/.claude/` → `.docker/claude/`에 bind mount (devcontainer.json, run_container.sh 모두 설정됨). 컨테이너 재시작 후에도 Claude Code 히스토리 유지.
