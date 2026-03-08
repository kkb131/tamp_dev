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

`--symlink-install` 사용 시 소스 파일 수정이 rebuild 없이 즉시 반영됨 (Python 파일).

## 아키텍처

```
tamp_dev/
├── standalone/                    # 핵심 Python 패키지 (MoveIt 독립)
│   ├── config.py                  # 공유 설정 (joints, paths, controller 상수)
│   ├── robot_backend.py           # ABC + create_backend() 팩토리
│   ├── ur_robot.py                # RTDEBackend (실제 로봇)
│   ├── sim_robot.py               # SimBackend (Isaac Sim)
│   ├── trajectory_executor.py     # 궤적 리샘플링 + 스트리밍
│   ├── cumotion/                  # GPU 모션 플래닝 서브패키지
│   │   ├── planner.py             # StandaloneMotionPlanner (curobo)
│   │   ├── test_standalone.py     # 단일 목표 테스트
│   │   ├── test_multi_goal.py     # 다중 목표 테스트
│   │   └── docs/user_guide.md     # 사용자 가이드
│   ├── servo/                     # 실시간 제어 서브패키지 (MoveIt 불필요)
│   │   ├── controller_utils.py    # ControllerSwitcher (rclpy)
│   │   ├── pinocchio_utils.py     # PinocchioIK (FK/Jacobian/DLS)
│   │   ├── keyboard_cartesian.py  # Pinocchio DLS 키보드 제어
│   │   ├── keyboard_forward.py    # 직접 joint 키보드 제어
│   │   ├── keyboard_servo_admittance.py  # F/T 어드미턴스
│   │   └── joystick_cartesian.py  # Xbox + Pinocchio
│   └── teleop/                    # 텔레옵 파이프라인
│       ├── main.py                # Entry point
│       ├── input_handler.py       # Keyboard/Xbox 입력
│       ├── pink_ik.py             # Pink IK solver
│       ├── exp_filter.py          # Exponential filter
│       ├── safety_monitor.py      # 안전 검사
│       ├── teleop_config.py       # 설정 로더
│       └── config/default.yaml    # 기본 설정
├── cumotion/isaac_ros_cumotion/   # Isaac ROS cuMotion 소스 (release-3.2)
│   ├── isaac_ros_cumotion/        # cuMotion planner ROS2 노드
│   ├── isaac_ros_cumotion_examples/  # ur.launch.py
│   ├── isaac_ros_cumotion_moveit/    # MoveIt planner plugin
│   └── isaac_ros_cumotion_robot_description/  # XRDF
├── ur/                            # Universal Robots ROS2 (Humble branch)
│   ├── Universal_Robots_ROS2_Driver/
│   ├── Universal_Robots_ROS2_Description/
│   └── Universal_Robots_Client_Library/
├── docker/                        # Dockerfile, build/run 스크립트
├── .devcontainer/devcontainer.json
├── .docker/assets/ur10e.urdf      # 캐시된 UR10e URDF
├── requirements.txt               # pip 의존성
├── cumotion.repos, ur.repos       # 소스 참조
└── .gitignore
```

## Standalone 실행

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# cuMotion standalone (MoveIt 불필요, GPU 필요)
python3 -m standalone.cumotion.test_standalone --plan-only
python3 -m standalone.cumotion.test_multi_goal --plan-only

# Servo (실시간 제어)
python3 -m standalone.servo.keyboard_cartesian
python3 -m standalone.servo.keyboard_forward
python3 -m standalone.servo.joystick_cartesian

# Teleop (통합 파이프라인)
python3 -m standalone.teleop.main --mode sim --input keyboard
python3 -m standalone.teleop.main --mode rtde --input xbox
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

## 핵심 파일

- `standalone/config.py` — 공유 설정 (JOINT_NAMES, 경로, 컨트롤러 상수)
- `standalone/cumotion/planner.py` — MoveIt 독립 cuMotion planner (curobo)
- `standalone/servo/pinocchio_utils.py` — Pinocchio 기반 IK (MoveIt 불필요)
- `standalone/teleop/main.py` — 텔레옵 파이프라인 엔트리포인트
- `cumotion/isaac_ros_cumotion/isaac_ros_cumotion/isaac_ros_cumotion/cumotion_planner.py` — cuMotion ROS2 노드
- `.docker/assets/ur10e.urdf` — cuMotion planner 시작 시 필요

## Docker 이미지

```bash
cd /workspaces/tamp_ws/src/tamp_dev/docker
./build_image.sh               # amd64 자동 감지
./build_image.sh --arch arm64  # Jetson AGX Orin
./build_image.sh --no-cache    # 캐시 무시 재빌드
```

Base image: `nvcr.io/nvidia/isaac/ros:{x86_64|aarch64}-ros2_humble_<hash>` (NGC)

## 중요 사항

**cuMotion GPU 필수**: `nvidia-smi`가 정상 출력되어야 함. NVIDIA Runtime 없이 시작하면 `RuntimeError: No CUDA GPUs are available`.

**ISAAC_ROS_WS 환경 변수**: `devcontainer.json`의 `containerEnv`에 설정됨. 직접 실행 시 `export ISAAC_ROS_WS=/workspaces/tamp_ws` 필요.

**Mock hardware**: `ur_control.launch.py`가 `use_fake_hardware:=true` 시 자동으로 `joint_trajectory_controller` 활성화. 수동 전환 불필요.

**Cartesian 플래닝**: `cumotion_planner.py`는 Joint-Space(`plan_single_js`)와 Cartesian(`plan_single`) 모두 지원. Cartesian 사용 시 `link_name="tool0"`, `PositionConstraint+OrientationConstraint` 필수.

## 세션 영속화

`/root/.claude/` → `.docker/claude/`에 bind mount. 컨테이너 재시작 후에도 Claude Code 히스토리 유지.
