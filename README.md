# tamp_dev — UR10e GPU Motion Planning

UR10e 로봇을 위한 GPU 가속 모션 플래닝 워크스페이스.
NVIDIA Isaac ROS cuMotion + Universal Robots ROS2 Driver + Standalone Python 플래너.

---

## 디렉토리 구조

```
tamp_dev/
├── standalone/                # MoveIt 독립 Python 모션 플래닝
│   ├── config.py              # 공유 설정 (관절, 경로, 안전 한계)
│   ├── core/                  # 로봇 통신 + 궤적 실행 인프라
│   │   ├── robot_backend.py   # RobotBackend ABC + create_backend()
│   │   ├── ur_robot.py        # RTDEBackend (실제 로봇)
│   │   ├── sim_robot.py       # SimBackend (시뮬레이션)
│   │   └── trajectory_executor.py  # 궤적 리샘플링 + 스트리밍
│   └── cumotion/              # GPU 모션 플래닝 (curobo MotionGen)
│       ├── planner.py         # StandaloneMotionPlanner
│       ├── test_standalone.py # 단일 목표 테스트
│       └── test_multi_goal.py # 다중 목표 테스트
├── cumotion/                  # Isaac ROS cuMotion (ROS2 플래너 노드)
│   └── isaac_ros_cumotion/    # ROS2 패키지 소스
├── ur/                        # Universal Robots ROS2 Driver
│   ├── Universal_Robots_ROS2_Driver/
│   ├── Universal_Robots_ROS2_Description/
│   └── Universal_Robots_Client_Library/
├── docker/                    # Docker 빌드 설정
    ├── Dockerfile
    ├── build_image.sh
    └── run_container.sh

```

---

## 빠른 시작

### Docker 빌드

```bash
cd src/tamp_dev/docker
./build_image.sh               # amd64 자동 감지
./build_image.sh --arch arm64  # Jetson AGX Orin
```

### ROS2 빌드

```bash
cd /workspaces/tamp_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Standalone cuMotion 실행

GPU만 있으면 로봇 연결 없이 모션 플래닝 테스트 가능.

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# 플래닝만 (GPU 필수, 로봇 불필요)
python3 -m standalone.cumotion.test_standalone --plan-only

# 다중 목표 순차 플래닝
python3 -m standalone.cumotion.test_multi_goal --plan-only --rounds 3

# 시뮬레이션에서 플래닝 + 실행
python3 -m standalone.cumotion.test_standalone --mode sim --execute --velocity-scale 0.1

# 실제 로봇에서 실행
python3 -m standalone.cumotion.test_standalone --mode rtde --robot-ip 192.168.0.2 --execute
```

---

## ROS2 cuMotion 스택 실행 (Mock Hardware)

3개 터미널에서 순서대로 실행:

```bash
# Terminal 1 — UR10e Mock Hardware Driver
source /workspaces/tamp_ws/install/setup.bash
ros2 launch ur_robot_driver ur10e.launch.py use_fake_hardware:=true robot_ip:=0.0.0.0

# Terminal 2 — MoveIt2 + RViz
source /workspaces/tamp_ws/install/setup.bash
ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e

# Terminal 3 — cuMotion Planner Node
source /workspaces/tamp_ws/install/setup.bash
XRDF=$(ros2 pkg prefix isaac_ros_cumotion_robot_description)/share/isaac_ros_cumotion_robot_description/xrdf/ur10e.xrdf
URDF=/workspaces/tamp_ws/src/tamp_dev/.docker/assets/ur10e.urdf
ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \
  cumotion_planner.robot:=${XRDF} cumotion_planner.urdf_path:=${URDF}
```

---

## 의존성

| 패키지 | 출처 | 용도 |
|--------|------|------|
| numpy (<2.0) | pip | 수치 연산 (curobo/torch ABI 호환) |
| curobo-core | apt (`ros-humble-curobo-core`) | GPU 모션 플래닝 |
| ur-rtde | pip | 실제 로봇 RTDE 통신 (optional) |
| torch | apt (curobo-core pip-shim) | GPU 텐서 연산 |
| rclpy | ROS2 | SimBackend ROS2 통신 |

---

## 중요 사항

- **GPU 필수**: cuMotion 사용 시 `nvidia-smi` 정상 출력 필요
- **ISAAC_ROS_WS**: `export ISAAC_ROS_WS=/workspaces/tamp_ws`
- **Mock hardware**: `use_fake_hardware:=true` 시 `joint_trajectory_controller` 자동 활성화
- **Cartesian 플래닝**: `link_name="tool0"`, `PositionConstraint` + `OrientationConstraint` 둘 다 필수
