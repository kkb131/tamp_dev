# Isaac ROS 3.2 (Humble) 브랜치 구성 계획

## Context

AGX Orin(JetPack 6.2.x, CUDA 12.6)에서 cuMotion을 실행하기 위해 Isaac ROS 3.2 + ROS2 Humble 환경이 필요.
현재 워크스페이스는 Isaac ROS 4.2 + Jazzy (CUDA 13.0)이며 Orin과 호환되지 않음.

**전략**: `humble` git 브랜치를 별도로 생성하여, 기존 Jazzy 환경은 유지하면서 Humble 환경을 구성.
로컬 PC(amd64)에서 먼저 테스트 후 AGX Orin(arm64)에 배포.

### 핵심 차이점

| 항목 | 현재 (Jazzy) | 목표 (Humble) |
|------|-------------|---------------|
| Ubuntu | 24.04 | **22.04** |
| Python | 3.12 | **3.10** |
| CUDA | 13.0 | **12.6** |
| Isaac ROS | 4.2 (release-4.2) | **3.2 (release-3.2)** |
| MoveIt | 2.10.x | **2.5.x** |
| UR Driver | jazzy (v3.7.0) | **humble (v2.3.x~2.4.x)** |

---

## Step 1: 브랜치 생성

```bash
cd /workspaces/tamp_ws/src/tamp_dev
git checkout -b humble
```

---

## Step 2: Docker 인프라 수정 (3개 파일)

### 2-1. `docker/Dockerfile`

- **Base image** 변경 (line 7): Isaac ROS 3.2 Humble NGC 이미지로 교체
  - amd64: `nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_<hash>` (NGC에서 정확한 태그 확인)
  - arm64: `nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_<hash>`
  - Ubuntu 22.04 기반, CUDA 12.6 포함
- **ROS_DISTRO** 변경 (line 12): `jazzy` → `humble`
- **Layer 0 (apt source fix)**: arm64 이미지에 맞게 확인/조정
- **모든 `ros-${ROS_DISTRO}-*` 패키지**: 자동으로 humble로 해석됨
- **Layer 7 cuMotion deps**: 패키지 존재 확인 필요
  - 확인됨: `ros-humble-curobo-core`, `ros-humble-isaac-ros-common`
  - 확인 필요: `ros-humble-isaac-ros-nitros-bridge-ros2`, `ros-humble-isaac-ros-pynitros` 등
  - 없는 패키지는 cumotion.repos에 추가하여 source build

### 2-2. `docker/build_image.sh`

- **BASE_IMAGES** 배열 (lines 31-34): 두 이미지 태그 모두 Isaac ROS 3.2 Humble 버전으로 변경

### 2-3. `docker/entrypoint.sh`

- **Line 37**: `/opt/ros/jazzy/...` 하드코딩 → `/opt/ros/${ROS_DISTRO}/...` 변수화

---

## Step 3: cuMotion 소스 교체

### 3-1. `cumotion.repos` 수정

- `version: release-4.2` → `version: release-3.2`

### 3-2. cuMotion 소스 교체

```bash
rm -rf cumotion/isaac_ros_cumotion
vcs import . < cumotion.repos  # release-3.2 클론
rm -rf cumotion/isaac_ros_cumotion/.git  # inline 커밋용
```

### 3-3. COLCON_IGNORE 재적용

- `cumotion/isaac_ros_cumotion/curobo_core/COLCON_IGNORE` 생성 확인

### 3-4. rclpy API 차이 확인 (클론 후)

- Humble rclpy: `goal_handle.succeed()` (인자 없음, return값이 result)
- Jazzy rclpy: `goal_handle.succeed(result)` (인자로 전달)
- release-3.2 소스에 이미 Humble 방식 적용되어 있을 가능성 높음 → 확인

---

## Step 4: UR Driver 소스 교체

### 4-1. UR 소스 디렉토리 교체

```bash
cd ur/
rm -rf Universal_Robots_ROS2_Driver Universal_Robots_ROS2_Description Universal_Robots_Client_Library
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
git clone -b master https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
rm -rf Universal_Robots_ROS2_Driver/.git Universal_Robots_ROS2_Description/.git Universal_Robots_Client_Library/.git
```

### 4-2. initial_positions.yaml 재적용

싱귤러리티 방지 초기 위치: `[2.24, -1.2808, 2.16, -0.8848, 2.24, 0.0]`
- `ur/Universal_Robots_ROS2_Description/config/initial_positions.yaml` 수정

### 4-3. apt `ur_client_library` 충돌 확인

- Humble에서 DashboardResponse 충돌 존재 여부 확인
- 없을 가능성 높음 (Humble UR driver는 다른 API 사용)
- Dockerfile Layer 6의 `apt-get remove` 라인: 필요 여부에 따라 유지/제거

### 4-4. `ur.repos` 버전 참조 갱신

---

## Step 5: MoveIt Servo 처리

### 5-1. apt 설치 전환

Humble MoveIt Servo는 Jazzy와 완전히 다른 API → 소스 빌드 대신 apt 패키지 사용:
- Dockerfile에 `ros-humble-moveit-servo` 추가
- `servo/moveit_servo/COLCON_IGNORE` 생성 (또는 디렉토리 삭제)

### 5-2. Servo 스크립트 수정

**`keyboard_servo.py`, `joystick_servo.py`:**
- `ServoCommandType` 서비스 관련 코드 제거 (Humble에 없음)

**Python 3.10 호환성 (6개 파일):**
- 각 파일 상단에 `from __future__ import annotations` 추가
- 영향: `keyboard_servo.py`, `keyboard_forward.py`, `keyboard_cartesian.py`,
  `controller_utils.py`, `pinocchio_utils.py`, `joystick_servo.py`

---

## Step 6: 테스트 스크립트

**수정 불필요** — `test_motion_plan*.py`, `test_collision_objects.py`, `go_to_init_pose*.py`는
표준 rclpy + moveit_msgs API 사용. Python 3.12 전용 문법 없음 확인됨.

---

## Step 7: 문서 업데이트

- `CLAUDE.md`: Jazzy → Humble 참조 변경
- `cumotion/docs/*.md`, `ur/docs/*.md`, `servo/docs/*.md`: 버전 참조 갱신

---

## 검증 절차

### 로컬 PC (amd64) 테스트

```bash
# 1. Docker 이미지 빌드
./build_image.sh --arch amd64 --no-cache

# 2. 기본 확인
./run_container.sh
python3 --version          # 3.10.x
echo $ROS_DISTRO           # humble
python3 -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())"

# 3. 소스 빌드
colcon build --symlink-install

# 4. Mock hardware 테스트
# T1: ros2 launch ur_robot_driver ur10e.launch.py use_mock_hardware:=true robot_ip:=0.0.0.0
# T2: ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e
# T3: ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py ...
# T4: python3 test_motion_plan.py --plan-only
```

### AGX Orin (arm64) 배포

```bash
./build_image.sh --arch arm64 --no-cache
./run_container.sh
# nvidia-smi + torch.cuda.is_available() == True 확인
# colcon build + cuMotion 실행
```

---

## 수정 대상 파일 요약

| 파일 | 변경 유형 |
|------|----------|
| `docker/Dockerfile` | 주요 수정 (Base image, ROS_DISTRO, apt 확인) |
| `docker/build_image.sh` | BASE_IMAGES 배열 변경 |
| `docker/entrypoint.sh` | hardcoded path 변수화 |
| `cumotion.repos` | version: release-3.2 |
| `cumotion/isaac_ros_cumotion/` | **전체 교체** (release-3.2) |
| `ur/Universal_Robots_ROS2_Driver/` | **전체 교체** (humble 브랜치) |
| `ur/Universal_Robots_ROS2_Description/` | **전체 교체** (humble 브랜치) |
| `ur/Universal_Robots_Client_Library/` | **전체 교체** (master 브랜치) |
| `ur.repos` | 버전 참조 갱신 |
| `servo/moveit_servo/` | COLCON_IGNORE 추가 |
| `servo/keyboard_servo.py` | ServoCommandType 제거, 타입 힌트 |
| `servo/joystick_servo.py` | ServoCommandType 제거, 타입 힌트 |
| `servo/*.py` (4개) | Python 3.10 타입 힌트 |
| `CLAUDE.md`, `docs/*.md` | Jazzy → Humble 참조 |

## 주요 리스크

| 리스크 | 확률 | 대응 |
|--------|------|------|
| NGC 이미지 태그 확인 필요 | 중 | NGC에서 정확한 Humble 태그 조회 |
| 일부 ros-humble-* apt 패키지 미존재 | 중 | source build로 대체 |
| MoveIt Servo API 완전 변경 | 확정 | apt 설치 + 스크립트 수정 |
| UR Humble driver 구조 차이 | 저 | 공식 humble 브랜치 사용 |
