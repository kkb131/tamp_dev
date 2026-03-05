# UR Robot Driver — User Guide

환경: **ROS 2 Jazzy** / Ubuntu 24.04

---

## 목차

1. [패키지 구성](#1-패키지-구성)
2. [소스 설치](#2-소스-설치)
3. [빌드](#3-빌드)
4. [Mock Hardware 테스트](#4-mock-hardware-테스트)
5. [실제 로봇 연결](#5-실제-로봇-연결)
6. [알려진 이슈 및 수정사항](#6-알려진-이슈-및-수정사항)
7. [트러블슈팅](#7-트러블슈팅)

---

## 1. 패키지 구성

| 소스 디렉토리 | upstream 브랜치 | 고정 버전 | 제공 패키지 |
|---|---|---|---|
| `Universal_Robots_ROS2_Driver` | `jazzy` | 3.7.0 (버그 수정 포함) | `ur_robot_driver`, `ur_controllers`, `ur_calibration`, `ur_dashboard_msgs`, `ur_moveit_config` |
| `Universal_Robots_ROS2_Description` | `jazzy` | 3.5.0+7 (b2d2899) | `ur_description` |
| `Universal_Robots_Client_Library` | `master` | 2.7.0+2 (7b57b66) | `ur_client_library` |

> **소스 관리 방식**: 3개 패키지 모두 tamp_dev 레포지토리에 **inline**(직접 커밋)으로 포함되어 있습니다.
> `git clone`만으로 모든 소스가 확보되며, submodule이나 `vcs import`가 필요 없습니다.
> 버전 참고: [`ur.repos`](../ur.repos)

지원 로봇 모델: `ur3`, `ur5`, `ur10`, `ur3e`, `ur5e`, `ur7e`, `ur10e`, `ur12e`, `ur16e`, `ur20`, `ur30`

---

## 2. 소스 설치

### 2-1. 저장소 클론

UR 패키지는 tamp_dev 레포에 inline으로 포함되어 있으므로 별도 클론이 불필요합니다.

```bash
git clone https://github.com/kkb131/tamp_dev.git /workspaces/tamp_ws/src/tamp_dev
```

### 2-2. apt `ur_client_library` 충돌 제거 (필수)

`ros-jazzy-ur-client-library` apt 패키지가 설치되어 있으면 **반드시 제거**해야 합니다.
apt 버전의 헤더(`/opt/ros/jazzy/include/ur_client_library/`)가 소스 빌드 버전보다
먼저 참조되어 `'DashboardResponse' is not a member of 'urcl'` 빌드 에러가 발생합니다.

```bash
# 충돌 패키지 확인
dpkg -l | grep ur-client-library

# 설치되어 있으면 제거
sudo apt remove ros-jazzy-ur-client-library
```

> **원인**: `ur_robot_driver` v3.7.0은 `urcl::DashboardResponse` 타입을 사용하며,
> 이 타입은 `ur_client_library` ≥ 2.5.0 (PolyScope X dashboard refactoring)에서 도입되었습니다.
> apt 버전(`99.2.0`)은 이 리팩토링 이전 API를 사용하므로 호환되지 않습니다.

### 2-3. 바이너리 의존성 설치

```bash
cd /workspaces/tamp_ws

rosdep update --rosdistro=jazzy

rosdep install \
    --from-paths src/tamp_dev/ur \
    --ignore-src \
    --rosdistro jazzy \
    --skip-keys "ur_client_library liburdfdom-tools backward_ros" \
    -y
```

> `ur_client_library`는 소스 빌드이므로 반드시 `--skip-keys`에 포함해야 합니다.
> 생략하면 apt 버전이 설치되어 충돌이 재발합니다.

---

## 3. 빌드

```bash
cd /workspaces/tamp_ws
source /opt/ros/jazzy/setup.bash

colcon build \
    --packages-up-to ur_robot_driver \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --symlink-install

source install/setup.bash
```

빌드 결과 (5개 패키지):
```
ur_dashboard_msgs  ur_client_library  ur_description  ur_controllers  ur_robot_driver
```

> `--symlink-install`을 사용하면 launch 파일 수정 시 재빌드 없이 반영됩니다.

---

## 4. Mock Hardware 테스트

물리적 로봇 없이 컨트롤러 및 인터페이스 동작을 검증할 때 사용합니다.

### 4-1. 사전 정리 (필수)

FastDDS 공유메모리(`/dev/shm/fastrtps_*`)에 이전 실행의 `/robot_description` 데이터가 남아있으면
새 실행에서 잘못된 하드웨어 플러그인이 로드될 수 있습니다. **매 테스트 전에 반드시 정리합니다.**

```bash
# 관련 프로세스 종료
ps aux | grep -E "robot_state|ros2_control|spawner|trajectory" \
    | grep -v grep | awk '{print $2}' | xargs -r kill -9 2>/dev/null
sleep 2

# ROS 2 daemon 종료 및 FastDDS 공유메모리 정리
source /opt/ros/jazzy/setup.bash
ros2 daemon stop 2>/dev/null
rm -f /dev/shm/fastrtps_* 2>/dev/null
```

### 4-2. 런치

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/tamp_ws/install/setup.bash
export ROS_DOMAIN_ID=42        # 격리된 도메인 사용 권장

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur10e \
    robot_ip:=192.168.56.101 \
    use_mock_hardware:=true \
    launch_rviz:=false
```

> `ur_type`은 `ur5e`, `ur10e` 등 원하는 모델로 변경합니다.
> `robot_ip`는 Mock 모드에서 실제로 연결하지 않으므로 임의 값 사용 가능합니다.

### 4-3. 검증 (별도 터미널)

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/tamp_ws/install/setup.bash
export ROS_DOMAIN_ID=42

# 컨트롤러 상태 확인
ros2 control list_controllers

# 하드웨어 인터페이스 확인
ros2 control list_hardware_interfaces

# joint_states 토픽 수신 확인
ros2 topic echo /joint_states --once
```

### 4-4. 정상 동작 기준

**컨트롤러 (active 상태여야 함)**

| 컨트롤러 | 타입 | 상태 |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | active |
| `scaled_joint_trajectory_controller` | `ur_controllers/ScaledJointTrajectoryController` | active |
| `io_and_status_controller` | `ur_controllers/GPIOController` | active |
| `speed_scaling_state_broadcaster` | `ur_controllers/SpeedScalingStateBroadcaster` | active |
| `force_torque_sensor_broadcaster` | `force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster` | active |
| `ur_configuration_controller` | `ur_controllers/URConfigurationController` | active |

**하드웨어 플러그인**

Mock 모드에서는 반드시 `mock_components/GenericSystem`이 로드되어야 합니다.
```
Loaded hardware 'ur10e' from plugin 'mock_components/GenericSystem'
```

**joint_states**

6개 관절 (shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3) 데이터가 수신됩니다.

---

## 5. 실제 로봇 연결

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/tamp_ws/install/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur10e \
    robot_ip:=<ROBOT_IP> \
    launch_rviz:=false
```

> 실제 로봇 연결 전 UR 티치 펜던트에서 **External Control URCap** 프로그램을 실행해야 합니다.

### 키네마틱 캘리브레이션 (권장)

실제 로봇의 정확한 캘리브레이션 파라미터를 추출합니다:

```bash
ros2 launch ur_calibration calibration_correction.launch.py \
    robot_ip:=<ROBOT_IP> \
    target_filename:="${HOME}/my_robot_calibration.yaml"
```

추출한 파일을 런치 시 지정합니다:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur10e \
    robot_ip:=<ROBOT_IP> \
    kinematics_params_file:="${HOME}/my_robot_calibration.yaml"
```

---

## 6. 알려진 이슈 및 수정사항

### `use_mock_hardware`가 전달되지 않는 문제

**증상**: `use_mock_hardware:=true` 지정 시에도 `URPositionHardwareInterface`가 로드됨

**원인**: ROS 2 Jazzy에서 `IncludeLaunchDescription`을 `OpaqueFunction` 내부에서 사용할 때,
`launch_arguments`를 명시하지 않으면 부모 컨텍스트의 LaunchConfiguration 값이 전달되지 않음

**수정 위치**: [`ur_robot_driver/launch/ur_control.launch.py`](../Universal_Robots_ROS2_Driver/ur_robot_driver/launch/ur_control.launch.py) (line ~224)

```python
# 수정 후 — use_mock_hardware 등을 launch_arguments에 명시
rsp = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(description_launchfile),
    launch_arguments={
        "robot_ip": robot_ip,
        "ur_type": ur_type,
        "use_mock_hardware": use_mock_hardware,
        "mock_sensor_commands": LaunchConfiguration("mock_sensor_commands"),
        "headless_mode": headless_mode,
    }.items(),
)
```

---

## 7. 트러블슈팅

### `URPositionHardwareInterface`가 로드됨 (Mock 모드인데)

FastDDS 공유메모리에 이전 실행의 stale `/robot_description`이 남아있는 경우입니다.

```bash
# 해결책: FastDDS 데이터 세그먼트 삭제 후 재실행
ros2 daemon stop 2>/dev/null
rm -f /dev/shm/fastrtps_* 2>/dev/null
export ROS_DOMAIN_ID=42   # 새 도메인 ID 사용
```

> `sem.fastrtps_port*_mutex` 파일은 포트 락 세마포어로 무해합니다. 삭제하지 않아도 됩니다.

---

### `ros2 control list_controllers` 실패

`ros-jazzy-ros2controlcli`가 설치되어 있지 않은 경우입니다.

```bash
apt-get install -y ros-jazzy-ros2controlcli
```

---

### `Could not enable FIFO RT scheduling policy`

컨테이너 환경에서 실시간 스케줄링 권한이 없을 때 나타나는 경고입니다. Mock 테스트에는 영향 없습니다.

실제 로봇 사용 시 성능을 위해 설정이 필요하다면:
[ros2_control RT scheduling 문서](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html) 참고

---

### 빌드 시 `liburdfdom-tools` / `backward_ros` 미설치 오류

rosdep이 해당 패키지를 찾지 못하는 경우입니다. `--skip-keys`로 처리합니다:

```bash
rosdep install \
    --from-paths src/tamp_dev/ur \
    --ignore-src \
    --rosdistro jazzy \
    --skip-keys "ur_client_library liburdfdom-tools backward_ros" \
    -y
```

---

### `'DashboardResponse' is not a member of 'urcl'` 빌드 에러

**증상**: `ur_robot_driver` 빌드 시 `urcl::DashboardResponse` 를 찾지 못함

**원인**: `ros-jazzy-ur-client-library` apt 패키지가 설치되어 있으면, `/opt/ros/jazzy/include/`의
구 버전 헤더가 소스 빌드 헤더보다 먼저 참조됨. apt 버전은 `DashboardResponse` 타입이 없는 구 API 사용.

**해결**:

```bash
# 1. apt 충돌 패키지 제거
sudo apt remove ros-jazzy-ur-client-library

# 2. 클린 빌드
cd /workspaces/tamp_ws
rm -rf build/ur_client_library build/ur_robot_driver \
       install/ur_client_library install/ur_robot_driver
colcon build --symlink-install
```

> 자세한 원인은 [2-2. apt `ur_client_library` 충돌 제거](#2-2-apt-ur_client_library-충돌-제거-필수) 참조.
