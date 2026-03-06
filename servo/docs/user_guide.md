# Servo 실시간 제어 사용자 가이드

## 개요

UR10e 로봇의 실시간(real-time) 텔레오퍼레이션을 위한 가이드입니다.
세 가지 제어 방식을 제공합니다:

| 방식 | 설명 | 제어 공간 | MoveIt Servo 필요 |
|------|------|-----------|-------------------|
| **Forward Position Controller** | ros2_control 직접 제어 | Joint 공간 | 아니오 |
| **Forward Cartesian (Pinocchio)** | Pinocchio DLS IK + 직접 제어 | Cartesian 공간 | 아니오 |
| **MoveIt Servo** | Jacobian 기반 실시간 제어 | Cartesian 공간 | 예 |

**입력 장치**: 키보드, Xbox 조이스틱

---

## 목차

1. [사전 요구사항](#1-사전-요구사항)
2. [파일 구조](#2-파일-구조)
3. [Forward Position Controller (키보드 Joint)](#3-forward-position-controller-키보드-joint)
4. [Forward Cartesian — 키보드](#4-forward-cartesian--키보드)
5. [Forward Cartesian — Xbox 조이스틱](#5-forward-cartesian--xbox-조이스틱)
6. [MoveIt Servo — 키보드 Cartesian](#6-moveit-servo--키보드-cartesian)
7. [MoveIt Servo — Xbox 조이스틱](#7-moveit-servo--xbox-조이스틱)
8. [설정 파라미터](#8-설정-파라미터)
9. [트러블슈팅](#9-트러블슈팅)

---

## 1. 사전 요구사항

### 1.1 빌드

moveit_servo는 소스 빌드가 필요합니다 (apt 버전은 사용하지 않음):

```bash
cd /workspaces/tamp_ws
source /opt/ros/humble/setup.bash

# moveit_servo 빌드
colcon build --packages-select moveit_servo --symlink-install
source install/setup.bash
```

### 1.2 패키지

| 패키지 | 설치 방법 | 용도 |
|--------|-----------|------|
| ros-humble-pinocchio | `apt install ros-humble-pinocchio` | Forward Cartesian (FK/Jacobian/DLS) |
| moveit_servo | 소스 빌드 (위 참조) | MoveIt Servo Cartesian 제어 |
| ros-humble-joy | `apt install ros-humble-joy` | Xbox 조이스틱 (선택) |

### 1.3 mock hardware 초기 자세

mock hardware의 초기 자세는 wrist singularity를 피하도록 설정되어 있습니다:

```
shoulder_pan: 2.24, shoulder_lift: -1.2808, elbow: 2.16
wrist_1: -0.8848, wrist_2: 2.24, wrist_3: 0.0
```

> **중요**: `wrist_2_joint = 0`이면 wrist singularity입니다. 이 자세에서는 Servo가 동작하지 않습니다.

---

## 2. 파일 구조

```
src/tamp_dev/servo/
├── controller_utils.py        # Controller 전환 유틸리티 (공용)
├── pinocchio_utils.py         # Pinocchio FK/Jacobian/DLS 유틸리티 (공용)
├── keyboard_forward.py        # [방식1] 키보드 → Joint 직접 제어
├── keyboard_cartesian.py      # [방식2] 키보드 → Cartesian (Pinocchio DLS)
├── joystick_cartesian.py      # [방식3] Xbox → Cartesian (Pinocchio DLS)
├── keyboard_servo.py          # [방식4] 키보드 → Cartesian (MoveIt Servo)
├── joystick_servo.py          # [방식5] Xbox → Cartesian (MoveIt Servo)
├── launch_servo.sh            # 실행 가이드 스크립트
├── moveit_servo/              # moveit_servo 소스 (빌드됨)
└── docs/
    └── user_guide.md          # 이 문서
```

---

## 3. Forward Position Controller (키보드 Joint)

MoveIt Servo 없이 `ros2_control`의 `forward_position_controller`를 직접 사용합니다.
Joint 단위로 개별 제어합니다.

### 3.1 실행

**터미널 2개 필요** (cuMotion planner 불필요):

```bash
# T1: UR Driver (mock hardware)
ros2 launch ur_robot_driver ur10e.launch.py use_mock_hardware:=true robot_ip:=0.0.0.0

# T2: MoveIt + RViz (시각화용)
ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e

# T3: 키보드 제어
cd /workspaces/tamp_ws/src/tamp_dev
python3 servo/keyboard_forward.py
```

### 3.2 키 매핑

| 키 | 동작 |
|----|------|
| `1` ~ `6` | Joint 1~6 선택 |
| `w` / `↑` | 선택된 joint 위치 증가 (+step) |
| `s` / `↓` | 선택된 joint 위치 감소 (-step) |
| `+` / `=` | step 크기 증가 |
| `-` | step 크기 감소 |
| `h` | Home 위치로 이동 |
| `p` | 현재 joint 상태 출력 |
| `Space` | 현재 위치 유지 (정지) |
| `q` / `Esc` | 종료 (원래 controller 복원) |

**step 크기**: 0.001, 0.005, **0.01** (기본), 0.02, 0.05, 0.1 rad

### 3.3 동작 원리

```
키보드 입력 → target_positions 업데이트 → Float64MultiArray 발행
                                            ↓
/forward_position_controller/commands → ros2_control → 로봇
```

- `/joint_states` 구독으로 현재 위치 추적
- 6개 joint 값을 항상 모두 발행 (변경하지 않은 joint는 현재 값 유지)
- QoS: RELIABLE + TRANSIENT_LOCAL (controller 요구사항)

---

## 4. Forward Cartesian — 키보드

Pinocchio의 Jacobian + Damped Least Squares (DLS)를 사용하여 end-effector를 Cartesian 공간에서 제어합니다.
MoveIt Servo **불필요**. `forward_position_controller`에 직접 joint position을 발행합니다.

### 4.1 실행

**터미널 2개 필요** (MoveIt Servo, cuMotion planner 불필요):

```bash
# T1: UR Driver (mock hardware)
ros2 launch ur_robot_driver ur10e.launch.py use_mock_hardware:=true robot_ip:=0.0.0.0

# T2: MoveIt + RViz (시각화용, launch_servo 불필요!)
ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e

# T3: 키보드 Cartesian 제어
cd /workspaces/tamp_ws/src/tamp_dev
python3 servo/keyboard_cartesian.py
```

### 4.2 키 매핑

**이동 (Translation)**:

| 키 | 방향 | 설명 |
|----|------|------|
| `w` / `s` | X | 전진 / 후진 |
| `a` / `d` | Y | 좌 / 우 |
| `q` / `e` | Z | 상승 / 하강 |

**회전 (Rotation)**:

| 키 | 축 | 설명 |
|----|-----|------|
| `u` / `o` | RX | Roll +/- |
| `i` / `k` | RY | Pitch +/- |
| `j` / `l` | RZ | Yaw +/- |

**제어**:

| 키 | 동작 |
|----|------|
| `+` / `=` | 속도 증가 |
| `-` | 속도 감소 |
| `f` | 프레임 전환 (base_link ↔ tool0) |
| `p` | 현재 EE pose 출력 (FK) |
| `Space` | 정지 |
| `x` / `Esc` | 종료 (원래 controller 복원) |

**속도 스케일**: 0.1, 0.2, **0.3** (기본), 0.5, 0.8, 1.0

### 4.3 동작 원리

```
키보드 입력 → twist (6D) 생성
                ↓
Pinocchio: Jacobian 계산 → DLS inverse → joint delta
                ↓
q_new = clamp(q + dq) → /forward_position_controller/commands → 로봇
```

- Pinocchio로 URDF에서 Jacobian을 직접 계산
- DLS (Damped Least Squares): `dq = J^T @ inv(J @ J^T + λ²I) @ twist * dt`
- λ = 0.05 (damping factor) — singularity 근처에서 joint velocity 자동 제한
- 50Hz 루프로 joint position 발행
- MoveIt Servo의 singularity emergency stop 없이 DLS로 자연스럽게 감속

### 4.4 프레임 설명

| 프레임 | 설명 |
|--------|------|
| `base_link` (기본) | 로봇 베이스 기준. X=전방, Y=좌, Z=상 (직관적) |
| `tool0` | 엔드이펙터 기준. 로봇 자세에 따라 축이 변함 |

`f` 키로 전환 가능합니다. 일반적으로 `base_link`가 더 직관적입니다.

---

## 5. Forward Cartesian — Xbox 조이스틱

Xbox 컨트롤러로 Cartesian 제어합니다 (Pinocchio DLS, MoveIt Servo 불필요).

### 5.1 사전 준비

```bash
sudo apt install -y ros-humble-joy
```

### 5.2 실행

**터미널 3개 필요**:

```bash
# T1: UR Driver (mock hardware)
ros2 launch ur_robot_driver ur10e.launch.py use_mock_hardware:=true robot_ip:=0.0.0.0

# T2: MoveIt + RViz (launch_servo 불필요!)
ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e

# T3: Joy 노드 (조이스틱 드라이버)
ros2 run joy joy_node

# T4: 조이스틱 Cartesian 제어
cd /workspaces/tamp_ws/src/tamp_dev
python3 servo/joystick_cartesian.py
```

### 5.3 Xbox 매핑

**이동 (Stick / Trigger)**:

| 입력 | 동작 |
|------|------|
| 왼쪽 스틱 X | Y 이동 (좌/우) |
| 왼쪽 스틱 Y | X 이동 (전진/후진) |
| 오른쪽 스틱 X | Yaw (RZ) 회전 |
| 오른쪽 스틱 Y | Pitch (RY) 회전 |
| LT (왼쪽 트리거) | Z 하강 |
| RT (오른쪽 트리거) | Z 상승 |

**버튼**:

| 버튼 | 동작 |
|------|------|
| LB (왼쪽 범퍼) | Roll (RX) - |
| RB (오른쪽 범퍼) | Roll (RX) + |
| A | 속도 감소 |
| B | 속도 증가 |
| X | 프레임 전환 (base_link ↔ tool0) |
| Y | EE pose 출력 (FK) |
| Start | 종료 |

- Deadzone: 0.1 (미세 떨림 무시)
- 아날로그 스틱은 비례 제어 (기울기에 따라 속도 변화)

---

## 6. MoveIt Servo — 키보드 Cartesian

MoveIt Servo를 통해 end-effector를 Cartesian 공간에서 실시간 제어합니다.

### 6.1 실행

**터미널 3개 필요**:

```bash
# T1: UR Driver (mock hardware)
ros2 launch ur_robot_driver ur10e.launch.py use_mock_hardware:=true robot_ip:=0.0.0.0

# T2: MoveIt + RViz + Servo (launch_servo:=true 필수!)
ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e launch_servo:=true

# T3: 키보드 제어
cd /workspaces/tamp_ws/src/tamp_dev
python3 servo/keyboard_servo.py
```

> **주의**: T2에서 `launch_servo:=true`를 반드시 지정해야 servo_node가 실행됩니다.

### 6.2 키 매핑

**이동 (Translation)**:

| 키 | 방향 | 설명 |
|----|------|------|
| `w` / `s` | X | 전진 / 후진 |
| `a` / `d` | Y | 좌 / 우 |
| `q` / `e` | Z | 상승 / 하강 |

**회전 (Rotation)**:

| 키 | 축 | 설명 |
|----|-----|------|
| `u` / `o` | RX | Roll +/- |
| `i` / `k` | RY | Pitch +/- |
| `j` / `l` | RZ | Yaw +/- |

**제어**:

| 키 | 동작 |
|----|------|
| `+` / `=` | 속도 증가 |
| `-` | 속도 감소 |
| `f` | 프레임 전환 (base_link ↔ tool0) |
| `Space` | 정지 |
| `x` / `Esc` | 종료 (원래 controller 복원) |

**속도 스케일**: 0.1, 0.2, **0.3** (기본), 0.5, 0.8, 1.0

### 6.3 동작 원리

```
키보드 입력 → TwistStamped 발행 → servo_node
                                    ↓
                              Jacobian 역변환
                                    ↓
                     /forward_position_controller/commands → 로봇
```

- `keyboard_servo.py`는 `/servo_node/delta_twist_cmds`에 TwistStamped를 발행
- servo_node가 Jacobian을 사용해 joint position으로 변환
- servo_node가 `/forward_position_controller/commands`에 Float64MultiArray 발행
- 시작 시 자동으로 `forward_position_controller` 활성화 + servo TWIST 모드 전환

### 6.4 프레임 설명

| 프레임 | 설명 |
|--------|------|
| `base_link` (기본) | 로봇 베이스 기준. X=전방, Y=좌, Z=상 (직관적) |
| `tool0` | 엔드이펙터 기준. 로봇 자세에 따라 축이 변함 |

`f` 키로 전환 가능합니다. 일반적으로 `base_link`가 더 직관적입니다.

---

## 7. MoveIt Servo — Xbox 조이스틱

Xbox 컨트롤러로 Cartesian 제어합니다 (MoveIt Servo 필요).

### 7.1 사전 준비

```bash
# joy 패키지 설치
sudo apt install -y ros-humble-joy
```

### 7.2 실행

**터미널 4개 필요**:

```bash
# T1: UR Driver (mock hardware)
ros2 launch ur_robot_driver ur10e.launch.py use_mock_hardware:=true robot_ip:=0.0.0.0

# T2: MoveIt + RViz + Servo
ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e launch_servo:=true

# T3: Joy 노드 (조이스틱 드라이버)
ros2 run joy joy_node

# T4: 조이스틱 제어
cd /workspaces/tamp_ws/src/tamp_dev
python3 servo/joystick_servo.py
```

### 7.3 Xbox 매핑

**이동 (Stick / Trigger)**:

| 입력 | 동작 |
|------|------|
| 왼쪽 스틱 X | Y 이동 (좌/우) |
| 왼쪽 스틱 Y | X 이동 (전진/후진) |
| 오른쪽 스틱 X | Yaw (RZ) 회전 |
| 오른쪽 스틱 Y | Pitch (RY) 회전 |
| LT (왼쪽 트리거) | Z 하강 |
| RT (오른쪽 트리거) | Z 상승 |

**버튼**:

| 버튼 | 동작 |
|------|------|
| LB (왼쪽 범퍼) | Roll (RX) - |
| RB (오른쪽 범퍼) | Roll (RX) + |
| A | 속도 감소 |
| B | 속도 증가 |
| X | 프레임 전환 (base_link ↔ tool0) |
| Start | 종료 |

- Deadzone: 0.1 (미세 떨림 무시)
- 아날로그 스틱은 비례 제어 (기울기에 따라 속도 변화)

---

## 8. 설정 파라미터

### 8.1 ur_servo.yaml (MoveIt Servo 전용)

위치: `ur_moveit_config/config/ur_servo.yaml`

**주요 파라미터**:

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| `command_in_type` | `speed_units` | 명령 단위 (m/s, rad/s) |
| `scale.linear` | 0.6 | 최대 선형 속도 [m/s] |
| `scale.rotational` | 0.3 | 최대 회전 속도 [rad/s] |
| `publish_period` | 0.004 | 명령 발행 주기 (250Hz) |
| `command_out_type` | `Float64MultiArray` | 출력 메시지 타입 |
| `move_group_name` | `ur_manipulator` | MoveIt 그룹 이름 |
| `planning_frame` | `base_link` | 계획 기준 프레임 |
| `ee_frame` | `tool0` | 엔드이펙터 프레임 |
| `robot_link_command_frame` | `tool0` | 명령 입력 프레임 |

**안전 파라미터**:

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| `incoming_command_timeout` | 0.1 | 명령 없으면 정지 (초) |
| `lower_singularity_threshold` | 100.0 | Singularity 감속 시작 (condition number) |
| `hard_stop_singularity_threshold` | 200.0 | Singularity 정지 (condition number) |
| `check_collisions` | true | 충돌 검사 활성화 |
| `collision_check_rate` | 5.0 | 충돌 검사 주기 (Hz) |
| `joint_limit_margins` | [0.1 x 6] | Joint 한계 여유 (rad) |

### 8.2 Controller 구조

```
joint_trajectory_controller (기본, MoveIt planning용)
    ↕ 전환
forward_position_controller (Servo / 직접 제어용)
```

- 모든 teleop 스크립트 시작 시 자동으로 `forward_position_controller`로 전환
- 종료 시 원래 controller (`joint_trajectory_controller`)로 자동 복원

---

## 9. 트러블슈팅

### "Waiting to receive robot state update" (servo_node)

**원인**: servo_node의 PlanningSceneMonitor가 joint state를 받지 못함.

**해결**:
- `launch_servo:=true`로 실행했는지 확인
- servo_node는 `component_container_mt` (멀티스레드 컨테이너)에서 실행되어야 함
- UR Driver(T1)가 먼저 실행되어 `/joint_states`를 발행 중인지 확인

### "Very close to a singularity, emergency stop"

**원인**: 로봇이 singularity 근처에 있음.

**해결**:
- `wrist_2_joint ≠ 0`인지 확인 (wrist singularity)
- mock hardware 재시작 시 기본 자세가 non-singular인지 확인
  - 설정 파일: `ur_description/config/initial_positions.yaml`
- singularity 상태에서는 먼저 `test_motion_plan.py`로 이동 후 servo 사용

**UR 로봇의 주요 singularity**:
- **Wrist singularity**: `wrist_2_joint ≈ 0` (joint 4, 6 축이 정렬)
- **Shoulder singularity**: wrist center가 joint 1 축을 통과
- **Elbow singularity**: 팔꿈치가 완전히 펴짐

### "Command type has not been set"

**원인**: servo_node에 TWIST 명령 타입이 설정되지 않음.

**해결**:
- `keyboard_servo.py`가 자동으로 `/servo_node/switch_command_type` 서비스를 호출하여 TWIST 모드 전환
- servo_node 초기화가 완료될 때까지 최대 10회 재시도 (약 20초)

### QoS 불일치 경고 (forward_position_controller)

```
offering incompatible QoS. Last incompatible policy: DURABILITY
```

**원인**: publisher QoS가 controller subscription의 TRANSIENT_LOCAL과 불일치.

**해결**: `keyboard_forward.py`는 이미 RELIABLE + TRANSIENT_LOCAL QoS로 설정됨. 이 경고가 나오면 최신 코드를 사용 중인지 확인.

### 로봇이 움직이지 않음 (keyboard_forward)

**확인 사항**:
1. `forward_position_controller`가 active 상태인지: `ros2 control list_controllers`
2. `/forward_position_controller/commands` 토픽에 메시지가 도착하는지: `ros2 topic echo /forward_position_controller/commands`
3. step 크기가 너무 작지 않은지 (+/- 키로 조절)

### servo_node 초기화 지연

servo_node는 시작 후 다음을 순서대로 수행합니다:
1. robot_description 파라미터 로드
2. PlanningSceneMonitor 초기화
3. joint_states 수신 대기 ("Waiting to receive robot state update")
4. 서비스 등록 (`switch_command_type`)
5. Servo loop 시작

이 과정에서 30초 이상 걸릴 수 있습니다. `keyboard_servo.py`는 자동으로 대기합니다.
