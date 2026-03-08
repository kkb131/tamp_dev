# Servo 구현 방식 조사 결과

## 현재 상태

| 항목 | 상태 |
|---|---|
| `forward_position_controller` | 설정 완료, inactive 상태 |
| `forward_velocity_controller` | 설정 완료, inactive 상태 |
| MoveIt Servo config (`ur_servo.yaml`) | 설정 파일 있음 |
| MoveIt Servo 패키지 | **미설치** (apt로 설치 가능) |
| UR hardware interface | position, velocity, effort 3개 모두 지원 |

## UR Controllers 전체 목록 (ur_controllers.yaml)

| Controller | Type | Interface | 기본 상태 |
|---|---|---|---|
| joint_state_broadcaster | JointStateBroadcaster | state | Active |
| io_and_status_controller | ur_controllers/GPIOController | - | Active |
| speed_scaling_state_broadcaster | ur_controllers/SpeedScalingStateBroadcaster | state | Active |
| force_torque_sensor_broadcaster | ForceTorqueSensorBroadcaster | state | Active |
| joint_trajectory_controller | JointTrajectoryController | position | Mock hw: Active |
| scaled_joint_trajectory_controller | ur_controllers/ScaledJointTrajectoryController | position | Real hw: Active |
| **forward_velocity_controller** | velocity_controllers/JointGroupVelocityController | velocity | Inactive |
| **forward_effort_controller** | effort_controllers/JointGroupEffortController | effort | Inactive |
| **forward_position_controller** | position_controllers/JointGroupPositionController | position | Inactive |
| force_mode_controller | ur_controllers/ForceModeController | - | Inactive |
| freedrive_mode_controller | ur_controllers/FreedriveModeController | - | Inactive |
| passthrough_trajectory_controller | ur_controllers/PassthroughTrajectoryController | - | Inactive |
| tcp_pose_broadcaster | pose_broadcaster/PoseBroadcaster | state | Inactive |
| ur_configuration_controller | ur_controllers/URConfigurationController | - | Active |
| tool_contact_controller | ur_controllers/ToolContactController | - | Inactive |

---

## 방식 1: MoveIt Servo (권장)

가장 완성도 높은 방식. UR Driver에 이미 설정이 다 되어 있음.

```
TwistStamped (Cartesian) ──→ MoveIt Servo ──→ forward_position_controller ──→ UR HW
JointJog (Joint)         ──→    (IK변환)   ──→ (Float64MultiArray)         ──→
```

### 장점
- Cartesian twist 명령 지원 (조이스틱/스페이스마우스 연동에 최적)
- 충돌 체크 + 특이점 감지 내장
- Butterworth 필터로 부드러운 움직임
- 250Hz 퍼블리시 (설정 가능)

### 필요한 작업
```bash
apt install ros-humble-moveit-servo
# 그 후 launch_servo:=true로 실행
```

### 이미 있는 설정 파일
- `ur/Universal_Robots_ROS2_Driver/ur_moveit_config/config/ur_servo.yaml`
  - 출력: `/forward_position_controller/commands` (Float64MultiArray)
  - 입력: `~/delta_twist_cmds` (TwistStamped) 또는 `~/delta_joint_cmds` (JointJog)
  - publish_period: 0.004 (250Hz)
  - smoothing_filter: ButterworthFilterPlugin
  - check_collisions: true

### MoveIt Servo 설정 상세 (ur_servo.yaml)
```yaml
command_in_type: "speed_units"      # m/s, rad/s
scale:
  linear: 0.6        # m/s
  rotational: 0.3    # rad/s
  joint: 0.01        # rad/s per publish period
publish_period: 0.004  # 250 Hz
low_latency_mode: false
command_out_type: std_msgs/Float64MultiArray
command_out_topic: /forward_position_controller/commands
publish_joint_positions: true
publish_joint_velocities: false
smoothing_filter_plugin_name: "online_signal_smoothing::ButterworthFilterPlugin"
move_group_name: ur_manipulator
planning_frame: base_link
ee_frame: tool0
robot_link_command_frame: tool0
cartesian_command_in_topic: ~/delta_twist_cmds
joint_command_in_topic: ~/delta_joint_cmds
check_collisions: true
collision_check_rate: 5.0
```

---

## 방식 2: Forward Controller 직접 사용 (경량 방식)

MoveIt 없이 ros2_control만으로 스트리밍.

```
Python Node (직접 IK) ──→ forward_velocity_controller ──→ UR HW
                          (/commands topic, Float64MultiArray)
```

### 장점
- 추가 설치 없음 (이미 모두 있음)
- 가장 낮은 레이턴시
- 구현이 단순

### 단점
- 충돌 체크 없음
- Cartesian→Joint 변환을 직접 구현해야 함
- 특이점 처리 없음

---

## 방식 3: cuMotion + Servo (미래 방향)

cuMotion의 GPU 가속 IK를 servo에 활용. 현재 cuMotion은 trajectory planning만 지원하지만, cuRobo의 IK solver를 직접 호출하면 가능.

---

## 추천 요약

| 목적 | 추천 방식 |
|---|---|
| 조이스틱/텔레옵 | **방식 1 (MoveIt Servo)** — Cartesian twist 지원, 안전 기능 내장 |
| 간단한 위치 스트리밍 | **방식 2 (Forward Controller)** — 설치 없이 바로 사용 |
| 속도 제어 실험 | **방식 2 (velocity controller)** — joint velocity 직접 제어 |

## Controller 전환 방법

현재 `joint_trajectory_controller`가 active 상태이므로, servo 시작 시 controller 전환 필요:

```bash
# trajectory → forward_position (for MoveIt Servo)
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{start_controllers: ['forward_position_controller'],
    stop_controllers: ['joint_trajectory_controller'],
    strictness: 2}"
```

servo 종료 후 다시 trajectory controller로 복귀하면 기존 cuMotion planning도 그대로 사용 가능.

---

## RL/IL Policy 배포 시 Controller 선택

RL/IL policy는 보통 10~50Hz로 joint position 또는 velocity를 출력함.
현재 UR10e에 **이미 설정되어 있고 설치 없이 바로 사용 가능**.

```
[Policy Model] ──(10~50Hz)──→ forward_position_controller ──(500Hz HW)──→ UR10e
  (PyTorch/TF)                 /commands (Float64MultiArray)
      ↑
  /joint_states (피드백)
  /ft_data (힘/토크 피드백)
```

### Policy 출력 타입별 Controller 선택

| Policy 출력 | Controller | Topic | 비고 |
|---|---|---|---|
| **Joint Position** (가장 흔함) | `forward_position_controller` | `/forward_position_controller/commands` | ACT, Diffusion Policy 등 |
| **Joint Velocity** | `forward_velocity_controller` | `/forward_velocity_controller/commands` | 안전 관리 주의 |
| **Joint Torque** | `forward_effort_controller` | `/forward_effort_controller/commands` | 고급, 동역학 모델 필요 |
| **Cartesian Pose** | MoveIt Servo | `~/delta_twist_cmds` | 설치 필요, IK 내장 |

### 핵심 스펙
- **HW 업데이트**: 500Hz (UR10e, `ur10e_update_rate.yaml`)
- **메시지**: `std_msgs/Float64MultiArray` (6개 joint 값)
- **레이턴시**: ~2-4ms (publish → HW command)
- **피드백**: `/joint_states` (pos/vel/effort), `/ft_data` (F/T 센서)

### 구현 패턴 (Python)
```python
# 1. Controller 전환 (trajectory → forward_position)
switch_controller(start=['forward_position_controller'],
                  stop=['joint_trajectory_controller'])

# 2. Policy 루프
while running:
    obs = get_observation()          # joint_states + ft_data + camera
    action = policy.predict(obs)     # [6] joint positions
    msg = Float64MultiArray(data=action)
    publisher.publish(msg)           # → /forward_position_controller/commands
    rate.sleep()                     # 10~50Hz

# 3. 복귀
switch_controller(start=['joint_trajectory_controller'],
                  stop=['forward_position_controller'])
```

### MoveIt Servo가 더 나은 경우
- Policy가 **Cartesian delta** (dx, dy, dz, drx, dry, drz)를 출력하는 경우
- 충돌 체크가 필수인 환경
- 특이점 근처 작업이 많은 경우
- → `apt install ros-humble-moveit-servo` 후 `launch_servo:=true`

---

## TAMP 정밀 이동: 3단계 파이프라인

```
[Phase 1] Coarse Motion        → cuMotion (trajectory planning)
[Phase 2] Approach + Contact   → FollowJointTrajectoryUntil (tool_contact 감지)
[Phase 3] Fine Control         → force_mode_controller (힘/위치 하이브리드)
```

### Phase 1: 대략적 이동 (기존 cuMotion)
```
cuMotion plan_single() → joint_trajectory_controller → UR10e
```
- 목표 근처까지 빠르게 이동 (현재 구현 완료)

### Phase 2: 접근 + 접촉 감지 (UR 전용 기능)
```
FollowJointTrajectoryUntil (TOOL_CONTACT) → 접촉 시 자동 정지
```
- **Action**: `/trajectory_until_node/execute`
- **Type**: `ur_msgs/FollowJointTrajectoryUntil`
- 삽입/조립 접근 시 접촉 감지하면 자동 정지
- 예시 코드: `ur/Universal_Robots_ROS2_Driver/ur_robot_driver/examples/move_until_example.py`

### Phase 3: 정밀 제어 (Force Mode)
```
force_mode_controller → 특정 축은 힘 제어, 나머지는 위치 유지
```

**Force Mode 서비스:**
- 시작: `/force_mode_controller/start_force_mode` (ur_msgs/SetForceMode)
- 정지: `/force_mode_controller/stop_force_mode` (std_srvs/Trigger)
- 예시 코드: `ur/Universal_Robots_ROS2_Driver/ur_robot_driver/examples/force_mode.py`

**파라미터:**
| 파라미터 | 설명 | 예시 (Z축 삽입) |
|---|---|---|
| task_frame | 힘 제어 기준 프레임 | TCP 기준 |
| selection_vector | 축별 컴플라이언스 ON/OFF | [0,0,1,0,0,0] (Z만 힘 제어) |
| wrench | 목표 힘/토크 | [0,0,10,0,0,0] (Z방향 10N) |
| speed_limits | 컴플라이언트 축 최대 속도 | 0.01 m/s |
| deviation_limits | 비컴플라이언트 축 최대 편차 | ±0.005m |
| damping | 감쇠 계수 [0,1] | 0.025 |
| gain_scaling | 이득 스케일 [0,2] | 0.5 |

### 추가 활용 가능한 기능

| 기능 | 용도 | 인터페이스 |
|---|---|---|
| **FTS 센서 피드백** | 실시간 힘/토크 모니터링 | `/ft_data` (WrenchStamped) |
| **FTS 영점** | 센서 캘리브레이션 | `/io_and_status_controller/zero_ftsensor` |
| **TCP Pose** | 실시간 위치 피드백 | TF2: `base` → `tool0_controller` |
| **Freedrive Mode** | 수동 교시 | `/freedrive_mode_controller/enable_freedrive_mode` |
| **Tool Contact** | 접촉 감지 | `/tool_contact_controller/detect_tool_contact` |
| **Admittance Controller** | 임피던스/컴플라이언스 제어 | ros2_control chainable (설치됨) |

---

## 전체 아키텍처 요약

```
                    ┌─────────────────────────────────┐
                    │         Task Planner (TAMP)      │
                    └──────────┬──────────────────────-┘
                               │
              ┌────────────────┼────────────────┐
              ▼                ▼                ▼
     [Coarse Motion]   [RL/IL Policy]   [Precision]
      cuMotion          forward_pos      force_mode
      trajectory        controller       controller
      controller        (10~50Hz)        (힘/위치 혼합)
              │                │                │
              └────────────────┼────────────────┘
                               ▼
                    ┌──────────────────────┐
                    │  Controller Manager  │
                    │     (500Hz, ros2     │
                    │      _control)       │
                    └──────────┬───────────┘
                               ▼
                    ┌──────────────────────┐
                    │       UR10e HW       │
                    │  (pos/vel/effort IF) │
                    └──────────────────────┘
```

### Controller 전환 흐름
```
cuMotion 사용 시:  joint_trajectory_controller (active)
Policy 배포 시:    forward_position_controller (active) ← switch
정밀 제어 시:      force_mode_controller (active) ← switch
교시 모드 시:      freedrive_mode_controller (active) ← switch
```

---

## 다음 구현 단계 (제안)

1. **forward_position_controller 테스트** — 간단한 sin파 joint 스트리밍으로 동작 확인
2. **force_mode_controller 테스트** — 예제 코드 기반 Z축 힘 제어 테스트
3. **controller 전환 유틸리티** — cuMotion ↔ forward ↔ force_mode 전환 함수 구현
4. **(선택) MoveIt Servo 설치** — Cartesian servo가 필요한 경우

## 관련 파일 경로

| 파일 | 경로 |
|---|---|
| UR Servo Config | `ur/Universal_Robots_ROS2_Driver/ur_moveit_config/config/ur_servo.yaml` |
| UR Controllers Config | `ur/Universal_Robots_ROS2_Driver/ur_robot_driver/config/ur_controllers.yaml` |
| UR10e Update Rate | `ur/Universal_Robots_ROS2_Driver/ur_robot_driver/config/ur10e_update_rate.yaml` |
| Force Mode 예제 | `ur/Universal_Robots_ROS2_Driver/ur_robot_driver/examples/force_mode.py` |
| Move Until 예제 | `ur/Universal_Robots_ROS2_Driver/ur_robot_driver/examples/move_until_example.py` |
| UR MoveIt Launch | `ur/Universal_Robots_ROS2_Driver/ur_moveit_config/launch/ur_moveit.launch.py` |
| cuMotion UR Launch | `cumotion/isaac_ros_cumotion/isaac_ros_cumotion_examples/launch/ur.launch.py` |
| HW Interface XACRO | `ur/Universal_Robots_ROS2_Driver/ur_robot_driver/urdf/ur.ros2_control.xacro` |
| Velocity Test Config | `ur/Universal_Robots_ROS2_Driver/ur_robot_driver/config/test_velocity_goal_publishers_config.yaml` |
| Velocity Test Launch | `ur/Universal_Robots_ROS2_Driver/ur_robot_driver/launch/test_forward_velocity_controller.launch.py` |
