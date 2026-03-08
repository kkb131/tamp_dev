# cuMotion UR10e 실제 로봇 사용 가이드

> **작성일:** 2026-02-26
> **대상:** 실제 UR10e 로봇에서 cuMotion을 사용하려는 사용자
> **선행 문서:** [user_guide.md](user_guide.md) (mock hardware 설정 및 cuMotion 기본 동작 검증 완료 후 이 문서를 참조)

> **참고**: 실제 로봇 제어는 `standalone/` 모듈(RTDE 모드)로도 가능합니다.
> `standalone/cumotion/docs/user_guide.md` 참조.

---

## 목차

1. [소스 코드 변경 사항 분석](#1-소스-코드-변경-사항-분석)
2. [실제 로봇 영향 요약](#2-실제-로봇-영향-요약)
3. [사전 요구사항 및 안전 체크리스트](#3-사전-요구사항-및-안전-체크리스트)
4. [하드웨어 설정](#4-하드웨어-설정)
5. [ROS2 스택 실행 절차](#5-ros2-스택-실행-절차)
6. [테스트 가이드](#6-테스트-가이드)
7. [비상 정지 절차](#7-비상-정지-절차)
8. [트러블슈팅](#8-트러블슈팅)

---

## 1. 소스 코드 변경 사항 분석

Mock hardware 테스트 과정에서 아래 파일들이 변경되었습니다.
각 변경이 실제 로봇에 미치는 영향과 조치 여부를 정리합니다.

### 1.1 `moveit_controllers.yaml`

**파일:** `ur/Universal_Robots_ROS2_Driver/ur_moveit_config/config/moveit_controllers.yaml`

| 항목 | 변경 전 (upstream) | 현재 상태 |
|---|---|---|
| `scaled_joint_trajectory_controller.default` | `true` | **`false` (mock hardware용으로 변경됨)** |
| `joint_trajectory_controller.default` | `false` | **`true` (mock hardware용으로 변경됨)** |

- **실제 로봇에서의 의미:** `scaled_joint_trajectory_controller`는 UR 로봇의 Teaching Pendant(TP) 속도 다이얼과 연동됩니다. 이 컨트롤러가 기본값이어야 TP에서 속도를 조절할 수 있습니다.
- **현재 상태:** mock hardware용 설정. **실제 로봇 사용 전 반드시 복원 필요** (아래 참조).

> **실제 로봇 전환 전 필수 작업**: 아래와 같이 복원 후 Terminal 2 재시작:
> ```yaml
> scaled_joint_trajectory_controller:
>   default: true   # 실제 로봇용
> joint_trajectory_controller:
>   default: false  # 실제 로봇용
> ```

### 1.2 `ur_controllers.yaml`

**파일:** `ur/Universal_Robots_ROS2_Driver/ur_robot_driver/config/ur_controllers.yaml`

| 항목 | 변경 전 | 현재 상태 |
|---|---|---|
| `joint_trajectory_controller.allow_nonzero_velocity_at_trajectory_end` | (없음, 기본 false) | `true` |
| `scaled_joint_trajectory_controller.allow_nonzero_velocity_at_trajectory_end` | (없음) | **`true` (추가됨)** |

- **변경 이유:** cuMotion이 생성하는 trajectory의 마지막 지점에 미세한 부동소수점 속도(~3.7x10^-7 rad/s)가 포함됩니다. JTC는 엄격한 `!= 0.0` 검사로 새 goal을 거부합니다.
- **실제 로봇에서의 영향:** cuMotion 사용 시 필수. 이 파라미터 없이는 두 번째 goal부터 CONTROL_FAILED(-4) 오류가 발생합니다.
- **안전성:** trajectory 마지막 점의 허용 오차이므로 안전에 직접적인 영향 없음.

### 1.3 `ur.ros2_control.xacro`

**파일:** `ur/Universal_Robots_ROS2_Driver/ur_robot_driver/urdf/ur.ros2_control.xacro`

| 항목 | 변경 전 | 현재 상태 |
|---|---|---|
| mock hardware: `calculate_dynamics` | `true` | `false` |

- **실제 로봇에서의 영향:** 이 파라미터는 `xacro:if value="${use_fake_hardware}"` 블록 내에 있습니다. 실제 로봇은 `URPositionHardwareInterface` 플러그인을 사용하므로 **실제 로봇에 영향 없음**.

### 1.4 `cumotion_planner.py`

**파일:** `cumotion/isaac_ros_cumotion/isaac_ros_cumotion/isaac_ros_cumotion/cumotion_planner.py`

| 항목 | 변경 전 | 현재 상태 |
|---|---|---|
| joint-space goal 처리 | FK -> Cartesian -> `plan_single()` | 직접 `plan_single_js()` |

- **핵심 버그 수정:** 기존 코드는 joint-space goal을 FK로 Cartesian pose로 변환한 뒤 Cartesian planner를 호출했습니다. IK solver가 동일한 Cartesian pose에 대해 다른 joint 구성(예: elbow-up vs elbow-down)을 선택해 목표 위치에 도달하지 못하는 문제가 있었습니다.
- **실제 로봇에서도 동일한 버그가 존재했을 것이며, 이 수정으로 개선됩니다.**

---

## 2. 실제 로봇 영향 요약

| 파일 | 실제 로봇 영향 | 상태 |
|---|---|---|
| `moveit_controllers.yaml` | `scaled_joint_trajectory_controller` 기본값 복원 필요 | 복원 후 안전 |
| `ur_controllers.yaml` | `allow_nonzero_velocity_at_trajectory_end` 추가 — cuMotion 필수 | 필요 |
| `ur.ros2_control.xacro` | mock hardware 전용, 실제 로봇 무관 | 안전 |
| `cumotion_planner.py` | joint-space 계획 버그 수정 — 실제 로봇에서도 개선됨 | 개선 |

---

## 3. 사전 요구사항 및 안전 체크리스트

### 3.1 소프트웨어 요구사항

- ROS2 Humble (Ubuntu 22.04)
- UR Robot Driver (ROS2): `ur_robot_driver`
- Isaac ROS cuMotion (release-3.2): `isaac_ros_cumotion`
- MoveIt2: `moveit_ros`
- CUDA 12.x + cuDNN (cuMotion GPU 연산용)
- `ISAAC_ROS_WS` 환경변수 설정:
  ```bash
  export ISAAC_ROS_WS=/workspaces/tamp_ws
  ```

### 3.2 하드웨어 요구사항

- UR10e 로봇 (Polyscope 5.x 이상)
- 로봇 컨트롤러와 PC 네트워크 연결 (기본 포트: 30001-30004, 50001-50004)
- 비상 정지 버튼 (E-stop) 접근 가능

### 3.3 실행 전 안전 체크리스트

```
[ ] 로봇 컨트롤러 전원 ON, Polyscope 정상 부팅 확인
[ ] 로봇 모드: Normal (not Protective Stop, not Emergency Stop)
[ ] 로봇을 home 위치로 수동 이동 완료
[ ] 작업 반경(최소 1m) 내 사람 없음
[ ] 주변 장애물 제거
[ ] 비상 정지 버튼 위치 확인 및 손이 닿는 곳에 준비
[ ] 티칭 펜던트(TP) 속도 다이얼 최저값(25% 이하)으로 설정
[ ] RViz에서 로봇 현재 위치 시각적 확인
```

---

## 4. 하드웨어 설정

### 4.1 로봇 IP 설정

Polyscope에서 로봇 IP 확인:
- 설정 -> 시스템 -> 네트워크

### 4.2 External Control URScript 설치

실제 로봇에서 UR Robot Driver를 사용하려면 `external_control.urscript`를 Polyscope에 설치해야 합니다.

```bash
# URScript 경로 확인
ros2 pkg prefix ur_client_library
# 일반적으로: /opt/ros/humble/share/ur_client_library/resources/
```

Polyscope에서 `ExternalControl` 프로그램 설치 방법은 [UR Robot Driver 공식 문서](https://docs.universal-robots.com)를 참조하세요.

### 4.3 kinematics 파라미터 추출

실제 로봇의 kinematics 캘리브레이션 파라미터를 추출합니다:

```bash
ros2 run ur_calibration calibration_correction \
  --ros-args \
  -p robot_ip:=<ROBOT_IP> \
  -p target_filename:=${HOME}/my_robot_calibration.yaml
```

---

## 5. ROS2 스택 실행 절차

### 5.1 moveit_controllers.yaml 복원 (필수)

실제 로봇 실행 전 반드시 컨트롤러 기본값을 복원합니다:

**파일:** `ur/Universal_Robots_ROS2_Driver/ur_moveit_config/config/moveit_controllers.yaml`

```yaml
scaled_joint_trajectory_controller:
  default: true   # 실제 로봇용
joint_trajectory_controller:
  default: false  # 실제 로봇용
```

### 5.2 Terminal 1: UR Robot Driver (실제 로봇)

```bash
source /workspaces/tamp_ws/install/setup.bash
export ISAAC_ROS_WS=/workspaces/tamp_ws

ros2 launch ur_robot_driver ur10e.launch.py \
  robot_ip:=<ROBOT_IP> \
  kinematics_params_file:=${HOME}/my_robot_calibration.yaml
```

> **중요:** `use_fake_hardware:=true` 옵션을 **절대 포함하지 마세요**.

정상 시작 확인:
```
[INFO] UR Robot connected.
[INFO] Robot mode: RUNNING
```

### 5.3 Terminal 2: MoveIt2 + RViz

```bash
source /workspaces/tamp_ws/install/setup.bash

ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e
```

RViz에서 로봇 현재 위치가 실제 로봇과 일치하는지 확인하세요.

### 5.4 Terminal 3: cuMotion 플래너

```bash
source /workspaces/tamp_ws/install/setup.bash
export ISAAC_ROS_WS=/workspaces/tamp_ws

XRDF=$(ros2 pkg prefix isaac_ros_cumotion_robot_description)/share/isaac_ros_cumotion_robot_description/xrdf/ur10e.xrdf
URDF=/workspaces/tamp_ws/src/tamp_dev/.docker/assets/ur10e.urdf

ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \
  cumotion_planner.robot:=${XRDF} \
  cumotion_planner.urdf_path:=${URDF}
```

`cuMotion is ready for planning queries!` 메시지 확인.

### 5.5 컨트롤러 확인

```bash
# 활성 컨트롤러 목록 확인
ros2 control list_controllers

# 실제 로봇: scaled_joint_trajectory_controller가 active 상태여야 함
# 예상 출력:
# scaled_joint_trajectory_controller[active]
# joint_trajectory_controller[inactive]
```

---

## 6. 테스트 가이드

### 6.1 RViz에서 수동 테스트

ROS2 스택이 실행된 상태에서 RViz의 MoveIt MotionPlanning 패널을 사용하여 테스트합니다:

1. **Motion Planning** -> **Planning** 탭
2. Goal State를 선택 (예: `up`, `home` 등 사전 정의 포즈)
3. **Plan** 버튼으로 경로 미리보기 (실행 없음)
4. 경로가 안전한지 시각적으로 확인
5. **Plan & Execute**로 실행

> **원칙:** 항상 plan-only로 시작 -> 계획 시각적 검증 -> 최저 속도로 실행 -> 점진적 속도 증가

### 6.2 Standalone 모듈 사용 (권장)

ROS2 스택 없이도 `standalone/` 모듈로 실제 로봇을 제어할 수 있습니다:

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# GPU 모션 플래닝 + RTDE 직접 실행 (plan-only)
python3 -m standalone.cumotion.test_standalone --mode rtde --robot-ip <ROBOT_IP>

# 플래닝 + 실행 (매우 낮은 속도)
python3 -m standalone.cumotion.test_standalone --mode rtde --robot-ip <ROBOT_IP> \
    --execute --velocity-scale 0.03

# 다중 목표 테스트
python3 -m standalone.cumotion.test_multi_goal --mode rtde --robot-ip <ROBOT_IP> \
    --execute --velocity-scale 0.03
```

> 자세한 사용법은 `standalone/cumotion/docs/user_guide.md` 참조.

### 6.3 속도 스케일 가이드

| 값 | 속도 | 용도 |
|----|------|------|
| 0.03 | 매우 느림 | 첫 테스트, 안전 확인 |
| 0.05 | 느림 | 일반 테스트 (기본값) |
| 0.1 | 보통 | 검증된 동작 반복 |
| 0.3 | 빠름 | 성능 테스트 |

> 실제 로봇 첫 테스트 시 반드시 0.03~0.05로 시작하세요.

### 6.4 Joint 단위 참고

| Joint | 이름 | 최대 속도 |
|---|---|---|
| 0 | shoulder_pan_joint | 120 deg/s (2.094 rad/s) |
| 1 | shoulder_lift_joint | 120 deg/s (2.094 rad/s) |
| 2 | elbow_joint | 180 deg/s (3.142 rad/s) |
| 3 | wrist_1_joint | 180 deg/s (3.142 rad/s) |
| 4 | wrist_2_joint | 180 deg/s (3.142 rad/s) |
| 5 | wrist_3_joint | 180 deg/s (3.142 rad/s) |

> **5% 속도 스케일 적용 시:** shoulder: 6 deg/s, wrist: 9 deg/s — 매우 느리고 안전합니다.

---

## 7. 비상 정지 절차

### 7.1 즉시 정지 방법 (우선순위 순)

1. **TP(티칭 펜던트) 비상 정지 버튼** — 물리적 E-stop (즉시 전원 차단)
2. **TP 일시정지** — Polyscope의 Stop 버튼 (프로그램 정지)
3. **ROS2 컨트롤러 중지:**
   ```bash
   ros2 control switch_controllers \
     --deactivate scaled_joint_trajectory_controller \
     --deactivate joint_trajectory_controller
   ```

### 7.2 E-stop 후 복구 절차

```
1. TP에서 E-stop 해제 (비상 정지 버튼 잠금 해제)
2. Polyscope에서 "FAULT RESET" 실행
3. 로봇을 안전한 위치로 수동 이동 (TP Free Drive 사용)
4. ROS2 노드 재시작 (Terminal 1 재시작)
5. 컨트롤러 상태 확인 후 재시작
```

### 7.3 Protective Stop 처리

Protective Stop이 발생하면:
```
1. 정지 원인 확인 (힘 제한, 속도 제한, 충돌 감지 등)
2. TP에서 "STOP" -> 원인 해결
3. TP에서 "RESUME" 또는 재시작
4. 속도/힘 설정 검토 후 더 낮은 속도로 재시도
```

---

## 8. 트러블슈팅

### 8.1 로봇이 연결되지 않음

**증상:** Terminal 1에서 `Connection refused` 또는 타임아웃

```bash
# 네트워크 연결 확인
ping <ROBOT_IP>

# UR 포트 확인 (30001: Primary, 30002: Secondary, 30003: RT)
nc -zv <ROBOT_IP> 30001
```

해결: Polyscope에서 ExternalControl 프로그램 실행 확인.

### 8.2 CONTROL_FAILED (-4) 오류

**증상:** `MoveItErrorCode=-4`

**원인 1:** 컨트롤러 비활성화
```bash
# 활성 컨트롤러 확인
ros2 control list_controllers
# scaled_joint_trajectory_controller[active] 확인
```

**원인 2:** 시작 위치 오차가 너무 큼 (`allowed_start_tolerance: 0.01 rad`)
```bash
# 현재 위치 확인
ros2 topic echo /joint_states --once
# RViz와 실제 로봇 위치 비교 후 home으로 수동 이동
```

### 8.3 cuMotion 계획 실패

**증상:** `Planning FAILED` 또는 `max_attempts` 초과

가능한 원인:
- 목표 위치가 self-collision 또는 workspace 경계 밖
- `allowed_planning_time` 부족

### 8.4 속도 스케일링이 동작하지 않음

**증상:** TP 다이얼을 돌려도 속도 변화 없음

```bash
# speed_scaling topic 확인
ros2 topic echo /speed_scaling_state

# 컨트롤러 확인: joint_trajectory_controller는 속도 스케일링 미지원
# scaled_joint_trajectory_controller로 전환 필요
ros2 control list_controllers
```

### 8.5 `ISAAC_ROS_WS` 오류

```bash
export ISAAC_ROS_WS=/workspaces/tamp_ws
```

---

*이 가이드는 cuMotion + UR10e ROS2 스택을 실제 로봇으로 확장하기 위한 절차를 다룹니다.
실제 로봇 테스트 시 항상 안전을 최우선으로 하고, 이상이 발생하면 즉시 비상 정지 절차를 따르세요.*
