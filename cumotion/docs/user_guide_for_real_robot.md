# cuMotion UR10e 실제 로봇 사용 가이드

> **작성일:** 2026-02-26
> **대상:** 실제 UR10e 로봇에서 cuMotion을 사용하려는 사용자
> **선행 문서:** [user_guide.md](user_guide.md) (mock hardware 설정 및 cuMotion 기본 동작 검증 완료 후 이 문서를 참조)

---

## 목차

1. [이번 개발에서 변경된 사항 분석](#1-이번-개발에서-변경된-사항-분석)
2. [실제 로봇 영향 요약](#2-실제-로봇-영향-요약)
3. [사전 요구사항 및 안전 체크리스트](#3-사전-요구사항-및-안전-체크리스트)
4. [하드웨어 설정](#4-하드웨어-설정)
5. [실행 절차](#5-실행-절차)
6. [단계별 테스트 가이드](#6-단계별-테스트-가이드)
7. [Cartesian 목표 테스트 (실제 로봇)](#7-cartesian-목표-테스트-실제-로봇)
8. [cuMotion 성능 검증](#8-cumotion-성능-검증)
9. [비상 정지 절차](#9-비상-정지-절차)
10. [트러블슈팅](#10-트러블슈팅)

---

## 1. 이번 개발에서 변경된 사항 분석

Mock hardware 테스트 과정에서 아래 파일들이 변경되었습니다.
각 변경이 실제 로봇에 미치는 영향과 조치 여부를 정리합니다.

### 1.1 `moveit_controllers.yaml`

**파일:** `ur_moveit_config/config/moveit_controllers.yaml`

| 항목 | 변경 전 (upstream) | 현재 상태 |
|---|---|---|
| `scaled_joint_trajectory_controller.default` | `true` | **`false` (mock hardware용으로 변경됨)** |
| `joint_trajectory_controller.default` | `false` | **`true` (mock hardware용으로 변경됨)** |

- **실제 로봇에서의 의미:** `scaled_joint_trajectory_controller`는 UR 로봇의 Teaching Pendant(TP) 속도 다이얼과 연동됩니다. 이 컨트롤러가 기본값이어야 TP에서 속도를 조절할 수 있습니다.
- **현재 상태:** mock hardware용 설정. **실제 로봇 사용 전 반드시 복원 필요** (아래 참조).
- **Mock hardware 실행 시:** 현재 설정이 이미 올바름 — 별도 전환 불필요.

> ⚠️ **실제 로봇 전환 전 필수 작업**: 아래와 같이 복원 후 Terminal 2 재시작:
> ```yaml
> scaled_joint_trajectory_controller:
>   default: true   # 실제 로봇용
> joint_trajectory_controller:
>   default: false  # 실제 로봇용
> ```

### 1.2 `ur_controllers.yaml`

**파일:** `ur_robot_driver/config/ur_controllers.yaml`

| 항목 | 변경 전 | 현재 상태 |
|---|---|---|
| `joint_trajectory_controller.allow_nonzero_velocity_at_trajectory_end` | (없음, 기본 false) | `true` |
| `scaled_joint_trajectory_controller.allow_nonzero_velocity_at_trajectory_end` | (없음) | **`true` (추가됨)** |

- **변경 이유:** cuMotion이 생성하는 trajectory의 마지막 지점에 미세한 부동소수점 속도(~3.7×10⁻⁷ rad/s)가 포함됩니다. JTC는 엄격한 `!= 0.0` 검사로 새 goal을 거부합니다.
- **실제 로봇에서의 영향:** cuMotion 사용 시 필수. 이 파라미터 없이는 두 번째 goal부터 CONTROL_FAILED(-4) 오류가 발생합니다.
- **안전성:** trajectory 마지막 점의 허용 오차이므로 안전에 직접적인 영향 없음.

### 1.3 `ur.ros2_control.xacro`

**파일:** `ur_robot_driver/urdf/ur.ros2_control.xacro`

| 항목 | 변경 전 | 현재 상태 |
|---|---|---|
| mock hardware: `calculate_dynamics` | `true` | `false` |

- **실제 로봇에서의 영향:** 이 파라미터는 `xacro:if value="${use_fake_hardware}"` 블록 내에 있습니다. 실제 로봇은 `URPositionHardwareInterface` 플러그인을 사용하므로 **실제 로봇에 영향 없음**.
- **변경 이유:** `calculate_dynamics: true` 상태에서 mock hardware가 물리 시뮬레이션을 수행해 위치 추적이 불안정해지는 문제 해결.

### 1.4 `cumotion_planner.py`

**파일:** `isaac_ros_cumotion/cumotion_planner.py`

| 항목 | 변경 전 | 현재 상태 |
|---|---|---|
| joint-space goal 처리 | FK → Cartesian → `plan_single()` | 직접 `plan_single_js()` |

- **핵심 버그 수정:** 기존 코드는 joint-space goal을 FK로 Cartesian pose로 변환한 뒤 Cartesian planner를 호출했습니다. IK solver가 동일한 Cartesian pose에 대해 다른 joint 구성(예: elbow-up vs elbow-down)을 선택해 목표 위치에 도달하지 못하는 문제가 있었습니다.
- **실제 로봇에서의 영향:** **실제 로봇에서도 동일한 버그가 존재했을 것이며, 이 수정으로 개선됩니다.** joint-space goal은 `plan_single_js()`로 직접 계획하므로 IK 모호성 문제가 없습니다.
- **Cartesian goal은 영향 없음:** Cartesian constraint를 사용하는 경우 기존 `plan_single()` 경로를 유지합니다.

### 1.5 알고리즘 확장 측면의 고려사항

나중에 다른 알고리즘(예: task and motion planning, TAMP)으로 확장할 때 주의할 점:

| 시나리오 | 잠재적 문제 | 권장 조치 |
|---|---|---|
| Cartesian space planning 추가 | `plan_single()` 경로는 유지되므로 직접 영향 없음 | goal 유형(joint vs Cartesian)을 명확히 구분해 올바른 함수 호출 확인 |
| 새 컨트롤러 추가 | JTC의 non-zero velocity 문제가 새 컨트롤러에도 발생 가능 | `allow_nonzero_velocity_at_trajectory_end: true` 를 새 JTC에도 추가 |
| 다중 로봇 지원 | `tf_prefix` 기반 파라미터 분리 필요 | UR driver 기본 구조(xacro tf_prefix) 유지 |

---

## 2. 실제 로봇 영향 요약

| 파일 | 실제 로봇 영향 | 상태 |
|---|---|---|
| `moveit_controllers.yaml` | `scaled_joint_trajectory_controller` 기본값 복원 완료 | ✅ 안전 |
| `ur_controllers.yaml` | `allow_nonzero_velocity_at_trajectory_end` 추가 — cuMotion 필수 | ✅ 필요 |
| `ur.ros2_control.xacro` | mock hardware 전용, 실제 로봇 무관 | ✅ 안전 |
| `cumotion_planner.py` | joint-space 계획 버그 수정 — 실제 로봇에서도 개선됨 | ✅ 개선 |

---

## 3. 사전 요구사항 및 안전 체크리스트

### 3.1 소프트웨어 요구사항

- ROS2 Humble (Ubuntu 22.04)
- UR Robot Driver (ROS2): `ur_robot_driver`
- Isaac ROS cuMotion (release-4.2): `isaac_ros_cumotion`
- MoveIt2: `moveit_ros`
- CUDA 12.x + cuDNN (cuMotion GPU 연산용)
- `ISAAC_ROS_WS` 환경변수 설정:
  ```bash
  export ISAAC_ROS_WS=/workspaces/tamp_ws
  # 영구 설정:
  echo 'export ISAAC_ROS_WS=/workspaces/tamp_ws' >> ~/.bashrc
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
    home: shoulder_pan=0°, shoulder_lift=-90°, elbow=0°,
          wrist_1=0°, wrist_2=0°, wrist_3=0°
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
- 설정 → 시스템 → 네트워크

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

## 5. 실행 절차

### 5.1 Terminal 1: UR Robot Driver (실제 로봇)

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

### 5.2 Terminal 2: MoveIt2 + RViz

```bash
source /workspaces/tamp_ws/install/setup.bash

ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e
```

RViz에서 로봇 현재 위치가 실제 로봇과 일치하는지 확인하세요.

### 5.3 Terminal 3: cuMotion 플래너

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

### 5.4 (선택) Terminal 4: 컨트롤러 확인

```bash
# 활성 컨트롤러 목록 확인
ros2 control list_controllers

# 실제 로봇: scaled_joint_trajectory_controller가 active 상태여야 함
# 예상 출력:
# scaled_joint_trajectory_controller[active]
# joint_trajectory_controller[inactive]
```

> **Mock hardware 실행 시:** `scaled_joint_trajectory_controller`를 사용할 수 없으므로 전환 필요:
> ```bash
> ros2 control switch_controllers \
>   --deactivate scaled_joint_trajectory_controller \
>   --activate joint_trajectory_controller
> ```

---

## 6. 단계별 테스트 가이드

**원칙: 항상 plan-only로 시작 → 계획 검증 → 최소 실행 → 점진적 확장**

### 6.1 Stage 0: 계획 검증 (plan-only, 필수)

로봇을 움직이지 않고 cuMotion이 올바른 경로를 계획하는지 확인합니다.

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# Stage minimal (1축, ~8.6° 이동) 계획만 검증
python3 test_motion_plan_real.py --stage minimal

# 예상 출력:
# [PLAN ONLY] 'verify_home': [+0.0°, -90.0°, +0.0°, +0.0°, +0.0°, +0.0°]
# OK (plan only): 'verify_home'
# [PLAN ONLY] 'micro_wrist': [+0.0°, -90.0°, +0.0°, -8.6°, +0.0°, +0.0°]
# OK (plan only): 'micro_wrist'
# [PLAN ONLY] 'safe_return': ...
# OK (plan only): 'safe_return'
```

모두 ✓ OK가 나오면 다음 단계로 진행하세요.

### 6.2 Stage 1: 최소 이동 실행 (5% 속도, 1축)

```bash
# 실행 전 체크리스트 재확인 (Section 3.3)
python3 test_motion_plan_real.py --stage minimal --execute --velocity-scale 0.05
```

**관찰 포인트:**
- TP 속도 다이얼이 반응하는지 확인 (`scaled_joint_trajectory_controller` 동작 검증)
- wrist_1 관절만 ~8.6° 이동하는지 확인
- 이동 속도가 매우 느린지 확인 (5%)

### 6.3 Stage 2: 표준 이동 실행 (5% 속도, 2축)

```bash
python3 test_motion_plan_real.py --stage standard --execute --velocity-scale 0.05
```

**관찰 포인트:**
- shoulder_pan과 wrist_1 두 축이 순서대로 이동하는지 확인
- 각 이동 후 home으로 정확히 복귀하는지 확인

### 6.4 Stage 3: 속도 증가 (10%, 20%)

```bash
# 10% 속도
python3 test_motion_plan_real.py --stage standard --execute --velocity-scale 0.1

# 20% 속도
python3 test_motion_plan_real.py --stage standard --execute --velocity-scale 0.2
```

각 속도에서 이상이 없으면 다음 속도로 진행하세요.

### 6.5 Stage 4: 전체 경로 (경험자용)

```bash
# 먼저 계획만 검증
python3 test_motion_plan_real.py --stage full

# 실행 (20% 속도부터 시작)
python3 test_motion_plan_real.py --stage full --execute --velocity-scale 0.2
```

### 6.6 커스텀 타겟 사용

특정 위치를 테스트하려면 JSON 파일로 정의하세요:

```bash
# custom_targets.json 작성
cat > custom_targets.json << 'EOF'
{
  "my_home":  [0.0, -1.5707, 0.0, 0.0,   0.0, 0.0],
  "my_pose1": [0.2, -1.5707, 0.0, -0.3,  0.0, 0.0],
  "my_home":  [0.0, -1.5707, 0.0, 0.0,   0.0, 0.0]
}
EOF

# 커스텀 타겟으로 계획 검증
python3 test_motion_plan_real.py --targets-file custom_targets.json

# 실행
python3 test_motion_plan_real.py --targets-file custom_targets.json --execute
```

---

## 7. Cartesian 목표 테스트 (실제 로봇)

### 7.1 개요

`cumotion_planner.py`는 Joint-Space 목표와 Cartesian 목표를 모두 지원합니다. `test_motion_plan_real_cartesian.py`는 실제 로봇에서 Cartesian goal을 안전하게 테스트하기 위한 전용 스크립트입니다.

**핵심 원칙:**
- 기본값: plan-only (반드시 `--execute` 플래그를 명시해야 실행됨)
- 기본 이동 거리: 2cm (보수적)
- 기본 속도: 5%
- 실행 전 확인 프롬프트

**Cartesian 목표 동작 원리:**
| 목표 유형 | 인식 조건 | cuMotion 내부 처리 |
|-----------|-----------|-------------------|
| Joint-Space | `joint_constraints` | `plan_single_js()` |
| Cartesian | `position_constraints` + `orientation_constraints` 모두 | `plan_single()` |

> **중요:** Cartesian goal의 `link_name`은 반드시 `"tool0"`이어야 합니다 (cuMotion XRDF ee_link).

### 7.2 Stage 0: Cartesian plan-only 검증 (필수)

로봇을 움직이지 않고 Cartesian 계획이 성공하는지 확인합니다.

```bash
cd /workspaces/tamp_ws/src/tamp_dev

# minimal: home(joint) → +2cm Z (Cartesian) → home(joint) — plan-only
python3 test_motion_plan_real_cartesian.py --stage minimal

# standard: home → +2cm Z → +2cm X → home — plan-only
python3 test_motion_plan_real_cartesian.py --stage standard
```

**예상 출력:**
```
[PLAN ONLY] 'home' → OK (plan only)
[PLAN ONLY] 'home+2cm_z (Cartesian)' → OK (plan only)
[PLAN ONLY] 'home (return)' → OK (plan only)
```

**cuMotion 로그 확인** (Terminal 3):
```
[INFO] Using goal from Pose   ← Cartesian 경로 인식됨
```

### 7.3 Stage 1: minimal Cartesian 실행

plan-only 검증 후 실제 이동을 수행합니다.

```bash
# 실행 전 체크리스트 (Section 3.3) 재확인 필수
python3 test_motion_plan_real_cartesian.py --stage minimal --execute --delta-cm 2.0
```

**확인 프롬프트 예시:**
```
============================================================
  SAFETY CHECK: About to execute Cartesian motion
  Delta: 2.0 cm, Velocity: 5.0%, Accel: 5.0%
  Stage: minimal (1 Cartesian move)
  Type 'yes' to proceed:
============================================================
```

`yes`를 입력해야 실행됩니다.

**관찰 포인트:**
- tool0이 base_link 기준 Z방향으로 2cm 위로 이동하는지 확인
- 방향(orientation)이 유지되는지 확인
- TF2 → joint_states로 실제 이동 위치 검증

### 7.4 Stage 2: standard Cartesian 실행

Z, X 각각 2cm 이동하는 2단계 Cartesian 시퀀스입니다.

```bash
python3 test_motion_plan_real_cartesian.py --stage standard --execute --delta-cm 2.0
```

### 7.5 이동 거리 증가 (경험자용)

충분한 검증 후 이동 거리를 점진적으로 늘립니다.

```bash
# 3cm
python3 test_motion_plan_real_cartesian.py --stage standard --execute --delta-cm 3.0

# 5cm (주의: 속도 다이얼 최저값 유지)
python3 test_motion_plan_real_cartesian.py --stage standard --execute --delta-cm 5.0
```

> 최대 허용 거리: 10cm (`MAX_DELTA_CM = 10.0`). 5cm 초과 시 경고 출력.

### 7.6 주요 옵션

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--stage` | `minimal` | `minimal` / `standard` / `full` |
| `--execute` | 미지정(plan-only) | 지정 시 실행 |
| `--delta-cm` | `2.0` | Cartesian 이동 거리 (cm) |
| `--velocity-scale` | `0.05` | 속도 스케일 (5%) |
| `--accel-scale` | `0.05` | 가속도 스케일 (5%) |

### 7.7 트러블슈팅

**`TF2 lookup failed after 5.0s timeout`:**
```bash
# TF 발행 확인
ros2 topic echo /tf --once
# robot_state_publisher가 실행 중인지 확인 (Terminal 2)
```

**`PLANNING_FAILED` (Cartesian goal):**
- tool0에서 2cm Z 이동이 workspace 경계를 벗어날 경우 발생
- `ros2 topic echo /joint_states --once`로 현재 위치 확인 후 home으로 이동 후 재시도

**`INVALID_LINK_NAME` 오류:**
- `link_name`이 `tool0`이 아닌 경우 — 스크립트는 `tool0`을 하드코딩하므로 일반적으로 발생하지 않음

---

## 8. cuMotion 성능 검증

### 8.1 계획 시간 모니터링

```bash
# cuMotion 로그에서 계획 시간 확인
# Terminal 3 출력에서 확인 가능:
# [INFO] Planning time: X.XXX s
```

cuMotion은 GPU 가속으로 일반적으로 0.1~0.5초 내에 계획을 완료합니다.

### 8.2 trajectory 품질 확인

RViz의 `MotionPlanning` 패널에서 계획된 경로를 시각적으로 확인:
1. `Motion Planning` → `Planning` 탭
2. `Plan` 버튼으로 경로 미리보기
3. 경로가 예상한 방향으로 이동하는지 확인

### 8.3 joint-space 목표 정확도 확인

```bash
# 실행 후 joint_states 확인
ros2 topic echo /joint_states --once

# 목표 위치와 현재 위치 비교 (허용 오차: 0.01 rad ≈ 0.57°)
```

### 8.4 속도 스케일링 동작 확인

```bash
# TP 속도 다이얼을 변경하면서 로봇 실제 속도 변화 확인
ros2 topic echo /speed_scaling_state
```

---

## 9. 비상 정지 절차

### 9.1 즉시 정지 방법 (우선순위 순)

1. **TP(티칭 펜던트) 비상 정지 버튼** — 물리적 E-stop (즉시 전원 차단)
2. **TP 일시정지** — Polyscope의 Stop 버튼 (프로그램 정지)
3. **ROS2 컨트롤러 중지:**
   ```bash
   ros2 control switch_controllers \
     --deactivate scaled_joint_trajectory_controller \
     --deactivate joint_trajectory_controller
   ```
4. **MoveIt2에서 목표 취소:**
   ```bash
   ros2 action send_goal /move_action moveit_msgs/action/MoveGroup \
     "{request: {group_name: ur_manipulator}, planning_options: {plan_only: true}}"
   # (새 plan-only goal로 현재 실행 중인 goal 인터럽트)
   ```

### 9.2 E-stop 후 복구 절차

```
1. TP에서 E-stop 해제 (비상 정지 버튼 잠금 해제)
2. Polyscope에서 "FAULT RESET" 실행
3. 로봇을 안전한 위치로 수동 이동 (TP Free Drive 사용)
4. ROS2 노드 재시작 (Terminal 1 재시작)
5. 컨트롤러 상태 확인 후 재시작
```

### 9.3 Protective Stop 처리

Protective Stop이 발생하면:
```
1. 정지 원인 확인 (힘 제한, 속도 제한, 충돌 감지 등)
2. TP에서 "STOP" → 원인 해결
3. TP에서 "RESUME" 또는 재시작
4. 속도/힘 설정 검토 후 더 낮은 속도로 재시도
```

---

## 10. 트러블슈팅

### 10.1 로봇이 연결되지 않음

**증상:** Terminal 1에서 `Connection refused` 또는 타임아웃

```bash
# 네트워크 연결 확인
ping <ROBOT_IP>

# UR 포트 확인 (30001: Primary, 30002: Secondary, 30003: RT)
nc -zv <ROBOT_IP> 30001
```

해결: Polyscope에서 ExternalControl 프로그램 실행 확인.

### 10.2 CONTROL_FAILED (-4) 오류

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

### 10.3 cuMotion 계획 실패

**증상:** `Planning FAILED` 또는 `max_attempts` 초과

가능한 원인:
- 목표 위치가 self-collision 또는 workspace 경계 밖
- `allowed_planning_time` 부족

```bash
# 계획 시간 증가 (test_motion_plan_real.py에서 변경)
goal.request.allowed_planning_time = 30.0  # 기본값 15.0 → 30.0
```

### 10.4 로봇이 목표 위치에 정확히 도달하지 못함

**증상:** 실행 후 joint_states가 목표와 크게 다름

- MoveIt2 start tolerance 확인 (0.01 rad = 0.57°)
- `ur_controllers.yaml`의 goal tolerance 확인:
  ```yaml
  constraints:
    goal_time: 0.0
    shoulder_pan_joint: { trajectory: 0.2, goal: 0.1 }
  ```
- cuMotion trajectory의 마지막 지점 속도 확인 (non-zero velocity 허용 여부)

### 10.5 속도 스케일링이 동작하지 않음

**증상:** TP 다이얼을 돌려도 속도 변화 없음

```bash
# speed_scaling topic 확인
ros2 topic echo /speed_scaling_state

# 컨트롤러 확인: joint_trajectory_controller는 속도 스케일링 미지원
# scaled_joint_trajectory_controller로 전환 필요
ros2 control list_controllers
```

### 10.6 cuMotion 시작 실패 (`ISAAC_ROS_WS` 오류)

```bash
export ISAAC_ROS_WS=/workspaces/tamp_ws
# 또는 ~/.bashrc에 영구 추가
```

---

## 부록 A: 파라미터 빠른 변경 가이드

### 테스트 스크립트 파라미터

```bash
# 속도/가속도 변경 (0.0~1.0, 1.0 = 최대)
python3 test_motion_plan_real.py --velocity-scale 0.1 --accel-scale 0.1

# 단계 선택
python3 test_motion_plan_real.py --stage minimal    # 1축, ~8.6° (기본)
python3 test_motion_plan_real.py --stage standard   # 2축
python3 test_motion_plan_real.py --stage full        # 전체 경로

# 커스텀 타겟
python3 test_motion_plan_real.py --targets-file targets.json
```

### Joint 단위 참고

| Joint | 이름 | 최대 속도 |
|---|---|---|
| 0 | shoulder_pan_joint | 120°/s (2.094 rad/s) |
| 1 | shoulder_lift_joint | 120°/s (2.094 rad/s) |
| 2 | elbow_joint | 180°/s (3.142 rad/s) |
| 3 | wrist_1_joint | 180°/s (3.142 rad/s) |
| 4 | wrist_2_joint | 180°/s (3.142 rad/s) |
| 5 | wrist_3_joint | 180°/s (3.142 rad/s) |

> **5% 속도 스케일 적용 시:** shoulder: 6°/s, wrist: 9°/s — 매우 느리고 안전합니다.

---

## 부록 B: 안전 타겟 JSON 예시

```json
{
  "_comment": "단위: rad. 순서: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3",
  "home":       [0.0,   -1.5707, 0.0,  0.0,   0.0,  0.0],
  "micro_pan":  [0.1,   -1.5707, 0.0,  0.0,   0.0,  0.0],
  "micro_lift": [0.0,   -1.4207, 0.0,  0.0,   0.0,  0.0],
  "micro_elbow":[0.0,   -1.5707, 0.15, 0.0,   0.0,  0.0],
  "home":       [0.0,   -1.5707, 0.0,  0.0,   0.0,  0.0]
}
```

---

*이 가이드는 mock hardware 환경에서 검증된 cuMotion + UR10e 시스템을 실제 로봇으로 확장하기 위한 절차를 다룹니다. 실제 로봇 테스트 시 항상 안전을 최우선으로 하고, 이상이 발생하면 즉시 비상 정지 절차를 따르세요.*
