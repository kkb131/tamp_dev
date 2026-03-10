# cumotion/ 학습 매뉴얼 — GPU 경로 계획 (난이도 ★★)

> **위치**: `standalone/cumotion/`
> **의존 core 모듈**: `robot_backend`, `trajectory_executor`
> **학습 선수 지식**: [core/ 매뉴얼](../../core/docs/manual.md) §2 (로봇 통신), §3 (궤적 실행)

---

## 1. 왜 cumotion/이 존재하는가

servo/나 teleop 모듈은 **실시간** 제어 (사용자 입력 → 즉시 이동)이다.
하지만 특정 상황에서는 **오프라인 경로 계획**이 필요하다:

- **충돌 회피**: 장애물이 있는 환경에서 안전한 경로 생성
- **시간 최적 경로**: 여러 관절을 동시에 최적으로 움직여 시간 단축
- **사전 검증**: 실행 전에 궤적을 계획하고 안전성을 확인

cumotion/은 NVIDIA의 **cuRobo** GPU 라이브러리를 래핑하여,
**MoveIt 없이** 독립적으로 모션 플래닝을 수행한다.

### teleop과의 관계

```
cumotion: "어디로 갈지" 계획 (오프라인, 충돌 회피)
teleop:   "지금 어떻게 움직일지" 제어 (실시간, 사용자 입력)
```

실무에서는 둘을 조합한다:
1. cumotion으로 목표 위치까지 **자동 이동**
2. 도착 후 teleop으로 **미세 조정**

---

## 2. cuRobo/cuMotion 개념

### cuRobo란?

NVIDIA의 GPU 가속 로봇 모션 플래닝 라이브러리.
**수천 개의 경로 후보**를 GPU에서 병렬로 평가하여 최적 경로를 찾는다.

### 핵심 구성 요소

```
┌─────────────────────────────────────────────────┐
│ StandaloneMotionPlanner (planner.py)             │
│                                                   │
│  ┌──────────────┐    ┌────────────────────────┐  │
│  │ MotionGen    │    │ CollisionChecker       │  │
│  │ (최적화 엔진) │    │ (MESH: GPU 충돌 검사)   │  │
│  │              │    │                        │  │
│  │ graph_plan + │    │ XRDF (로봇 충돌 모델)   │  │
│  │ trajopt      │    │ + cuboid obstacles     │  │
│  └──────┬───────┘    └────────────────────────┘  │
│         │                                         │
│         ▼                                         │
│  결과: InterpolatedPlan (40Hz 궤적)               │
└─────────────────────────────────────────────────┘
          │
          ▼  core/trajectory_executor.py
  resample(125Hz) → execute(servoJ)
```

### ROS2 노드 vs Standalone 차이

| 항목 | ROS2 노드 (`isaac_ros_cumotion`) | Standalone (`planner.py`) |
|------|-------------------------------|--------------------------|
| 의존성 | ROS2 + MoveIt + launch 파일 | Python + cuRobo만 |
| 입력 | MoveIt `MotionPlanRequest` | Python 함수 호출 |
| 충돌 맵 | nvblox ESDF (3D 스캔) | MESH + 수동 cuboid |
| 장점 | RViz 시각화, MoveIt 연동 | 빠른 프로토타이핑, 독립 실행 |

---

## 3. planner.py 핵심 분석

### StandaloneMotionPlanner 초기화

```python
# planner.py:28-79
class StandaloneMotionPlanner:
    def __init__(self, xrdf_path, urdf_path, add_ground_plane=True, ...):
        # 1. 로봇 설정 로드 (XRDF + URDF)
        content_path = ContentPath(
            robot_xrdf_absolute_path=xrdf_path,
            robot_urdf_absolute_path=urdf_path,
        )
        robot_config = load_robot_yaml(content_path)

        # 2. 월드 설정 (기본: 바닥 평면)
        world_dict = {"cuboid": {"table": {
            "pose": [0, 0, -0.05, 1, 0, 0, 0],
            "dims": [2.0, 2.0, 0.1],
        }}}

        # 3. MotionGen 엔진 빌드 (GPU)
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_dict, world_file, tensor_args,
            collision_checker_type=CollisionCheckerType.MESH,
            ...
        )
        self.motion_gen = MotionGen(motion_gen_config)

        # 4. GPU 워밍업 (최초 1회, 수 초 소요)
        self.motion_gen.warmup(enable_graph=True)
```

**XRDF**: cuRobo 전용 로봇 기술 파일. 충돌 구(spheres), 관절 한계, 속도 한계 등을 정의.
**URDF**: 표준 로봇 기술 형식. 기구학 + 시각화용.
두 파일을 함께 사용하여 정확한 충돌 검사와 운동학을 제공한다.

### plan_joint() — 관절 공간 플래닝

```python
# planner.py:81-112
def plan_joint(self, start_joints, goal_joints, velocity_scale=0.5):
    # cuRobo JointState 생성
    start_state = CuJointState.from_position(
        position=self.tensor_args.to_device(start_joints).unsqueeze(0),
        joint_names=self.joint_names,
    )
    goal_state = CuJointState.from_position(...)

    # 플래닝 설정
    plan_config = MotionGenPlanConfig(
        max_attempts=MAX_ATTEMPTS,        # 최대 10회 시도
        enable_graph_attempt=1,            # 1회째부터 graph planner 활성화
        time_dilation_factor=velocity_scale,  # 속도 스케일 (0.05 = 5%)
    )

    # 실행
    result = self.motion_gen.plan_single_js(start_state, goal_state, plan_config)

    if not result.success.item():
        return None
    return self._extract_trajectory(result, plan_time)
```

### plan_cartesian() — 카르테시안 플래닝

```python
# planner.py:114-148
def plan_cartesian(self, start_joints, goal_position, goal_quaternion, ...):
    # 카르테시안 목표: 위치 + 자세 (quaternion)
    goal_pose = Pose(
        position=self.tensor_args.to_device(goal_position).unsqueeze(0),
        quaternion=self.tensor_args.to_device(goal_quaternion).unsqueeze(0),
    )
    result = self.motion_gen.plan_single(start_state, goal_pose, plan_config)
    ...
```

**plan_joint vs plan_cartesian:**

| 항목 | plan_joint | plan_cartesian |
|------|-----------|---------------|
| 목표 지정 | 관절 각도 [6] | 위치[3] + quaternion[4] |
| 내부 함수 | `plan_single_js()` | `plan_single()` |
| 사용 시 | 정확한 관절 구성이 필요할 때 | 엔드이펙터 위치만 중요할 때 |
| IK | 불필요 (이미 관절 공간) | cuRobo 내부에서 자동 수행 |

### _extract_trajectory() — GPU → numpy 변환

```python
# planner.py:150-174
def _extract_trajectory(self, result, plan_time):
    js = result.get_interpolated_plan()    # 보간된 궤적 (40Hz)
    dt = result.interpolation_dt

    positions = js.position.cpu().view(-1, js.position.shape[-1]).numpy()
    velocities = js.velocity.cpu().view(-1, js.velocity.shape[-1]).numpy()

    return {
        "positions": positions,    # (N, 6) ndarray
        "velocities": velocities,  # (N, 6) ndarray
        "dt": dt,                  # 0.025s (40Hz)
        "n_points": len(positions),
        "total_time": timestamps[-1],
        "plan_time": plan_time,
    }
```

`.cpu().numpy()`: GPU 텐서(torch.Tensor)를 CPU numpy 배열로 변환.
`get_interpolated_plan()`: trajopt 결과를 `interpolation_dt` 간격으로 보간한 밀집 궤적.

### add_cuboid() — 장애물 추가

```python
# planner.py:176-181
def add_cuboid(self, name, pose, dims):
    """pose: [x,y,z,qw,qx,qy,qz], dims: [dx,dy,dz]"""
    cuboid = Cuboid(name=name, pose=pose, dims=dims)
    world = WorldConfig(cuboid=[cuboid]).get_collision_check_world()
    self.motion_gen.update_world(world)
```

런타임에 장애물을 추가할 수 있다. 추가된 장애물은 플래닝 시 충돌 검사에 포함된다.

---

## 4. 테스트 스크립트 흐름

### test_standalone.py — 단일 목표

```
HOME → UP (또는 지정 목표) 플래닝
옵션:
  --plan-only     : 로봇 연결 없이 플래닝만 수행 (GPU만 필요)
  --execute       : 실제 궤적 실행
  --goal-type     : joint (관절 공간) 또는 cartesian (카르테시안)
  --velocity-scale: 속도 배율 (default: 0.05 = 5%)
```

**실행 흐름 (plan-only):**
1. `StandaloneMotionPlanner` 초기화 (GPU 워밍업)
2. `plan_joint(HOME, UP)` 호출
3. 궤적 정보 출력 (시간, 포인트 수, 시작/끝 관절)

**실행 흐름 (with backend):**
1. 초기화 + `create_backend()` 연결
2. `robot.get_joint_positions()` → 현재 위치를 시작점으로 사용
3. 플래닝 → `validate_trajectory()` → `check_start_match()`
4. 사용자 확인 → `execute_trajectory()` 실행

### test_multi_goal.py — 다중 목표 순차

```
HOME → A → B → C → D → E → HOME (1 라운드)
config.py의 NEAR_HOME_WAYPOINTS 5개를 순회
```

```python
# test_multi_goal.py:49-58
def build_sequence(rounds, return_home):
    seq = []
    for _ in range(rounds):
        for i, wp in enumerate(NEAR_HOME_WAYPOINTS):
            label = chr(ord("A") + i)
            seq.append((label, wp))
        if return_home:
            seq.append(("HOME", HOME_JOINTS))
    return seq
```

플래너 신뢰성 검증에 유용: 여러 목표를 순차 플래닝하여 **실패율, 평균 계획 시간** 등을 확인.

---

## 5. 궤적 실행 파이프라인

cumotion/은 경로를 **계획**하지만, 실행은 core/trajectory_executor.py가 담당한다.

```
cuMotion planner → trajectory dict → trajectory_executor
   (40Hz, GPU)        {"positions",      (resample 125Hz
                        "dt": 0.025}       + busy-wait)
                                             → servoJ / topic publish
```

### 전체 코드 흐름

```python
# test_standalone.py:94-158 (요약)
planner = StandaloneMotionPlanner(XRDF_PATH, URDF_PATH)

with create_backend(mode, robot_ip=ip) as robot:
    current = robot.get_joint_positions()     # 1. 현재 위치 읽기
    traj = planner.plan_joint(current, goal)  # 2. GPU 플래닝
    validate_trajectory(traj)                  # 3. 안전 검증
    check_start_match(traj["positions"][0], current)  # 4. 시작점 확인
    execute_trajectory(robot, traj, command_dt=SERVOJ_DT)  # 5. 실행
```

`validate_trajectory()`는 속도/가속도 한계를 검사하고,
`check_start_match()`는 로봇이 궤적 시작점에 있는지 확인한다 (tolerance: 0.05 rad).
두 검사를 통과해야 실행이 진행된다.

---

## 6. 주의 사항

### GPU 필수
cuMotion은 NVIDIA GPU가 필수이다. `nvidia-smi`가 동작하지 않으면 사용 불가.
Docker 환경에서는 `--gpus all` 옵션이 필요하다.

### 워밍업 시간
`motion_gen.warmup()`은 최초 1회 실행에 수 초~수십 초 소요.
이후 플래닝은 보통 0.1~1.0초 이내.

### Quaternion 순서
cuRobo는 `[qw, qx, qy, qz]` 순서를 사용한다.
Pinocchio/Pink는 `[x, y, z, w]` 순서. 변환 시 주의.

---

## 다음 단계

- [teleop_admittance/ 매뉴얼](../../teleop_admittance/docs/manual.md) — 실시간 어드미턴스 텔레옵
- [teleop_impedance/ 매뉴얼](../../teleop_impedance/docs/manual.md) — 임피던스 텔레옵

> 실행 방법, CLI 옵션, 트러블슈팅은 [cumotion/docs/user_guide.md](user_guide.md)를 참조하세요.
