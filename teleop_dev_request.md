# UR 로봇 텔레오퍼레이션 테스트 시스템 개발 요청

## 프로젝트 개요

UR 로봇팔을 키보드 또는 Xbox 컨트롤러로 실시간 제어하는 텔레오퍼레이션 테스트 시스템을 개발해줘.
무선 통신 결합 전 로봇 PC 단독 테스트용이야.

---

## 제어 파이프라인 (순서대로 구현)

```
키보드 or Xbox 컨트롤러 입력
        ↓
Exponential Filter (jitter 흡수)
        ↓
PINK IK (CPU 기반 실시간 IK)
        ↓
Safety Monitor (4단계)
        ↓
ur_rtde servoJ (500Hz)
        ↓
UR 로봇
```

---

## 개발 환경

- OS: Ubuntu 22.04
- Python: 3.10+
- 로봇: UR (UR10e 기준, 다른 모델도 설정으로 변경 가능하게)
- 로봇 IP: 설정 파일로 분리 (기본값 192.168.1.100)
- 필요 라이브러리 설치:
  ```bash
  pip install pin pink inputs pygame ur_rtde numpy
  ```

---

## 디렉토리 구조

```
teleop_test/
├── config/
│   └── config.yaml          # 로봇 IP, 안전 파라미터 등 설정
├── src/
│   ├── input_handler.py     # 키보드/Xbox 입력 처리
│   ├── exp_filter.py        # Exponential Filter
│   ├── ik_controller.py     # PINK IK 제어기
│   ├── safety_monitor.py    # Safety Monitor 4단계
│   └── robot_interface.py   # ur_rtde 래퍼
├── main.py                  # 메인 실행 파일
└── README.md
```

---

## 각 모듈 상세 명세

### 1. config/config.yaml

```yaml
robot:
  ip: "192.168.1.100"
  model: "ur5e"            # ur3e, ur5e, ur10e, ur16e
  urdf_path: "urdf/ur5e.urdf"

control:
  frequency: 500           # Hz, servoJ 제어 주기
  lookahead_time: 0.1      # servoJ lookahead
  gain: 300                # servoJ gain

input:
  type: "keyboard"         # "keyboard" or "xbox"
  cartesian_step: 0.005    # 키 1회 입력 시 이동량 (m)
  rotation_step: 0.05      # 키 1회 입력 시 회전량 (rad)
  xbox_linear_scale: 0.02  # Xbox 스틱 → 선속도 스케일
  xbox_angular_scale: 0.05 # Xbox 스틱 → 각속도 스케일

filter:
  alpha: 0.7               # Exponential filter 계수 (0~1, 클수록 반응 빠름)

safety:
  packet_timeout_ms: 200   # 이 이상 입력 없으면 정지
  max_joint_vel: 0.5       # rad/s
  max_joint_acc: 0.3       # rad/s²
  max_ee_velocity: 0.1     # m/s
  workspace:
    x: [-0.8, 0.8]         # 로봇 베이스 기준 (m)
    y: [-0.8, 0.8]
    z: [0.05, 1.2]
```

---

### 2. src/input_handler.py

**키보드 입력 매핑:**

| 키 | 동작 |
|---|---|
| W / S | end-effector X축 +/- |
| A / D | end-effector Y축 +/- |
| Q / E | end-effector Z축 +/- |
| I / K | end-effector Roll +/- |
| J / L | end-effector Pitch +/- |
| U / O | end-effector Yaw +/- |
| Space | E-Stop 발동 |
| R | E-Stop 해제 (reset) |
| ESC | 프로그램 종료 |

**Xbox 컨트롤러 매핑:**

| 입력 | 동작 |
|---|---|
| 왼쪽 스틱 (X/Y) | end-effector X/Y 이동 |
| 왼쪽 트리거/범퍼 | end-effector Z -/+ |
| 오른쪽 스틱 (X/Y) | Roll/Pitch 회전 |
| 오른쪽 트리거/범퍼 | Yaw +/- |
| B 버튼 | E-Stop 발동 |
| A 버튼 | E-Stop 해제 |
| Start | 프로그램 종료 |

**출력 형식:** 6D velocity command `[dx, dy, dz, droll, dpitch, dyaw]` numpy array (단위: m/frame, rad/frame)

구현 시 `pygame` 라이브러리로 Xbox 컨트롤러 지원.
키보드는 논블로킹 방식으로 구현 (pynput 또는 pygame).

---

### 3. src/exp_filter.py

Exponential Moving Average Filter 구현:

```
filtered = alpha * new_value + (1 - alpha) * prev_filtered
```

- 입력: 6D target pose (position + quaternion)
- position과 orientation을 **분리해서** 필터링
  - orientation은 quaternion slerp 방식으로 필터링 (단순 선형 평균 금지)
- alpha 값은 config에서 로드
- `reset(current_pose)` 메서드: 현재 로봇 pose로 필터 초기화

---

### 4. src/ik_controller.py

**PINK 라이브러리** 기반 IK 구현.

- `pin` (pinocchio) + `pink` 라이브러리 사용
- URDF는 `config.yaml`에서 경로 로드
- 매 루프마다 `pink.solve_ik()` 호출

Task 구성:
```python
tasks = [
    pink.tasks.FrameTask(
        "tool0",              # UR URDF end-effector 프레임명
        position_cost=1.0,
        orientation_cost=0.5,
    ),
    pink.tasks.PostureTask(
        cost=1e-3             # 기본 자세 유지 (중복 자유도 처리)
    ),
]
```

- 특이점 근처에서도 안정적으로 동작하도록 `damping` 설정
- 매 스텝 `config.integrate(velocity, dt)` 로 joint 업데이트
- IK 실패 시 (solution 없음) 현재 joint 유지 + 경고 로그

---

### 5. src/safety_monitor.py

4단계 안전 레이어를 하나의 클래스로 통합:

```python
class SafetyMonitor:
    def check_and_apply(
        self,
        q_target,        # IK 결과 joint 목표값
        q_current,       # 현재 joint 값
        ee_pose_target,  # 목표 end-effector pose
        dt               # 제어 주기
    ) -> tuple[bool, np.ndarray]:
        # Returns: (is_safe, q_safe)
        # is_safe=False 이면 로봇 정지 명령 이미 내림
```

**Level 1: 패킷 타임아웃**
- 마지막 입력으로부터 `config.safety.packet_timeout_ms` 초과 시
- `rtde_c.speedStop(deceleration=2.0)` 호출
- 신호 복구 시 자동 재개

**Level 2: 속도 제한**
- joint velocity 계산 = `(q_target - q_current) / dt`
- `config.safety.max_joint_vel` 초과 시 스케일링으로 클리핑
- `config.safety.max_ee_velocity` 초과 시도 체크

**Level 3: 작업공간 제한 (클램핑 방식)**
- `ee_pose_target` position이 `config.safety.workspace` 벗어나면
- 경계값으로 클램핑 후 IK 재계산 (차단하지 않고 경계에 붙임)
- 클램핑 발생 시 터미널에 경고 표시

**Level 4: E-Stop**
- `trigger()`: 즉시 `rtde_c.stopJ(deceleration=5.0)` 호출
- `reset()`: 조작자가 명시적으로 호출해야만 재동작
- E-Stop 상태일 때 `check_and_apply()` 는 항상 `(False, q_current)` 반환

---

### 6. src/robot_interface.py

ur_rtde 래퍼 클래스:

```python
class URRobotInterface:
    def __init__(self, ip, frequency=500)
    def connect(self) -> bool
    def disconnect(self)
    def get_joint_positions(self) -> np.ndarray   # shape: (6,)
    def get_ee_pose(self) -> pin.SE3               # pinocchio SE3
    def servo_j(self, q_target, vel, acc, t, lookahead, gain)
    def speed_stop(self, deceleration=2.0)
    def stop_j(self, deceleration=5.0)
    def is_connected(self) -> bool
```

- `RTDEControlInterface` + `RTDEReceiveInterface` 모두 초기화
- 연결 실패 시 재시도 로직 (3회)
- 연결 상태 모니터링 (별도 스레드 또는 heartbeat)

---

### 7. main.py

전체 파이프라인 통합 및 실행:

```python
# 제어 루프 구조 (500Hz)
while running:
    # 1. 입력 읽기
    delta_cmd = input_handler.get_command()   # [dx,dy,dz,dr,dp,dy]
    
    # 2. 현재 로봇 상태
    q_current = robot.get_joint_positions()
    ee_current = robot.get_ee_pose()
    
    # 3. 목표 pose 계산 (현재 pose에 delta 적용)
    ee_target_raw = apply_delta(ee_current, delta_cmd)
    
    # 4. Exponential Filter
    ee_target = exp_filter.update(ee_target_raw)
    
    # 5. PINK IK
    q_target = ik_controller.solve(ee_target, q_current)
    
    # 6. Safety Monitor
    is_safe, q_safe = safety_monitor.check_and_apply(
        q_target, q_current, ee_target, dt=1/500
    )
    
    # 7. 로봇 명령
    if is_safe:
        robot.servo_j(q_safe, ...)
    
    # 8. 루프 타이밍 유지
    rate.sleep()
```

터미널 상태 출력 (0.1초마다 갱신):
```
=== UR Teleop Test ===
Mode    : keyboard
EE Pos  : x=0.312  y=-0.145  z=0.487
EE RPY  : r=180.0  p=0.0     y=90.0
Joint   : [0.00, -1.57, 1.57, -1.57, -1.57, 0.00] rad
Velocity: 0.023 m/s
Filter α: 0.70
Safety  : ✅ OK  (workspace: OK, vel: OK, timeout: 45ms)
E-Stop  : ❌ 비활성  (Space: 발동 / R: 해제)
```

---

## 추가 요구사항

### URDF 자동 다운로드
- `ur_description` ROS 패키지가 없는 경우를 대비해
- UR 공식 GitHub에서 URDF를 직접 다운로드하는 헬퍼 스크립트 포함
  ```bash
  python download_urdf.py --model ur5e
  ```

### 시뮬레이션 모드 (--sim 플래그)
- 실제 로봇 없이 테스트할 수 있도록
- `--sim` 플래그 시 ur_rtde 없이 pinocchio 모델만으로 시뮬레이션
- 터미널에 joint 값만 출력

### 로그 저장
- `--log` 플래그 시 CSV 저장
  - timestamp, ee_pos(x,y,z), ee_rpy, joint_angles(6), velocity, safety_status
  - 나중에 무선 통신 레이턴시 분석에 활용 가능하도록

### 에러 처리
- ur_rtde 연결 끊김 시 graceful shutdown
- PINK IK 수렴 실패 시 현재 joint 유지 (로봇 정지 아님)
- 설정 파일 없을 시 기본값으로 동작

---

## 실행 방법 (README에 포함)

```bash
# 설치
pip install pin pink inputs pygame ur_rtde numpy pyyaml

# 키보드 모드 (실제 로봇)
python main.py --input keyboard --robot-ip 192.168.1.100

# Xbox 모드 (실제 로봇)
python main.py --input xbox --robot-ip 192.168.1.100

# 시뮬레이션 모드 (로봇 없이 테스트)
python main.py --input keyboard --sim

# 로그 저장
python main.py --input keyboard --sim --log
```

---

## 개발 우선순위

1. **필수 구현** (MVP)
   - [ ] config.yaml 로드
   - [ ] 키보드 입력 (논블로킹)
   - [ ] Exponential Filter
   - [ ] PINK IK (UR5e URDF 기준)
   - [ ] Safety Monitor 4단계
   - [ ] ur_rtde servoJ 연동
   - [ ] --sim 모드

2. **선택 구현** (시간 여유 시)
   - [ ] Xbox 컨트롤러 지원
   - [ ] 터미널 상태 UI (curses)
   - [ ] CSV 로깅
   - [ ] URDF 자동 다운로드 헬퍼

---

## 참고 사항

- PINK IK 공식 문서: https://github.com/stephane-caron/pink
- ur_rtde 문서: https://sdurobotics.gitlab.io/ur_rtde/
- UR URDF: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
- servoJ의 `t` 파라미터는 제어 주기(1/frequency)로 설정
- servoJ의 `lookahead_time`은 0.03~0.2 사이 튜닝 필요
- pinocchio의 SE3는 `pin.SE3(rotation_matrix, translation_vector)` 형태
