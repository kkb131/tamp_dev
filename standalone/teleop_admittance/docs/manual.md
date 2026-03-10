# teleop_admittance/ 학습 매뉴얼 — 어드미턴스 텔레옵 (난이도 ★★★)

> **위치**: `standalone/teleop_admittance/`
> **의존 core 모듈**: `robot_backend`, `pink_ik`, `exp_filter`, `input_handler`, `compliant_control`, `ft_source`, `controller_utils`
> **학습 선수 지식**: [core/ 매뉴얼](../../core/docs/manual.md) §2 (로봇 통신), §4.2 (Pink IK), §6 (필터), §8 (컴플라이언스)

---

## 1. 왜 teleop_admittance/가 존재하는가

servo/ 모듈은 가장 단순한 텔레옵을 제공하지만, 실제 로봇 조작에서는 부족한 점이 있다:

| servo/의 한계 | teleop_admittance/의 해결 |
|-------------|------------------------|
| 관절 한계 무시 (DLS IK) | Pink QP IK로 관절 한계 제약 |
| 외력 대응 불가 | F/T 센서 + 어드미턴스 제어로 외력 순응 |
| 안전 시스템 인라인 | 4단계 독립 SafetyMonitor |
| 설정 하드코딩 | YAML 기반 설정 시스템 |
| 단일 파일 스크립트 | 모듈 분리 (main, safety, admittance, config) |

### 어드미턴스 제어란?

**어드미턴스 제어**는 외력을 감지하여 **위치를 보정**하는 방식이다:

```
외력(F) → 어드미턴스 동역학(M·ẍ + D·ẋ + K·x = F) → 위치 변위(Δx) → IK → 관절 명령
```

인과관계: **힘 → 위치** (Force → Position).
로봇이 외부 힘에 순응하여 부드럽게 밀리는 동작을 구현한다.

---

## 2. 제어 파이프라인 상세

```
Input → ExpFilter → Workspace Clamp → [Admittance Δx] → Pink IK → SafetyMonitor → Robot
  ↑                                        ↑
  keyboard/xbox                       F/T sensor
```

### 10단계 제어 루프 (`_control_loop()`)

```python
# main.py:297-395
while self.running:
    t_start = time.perf_counter()

    # 1. 입력 읽기 — cmd.velocity[6] (linear + angular)
    cmd = self.input_handler.get_command(timeout=0.001)

    # 2. 목표 위치 누적 — 키를 누르고 있으면 계속 이동
    target_pos = target_pos + cmd.velocity[:3]
    target_quat = apply_rotation_delta(target_quat, cmd.velocity[3:], 1.0)

    # 3. 지수 필터 — 급격한 변화 완화
    filt_pos, filt_quat = self.exp_filter.update(target_pos, target_quat)

    # 4. 작업 공간 클램핑 — EE 위치를 안전 범위로 제한
    clamped_pos = self.safety.clamp_workspace(filt_pos)

    # 4.5 어드미턴스 변위 — F/T 센서 기반 위치 보정
    adm_disp = self.admittance.compute_displacement(self.q_current, dt)
    compliant_pos = clamped_pos + adm_disp[:3]

    # 5. Pink IK — 카르테시안 → 관절 공간 변환 (QP, 관절 한계 준수)
    q_target = self.ik.solve(compliant_pos, compliant_quat, dt)

    # 6. 안전 검사 — 속도 제한, 타임아웃, E-Stop
    result = self.safety.check_and_apply(q_target, self.q_current, dt)

    # 7. EE 속도 계산 (디스플레이용)
    # 8. 로봇에 명령 전송 (안전하면 전송, 아니면 현재 위치 유지)
    # 9. 상태 디스플레이 + CSV 로그
    # 10. 루프 타이밍 (dt 맞추기 위한 sleep)
```

**핵심 포인트**: 어드미턴스 변위(4.5단계)는 사용자 입력과 **합산**된다. 사용자가 키보드로 로봇을 이동시키면서 동시에 외력에 의해 순응 동작이 추가된다.

---

## 3. 핵심 모듈 분석

### 3.1 main.py — TeleopController

#### 초기화 순서

```python
# main.py:79-98
class TeleopController:
    def __init__(self, config: TeleopConfig, log_path=None):
        self.exp_filter = ExpFilter(
            config.filter.alpha_position,      # 위치 EMA (0.85)
            config.filter.alpha_orientation,   # 자세 slerp (0.85)
        )
        self.ik = PinkIK(
            config.urdf_path,
            ee_frame="tool0",
            position_cost=config.ik.position_cost,      # 1.0
            orientation_cost=config.ik.orientation_cost,  # 0.5
            posture_cost=config.ik.posture_cost,          # 1e-3
            damping=config.ik.damping,                    # 1e-12
        )
```

- `position_cost > orientation_cost`: 위치 추종이 자세보다 우선
- `posture_cost = 1e-3`: 관절 중립 위치로 복귀 (약한 힘)
- `damping = 1e-12`: QP 수치 안정성용 (거의 0)

#### run() 초기화 흐름

```python
# main.py:223-283
def run(self):
    self.backend = create_backend(cfg.robot.mode, ...)   # 1. 백엔드 생성
    self.input_handler = create_input(cfg.input.type, ...)  # 2. 입력 장치
    if cfg.robot.mode == "sim":
        self._setup_sim_controller()  # 3. sim: forward_position_controller 전환
    # ...
    self.q_current = np.array(self.backend.get_joint_positions())
    self.ik.initialize(self.q_current)          # 4. Pink IK 초기화
    self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)
    self.exp_filter.reset(self.ee_pos, self.ee_quat)  # 5. 필터 초기화
    self.safety = SafetyMonitor(cfg.safety, self.backend)  # 6. 안전 모니터
    self.admittance = AdmittanceLayer(...)       # 7. 어드미턴스 레이어
```

sim 모드에서 ROS2 mock hardware를 사용할 때, `forward_position_controller`로 전환해야 실시간 위치 명령이 가능하다. Isaac Sim은 controller_manager가 없으므로 자동 스킵된다.

### 3.2 admittance_layer.py — AdmittanceLayer

core/의 `AdmittanceController`와 `FTSource`를 래핑하여 텔레옵 파이프라인에 통합하는 역할이다.

#### 핵심 메서드: compute_displacement()

```python
# admittance_layer.py:76-97
def compute_displacement(self, q: np.ndarray, dt: float) -> np.ndarray:
    if not self._enabled:
        return np.zeros(6)

    # 1. 도구 프레임에서 렌치(wrench) 읽기
    wrench_tool = self._ft_source.get_wrench()

    # 2. 도구 프레임 → 기저 프레임 변환
    _, R = self._kin.get_ee_pose(q)  # FK로 현재 EE 회전 행렬
    f_base = R @ wrench_tool[:3]      # 힘 벡터 변환
    t_base = R @ wrench_tool[3:]      # 토크 벡터 변환
    wrench_base = np.concatenate([f_base, t_base])

    # 3. 어드미턴스 동역학 계산 (core/compliant_control.py)
    return self._controller.update(wrench_base, dt)
```

**왜 프레임 변환이 필요한가?**
F/T 센서는 도구(tool) 프레임에서 측정하지만, 로봇의 위치 제어는 기저(base) 프레임에서 이루어진다. 예: 로봇 손목이 아래를 향하고 있을 때, 도구 프레임의 +Z 힘은 기저 프레임의 -Z가 된다. 변환 없이 사용하면 로봇이 반대 방향으로 움직인다.

#### 모드별 동작

```python
# admittance_layer.py:38-42
if mode == "rtde":
    self._ft_source: FTSource = RTDEFTSource(backend)   # 실제 F/T 센서
else:
    self._ft_source = NullFTSource()   # 항상 0 반환 → 어드미턴스 무효
```

- **RTDE 모드**: 실제 F/T 센서 데이터 사용 → 어드미턴스 활성 가능
- **Sim 모드**: `NullFTSource` → 항상 0 렌치 → `compute_displacement()`도 항상 0 반환

#### 런타임 제어

| 키 | 동작 | 코드 |
|----|------|------|
| `t` | 어드미턴스 ON/OFF 토글 | `toggle()` |
| `z` | F/T 센서 제로잉 | `zero_sensor()` |
| `1/2/3` | STIFF/MEDIUM/SOFT 프리셋 | `set_preset("STIFF")` 등 |

### 3.3 safety_monitor.py — SafetyMonitor

4단계 안전 시스템. 모든 결과는 `SafetyResult` 데이터클래스로 반환된다.

```python
@dataclass
class SafetyResult:
    is_safe: bool
    q_safe: np.ndarray     # 안전한 관절 명령 (스케일링 적용 후)
    level: str = "OK"       # "OK", "TIMEOUT", "VEL_LIMIT", "WS_CLAMP", "ESTOP"
    message: str = ""
```

#### 4단계 계층 구조

| 우선순위 | 레벨 | 검사 대상 | 동작 |
|---------|------|---------|------|
| 4 (최고) | ESTOP | 사용자 Space 키 | 즉시 정지, 수동 리셋(R키) 필요 |
| 3 | WS_CLAMP | EE 위치 | 작업 공간 경계로 클램핑 (거부 아닌 제한) |
| 2 | VEL_LIMIT | 관절 속도 | 속도 비례 스케일링 `q_safe = q_current + Δq * scale` |
| 1 | TIMEOUT | 입력 간격 | 200ms 무입력 시 speed_stop() |

#### 속도 스케일링 핵심 코드

```python
# safety_monitor.py:117-126
joint_vel = (q_target - q_current) / dt
max_vel = np.max(np.abs(joint_vel))

if max_vel > self._config.max_joint_vel:
    scale = self._config.max_joint_vel / max_vel   # 0~1 사이 비율
    q_safe = q_current + (q_target - q_current) * scale
```

**모든 관절을 동일 비율로 스케일링**: 특정 관절만 줄이면 카르테시안 경로가 왜곡된다. 전체를 같은 비율로 줄여 직선 경로를 유지한다.

### 3.4 teleop_config.py — TeleopConfig

YAML → Python 데이터클래스 매핑.

```python
# teleop_config.py:86-107
@dataclass
class TeleopConfig:
    robot: RobotConfig           # ip, mode
    control: ControlConfig       # frequency_sim(50Hz), frequency_rtde(125Hz)
    input: InputConfig           # type, cartesian_step, rotation_step
    filter: FilterConfig         # alpha_position, alpha_orientation
    ik: IKConfig                 # position_cost, orientation_cost, damping
    safety: SafetyConfig         # timeout, max_vel, workspace
    admittance: AdmittanceConfig # preset, max_displacement, deadzone

    @property
    def frequency(self) -> int:
        if self.robot.mode == "rtde":
            return self.control.frequency_rtde   # 125Hz (servoJ에 맞춤)
        return self.control.frequency_sim         # 50Hz (시뮬레이션 충분)
```

`frequency` 프로퍼티가 모드에 따라 자동 전환되므로, 나머지 코드는 `config.frequency`만 참조하면 된다.

#### YAML 설정 주요 항목 (config/default.yaml)

```yaml
safety:
  packet_timeout_ms: 200     # admittance: 200ms (위치 제어이므로 여유)
  max_joint_vel: 0.5         # rad/s — 보수적 (safety first)
  max_ee_velocity: 0.1       # m/s

admittance:
  enabled_by_default: false        # 't' 키로 수동 활성화
  default_preset: "MEDIUM"         # STIFF / MEDIUM / SOFT
  max_displacement_trans: 0.05     # 최대 5cm 변위
  force_deadzone: [3.0, 3.0, 3.0, 0.3, 0.3, 0.3]  # N / Nm
```

- `force_deadzone`: 이 이하의 힘은 무시 (센서 노이즈 필터링)
- `max_displacement_trans`: 어드미턴스 변위 포화값 (안전 장치)

---

## 4. 어드미턴스 vs 임피던스: 근본적 차이

이 모듈(teleop_admittance)과 다음 모듈(teleop_impedance)의 근본적 차이를 이해하는 것이 중요하다.

| 항목 | 어드미턴스 (이 모듈) | 임피던스 (teleop_impedance) |
|------|---------------------|--------------------------|
| 인과관계 | F → x (힘 → 위치) | x → τ (위치 → 토크) |
| 제어 출력 | 위치 명령 (servoJ) | 토크 명령 (direct_torque) |
| 순응성 구현 | 소프트웨어 동역학 시뮬레이션 | 물리적 스프링-댐퍼 (PD 토크) |
| F/T 센서 | **필수** (외력 측정) | 불필요 (위치 오차가 곧 힘) |
| 안전성 | 높음 (위치 제어 기반) | 낮음 (토크 직접 제어) |
| 접촉 품질 | 간접적 (센서→계산→명령) | 직접적 (물리적 컴플라이언스) |

**비유**:
- 어드미턴스 = "센서로 힘을 측정하고, 컴퓨터가 계산해서, 로봇에게 새 위치를 알려줌"
- 임피던스 = "스프링으로 연결된 것처럼, 위치 차이가 곧바로 힘이 됨"

---

## 5. 주요 패턴과 설계 결정

### 영속적 목표(Persistent Target) 패턴

```python
# main.py:303-304
target_pos = self.ee_pos.copy()   # 초기화: 현재 EE 위치
target_quat = self.ee_quat.copy()

# main.py:344-345
target_pos = target_pos + cmd.velocity[:3]   # 누적 (+=)
target_quat = apply_rotation_delta(target_quat, cmd.velocity[3:], 1.0)
```

키를 누르면 `target_pos`가 **계속 누적**된다. 키를 놓으면 `cmd.velocity = 0`이므로 목표가 유지된다.
이전 루프에서 얼마나 이동했는지와 무관하게, 목표 자체가 독립적으로 전진한다.

### E-Stop 후 상태 재동기화

```python
# main.py:321-329
if cmd.reset:
    self.safety.reset_estop()
    self.q_current = np.array(self.backend.get_joint_positions())
    self.ik.sync_configuration(self.q_current)      # Pink IK 상태 리셋
    self.ee_pos, self.ee_quat = self.ik.get_ee_pose(self.q_current)
    self.exp_filter.reset(self.ee_pos, self.ee_quat) # 필터 리셋
    self.admittance.reset()                           # 어드미턴스 리셋
    target_pos = self.ee_pos.copy()                  # 목표도 현재 위치로
```

E-Stop 해제 시 모든 상태를 현재 로봇 위치로 **재동기화**한다. 이를 빠뜨리면 누적된 목표와 현재 위치의 차이로 인해 로봇이 급격히 점프한다.

---

## 다음 단계

- [teleop_impedance/ 매뉴얼](../../teleop_impedance/docs/manual.md) — 임피던스 텔레옵 (토크 직접 제어)
- [core/ 매뉴얼](../../core/docs/manual.md) — 공유 인프라 심화 학습

> 실행 방법, CLI 옵션, 트러블슈팅은 [teleop_admittance/docs/user_guide.md](user_guide.md)를 참조하세요 (있는 경우).
