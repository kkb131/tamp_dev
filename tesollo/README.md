# Tesollo DG5F Hand Control

Tesollo DG 5F M 핸드 제어 패키지. ROS2 드라이버 또는 Modbus TCP 직접 통신을 지원.

## 구조

```
tesollo/
├── dg5f_ros2_client.py     # ROS2 JointTrajectory 클라이언트 (권장)
├── dg5f_client.py           # Modbus TCP 직접 통신 클라이언트 (pymodbus)
├── receiver.py              # Manus UDP 수신 → 리타게팅 → 핸드 제어 루프
├── retarget.py              # Manus → DG5F 관절 각도 변환 (순수 수학, HW 독립)
├── tesollo_config.py        # YAML 설정 로더
├── config/
│   └── default.yaml         # 기본 설정 (네트워크, 핸드, 리타게팅, 제어)
└── tests/
    ├── test_zero_ros2.py    # ROS2로 전체 관절 0도 이동
    ├── test_zero_position.py # Modbus로 전체 관절 0도 이동
    ├── test_modbus.py       # Modbus TCP 연결 테스트
    ├── test_scan_slave.py   # Modbus slave ID 스캔
    ├── test_retarget.py     # 리타게팅 로직 단위 테스트
    └── test_e2e.py          # End-to-end 파이프라인 테스트 (mock)
```

## ROS2 드라이버 방식 (권장)

### 의존성 설치

```bash
cd /workspaces/tamp_ws/src
git clone https://github.com/tesollodelto/dg5f_ros2.git
git clone https://github.com/tesollodelto/dg_hardware.git
git clone https://github.com/tesollodelto/dg_tcp_comm.git

cd /workspaces/tamp_ws
rosdep install --from-paths src/dg5f_ros2/dg5f_driver --ignore-src -r -y
colcon build --symlink-install --packages-select delto_tcp_comm delto_hardware dg5f_driver dg5f_description
```

### 실행

```bash
# Terminal 1: 드라이버 실행
source /workspaces/tamp_ws/install/setup.bash
ros2 launch dg5f_driver dg5f_right_driver.launch.py delto_ip:=169.254.186.72

# Terminal 2: 0도 이동 테스트
source /workspaces/tamp_ws/install/setup.bash
cd /workspaces/tamp_ws/src/tamp_dev
python3 -m tesollo.tests.test_zero_ros2 --hand right
```

### ROS2 토픽

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/dg5f_right/dg5f_right_controller/joint_trajectory` | `JointTrajectory` | 관절 명령 |
| `/dg5f_right/joint_states` | `JointState` | 관절 피드백 |

## Modbus TCP 방식 (대안)

pymodbus >= 3.10 필요:

```bash
pip install 'pymodbus>=3.10,<4'

cd /workspaces/tamp_ws/src/tamp_dev
python3 -m tesollo.tests.test_zero_position --ip 169.254.186.72
```

## 핸드 사양

- 5 fingers x 4 joints = 20 DOF
- Modbus TCP: IP `169.254.186.72`, Port `502`, Slave ID `1`
- 관절 이름: `rj_dg_{finger}_{joint}` (right), `lj_dg_{finger}_{joint}` (left)
