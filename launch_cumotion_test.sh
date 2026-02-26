#!/bin/bash
# =============================================================================
# launch_cumotion_test.sh - cuMotion UR10e Mock Hardware 테스트 실행 가이드
#
# Usage:
#   bash launch_cumotion_test.sh          # 실행 지침 출력
#   bash launch_cumotion_test.sh --tmux   # tmux로 자동 실행 (tmux 설치 필요)
# =============================================================================

set -uo pipefail

WS=/workspaces/tamp_ws
TAMP_DEV="${WS}/src/tamp_dev"
ASSETS="${TAMP_DEV}/.docker/assets"

# workspace가 빌드되어 있는지 확인
if [ ! -f "${WS}/install/setup.bash" ]; then
    echo "ERROR: Workspace not built. Run:"
    echo "  cd ${WS} && colcon build --symlink-install"
    exit 1
fi

# URDF 캐시 확인 (생성이 필요한 경우)
if [ ! -f "${ASSETS}/ur10e.urdf" ]; then
    echo "WARNING: ${ASSETS}/ur10e.urdf not found. Please run in a sourced shell:"
    echo "  source ${WS}/install/setup.bash"
    echo "  xacro \$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro ur_type:=ur10e name:=ur > ${ASSETS}/ur10e.urdf"
fi

# 경로 직접 계산 (ros2 pkg prefix 대신 install 경로 사용)
XRDF_PATH="${WS}/install/isaac_ros_cumotion_robot_description/share/isaac_ros_cumotion_robot_description/xrdf/ur10e.xrdf"
URDF_PATH="${ASSETS}/ur10e.urdf"

SETUP="source ${WS}/install/setup.bash"
CMD_T1="${SETUP} && ros2 launch ur_robot_driver ur10e.launch.py use_mock_hardware:=true robot_ip:=0.0.0.0"
CMD_T2="${SETUP} && ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e"
CMD_T3="${SETUP} && ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py cumotion_planner.robot:=${XRDF_PATH} cumotion_planner.urdf_path:=${URDF_PATH}"

# =============================================================================
# tmux 자동 실행 모드
# =============================================================================
if [ "${1:-}" = "--tmux" ]; then
    if ! command -v tmux &>/dev/null; then
        echo "ERROR: tmux not installed. Using manual mode."
        exit 1
    fi

    SESSION="cumotion_test"

    if tmux has-session -t "${SESSION}" 2>/dev/null; then
        echo "[tmux] Session '${SESSION}' already exists. Attaching..."
        tmux attach-session -t "${SESSION}"
        exit 0
    fi

    echo "[tmux] Creating session: ${SESSION}"

    # Window 0: UR driver (mock hardware)
    tmux new-session -d -s "${SESSION}" -n "ur_driver" -x 220 -y 50
    tmux send-keys -t "${SESSION}:0" "${CMD_T1}" Enter

    # Window 1: MoveIt + RViz (5초 대기)
    tmux new-window -t "${SESSION}:1" -n "moveit_rviz"
    tmux send-keys -t "${SESSION}:1" "sleep 5 && ${CMD_T2}" Enter

    # Window 2: cuMotion planner (10초 대기)
    tmux new-window -t "${SESSION}:2" -n "cumotion"
    tmux send-keys -t "${SESSION}:2" "sleep 10 && ${CMD_T3}" Enter

    # Window 3: 테스트 터미널
    tmux new-window -t "${SESSION}:3" -n "test"
    tmux send-keys -t "${SESSION}:3" "${SETUP} && echo '[Test terminal ready]'" Enter

    tmux select-window -t "${SESSION}:3"
    tmux attach-session -t "${SESSION}"
    exit 0
fi

# =============================================================================
# 수동 실행 가이드 출력
# =============================================================================
cat << EOF
============================================================
  cuMotion UR10e Mock Hardware Test - 실행 가이드
  (또는: bash ${TAMP_DEV}/launch_cumotion_test.sh --tmux)
============================================================

3개 터미널을 열어 순서대로 실행하세요.
(각 명령 사이 약 5초 대기)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[Terminal 1] UR10e Mock Hardware Driver
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ${CMD_T1}

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[Terminal 2] MoveIt2 + RViz  (Terminal 1 시작 후 실행)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ${CMD_T2}

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[Terminal 3] cuMotion Planner Node  (move_group 시작 후 실행)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ${CMD_T3}

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[Stage 1] 경로 이동 테스트  (새 터미널)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ${SETUP}
  python3 ${TAMP_DEV}/test_motion_plan.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[Stage 2] 장애물 회피 테스트  (새 터미널)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  # 1. 장애물 추가
  python3 ${TAMP_DEV}/test_collision_objects.py

  # 2. 회피 경로 테스트
  python3 ${TAMP_DEV}/test_motion_plan.py --obstacle-test

  # 3. 장애물 제거
  python3 ${TAMP_DEV}/test_collision_objects.py --clear

============================================================
XRDF : ${XRDF_PATH}
URDF : ${URDF_PATH}
============================================================
EOF
