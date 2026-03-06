#!/bin/bash
# switch_to_cumotion.sh
#
# move_group의 default_planning_pipeline을 isaac_ros_cumotion으로 전환합니다.
# cuMotion planner 노드(Terminal 3) 실행 후 사용하세요.
#
# 사용법:
#   bash switch_to_cumotion.sh          # cuMotion으로 전환
#   bash switch_to_cumotion.sh ompl     # OMPL로 복원
#   bash switch_to_cumotion.sh status   # 현재 상태 확인

set -e

PIPELINE="${1:-isaac_ros_cumotion}"

if [ "$PIPELINE" = "status" ]; then
    echo "Current default_planning_pipeline:"
    ros2 param get /move_group default_planning_pipeline
    exit 0
fi

echo "Switching default_planning_pipeline → ${PIPELINE}"
ros2 param set /move_group default_planning_pipeline "$PIPELINE"
echo "Done. Verifying:"
ros2 param get /move_group default_planning_pipeline
