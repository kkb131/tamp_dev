#!/usr/bin/env python3
"""
cuMotion UR10e 모션 플래닝 테스트.

전제 조건:
  - Terminal 1: ros2 launch ur_robot_driver ur10e.launch.py use_mock_hardware:=true robot_ip:=0.0.0.0
  - Terminal 2: ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e
  - Terminal 3: ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \\
                  cumotion_planner.robot:=<xrdf> cumotion_planner.urdf_path:=<urdf>

Usage:
  python3 test_motion_plan.py               # Stage 1: 경로 이동 테스트
  python3 test_motion_plan.py --obstacle-test  # Stage 2: 장애물 회피 테스트
  python3 test_motion_plan.py --plan-only      # 플래닝만 (실행 안 함)

Joint 순서: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
"""

import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

# UR10e 조인트 이름 (UR SRDF 기준)
UR_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# SRDF named states + 테스트용 위치 (단위: rad)
# home:              수직 방향 대기 자세
# up:                팔을 위로 편 자세
# test_configuration: 측면 비틀린 자세 (SRDF에 정의됨)
# obstacle_test:     전방 장애물(0.7,0,0.5) 측면을 돌아가는 목표
TARGETS = {
    'home':               [2.24, -1.2808, 2.16, -0.8848, 2.24, 0.0],
    'up':                 [2.24, -1.2808, 2.16, 1.56, 2.24, 0.0],
    'test_configuration': [1.54,  -1.62,    1.4,  -1.2,   -1.6,   -0.11],
    # 장애물 회피 테스트: 좌측 경유로 목표 도달 (전방 장애물 우회)
    'obstacle_test':      [-0.5,  -1.0,     1.5,  -2.0,   -1.5707, 0.0],
}

MOVE_GROUP_ACTION = '/move_action'
PLANNING_GROUP = 'ur_manipulator'

# moveit_msgs/MoveItErrorCodes
SUCCESS = 1


def make_joint_goal(values, tol=0.001):
    c = Constraints()
    for name, val in zip(UR_JOINTS, values):
        jc = JointConstraint(
            joint_name=name,
            position=float(val),
            tolerance_above=tol,
            tolerance_below=tol,
            weight=1.0,
        )
        c.joint_constraints.append(jc)
    return c


class MotionPlanTester(Node):
    def __init__(self):
        super().__init__('test_motion_plan')
        self._client = ActionClient(self, MoveGroup, MOVE_GROUP_ACTION)
        self.get_logger().info(f'Waiting for {MOVE_GROUP_ACTION}...')
        if not self._client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError(
                f'{MOVE_GROUP_ACTION} not available. '
                'Is move_group running? (Terminal 2)'
            )
        self.get_logger().info('Connected to move_group.')

    def move_to(self, name: str, joints: list, plan_only: bool = False) -> bool:
        goal = MoveGroup.Goal()

        # MotionPlanRequest
        goal.request.group_name = PLANNING_GROUP
        goal.request.planner_id = ''  # move_group의 default pipeline 사용 (isaac_ros_cumotion)
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.goal_constraints.append(make_joint_goal(joints))

        # PlanningOptions
        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        vals_str = ', '.join(f'{v:.3f}' for v in joints)
        self.get_logger().info(f"→ Target '{name}': [{vals_str}]")

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)

        if not send_future.done():
            self.get_logger().error('Timeout sending goal.')
            return False

        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by move_group.')
            return False

        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        if not result_future.done():
            self.get_logger().error('Timeout waiting for result.')
            return False

        result = result_future.result().result
        ec = result.error_code.val
        if ec == SUCCESS:
            label = '(plan only)' if plan_only else '(executed)'
            self.get_logger().info(f"SUCCESS {label}: '{name}'")
            return True
        else:
            self.get_logger().error(
                f"FAILED '{name}'. MoveItErrorCode={ec}"
            )
            return False


# =============================================================================
# Stage 1: 경로 이동 테스트
# =============================================================================
def run_stage1(node: MotionPlanTester, plan_only: bool) -> bool:
    print('\n' + '=' * 60)
    print('  Stage 1: 경로 이동 테스트 (장애물 없음)')
    print('  home → up → test_configuration → home')
    print('=' * 60)

    sequence = ['home', 'up', 'test_configuration', 'home']
    results = []

    for target in sequence:
        print(f'\n  ▶ {target}')
        ok = node.move_to(target, TARGETS[target], plan_only=plan_only)
        results.append((target, ok))
        if ok:
            time.sleep(1.0)
        else:
            print(f'  ✗ 실패: {target} — 중단.')
            break

    _print_results(results)
    return all(ok for _, ok in results)


# =============================================================================
# Stage 2: 장애물 회피 테스트
# =============================================================================
def run_stage2(node: MotionPlanTester, plan_only: bool) -> bool:
    print('\n' + '=' * 60)
    print('  Stage 2: 장애물 회피 테스트')
    print('  장애물 배치:')
    print('    table:          (0.0,  0.0, -0.025) 2×2×0.05m')
    print('    obstacle_front: (0.7,  0.0,  0.5)   0.2×0.2×0.6m')
    print('    obstacle_right: (0.3, -0.6,  0.4)   0.15×0.15×0.4m')
    print('  경로: home → obstacle_test (장애물 우회) → home')
    print('=' * 60)
    print()
    print('  ※ 장애물 미추가 시 먼저 실행하세요:')
    print('     python3 test_collision_objects.py')

    sequence = ['home', 'obstacle_test', 'home']
    results = []

    for target in sequence:
        print(f'\n  ▶ {target}')
        ok = node.move_to(target, TARGETS[target], plan_only=plan_only)
        results.append((target, ok))
        if ok:
            time.sleep(1.0)
        else:
            print(f'  ✗ 실패: {target}')
            print('     → cuMotion이 장애물 회피 경로를 찾지 못했을 수 있습니다.')
            print('     → RViz MotionPlanning 패널에서 시각적으로 확인하세요.')

    _print_results(results)
    return all(ok for _, ok in results)


def _print_results(results):
    print('\n  Results:')
    for target, ok in results:
        mark = '✓ OK  ' if ok else '✗ FAIL'
        print(f'    {mark}  {target}')


# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description='cuMotion UR10e 모션 플래닝 테스트 (Stage 1 & 2)'
    )
    parser.add_argument(
        '--obstacle-test', action='store_true',
        help='Stage 2: 장애물 회피 테스트 (기본: Stage 1 경로 이동)'
    )
    parser.add_argument(
        '--plan-only', action='store_true',
        help='플래닝만 수행 (trajectory 실행 안 함)'
    )
    args = parser.parse_args()

    rclpy.init()
    success = False
    try:
        node = MotionPlanTester()
        if args.obstacle_test:
            success = run_stage2(node, plan_only=args.plan_only)
        else:
            success = run_stage1(node, plan_only=args.plan_only)
        node.destroy_node()
    except RuntimeError as e:
        print(f'Error: {e}', file=sys.stderr)
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
