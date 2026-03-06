#!/usr/bin/env python3
"""
UR10e 초기 위치 이동 스크립트 — Mock Hardware 버전.

go_to_init_pose.py의 mock hardware 버전.
안전 확인 프롬프트 없음, 기본 속도 50%, 기본값 plan-only.

초기 위치 (rad):
  shoulder_pan : 2.24
  shoulder_lift: -1.2808
  elbow        : 2.16
  wrist_1      : -0.8848
  wrist_2      : 2.24
  wrist_3      : 0.0

전제 조건:
  Terminal 1: ros2 launch ur_robot_driver ur10e.launch.py use_fake_hardware:=true robot_ip:=0.0.0.0
  Terminal 2: ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e
  Terminal 3: ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \\
                cumotion_planner.robot:=<xrdf> cumotion_planner.urdf_path:=<urdf>

사용법:
  # 계획만 확인 (기본값):
  python3 go_to_init_pose_mock.py

  # 실행 포함:
  python3 go_to_init_pose_mock.py --execute

  # 속도 조정:
  python3 go_to_init_pose_mock.py --execute --velocity-scale 0.3

Joint 순서: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
단위: rad
"""

import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

UR_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

INIT_POSE = [2.24, -1.2808, 2.16, -0.8848, 2.24, 0.0]

DEFAULT_VELOCITY_SCALE = 0.5
DEFAULT_ACCEL_SCALE    = 0.5

MOVE_GROUP_ACTION = '/move_action'
PLANNING_GROUP    = 'ur_manipulator'
SUCCESS           = 1


def deg(rad_val: float) -> float:
    return math.degrees(rad_val)


def make_joint_goal(values: list, tol: float = 0.001) -> Constraints:
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


class InitPoseMover(Node):
    def __init__(self, velocity_scale: float, accel_scale: float):
        super().__init__('go_to_init_pose_mock')
        self.velocity_scale = velocity_scale
        self.accel_scale    = accel_scale

        self._client = ActionClient(self, MoveGroup, MOVE_GROUP_ACTION)
        self.get_logger().info(
            f'move_group 연결 중... '
            f'(속도={velocity_scale*100:.0f}%, 가속도={accel_scale*100:.0f}%)'
        )
        if not self._client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError(
                f'{MOVE_GROUP_ACTION} 서버를 찾을 수 없습니다.\n'
                'move_group이 실행 중인지 확인하세요. (Terminal 2)'
            )
        self.get_logger().info('move_group 연결 완료.')

    def move_to_init(self, execute: bool = False) -> bool:
        plan_only = not execute
        goal = MoveGroup.Goal()

        goal.request.group_name                      = PLANNING_GROUP
        goal.request.planner_id                      = ''
        goal.request.num_planning_attempts           = 5
        goal.request.allowed_planning_time           = 15.0
        goal.request.max_velocity_scaling_factor     = float(self.velocity_scale)
        goal.request.max_acceleration_scaling_factor = float(self.accel_scale)
        goal.request.goal_constraints.append(make_joint_goal(INIT_POSE))

        goal.planning_options.plan_only      = plan_only
        goal.planning_options.replan         = True
        goal.planning_options.replan_attempts = 3

        degs_str = ', '.join(f'{deg(v):+.2f}°' for v in INIT_POSE)
        mode = 'PLAN ONLY' if plan_only else 'EXECUTE'
        self.get_logger().info(f'[{mode}] 초기 위치: [{degs_str}]')

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)

        if not send_future.done():
            self.get_logger().error('목표 전송 타임아웃.')
            return False

        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error('move_group이 목표를 거부했습니다.')
            return False

        self.get_logger().info('목표 수락됨. 결과 대기 중...')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        if not result_future.done():
            self.get_logger().error('결과 대기 타임아웃.')
            return False

        result = result_future.result().result
        ec = result.error_code.val
        if ec == SUCCESS:
            label = '(plan only)' if plan_only else '(executed)'
            self.get_logger().info(f'초기 위치 이동 성공 {label}')
            return True
        else:
            self.get_logger().error(f'초기 위치 이동 실패. MoveItErrorCode={ec}')
            return False


def main():
    parser = argparse.ArgumentParser(
        description='UR10e 초기 위치 이동 — Mock Hardware 버전 (안전 확인 없음)',
    )
    parser.add_argument(
        '--execute',
        action='store_true',
        default=False,
        help='실행까지 수행 (없으면 plan-only)',
    )
    parser.add_argument(
        '--velocity-scale',
        type=float,
        default=DEFAULT_VELOCITY_SCALE,
        metavar='SCALE',
        help=f'최대 속도 스케일 (0.0~1.0, 기본값: {DEFAULT_VELOCITY_SCALE})',
    )
    parser.add_argument(
        '--accel-scale',
        type=float,
        default=DEFAULT_ACCEL_SCALE,
        metavar='SCALE',
        help=f'최대 가속도 스케일 (0.0~1.0, 기본값: {DEFAULT_ACCEL_SCALE})',
    )
    args = parser.parse_args()

    if not 0.0 < args.velocity_scale <= 1.0:
        print(f'Error: --velocity-scale은 (0.0, 1.0] 범위여야 합니다. 입력값: {args.velocity_scale}')
        sys.exit(1)
    if not 0.0 < args.accel_scale <= 1.0:
        print(f'Error: --accel-scale은 (0.0, 1.0] 범위여야 합니다. 입력값: {args.accel_scale}')
        sys.exit(1)

    print('\n' + '=' * 60)
    print('  UR10e 초기 위치 이동 (Mock Hardware)')
    print('=' * 60)
    print(f'  Mode       : {"EXECUTE" if args.execute else "PLAN ONLY"}')
    print(f'  Vel. scale : {args.velocity_scale * 100:.0f}%')
    print(f'  Accel scale: {args.accel_scale * 100:.0f}%')
    print()
    print('  목표 초기 위치:')
    labels = ['shoulder_pan', 'shoulder_lift', 'elbow',
              'wrist_1',      'wrist_2',       'wrist_3']
    for label, val in zip(labels, INIT_POSE):
        print(f'    {label:14s}: {val:+.4f} rad  ({deg(val):+.2f}°)')
    print()

    rclpy.init()
    success = False
    try:
        node = InitPoseMover(args.velocity_scale, args.accel_scale)
        success = node.move_to_init(execute=args.execute)
        node.destroy_node()
    except RuntimeError as e:
        print(f'\nError: {e}', file=sys.stderr)
    except KeyboardInterrupt:
        print('\n중단됨.')
    finally:
        rclpy.shutdown()

    mode = '실행' if args.execute else '플래닝'
    print(f'\n  초기 위치 {mode} {"완료." if success else "실패."}')
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
