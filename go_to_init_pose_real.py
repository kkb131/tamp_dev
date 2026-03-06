#!/usr/bin/env python3
"""
UR10e 초기 위치 이동 스크립트.

로봇을 처음 켰을 때 현재 위치를 알 수 없으므로,
매우 낮은 속도로 커스텀 초기 위치로 이동합니다.

초기 위치 (rad):
  shoulder_pan : 2.24
  shoulder_lift: -1.2808
  elbow        : 2.16
  wrist_1      : -0.8848
  wrist_2      : 2.24
  wrist_3      : 0.0

전제 조건:
  Terminal 1: ros2 launch ur_robot_driver ur10e.launch.py robot_ip:=<ROBOT_IP>
  Terminal 2: ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e
  Terminal 3: ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \\
                cumotion_planner.robot:=<xrdf> cumotion_planner.urdf_path:=<urdf>

사용법:
  # 계획만 확인 (기본값, 실행 없음):
  python3 go_to_init_pose.py

  # 실제 실행 (매우 천천히, 5% 속도):
  python3 go_to_init_pose.py --execute

  # 속도 조정 (기본 5%):
  python3 go_to_init_pose.py --execute --velocity-scale 0.03

  # 확인 프롬프트 없이 실행 (자동화용):
  python3 go_to_init_pose.py --execute --no-confirm

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

# UR10e 조인트 이름 (UR SRDF 기준)
UR_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# 커스텀 초기 위치 (단위: rad)
INIT_POSE = [2.24, -1.2808, 2.16, -0.8848, 2.24, 0.0]

# 초기 이동 기본 속도/가속도 — 시작 위치 불명 → 매우 천천히
DEFAULT_VELOCITY_SCALE = 0.05   # 5%
DEFAULT_ACCEL_SCALE    = 0.05   # 5%

MOVE_GROUP_ACTION = '/move_action'
PLANNING_GROUP    = 'ur_manipulator'
SUCCESS           = 1  # moveit_msgs/MoveItErrorCodes


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
        super().__init__('go_to_init_pose')
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

        goal.request.group_name                    = PLANNING_GROUP
        goal.request.pipeline_id                   = 'isaac_ros_cumotion'
        goal.request.planner_id                    = ''
        goal.request.num_planning_attempts         = 5
        goal.request.allowed_planning_time         = 15.0
        goal.request.max_velocity_scaling_factor   = float(self.velocity_scale)
        goal.request.max_acceleration_scaling_factor = float(self.accel_scale)
        goal.request.goal_constraints.append(make_joint_goal(INIT_POSE))

        goal.planning_options.plan_only     = plan_only
        goal.planning_options.replan        = True
        goal.planning_options.replan_attempts = 3

        degs_str = ', '.join(f'{deg(v):+.2f}°' for v in INIT_POSE)
        mode = 'PLAN ONLY' if plan_only else 'EXECUTE'
        self.get_logger().info(f'[{mode}] 초기 위치: [{degs_str}]')

        # 목표 전송
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
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)

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


def _print_header(velocity_scale: float, accel_scale: float, execute: bool):
    print('\n' + '=' * 60)
    print('  UR10e 초기 위치 이동')
    print('=' * 60)
    print(f'  Mode       : {"EXECUTE" if execute else "PLAN ONLY"}')
    print(f'  Vel. scale : {velocity_scale * 100:.0f}%')
    print(f'  Accel scale: {accel_scale * 100:.0f}%')
    print()
    print('  목표 초기 위치:')
    labels = ['shoulder_pan', 'shoulder_lift', 'elbow',
              'wrist_1',      'wrist_2',       'wrist_3']
    for label, val in zip(labels, INIT_POSE):
        print(f'    {label:14s}: {val:+.4f} rad  ({deg(val):+.2f}°)')
    print()


def _confirm_execution(velocity_scale: float, accel_scale: float) -> bool:
    print('!' * 60)
    print('  !! 실제 로봇 실행 확인 !!')
    print('!' * 60)
    print(f'  속도 스케일 : {velocity_scale * 100:.0f}%')
    print(f'  가속도 스케일: {accel_scale * 100:.0f}%')
    print()
    print('  ⚠  경고: 로봇의 현재 위치를 알 수 없습니다.')
    print('     낮은 속도로 이동하지만, 예상치 못한 움직임이 발생할 수 있습니다.')
    print()
    print('  실행 전 체크리스트:')
    print('  [ ] 로봇 주변 작업 반경 내 사람 없음')
    print('  [ ] 비상 정지 버튼 손이 닿는 위치에 준비됨')
    print('  [ ] 로봇과 주변 환경에 장애물 없음')
    print()
    answer = input("  실행하려면 'yes'를 입력하세요: ").strip().lower()
    return answer == 'yes'


def main():
    parser = argparse.ArgumentParser(
        description='UR10e 초기 위치 이동 스크립트 (천천히 이동)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예시:
  # 계획만 확인 (안전, 기본값):
  python3 go_to_init_pose.py

  # 실제 실행 (5% 속도):
  python3 go_to_init_pose.py --execute

  # 더 천천히 (3% 속도):
  python3 go_to_init_pose.py --execute --velocity-scale 0.03

  # 확인 없이 실행 (자동화용):
  python3 go_to_init_pose.py --execute --no-confirm
        """,
    )
    parser.add_argument(
        '--execute',
        action='store_true',
        default=False,
        help='실제로 로봇을 실행합니다 (없으면 plan-only)',
    )
    parser.add_argument(
        '--velocity-scale',
        type=float,
        default=DEFAULT_VELOCITY_SCALE,
        metavar='SCALE',
        help=f'최대 속도 스케일 (0.0~1.0, 기본값: {DEFAULT_VELOCITY_SCALE} = {DEFAULT_VELOCITY_SCALE*100:.0f}%%)',
    )
    parser.add_argument(
        '--accel-scale',
        type=float,
        default=DEFAULT_ACCEL_SCALE,
        metavar='SCALE',
        help=f'최대 가속도 스케일 (0.0~1.0, 기본값: {DEFAULT_ACCEL_SCALE} = {DEFAULT_ACCEL_SCALE*100:.0f}%%)',
    )
    parser.add_argument(
        '--no-confirm',
        action='store_true',
        default=False,
        help='실행 전 확인 프롬프트 건너뜀 (자동화용, --execute와 함께 사용 시 주의)',
    )
    args = parser.parse_args()

    # 입력값 검증
    if not 0.0 < args.velocity_scale <= 1.0:
        print(f'Error: --velocity-scale은 (0.0, 1.0] 범위여야 합니다. 입력값: {args.velocity_scale}')
        sys.exit(1)
    if not 0.0 < args.accel_scale <= 1.0:
        print(f'Error: --accel-scale은 (0.0, 1.0] 범위여야 합니다. 입력값: {args.accel_scale}')
        sys.exit(1)

    _print_header(args.velocity_scale, args.accel_scale, args.execute)

    # 실행 전 안전 확인
    if args.execute and not args.no_confirm:
        if not _confirm_execution(args.velocity_scale, args.accel_scale):
            print('\n  실행이 취소되었습니다.')
            sys.exit(0)

    # ROS2 초기화 및 이동 실행
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

    if success:
        mode = '실행' if args.execute else '플래닝'
        print(f'\n  초기 위치 {mode} 완료.')
    else:
        print('\n  초기 위치 이동 실패.')
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
