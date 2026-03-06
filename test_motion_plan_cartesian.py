#!/usr/bin/env python3
"""
cuMotion UR10e Cartesian + Joint 모션 플래닝 테스트 (Mock Hardware).

cumotion_planner.py는 이미 두 가지 goal 유형을 지원합니다:
  - Joint goal (JointConstraint): plan_single_js() 호출
  - Cartesian goal (PositionConstraint + OrientationConstraint): plan_single() 호출

이 스크립트는 두 유형 모두를 테스트합니다.

전제 조건:
  Terminal 1: ros2 launch ur_robot_driver ur10e.launch.py use_fake_hardware:=true robot_ip:=0.0.0.0
  Terminal 2: ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e
  Terminal 3: ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \\
                cumotion_planner.robot:=<xrdf> cumotion_planner.urdf_path:=<urdf>

  실행(execute) 테스트 시 컨트롤러 전환 필요:
    ros2 control switch_controllers \\
      --deactivate scaled_joint_trajectory_controller \\
      --activate joint_trajectory_controller

사용법:
  # Joint goal만 (plan-only):
  python3 test_motion_plan_cartesian.py --goal-type joint

  # Cartesian goal만 (plan-only):
  python3 test_motion_plan_cartesian.py --goal-type cartesian

  # 둘 다 (실행):
  python3 test_motion_plan_cartesian.py --goal-type both --execute

  # Cartesian 이동 거리 변경 (기본: 5cm):
  python3 test_motion_plan_cartesian.py --goal-type cartesian --delta-cm 10.0 --execute

Cartesian goal 구조:
  - frame_id: 'base_link' (robot base frame)
  - ee_link:  'tool0'    (cuMotion ee_link, XRDF에 정의됨)
  - Position: PositionConstraint.constraint_region.primitive_poses[0]
  - Orientation: OrientationConstraint.orientation
"""

import argparse
import copy
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import Point, Pose as GeoPose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)

# ---------------------------------------------------------------------------
# 상수
# ---------------------------------------------------------------------------
UR_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

PLANNING_GROUP = 'ur_manipulator'
MOVE_GROUP_ACTION = '/move_action'
BASE_FRAME = 'base_link'
EE_LINK = 'tool0'   # cuMotion XRDF tool_frames 에 정의된 엔드이펙터

SUCCESS = 1  # moveit_msgs/MoveItErrorCodes

# Joint-space 목표 (test_motion_plan.py와 동일 기준)
# home: 실제 초기 위치 [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
JOINT_TARGETS = {
    'home':               [2.24,  -1.2808, 2.16,  -0.8848,  2.24,   0.0],
    'up':                 [2.24,  -1.2808, 2.16,   1.56,    2.24,   0.0],
    'test_configuration': [1.54,  -1.62,   1.4,   -1.2,    -1.6,  -0.11],
}

JOINT_SEQUENCE = ['home', 'up', 'test_configuration', 'home']


# ---------------------------------------------------------------------------
# Goal 생성 헬퍼
# ---------------------------------------------------------------------------
def make_joint_goal(values: list, tol: float = 0.001) -> Constraints:
    """JointConstraint 기반 Constraints 생성."""
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


def make_cartesian_goal(
    x: float, y: float, z: float,
    qx: float, qy: float, qz: float, qw: float,
    frame_id: str = BASE_FRAME,
    ee_link: str = EE_LINK,
) -> Constraints:
    """
    PositionConstraint + OrientationConstraint 기반 Constraints 생성.

    cumotion_planner.py의 Cartesian 처리 경로 (lines 829-876):
      - position: constraint_region.primitive_poses[0].position
      - orientation: orientation_constraints[0].orientation
      - link_name: 반드시 cuMotion ee_link ('tool0')와 일치해야 함
    """
    pose = GeoPose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

    pc = PositionConstraint()
    pc.header.frame_id = frame_id
    pc.link_name = ee_link
    pc.constraint_region.primitive_poses.append(pose)
    pc.weight = 1.0

    oc = OrientationConstraint()
    oc.header.frame_id = frame_id
    oc.link_name = ee_link
    oc.orientation = pose.orientation
    oc.weight = 1.0

    c = Constraints()
    c.position_constraints.append(pc)
    c.orientation_constraints.append(oc)
    return c


# ---------------------------------------------------------------------------
# 테스터 노드
# ---------------------------------------------------------------------------
class CartesianMotionTester(Node):
    def __init__(self, velocity_scale: float, accel_scale: float):
        super().__init__('test_motion_plan_cartesian')
        self.velocity_scale = velocity_scale
        self.accel_scale = accel_scale

        # TF2 리스너 (Cartesian goal을 위한 현재 tool0 pose 조회)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveGroup action client
        self._client = ActionClient(self, MoveGroup, MOVE_GROUP_ACTION)
        self.get_logger().info(f'Waiting for {MOVE_GROUP_ACTION}...')
        if not self._client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError(
                f'{MOVE_GROUP_ACTION} not available. '
                'Is move_group running? (Terminal 2)'
            )
        self.get_logger().info('Connected to move_group.')

    def lookup_tool_pose(self, timeout_sec: float = 5.0):
        """
        base_link → tool0 TF를 조회합니다.
        반환: transform (translation + rotation) 또는 None
        """
        self.get_logger().info(
            f'Looking up TF: {BASE_FRAME} → {EE_LINK} (timeout={timeout_sec}s)...'
        )
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            try:
                t = self.tf_buffer.lookup_transform(
                    BASE_FRAME, EE_LINK, rclpy.time.Time()
                )
                tx = t.transform.translation
                rot = t.transform.rotation
                self.get_logger().info(
                    f'  tool0 in base_link: '
                    f'pos=({tx.x:.4f}, {tx.y:.4f}, {tx.z:.4f}), '
                    f'quat=({rot.x:.4f}, {rot.y:.4f}, {rot.z:.4f}, {rot.w:.4f})'
                )
                return t.transform
            except Exception as e:
                self.get_logger().debug(f'TF lookup: {e}')
                rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error(
            f'TF lookup timed out after {timeout_sec}s. '
            'Is the UR driver running? (Terminal 1)'
        )
        return None

    def send_goal(
        self,
        label: str,
        constraints: Constraints,
        execute: bool,
        allowed_planning_time: float = 15.0,
    ) -> bool:
        """MoveGroup goal을 전송하고 결과를 기다립니다."""
        goal = MoveGroup.Goal()
        goal.request.group_name = PLANNING_GROUP
        goal.request.planner_id = ''
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = allowed_planning_time
        goal.request.max_velocity_scaling_factor = float(self.velocity_scale)
        goal.request.max_acceleration_scaling_factor = float(self.accel_scale)
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = not execute
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        mode = 'EXECUTE' if execute else 'PLAN ONLY'
        self.get_logger().info(f'[{mode}] → {label}')

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
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=90.0)

        if not result_future.done():
            self.get_logger().error('Timeout waiting for result.')
            return False

        ec = result_future.result().result.error_code.val
        if ec == SUCCESS:
            label_suffix = '(plan only)' if not execute else '(executed)'
            self.get_logger().info(f'OK {label_suffix}: {label}')
            return True
        else:
            self.get_logger().error(f'FAILED: {label}  MoveItErrorCode={ec}')
            return False


# ---------------------------------------------------------------------------
# 테스트 시퀀스
# ---------------------------------------------------------------------------
def run_joint_test(node: CartesianMotionTester, execute: bool) -> bool:
    """기존 joint-space 시퀀스: home → up → test_configuration → home."""
    print('\n' + '=' * 60)
    print('  [Joint] home → up → test_configuration → home')
    print('=' * 60)

    results = []
    for name in JOINT_SEQUENCE:
        joints = JOINT_TARGETS[name]
        constraints = make_joint_goal(joints)
        ok = node.send_goal(f'joint:{name}', constraints, execute=execute)
        results.append((name, ok))
        if ok:
            time.sleep(1.0)
        else:
            print(f'  FAILED: {name} — abort.')
            break

    _print_results(results)
    return all(ok for _, ok in results)


def run_cartesian_test(
    node: CartesianMotionTester,
    execute: bool,
    delta_m: float,
) -> bool:
    """
    Cartesian 시퀀스:
      1. joint-space로 home 이동 (기준 위치 확보)
      2. TF2로 현재 tool0 pose 조회
      3. home_pose + (0, 0, +delta_m)  — 위로 이동
      4. home_pose + (+delta_m, 0, 0)  — 전방으로 이동
      5. joint-space로 home 복귀
    """
    print('\n' + '=' * 60)
    print(f'  [Cartesian] home(joint) → +{delta_m*100:.0f}cm Z → +{delta_m*100:.0f}cm X → home(joint)')
    print(f'  frame: {BASE_FRAME}, ee_link: {EE_LINK}')
    print('=' * 60)

    results = []

    # Step 1: joint-space로 home 이동
    print('\n  Step 1: Move to home (joint-space, for TF reference)')
    home_constraints = make_joint_goal(JOINT_TARGETS['home'])
    ok = node.send_goal('joint:home (baseline)', home_constraints, execute=execute)
    results.append(('joint:home', ok))
    if not ok:
        print('  FAILED: Cannot establish home position.')
        _print_results(results)
        return False

    if execute:
        time.sleep(2.0)  # 로봇이 실제로 이동할 시간 확보

    # Step 2: TF2 조회
    print('\n  Step 2: TF2 lookup for tool0 pose...')
    tf = node.lookup_tool_pose(timeout_sec=5.0)
    if tf is None:
        print('  FAILED: Could not get tool0 TF.')
        _print_results(results)
        return False

    x0 = tf.translation.x
    y0 = tf.translation.y
    z0 = tf.translation.z
    qx = tf.rotation.x
    qy = tf.rotation.y
    qz = tf.rotation.z
    qw = tf.rotation.w

    print(f'  tool0 at home: ({x0:.4f}, {y0:.4f}, {z0:.4f})')

    # Step 3: Cartesian 이동 — +Z
    print(f'\n  Step 3: Cartesian move +{delta_m*100:.0f}cm in Z')
    cart_z = make_cartesian_goal(x0, y0, z0 + delta_m, qx, qy, qz, qw)
    ok = node.send_goal(
        f'cartesian: ({x0:.3f}, {y0:.3f}, {z0+delta_m:.3f})',
        cart_z, execute=execute,
    )
    results.append(('cartesian:+Z', ok))
    if ok:
        time.sleep(1.0)
    else:
        print('  FAILED: Cartesian +Z move.')

    # Step 4: Cartesian 이동 — +X (home_z 복귀 후)
    if ok:
        print(f'\n  Step 4: Cartesian move +{delta_m*100:.0f}cm in X (from home)')
        cart_x = make_cartesian_goal(x0 + delta_m, y0, z0, qx, qy, qz, qw)
        ok2 = node.send_goal(
            f'cartesian: ({x0+delta_m:.3f}, {y0:.3f}, {z0:.3f})',
            cart_x, execute=execute,
        )
        results.append(('cartesian:+X', ok2))
        if ok2:
            time.sleep(1.0)
        else:
            print('  FAILED: Cartesian +X move.')

    # Step 5: joint-space로 home 복귀
    print('\n  Step 5: Return to home (joint-space)')
    ok_ret = node.send_goal('joint:home (return)', home_constraints, execute=execute)
    results.append(('joint:home (return)', ok_ret))

    _print_results(results)
    return all(ok for _, ok in results)


def _print_results(results):
    print('\n  Results:')
    for name, ok in results:
        mark = 'OK  ' if ok else 'FAIL'
        print(f'    {mark}  {name}')


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description='cuMotion UR10e Cartesian + Joint 플래닝 테스트 (Mock Hardware)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예시:
  python3 test_motion_plan_cartesian.py --goal-type cartesian          # plan-only
  python3 test_motion_plan_cartesian.py --goal-type cartesian --execute  # 실행
  python3 test_motion_plan_cartesian.py --goal-type both --execute        # 둘 다
  python3 test_motion_plan_cartesian.py --goal-type cartesian --delta-cm 10.0 --execute

주의:
  --execute 사용 시 먼저 컨트롤러를 전환하세요:
    ros2 control switch_controllers \\
      --deactivate scaled_joint_trajectory_controller \\
      --activate joint_trajectory_controller
        """,
    )
    parser.add_argument(
        '--goal-type',
        choices=['joint', 'cartesian', 'both'],
        default='cartesian',
        help='테스트할 goal 유형 (기본: cartesian)',
    )
    parser.add_argument(
        '--execute',
        action='store_true',
        default=False,
        help='실제 실행 (없으면 plan-only)',
    )
    parser.add_argument(
        '--delta-cm',
        type=float,
        default=5.0,
        metavar='CM',
        help='Cartesian 이동 거리 cm (기본: 5.0)',
    )
    parser.add_argument(
        '--velocity-scale',
        type=float,
        default=0.5,
        metavar='SCALE',
        help='최대 속도 스케일 0~1 (기본: 0.5)',
    )
    parser.add_argument(
        '--accel-scale',
        type=float,
        default=0.5,
        metavar='SCALE',
        help='최대 가속도 스케일 0~1 (기본: 0.5)',
    )
    args = parser.parse_args()

    delta_m = args.delta_cm / 100.0

    print('\n' + '=' * 60)
    print('  cuMotion UR10e Cartesian/Joint 플래닝 테스트')
    print('=' * 60)
    print(f'  Goal type   : {args.goal_type}')
    print(f'  Mode        : {"EXECUTE" if args.execute else "PLAN ONLY"}')
    print(f'  Delta       : {args.delta_cm:.1f} cm')
    print(f'  Vel. scale  : {args.velocity_scale * 100:.0f}%')
    print(f'  Accel scale : {args.accel_scale * 100:.0f}%')

    rclpy.init()
    success = True
    try:
        node = CartesianMotionTester(args.velocity_scale, args.accel_scale)

        if args.goal_type in ('joint', 'both'):
            success &= run_joint_test(node, execute=args.execute)

        if args.goal_type in ('cartesian', 'both'):
            success &= run_cartesian_test(node, execute=args.execute, delta_m=delta_m)

        node.destroy_node()
    except RuntimeError as e:
        print(f'\nError: {e}', file=sys.stderr)
        success = False
    except KeyboardInterrupt:
        print('\nInterrupted.')
        success = False
    finally:
        rclpy.shutdown()

    print('\n  ' + ('ALL OK.' if success else 'SOME FAILED.'))
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
