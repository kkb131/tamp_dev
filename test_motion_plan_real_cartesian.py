#!/usr/bin/env python3
"""
cuMotion UR10e Cartesian 실제 로봇 안전 테스트 스크립트.

⚠️  경고: 이 스크립트는 실제 로봇을 움직입니다.
    --execute 플래그 없이는 계획(plan)만 수행합니다.
    처음에는 반드시 plan-only로 시작하세요.

Cartesian 제어 원리:
  - 엔드이펙터(tool0)의 위치/방향을 직접 지정합니다.
  - cuMotion이 IK를 풀어 joint trajectory를 생성합니다.
  - 방향(orientation)은 고정하고 위치(position)만 변경합니다.
  - 기준 좌표계: base_link (로봇 베이스)

전제 조건:
  Terminal 1: ros2 launch ur_robot_driver ur10e.launch.py robot_ip:=<ROBOT_IP>
  Terminal 2: ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e
  Terminal 3: ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \\
                cumotion_planner.robot:=<xrdf> cumotion_planner.urdf_path:=<urdf>

사용법:
  # 계획만 검증 (기본, 안전):
  python3 test_motion_plan_real_cartesian.py --stage minimal

  # 실행 (2cm 이동, 5% 속도):
  python3 test_motion_plan_real_cartesian.py --stage minimal --execute

  # 이동 거리 변경 (최대 5cm 권장):
  python3 test_motion_plan_real_cartesian.py --stage minimal --execute --delta-cm 3.0

  # 속도 변경:
  python3 test_motion_plan_real_cartesian.py --stage standard --execute \\
    --delta-cm 2.0 --velocity-scale 0.1

Stage 설명:
  minimal  : home(joint) → +deltaZ → home(joint)  [1 Cartesian 이동]
  standard : home(joint) → +deltaZ → +deltaX → home(joint)  [2 Cartesian 이동]
  full     : home → +deltaZ → +deltaX → -deltaZ+5cm → home  [3 Cartesian 이동]
"""

import argparse
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
EE_LINK = 'tool0'

SUCCESS = 1

# Home joint 위치 — 로봇이 이 위치에 있다고 가정
HOME_JOINTS = [0.0, -1.5707, 0.0, 0.0, 0.0, 0.0]

# Cartesian 이동 거리 제한 (실제 로봇 안전)
MAX_DELTA_CM = 10.0   # 절대 최대 (이 이상은 허용 안 함)
WARN_DELTA_CM = 5.0   # 이 이상이면 경고


# ---------------------------------------------------------------------------
# Goal 생성 헬퍼
# ---------------------------------------------------------------------------
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


def make_cartesian_goal(
    x: float, y: float, z: float,
    qx: float, qy: float, qz: float, qw: float,
    frame_id: str = BASE_FRAME,
    ee_link: str = EE_LINK,
) -> Constraints:
    """
    PositionConstraint + OrientationConstraint 기반 Constraints.
    link_name은 반드시 cuMotion ee_link ('tool0')와 일치해야 함.
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
class RealCartesianTester(Node):
    def __init__(self, velocity_scale: float, accel_scale: float):
        super().__init__('test_motion_plan_real_cartesian')
        self.velocity_scale = velocity_scale
        self.accel_scale = accel_scale

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._client = ActionClient(self, MoveGroup, MOVE_GROUP_ACTION)
        self.get_logger().info(
            f'Connecting to {MOVE_GROUP_ACTION} '
            f'(vel={velocity_scale*100:.0f}%, accel={accel_scale*100:.0f}%)...'
        )
        if not self._client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError(
                f'{MOVE_GROUP_ACTION} not available. '
                'Is move_group running? (Terminal 2)'
            )
        self.get_logger().info('Connected to move_group.')

    def lookup_tool_pose(self, timeout_sec: float = 8.0):
        """base_link → tool0 TF 조회."""
        self.get_logger().info(f'TF lookup: {BASE_FRAME} → {EE_LINK}...')
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            try:
                t = self.tf_buffer.lookup_transform(
                    BASE_FRAME, EE_LINK, rclpy.time.Time()
                )
                tx = t.transform.translation
                rot = t.transform.rotation
                self.get_logger().info(
                    f'  tool0: pos=({tx.x:.4f}, {tx.y:.4f}, {tx.z:.4f}), '
                    f'quat=({rot.x:.4f}, {rot.y:.4f}, {rot.z:.4f}, {rot.w:.4f})'
                )
                return t.transform
            except Exception as e:
                self.get_logger().debug(f'TF: {e}')
                rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error('TF lookup timed out.')
        return None

    def send_goal(
        self,
        label: str,
        constraints: Constraints,
        execute: bool,
    ) -> bool:
        goal = MoveGroup.Goal()
        goal.request.group_name = PLANNING_GROUP
        goal.request.pipeline_id = 'isaac_ros_cumotion'
        goal.request.planner_id = ''
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 20.0
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

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)

        if not result_future.done():
            self.get_logger().error('Timeout waiting for result.')
            return False

        ec = result_future.result().result.error_code.val
        if ec == SUCCESS:
            suffix = '(plan only)' if not execute else '(executed)'
            self.get_logger().info(f'OK {suffix}: {label}')
            return True
        else:
            self.get_logger().error(f'FAILED: {label}  MoveItErrorCode={ec}')
            return False


# ---------------------------------------------------------------------------
# 테스트 실행
# ---------------------------------------------------------------------------
def run_test(
    node: RealCartesianTester,
    stage: str,
    delta_m: float,
    execute: bool,
) -> bool:
    results = []

    # Step 1: joint-space로 home 이동
    print('\n  Step 1: Move to home (joint-space)')
    home_c = make_joint_goal(HOME_JOINTS)
    ok = node.send_goal('joint:home', home_c, execute=execute)
    results.append(('joint:home', ok))
    if not ok:
        print('  FAILED: Cannot move to home.')
        _print_results(results)
        return False

    if execute:
        time.sleep(3.0)  # 실제 로봇 이동 완료 대기

    # Step 2: TF 조회
    print('\n  Step 2: TF2 lookup for tool0 pose at home...')
    tf = node.lookup_tool_pose(timeout_sec=8.0)
    if tf is None:
        print('  FAILED: Could not get tool0 pose.')
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

    # Step 3: +deltaZ Cartesian 이동
    print(f'\n  Step 3: Cartesian +{delta_m*100:.1f}cm in Z')
    c_z = make_cartesian_goal(x0, y0, z0 + delta_m, qx, qy, qz, qw)
    ok = node.send_goal(
        f'cartesian: pos=({x0:.3f},{y0:.3f},{z0+delta_m:.3f})',
        c_z, execute=execute,
    )
    results.append(('cartesian:+Z', ok))
    if ok and execute:
        time.sleep(2.0)

    # standard / full: +deltaX 이동
    if stage in ('standard', 'full') and ok:
        print(f'\n  Step 4: Cartesian +{delta_m*100:.1f}cm in X (from home z)')
        c_x = make_cartesian_goal(x0 + delta_m, y0, z0, qx, qy, qz, qw)
        ok2 = node.send_goal(
            f'cartesian: pos=({x0+delta_m:.3f},{y0:.3f},{z0:.3f})',
            c_x, execute=execute,
        )
        results.append(('cartesian:+X', ok2))
        if ok2 and execute:
            time.sleep(2.0)

    # full: 추가 대각선 이동
    if stage == 'full' and ok:
        print(f'\n  Step 5: Cartesian diagonal (+X, -Z small)')
        dz_small = delta_m * 0.5
        c_diag = make_cartesian_goal(
            x0 + delta_m, y0, z0 - dz_small, qx, qy, qz, qw
        )
        ok3 = node.send_goal(
            f'cartesian: pos=({x0+delta_m:.3f},{y0:.3f},{z0-dz_small:.3f})',
            c_diag, execute=execute,
        )
        results.append(('cartesian:diagonal', ok3))
        if ok3 and execute:
            time.sleep(2.0)

    # 마지막: home 복귀
    print('\n  Final: Return to home (joint-space)')
    ok_ret = node.send_goal('joint:home (return)', home_c, execute=execute)
    results.append(('joint:home (return)', ok_ret))

    _print_results(results)
    return all(ok for _, ok in results)


def _print_results(results):
    print('\n  Results:')
    for name, ok in results:
        mark = 'OK  ' if ok else 'FAIL'
        print(f'    {mark}  {name}')


def _confirm_execution(stage: str, delta_cm: float, vel: float) -> bool:
    print('\n' + '!' * 60)
    print('  !! 실제 로봇 Cartesian 실행 확인 !!')
    print('!' * 60)
    print(f'  Stage     : {stage}')
    print(f'  Delta     : {delta_cm:.1f} cm (각 Cartesian 이동)')
    print(f'  Vel scale : {vel*100:.0f}%')
    print()
    print('  체크리스트:')
    print('  [ ] 로봇 주변 작업 반경(최소 1m) 내 사람 없음')
    print('  [ ] 비상 정지 버튼 손이 닿는 위치')
    print('  [ ] 로봇이 현재 home 위치에 있음')
    print('  [ ] TP 속도 다이얼 25% 이하')
    print()
    answer = input("  실행하려면 'yes'를 입력하세요: ").strip().lower()
    return answer == 'yes'


def main():
    parser = argparse.ArgumentParser(
        description='cuMotion UR10e Cartesian 실제 로봇 안전 테스트',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예시:
  # 계획만 (기본, 안전):
  python3 test_motion_plan_real_cartesian.py --stage minimal

  # 실행 — 2cm, 5% 속도:
  python3 test_motion_plan_real_cartesian.py --stage minimal --execute

  # 실행 — 3cm, 10% 속도:
  python3 test_motion_plan_real_cartesian.py --stage standard --execute \\
    --delta-cm 3.0 --velocity-scale 0.1
        """,
    )
    parser.add_argument(
        '--stage',
        choices=['minimal', 'standard', 'full'],
        default='minimal',
        help=(
            'minimal: +deltaZ 1회 | '
            'standard: +deltaZ +deltaX | '
            'full: 3 Cartesian 이동'
        ),
    )
    parser.add_argument(
        '--execute',
        action='store_true',
        default=False,
        help='실제 로봇 실행 (없으면 plan-only)',
    )
    parser.add_argument(
        '--delta-cm',
        type=float,
        default=2.0,
        metavar='CM',
        help=f'Cartesian 이동 거리 cm (기본: 2.0, 최대: {MAX_DELTA_CM})',
    )
    parser.add_argument(
        '--velocity-scale',
        type=float,
        default=0.05,
        metavar='SCALE',
        help='최대 속도 스케일 (기본: 0.05 = 5%%)',
    )
    parser.add_argument(
        '--accel-scale',
        type=float,
        default=0.05,
        metavar='SCALE',
        help='최대 가속도 스케일 (기본: 0.05 = 5%%)',
    )
    parser.add_argument(
        '--no-confirm',
        action='store_true',
        default=False,
        help='실행 전 확인 프롬프트 건너뜀 (자동화용)',
    )
    args = parser.parse_args()

    # 입력값 검증
    if args.delta_cm > MAX_DELTA_CM:
        print(
            f'Error: --delta-cm {args.delta_cm} exceeds maximum {MAX_DELTA_CM} cm '
            'for real robot safety.',
            file=sys.stderr,
        )
        sys.exit(1)
    if not 0.0 < args.velocity_scale <= 1.0:
        print(f'Error: --velocity-scale must be in (0, 1]', file=sys.stderr)
        sys.exit(1)

    delta_m = args.delta_cm / 100.0

    # 헤더 출력
    print('\n' + '=' * 60)
    print('  cuMotion UR10e Cartesian 실제 로봇 안전 테스트')
    print('=' * 60)
    print(f'  Stage     : {args.stage}')
    print(f'  Mode      : {"EXECUTE" if args.execute else "PLAN ONLY"}')
    print(f'  Delta     : {args.delta_cm:.1f} cm')
    print(f'  Vel scale : {args.velocity_scale*100:.0f}%')
    print(f'  Frame     : {BASE_FRAME} → {EE_LINK}')

    if args.delta_cm > WARN_DELTA_CM:
        print(f'\n  [WARNING] delta > {WARN_DELTA_CM}cm — 실제 로봇에서 신중하게 사용하세요.')

    # 실행 전 확인
    if args.execute and not args.no_confirm:
        if not _confirm_execution(args.stage, args.delta_cm, args.velocity_scale):
            print('\n  실행 취소.')
            sys.exit(0)

    rclpy.init()
    success = False
    try:
        node = RealCartesianTester(args.velocity_scale, args.accel_scale)
        success = run_test(node, args.stage, delta_m, execute=args.execute)
        node.destroy_node()
    except RuntimeError as e:
        print(f'\nError: {e}', file=sys.stderr)
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        rclpy.shutdown()

    print('\n  ' + ('ALL OK.' if success else 'SOME FAILED.'))
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
