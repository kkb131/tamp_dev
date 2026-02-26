#!/usr/bin/env python3
"""
cuMotion UR10e 실제 로봇 모션 플래닝 안전 테스트 스크립트.

⚠️  경고: 이 스크립트는 실제 로봇을 움직입니다.
    --execute 플래그 없이는 계획(plan)만 수행합니다.
    처음에는 반드시 --plan-only 또는 --execute 없이 시작하세요.

전제 조건:
  Terminal 1: ros2 launch ur_robot_driver ur10e.launch.py robot_ip:=<ROBOT_IP>
  Terminal 2: ros2 launch isaac_ros_cumotion_examples ur.launch.py ur_type:=ur10e
  Terminal 3: ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \\
                cumotion_planner.robot:=<xrdf> cumotion_planner.urdf_path:=<urdf>

사용법:
  # 계획만 (실행 없음 — 기본값):
  python3 test_motion_plan_real.py --stage minimal

  # 실제 실행 (명시적 확인 필요):
  python3 test_motion_plan_real.py --stage minimal --execute

  # 속도/가속도 변경:
  python3 test_motion_plan_real.py --stage minimal --execute --velocity-scale 0.1

  # 커스텀 타겟 파일 사용:
  python3 test_motion_plan_real.py --targets-file my_targets.json --execute

  # 전체 테스트 (경험자용):
  python3 test_motion_plan_real.py --stage full --execute --velocity-scale 0.2

Joint 순서: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
단위: rad
"""

import argparse
import json
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

# ---------------------------------------------------------------------------
# 안전 타겟 정의 — 실제 로봇 테스트용 (최소 이동)
# ---------------------------------------------------------------------------
# verify_home : 기준 위치 확인 (이동 없음, 현재 위치가 home인 경우)
# micro_wrist : wrist_1만 ~8.6° 이동 — 가장 안전한 단일 축 테스트
# micro_pan   : shoulder_pan만 ~5.7° 이동 — 두 번째 축 테스트
# safe_return : home 복귀
#
# 주의: 로봇이 이미 home 위치에 있다고 가정합니다.
#        실행 전 RViz나 TP(Teaching Pendant)로 현재 위치를 반드시 확인하세요.
SAFE_TARGETS = {
    'verify_home': [0.0,    -1.5707, 0.0,  0.0,    0.0,  0.0],   # home 기준점
    'micro_wrist': [0.0,    -1.5707, 0.0, -0.15,   0.0,  0.0],   # wrist_1: -8.6°
    'micro_pan':   [0.1,    -1.5707, 0.0,  0.0,    0.0,  0.0],   # shoulder_pan: +5.7°
    'safe_return': [0.0,    -1.5707, 0.0,  0.0,    0.0,  0.0],   # home 복귀
}

# 전체 테스트용 타겟 (--stage full에서 사용)
FULL_TARGETS = {
    'home':               [0.0,   -1.5707,  0.0,   0.0,    0.0,    0.0],
    'up':                 [0.0,   -1.5707,  0.0,  -1.5707, 0.0,    0.0],
    'test_configuration': [1.54,  -1.62,    1.4,  -1.2,   -1.6,   -0.11],
}

# 테스트 단계별 시퀀스
STAGE_SEQUENCES = {
    'minimal':  ['verify_home', 'micro_wrist', 'safe_return'],
    'standard': ['verify_home', 'micro_wrist', 'micro_pan', 'safe_return'],
    'full':     ['home', 'up', 'test_configuration', 'home'],
}

MOVE_GROUP_ACTION = '/move_action'
PLANNING_GROUP = 'ur_manipulator'
SUCCESS = 1  # moveit_msgs/MoveItErrorCodes


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


def deg(rad_val):
    """라디안을 도(degree)로 변환 (표시용)."""
    return rad_val * 180.0 / 3.14159265


class RealRobotMotionTester(Node):
    def __init__(self, velocity_scale: float, accel_scale: float):
        super().__init__('test_motion_plan_real')
        self.velocity_scale = velocity_scale
        self.accel_scale = accel_scale
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

    def move_to(self, name: str, joints: list, execute: bool = False) -> bool:
        plan_only = not execute
        goal = MoveGroup.Goal()

        goal.request.group_name = PLANNING_GROUP
        goal.request.planner_id = ''
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 15.0
        goal.request.max_velocity_scaling_factor = float(self.velocity_scale)
        goal.request.max_acceleration_scaling_factor = float(self.accel_scale)
        goal.request.goal_constraints.append(make_joint_goal(joints))

        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        vals_str = ', '.join(f'{deg(v):+.1f}°' for v in joints)
        mode = 'PLAN ONLY' if plan_only else 'EXECUTE'
        self.get_logger().info(f"[{mode}] '{name}': [{vals_str}]")

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
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)

        if not result_future.done():
            self.get_logger().error('Timeout waiting for result.')
            return False

        result = result_future.result().result
        ec = result.error_code.val
        if ec == SUCCESS:
            label = '(plan only)' if plan_only else '(executed)'
            self.get_logger().info(f"OK {label}: '{name}'")
            return True
        else:
            self.get_logger().error(f"FAILED '{name}'. MoveItErrorCode={ec}")
            return False


def _print_target_info(targets: dict, sequence: list):
    print('\n  타겟 정보:')
    for name in sequence:
        if name in targets:
            vals = targets[name]
            degs = ', '.join(f'{deg(v):+.1f}°' for v in vals)
            print(f'    {name:20s}: [{degs}]')


def _print_results(results):
    print('\n  Results:')
    for target, ok in results:
        mark = 'OK  ' if ok else 'FAIL'
        print(f'    {mark}  {target}')


def _confirm_execution(stage: str, velocity_scale: float, accel_scale: float) -> bool:
    print('\n' + '!' * 60)
    print('  !! 실제 로봇 실행 확인 !!')
    print('!' * 60)
    print(f'  단계(stage): {stage}')
    print(f'  속도 스케일: {velocity_scale * 100:.0f}%')
    print(f'  가속도 스케일: {accel_scale * 100:.0f}%')
    print()
    print('  실행 전 체크리스트:')
    print('  [ ] 로봇 주변 작업 반경 내 사람 없음')
    print('  [ ] 비상 정지 버튼 손이 닿는 위치에 준비됨')
    print('  [ ] 로봇이 현재 home 위치에 있음')
    print('  [ ] 장애물 없음')
    print()
    answer = input("  실행하려면 'yes'를 입력하세요: ").strip().lower()
    return answer == 'yes'


def load_custom_targets(path: str) -> dict:
    with open(path, 'r') as f:
        data = json.load(f)
    # 검증
    for name, vals in data.items():
        if len(vals) != 6:
            raise ValueError(f"Target '{name}' must have 6 joint values, got {len(vals)}")
    return data


def run_test(
    node: RealRobotMotionTester,
    targets: dict,
    sequence: list,
    execute: bool,
) -> bool:
    results = []
    for target in sequence:
        if target not in targets:
            print(f'  [ERROR] Unknown target: {target}')
            return False
        print(f'\n  >> {target}')
        ok = node.move_to(target, targets[target], execute=execute)
        results.append((target, ok))
        if ok:
            time.sleep(1.0)
        else:
            print(f'  FAILED: {target} — 중단.')
            break

    _print_results(results)
    return all(ok for _, ok in results)


def main():
    parser = argparse.ArgumentParser(
        description='cuMotion UR10e 실제 로봇 안전 테스트 스크립트',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예시:
  # 계획만 (안전, 기본값):
  python3 test_motion_plan_real.py --stage minimal

  # 실제 실행 — 5% 속도:
  python3 test_motion_plan_real.py --stage minimal --execute

  # 실제 실행 — 10% 속도, standard 단계:
  python3 test_motion_plan_real.py --stage standard --execute --velocity-scale 0.1

  # 커스텀 타겟 (JSON):
  python3 test_motion_plan_real.py --targets-file custom.json --execute

커스텀 타겟 JSON 형식:
  {
    "target_name": [pan, lift, elbow, wrist1, wrist2, wrist3],
    ...
  }
        """,
    )
    parser.add_argument(
        '--stage',
        choices=['minimal', 'standard', 'full'],
        default='minimal',
        help=(
            'minimal: home→micro_wrist→home (1축, ~8.6°) | '
            'standard: home→micro_wrist→micro_pan→home (2축) | '
            'full: 전체 경로 (home→up→test_config→home)'
        ),
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
        default=0.05,
        metavar='SCALE',
        help='최대 속도 스케일 (0.0~1.0, 기본값: 0.05 = 5%%)',
    )
    parser.add_argument(
        '--accel-scale',
        type=float,
        default=0.05,
        metavar='SCALE',
        help='최대 가속도 스케일 (0.0~1.0, 기본값: 0.05 = 5%%)',
    )
    parser.add_argument(
        '--targets-file',
        type=str,
        default=None,
        metavar='JSON_FILE',
        help='커스텀 타겟 JSON 파일 경로 (--stage 무시)',
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
        print(f'Error: --velocity-scale must be in (0.0, 1.0], got {args.velocity_scale}')
        sys.exit(1)
    if not 0.0 < args.accel_scale <= 1.0:
        print(f'Error: --accel-scale must be in (0.0, 1.0], got {args.accel_scale}')
        sys.exit(1)

    # 타겟 결정
    if args.targets_file:
        try:
            custom_targets = load_custom_targets(args.targets_file)
        except Exception as e:
            print(f'Error loading targets file: {e}')
            sys.exit(1)
        targets = custom_targets
        sequence = list(custom_targets.keys())
        stage_label = f'custom ({args.targets_file})'
    else:
        if args.stage == 'full':
            targets = FULL_TARGETS
        else:
            targets = SAFE_TARGETS
        sequence = STAGE_SEQUENCES[args.stage]
        stage_label = args.stage

    # 헤더 출력
    print('\n' + '=' * 60)
    print('  cuMotion UR10e 실제 로봇 안전 테스트')
    print('=' * 60)
    print(f'  Stage      : {stage_label}')
    print(f'  Mode       : {"EXECUTE" if args.execute else "PLAN ONLY"}')
    print(f'  Vel. scale : {args.velocity_scale * 100:.0f}%')
    print(f'  Accel scale: {args.accel_scale * 100:.0f}%')
    print(f'  Sequence   : {" → ".join(sequence)}')
    _print_target_info(targets, sequence)

    # 실행 전 안전 확인
    if args.execute and not args.no_confirm:
        if not _confirm_execution(stage_label, args.velocity_scale, args.accel_scale):
            print('\n  실행이 취소되었습니다.')
            sys.exit(0)

    # ROS2 초기화 및 테스트 실행
    rclpy.init()
    success = False
    try:
        node = RealRobotMotionTester(args.velocity_scale, args.accel_scale)
        success = run_test(node, targets, sequence, execute=args.execute)
        node.destroy_node()
    except RuntimeError as e:
        print(f'\nError: {e}', file=sys.stderr)
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        rclpy.shutdown()

    if success:
        print('\n  모든 테스트 완료.')
    else:
        print('\n  일부 테스트 실패.')
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
