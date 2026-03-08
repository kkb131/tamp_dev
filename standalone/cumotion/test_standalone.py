#!/usr/bin/env python3
"""Standalone cuMotion test — dual mode (RTDE / Isaac Sim).

Usage:
    # Plan-only (no robot/sim connection needed)
    python3 -m standalone.cumotion.test_standalone --plan-only

    # Isaac Sim mode (default)
    python3 -m standalone.cumotion.test_standalone --mode sim --execute --velocity-scale 0.1

    # Real robot (RTDE) mode
    python3 -m standalone.cumotion.test_standalone --mode rtde --robot-ip 192.168.0.2 --execute

    # Cartesian goal
    python3 -m standalone.cumotion.test_standalone --mode sim --goal-type cartesian --execute
"""

import argparse
import sys

import numpy as np

from standalone.config import (
    DEFAULT_MODE,
    DEFAULT_ROBOT_IP,
    DEFAULT_VELOCITY_SCALE,
    HOME_JOINTS,
    SERVOJ_DT,
    UP_JOINTS,
    XRDF_PATH,
    URDF_PATH,
)
from standalone.cumotion.planner import StandaloneMotionPlanner


def print_trajectory_info(traj: dict):
    """Print trajectory summary."""
    print(f"\n{'='*50}")
    print(f"  Planning time  : {traj['plan_time']:.3f}s")
    print(f"  Waypoints      : {traj['n_points']}")
    print(f"  Time step (dt) : {traj['dt']:.4f}s")
    print(f"  Total duration : {traj['total_time']:.3f}s")
    print(f"  Start joints   : [{', '.join(f'{v:.4f}' for v in traj['positions'][0])}]")
    print(f"  End joints     : [{', '.join(f'{v:.4f}' for v in traj['positions'][-1])}]")
    print(f"{'='*50}\n")


def confirm_execution(velocity_scale: float, mode: str) -> bool:
    """Interactive safety confirmation."""
    print("\n" + "!" * 50)
    label = "REAL ROBOT" if mode == "rtde" else "ISAAC SIM"
    print(f"  {label} EXECUTION")
    print(f"  Velocity scale: {velocity_scale*100:.1f}%")
    if mode == "rtde":
        print("  Checklist:")
        print("    - Work area clear?")
        print("    - E-stop accessible?")
        print("    - Robot in expected position?")
    print("!" * 50)
    ans = input("\nProceed? (yes/no): ").strip().lower()
    return ans == "yes"


def run_plan_only(args):
    """Plan-only mode — no robot/sim connection needed."""
    planner = StandaloneMotionPlanner(XRDF_PATH, URDF_PATH)

    start = list(args.start_joints) if args.start_joints else HOME_JOINTS

    if args.goal_type == "joint":
        goal = list(args.goal_joints) if args.goal_joints else UP_JOINTS
        print(f"\n[Test] Joint planning: start -> goal")
        traj = planner.plan_joint(start, goal, velocity_scale=args.velocity_scale)
    else:
        goal_joints = list(args.goal_joints) if args.goal_joints else UP_JOINTS
        ee = planner.get_ee_pose(goal_joints)
        print(f"\n[Test] Cartesian planning to EE: pos={ee['position']}, quat={ee['quaternion']}")
        traj = planner.plan_cartesian(
            start, ee["position"], ee["quaternion"], velocity_scale=args.velocity_scale
        )

    if traj is None:
        print("[Test] Planning FAILED")
        return False

    print("[Test] Planning SUCCESS")
    print_trajectory_info(traj)
    return True


def run_with_backend(args):
    """Plan and execute using the selected backend (rtde or sim)."""
    from standalone.robot_backend import create_backend
    from standalone.trajectory_executor import (
        check_start_match,
        execute_trajectory,
        validate_trajectory,
    )

    planner = StandaloneMotionPlanner(XRDF_PATH, URDF_PATH)

    backend_kwargs = {"robot_ip": args.robot_ip}
    with create_backend(args.mode, **backend_kwargs) as robot:
        # Get current joint positions
        current = robot.get_joint_positions()
        print(f"\n[Robot] Current joints: [{', '.join(f'{v:.4f}' for v in current)}]")

        # Get current EE pose
        ee = planner.get_ee_pose(current)
        print(f"[Robot] Current EE pos: {ee['position']}")

        # Plan
        if args.goal_type == "joint":
            goal = list(args.goal_joints) if args.goal_joints else UP_JOINTS
            print(f"\n[Test] Joint planning: current -> goal")
            traj = planner.plan_joint(current, goal, velocity_scale=args.velocity_scale)
        else:
            goal_joints = list(args.goal_joints) if args.goal_joints else UP_JOINTS
            goal_ee = planner.get_ee_pose(goal_joints)
            print(f"\n[Test] Cartesian planning to: pos={goal_ee['position']}")
            traj = planner.plan_cartesian(
                current, goal_ee["position"], goal_ee["quaternion"],
                velocity_scale=args.velocity_scale,
            )

        if traj is None:
            print("[Test] Planning FAILED")
            return False

        print("[Test] Planning SUCCESS")
        print_trajectory_info(traj)

        # Validate
        warnings = validate_trajectory(traj)
        if warnings:
            print("[Safety] Warnings:")
            for w in warnings:
                print(f"  - {w}")

        # Start state check
        if not check_start_match(traj["positions"][0], current):
            print("[Safety] Start state mismatch -- aborting")
            return False

        if args.execute:
            if not args.no_confirm and not confirm_execution(args.velocity_scale, args.mode):
                print("[Test] Cancelled by user")
                return False

            execute_trajectory(robot, traj, command_dt=SERVOJ_DT)
            print("[Test] Execution complete!")

            # Print final position
            final = robot.get_joint_positions()
            print(f"[Robot] Final joints: [{', '.join(f'{v:.4f}' for v in final)}]")
        else:
            print("[Test] Plan-only mode (add --execute to run)")

    return True


def main():
    parser = argparse.ArgumentParser(
        description="Standalone cuMotion test (RTDE / Isaac Sim dual-mode)"
    )
    parser.add_argument(
        "--mode", choices=["rtde", "sim"], default=DEFAULT_MODE,
        help=f"Backend mode (default: {DEFAULT_MODE})"
    )
    parser.add_argument("--robot-ip", default=DEFAULT_ROBOT_IP, help="UR10e IP (rtde mode)")
    parser.add_argument(
        "--goal-type", choices=["joint", "cartesian"], default="joint",
        help="Goal type (default: joint)"
    )
    parser.add_argument("--execute", action="store_true", help="Execute trajectory")
    parser.add_argument("--plan-only", action="store_true", help="Plan without connection")
    parser.add_argument(
        "--velocity-scale", type=float, default=DEFAULT_VELOCITY_SCALE,
        help=f"Velocity scale 0.0-1.0 (default: {DEFAULT_VELOCITY_SCALE})"
    )
    parser.add_argument(
        "--start-joints", type=float, nargs=6, metavar="J",
        help="Start joint positions (6 values, radians). Default: HOME"
    )
    parser.add_argument(
        "--goal-joints", type=float, nargs=6, metavar="J",
        help="Goal joint positions (6 values, radians). Default: UP"
    )
    parser.add_argument("--no-confirm", action="store_true", help="Skip safety confirmation")
    args = parser.parse_args()

    if args.plan_only:
        success = run_plan_only(args)
    else:
        success = run_with_backend(args)

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
