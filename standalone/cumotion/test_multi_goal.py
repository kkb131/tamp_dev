#!/usr/bin/env python3
"""Multi-goal sequential planning & execution test.

Plans and (optionally) executes a series of moves between near-home waypoints.
Useful for verifying planner reliability and robot motion across multiple plans.

Usage:
    # Plan-only (no connection needed)
    python3 -m standalone.cumotion.test_multi_goal --plan-only

    # Isaac Sim
    python3 -m standalone.cumotion.test_multi_goal --mode sim --execute --velocity-scale 0.1

    # Real robot
    python3 -m standalone.cumotion.test_multi_goal --mode rtde --robot-ip 192.168.0.2 --execute

    # Custom rounds and return-home
    python3 -m standalone.cumotion.test_multi_goal --plan-only --rounds 3 --no-return-home
"""

import argparse
import sys
import time

import numpy as np

from standalone.config import (
    DEFAULT_MODE,
    DEFAULT_ROBOT_IP,
    DEFAULT_VELOCITY_SCALE,
    HOME_JOINTS,
    NEAR_HOME_WAYPOINTS,
    SERVOJ_DT,
    XRDF_PATH,
    URDF_PATH,
)
from standalone.cumotion.planner import StandaloneMotionPlanner


def print_trajectory_info(traj: dict, label: str = ""):
    prefix = f"[{label}] " if label else ""
    print(
        f"  {prefix}plan={traj['plan_time']:.3f}s  "
        f"pts={traj['n_points']}  "
        f"dur={traj['total_time']:.2f}s"
    )


def build_sequence(rounds: int, return_home: bool) -> list:
    """Build the waypoint sequence: HOME → A → B → ... → (HOME) × rounds."""
    seq = []
    for _ in range(rounds):
        for i, wp in enumerate(NEAR_HOME_WAYPOINTS):
            label = chr(ord("A") + i)
            seq.append((label, wp))
        if return_home:
            seq.append(("HOME", HOME_JOINTS))
    return seq


def run_plan_only(args):
    planner = StandaloneMotionPlanner(XRDF_PATH, URDF_PATH)
    sequence = build_sequence(args.rounds, args.return_home)

    current = list(HOME_JOINTS)
    total_plans = len(sequence)
    success_count = 0
    fail_count = 0
    total_plan_time = 0.0

    print(f"\n[MultiGoal] Plan-only: {total_plans} moves, {args.rounds} round(s)")
    print(f"[MultiGoal] Start: HOME {HOME_JOINTS}\n")

    for i, (label, goal) in enumerate(sequence, 1):
        print(f"[{i}/{total_plans}] → {label}", end="  ")
        traj = planner.plan_joint(current, goal, velocity_scale=args.velocity_scale)

        if traj is None:
            print("FAILED")
            fail_count += 1
        else:
            print_trajectory_info(traj)
            success_count += 1
            total_plan_time += traj["plan_time"]
            current = traj["positions"][-1].tolist()

    print(f"\n{'='*50}")
    print(f"  Results: {success_count}/{total_plans} succeeded, {fail_count} failed")
    print(f"  Total planning time: {total_plan_time:.3f}s")
    print(f"{'='*50}\n")
    return fail_count == 0


def run_with_backend(args):
    from standalone.core.robot_backend import create_backend
    from standalone.core.trajectory_executor import (
        check_start_match,
        execute_trajectory,
        validate_trajectory,
    )

    planner = StandaloneMotionPlanner(XRDF_PATH, URDF_PATH)
    sequence = build_sequence(args.rounds, args.return_home)
    total_plans = len(sequence)

    backend_kwargs = {"robot_ip": args.robot_ip}
    with create_backend(args.mode, **backend_kwargs) as robot:
        current = robot.get_joint_positions()
        print(f"\n[MultiGoal] Current joints: [{', '.join(f'{v:.4f}' for v in current)}]")
        print(f"[MultiGoal] {total_plans} moves, {args.rounds} round(s)")

        if args.execute and not args.no_confirm:
            label = "REAL ROBOT" if args.mode == "rtde" else "ISAAC SIM"
            print(f"\n{'!'*50}")
            print(f"  {label} — {total_plans} sequential moves")
            print(f"  Velocity scale: {args.velocity_scale*100:.1f}%")
            print(f"{'!'*50}")
            ans = input("\nProceed with all moves? (yes/no): ").strip().lower()
            if ans != "yes":
                print("[MultiGoal] Cancelled")
                return False

        success_count = 0
        fail_count = 0

        for i, (label, goal) in enumerate(sequence, 1):
            current = robot.get_joint_positions()
            print(f"\n[{i}/{total_plans}] → {label}")
            traj = planner.plan_joint(current, goal, velocity_scale=args.velocity_scale)

            if traj is None:
                print("  Planning FAILED — skipping")
                fail_count += 1
                continue

            print_trajectory_info(traj)

            warnings = validate_trajectory(traj)
            if warnings:
                for w in warnings:
                    print(f"  WARNING: {w}")

            if not check_start_match(traj["positions"][0], current):
                print("  Start mismatch — skipping")
                fail_count += 1
                continue

            success_count += 1

            if args.execute:
                execute_trajectory(robot, traj, command_dt=SERVOJ_DT)
                final = robot.get_joint_positions()
                goal_err = np.max(np.abs(np.array(final) - np.array(goal)))
                print(f"  Reached (max goal err: {goal_err:.4f} rad)")
            else:
                print("  Plan OK (add --execute to run)")

        print(f"\n{'='*50}")
        print(f"  Results: {success_count}/{total_plans} succeeded, {fail_count} failed")
        if args.execute:
            final = robot.get_joint_positions()
            print(f"  Final joints: [{', '.join(f'{v:.4f}' for v in final)}]")
        print(f"{'='*50}\n")

    return fail_count == 0


def main():
    parser = argparse.ArgumentParser(
        description="Multi-goal sequential planning & execution test"
    )
    parser.add_argument(
        "--mode", choices=["rtde", "sim"], default=DEFAULT_MODE,
        help=f"Backend mode (default: {DEFAULT_MODE})"
    )
    parser.add_argument("--robot-ip", default=DEFAULT_ROBOT_IP, help="UR10e IP (rtde mode)")
    parser.add_argument("--execute", action="store_true", help="Execute trajectories")
    parser.add_argument("--plan-only", action="store_true", help="Plan without connection")
    parser.add_argument(
        "--velocity-scale", type=float, default=DEFAULT_VELOCITY_SCALE,
        help=f"Velocity scale 0.0-1.0 (default: {DEFAULT_VELOCITY_SCALE})"
    )
    parser.add_argument(
        "--rounds", type=int, default=1,
        help="Number of rounds through all waypoints (default: 1)"
    )
    parser.add_argument(
        "--no-return-home", action="store_true",
        help="Skip returning to HOME between rounds"
    )
    parser.add_argument("--no-confirm", action="store_true", help="Skip safety confirmation")
    args = parser.parse_args()
    args.return_home = not args.no_return_home

    if args.plan_only:
        success = run_plan_only(args)
    else:
        success = run_with_backend(args)

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
