"""Pure admittance control tester (no safety monitor).

Minimal pipeline: F/T sensor → wrench transform → AdmittanceController → PinkIK → servoJ.
Push the robot by hand and it should follow compliantly.

Wrench transform candidates (A-D) and rotation displacement candidates (a-d)
are provided as commented code. Uncomment ONE of each to test which mapping
is correct for your setup.

Usage:
    cd /workspaces/tamp_ws/src/tamp_dev
    python3 -m standalone.teleop_admittance.test_admittance --robot-ip 192.168.0.2
"""

import argparse
import sys
import time
import select
import termios
import tty

import numpy as np
import pinocchio as pin

from standalone.config import URDF_PATH, RTDE_FREQUENCY
from standalone.core.robot_backend import create_backend
from standalone.core.ft_source import RTDEFTSource
from standalone.core.pink_ik import PinkIK
from standalone.core.compliant_control import (
    AdmittanceController,
    COMPLIANCE_PRESETS,
)


PRESET_KEYS = {"1": "SOFT", "2": "MEDIUM", "3": "STIFF"}


def get_key_nonblocking():
    """Non-blocking single key read from stdin."""
    if select.select([sys.stdin], [], [], 0.0)[0]:
        return sys.stdin.read(1)
    return None


def quat_to_matrix(quat_xyzw: np.ndarray) -> np.ndarray:
    """Convert quaternion (x,y,z,w) to 3x3 rotation matrix via Pinocchio."""
    q = pin.Quaternion(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2])
    return q.matrix()


def apply_rotation_delta(
    quat_xyzw: np.ndarray, angular_vel: np.ndarray, dt: float
) -> np.ndarray:
    """Apply angular velocity delta to a quaternion."""
    angle = np.linalg.norm(angular_vel) * dt
    if angle < 1e-10:
        return quat_xyzw.copy()
    axis = angular_vel / (np.linalg.norm(angular_vel) + 1e-15)
    aa = pin.AngleAxis(angle, axis)
    dR = aa.matrix()
    q_pin = pin.Quaternion(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2])
    R_new = dR @ q_pin.matrix()
    q_new = pin.Quaternion(R_new)
    return np.array([q_new.x, q_new.y, q_new.z, q_new.w])


def run(backend):
    """Run pure admittance control loop."""
    dt = 1.0 / RTDE_FREQUENCY

    # Wait for valid joint state
    print("[Admittance] Waiting for valid joint state...")
    for _ in range(50):
        q = backend.get_joint_positions()
        if any(v != 0.0 for v in q):
            break
        time.sleep(0.1)
    else:
        print("[Admittance] ERROR: No valid joint state received!")
        return

    # F/T source (raw TCP wrench with bias correction, NO frame transform)
    raw_ft = RTDEFTSource(backend)

    # IK — use initialize() to set PostureTask target (not sync_configuration)
    ik = PinkIK(URDF_PATH)
    q_current = np.array(backend.get_joint_positions())
    ik.initialize(q_current)
    home_pos, home_quat = ik.get_ee_pose(q_current)

    # Admittance controller
    preset_name = "SOFT"
    controller = AdmittanceController(
        params=COMPLIANCE_PRESETS[preset_name],
        max_disp_trans=0.05,
        max_disp_rot=0.15,
        force_deadzone=np.array([3.0, 3.0, 3.0, 0.3, 0.3, 0.3]),
    )

    print(f"[Admittance] Home: [{home_pos[0]:.3f}, {home_pos[1]:.3f}, {home_pos[2]:.3f}]")
    print("[Admittance] Ready. Push the robot to test. [z]zero [1]SOFT [2]MEDIUM [3]STIFF [q]quit")

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        while True:
            t_start = time.perf_counter()

            # Key handling
            key = get_key_nonblocking()
            if key == "q":
                break
            elif key == "z":
                raw_ft.zero_sensor()
                controller.reset()
                q_current = np.array(backend.get_joint_positions())
                ik.initialize(q_current)
                home_pos, home_quat = ik.get_ee_pose(q_current)
                print("\033[2K\r[Admittance] Sensor zeroed, home reset.")
                time.sleep(0.3)
            elif key in PRESET_KEYS:
                preset_name = PRESET_KEYS[key]
                controller.set_params(COMPLIANCE_PRESETS[preset_name])
                controller.reset()
                q_current = np.array(backend.get_joint_positions())
                ik.initialize(q_current)
                home_pos, home_quat = ik.get_ee_pose(q_current)

            # --- Wrench transform: TCP frame → base_link frame ---
            # Read raw wrench in TCP frame
            wrench_tcp = raw_ft.get_wrench()

            # Get tool0 rotation matrix in base_link frame via Pinocchio FK
            q_current = np.array(backend.get_joint_positions())
            ee_pos_now, ee_quat_now = ik.get_ee_pose(q_current)
            R = quat_to_matrix(ee_quat_now)  # base_link ← tool0 rotation

            # --- Wrench transform candidates (uncomment ONE) ---
            # (A) Pinocchio FK R @ wrench — theoretically correct (base_link frame)
            wrench_base = np.concatenate([R @ wrench_tcp[:3], R @ wrench_tcp[3:]])
            # (B) R.T @ wrench — inverse rotation
            # wrench_base = np.concatenate([R.T @ wrench_tcp[:3], R.T @ wrench_tcp[3:]])
            # (C) negate X,Y only — maps TCP→UR Base (NOT base_link)
            # wrench_base = np.array([-wrench_tcp[0], -wrench_tcp[1], wrench_tcp[2],
            #                          -wrench_tcp[3], -wrench_tcp[4], wrench_tcp[5]])
            # (D) raw — no transform
            # wrench_base = wrench_tcp.copy()

            # Admittance dynamics → displacement in base_link frame
            disp = controller.update(wrench_base, dt)

            # --- Rotation displacement candidates (uncomment ONE) ---
            # (a) raw — use admittance rotation displacement as-is
            rot_disp = disp[3:]
            # (b) negate RX, RY only
            # rot_disp = np.array([-disp[3], -disp[4], disp[5]])
            # (c) negate all rotation
            # rot_disp = -disp[3:]
            # (d) disable rotation (translation only)
            # rot_disp = np.zeros(3)

            # Target = home + displacement
            target_pos = home_pos + disp[:3]
            target_quat = apply_rotation_delta(home_quat, rot_disp, 1.0)

            # IK
            ik.sync_configuration(q_current)
            q_target = ik.solve(target_pos, target_quat, dt)
            if q_target is None:
                q_target = q_current.copy()

            # Send command
            backend.send_joint_command(q_target.tolist())

            # Current EE pose
            ee_pos, _ = ik.get_ee_pose(q_target)

            # Display
            p = COMPLIANCE_PRESETS[preset_name]
            disp_mm = disp[:3] * 1000
            disp_deg = np.degrees(rot_disp)
            lines = ["\033[2J\033[H"]
            lines.append("=== Admittance Control Test (No Safety) ===")
            lines.append(
                f"Preset: [{preset_name}]  M={p.M[0]:.0f} D={p.D[0]:.0f} K={p.K[0]:.0f}"
            )
            lines.append(
                f"TCP:    [{ee_pos[0]:+.3f}, {ee_pos[1]:+.3f}, {ee_pos[2]:+.3f}]"
            )
            lines.append(
                f"Transform: (A) R@wrench [Pinocchio FK]"
            )
            lines.append("")
            lines.append(
                f"Wrench(TCP): [{wrench_tcp[0]:+6.1f}, {wrench_tcp[1]:+6.1f},"
                f" {wrench_tcp[2]:+6.1f} |"
                f" {wrench_tcp[3]:+5.2f}, {wrench_tcp[4]:+5.2f}, {wrench_tcp[5]:+5.2f}]"
            )
            lines.append(
                f"Wrench(base): [{wrench_base[0]:+6.1f}, {wrench_base[1]:+6.1f},"
                f" {wrench_base[2]:+6.1f} |"
                f" {wrench_base[3]:+5.2f}, {wrench_base[4]:+5.2f}, {wrench_base[5]:+5.2f}]"
            )
            lines.append(
                f"Displacement: [{disp_mm[0]:+6.1f}, {disp_mm[1]:+6.1f},"
                f" {disp_mm[2]:+6.1f}]mm |"
                f" [{disp_deg[0]:+5.1f}, {disp_deg[1]:+5.1f}, {disp_deg[2]:+5.1f}]deg"
            )
            lines.append("")
            lines.append("[1]SOFT [2]MEDIUM [3]STIFF  [z]zero+reset  [q]quit")
            lines.append("Push the robot -> it should follow your hand.")

            sys.stdout.write("\n".join(lines))
            sys.stdout.flush()

            # Loop timing
            elapsed = time.perf_counter() - t_start
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        backend.on_trajectory_done()


def main():
    parser = argparse.ArgumentParser(description="Pure admittance control test")
    parser.add_argument("--robot-ip", default="192.168.0.2")
    args = parser.parse_args()

    backend = create_backend("rtde", robot_ip=args.robot_ip)
    with backend:
        run(backend)
    print("\n[Admittance] Done.")


if __name__ == "__main__":
    main()
