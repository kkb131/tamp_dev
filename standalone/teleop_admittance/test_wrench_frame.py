"""Wrench frame transformation diagnostic tester.

Connects to UR10e via RTDE, reads F/T sensor + TCP pose, and displays
multiple transformation candidates side-by-side so you can identify
which one maps correctly to the base frame.

No servo, no safety, no IK — pure sensor reading + math.

Usage:
    cd /workspaces/tamp_ws/src/tamp_dev
    python3 -m standalone.teleop_admittance.test_wrench_frame --robot-ip 192.168.0.2
"""

import argparse
import sys
import time
import select
import termios
import tty

import numpy as np

from standalone.core.robot_backend import create_backend
from standalone.core.ft_source import rotvec_to_matrix


def get_key_nonblocking():
    """Non-blocking single key read from stdin."""
    if select.select([sys.stdin], [], [], 0.0)[0]:
        return sys.stdin.read(1)
    return None


def main():
    parser = argparse.ArgumentParser(description="Wrench frame diagnostic")
    parser.add_argument("--robot-ip", default="192.168.0.2")
    args = parser.parse_args()

    # Connect
    backend = create_backend("rtde", args.robot_ip)
    backend.connect()
    print("[Diag] Connected. Press 'z' to zero sensor, 'q' to quit.\n")

    bias = np.zeros(6)

    # Set terminal to raw mode for non-blocking key input
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        while True:
            # Key handling
            key = get_key_nonblocking()
            if key == "q":
                break
            elif key == "z":
                bias = np.array(backend.get_tcp_force())
                print("\033[2K\r[Diag] Sensor zeroed!")
                time.sleep(0.3)

            # Read sensor data
            raw_wrench = np.array(backend.get_tcp_force()) - bias
            tcp_pose = backend.get_tcp_pose()
            rotvec = np.array(tcp_pose[3:6])
            R = rotvec_to_matrix(rotvec)
            angle_deg = np.degrees(np.linalg.norm(rotvec))

            f = raw_wrench[:3]  # force in TCP frame
            t = raw_wrench[3:]  # torque in TCP frame

            # Candidate transformations (force only for display clarity)
            candidates = {
                "A  raw     ": f,
                "B  R@f     ": R @ f,
                "C  R.T@f   ": R.T @ f,
                "D  neg(XY) ": np.array([-f[0], -f[1], f[2]]),
                "E  R+negXY ": (lambda v: np.array([-v[0], -v[1], v[2]]))(R @ f),
            }

            # Build display
            lines = []
            lines.append("\033[2J\033[H")  # clear screen
            lines.append("=== Wrench Frame Diagnostic ===")
            lines.append(
                f"TCP pose: [{tcp_pose[0]:+.3f}, {tcp_pose[1]:+.3f}, {tcp_pose[2]:+.3f}"
                f" | {rotvec[0]:+.3f}, {rotvec[1]:+.3f}, {rotvec[2]:+.3f}]"
            )
            lines.append(f"TCP rotvec angle: {angle_deg:.1f} deg")
            lines.append("")
            lines.append("Push the robot and check which row matches expected direction.")
            lines.append("Base frame: X=right, Y=forward(away from base), Z=up")
            lines.append("[z] zero sensor  [q] quit")
            lines.append("")
            lines.append(f"{'':13s} {'Force X':>9s} {'Force Y':>9s} {'Force Z':>9s}")
            lines.append(f"{'':13s} {'-------':>9s} {'-------':>9s} {'-------':>9s}")
            for name, fv in candidates.items():
                lines.append(f"{name}  {fv[0]:+9.2f} {fv[1]:+9.2f} {fv[2]:+9.2f}")

            lines.append("")
            lines.append(f"{'':13s} {'Torq X':>9s} {'Torq Y':>9s} {'Torq Z':>9s}")
            lines.append(f"{'':13s} {'------':>9s} {'------':>9s} {'------':>9s}")
            # Same transformations for torque
            t_candidates = {
                "A  raw     ": t,
                "B  R@t     ": R @ t,
                "C  R.T@t   ": R.T @ t,
                "D  neg(XY) ": np.array([-t[0], -t[1], t[2]]),
                "E  R+negXY ": (lambda v: np.array([-v[0], -v[1], v[2]]))(R @ t),
            }
            for name, tv in t_candidates.items():
                lines.append(f"{name}  {tv[0]:+9.3f} {tv[1]:+9.3f} {tv[2]:+9.3f}")

            sys.stdout.write("\n".join(lines))
            sys.stdout.flush()

            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        backend.disconnect()
        print("\n[Diag] Done.")


if __name__ == "__main__":
    main()
