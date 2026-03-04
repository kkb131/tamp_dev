#!/usr/bin/env python3
"""Keyboard teleop for Cartesian control via forward_position_controller.

Uses Pinocchio for FK/Jacobian and Damped Least Squares (DLS) to convert
keyboard twist inputs to joint position commands, published directly to
forward_position_controller. No MoveIt Servo required.

Key mappings:
  W/S       : X forward/backward
  A/D       : Y left/right
  Q/E       : Z up/down
  U/O       : Roll  (RX) +/-
  I/K       : Pitch (RY) +/-
  J/L       : Yaw   (RZ) +/-
  +/=       : Increase speed scale
  -         : Decrease speed scale
  f         : Toggle frame (base_link / tool0)
  p         : Print current EE pose (FK)
  Space     : Stop (hold current position)
  Esc / x   : Quit (restore original controller)

Usage:
  python3 keyboard_cartesian.py
"""

import sys
import termios
import tty
import select
import signal
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from controller_utils import (
    ControllerSwitcher,
    JOINT_NAMES,
    FORWARD_POSITION_CONTROLLER,
)
from pinocchio_utils import PinocchioIK

# Frames
BASE_FRAME = 'base_link'
EE_FRAME = 'tool0'

# Speed scales (applied to twist input)
SPEED_SCALES = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0]
DEFAULT_SPEED_IDX = 2  # 0.3

# DLS damping factor
DAMPING = 0.05

# Publish rate (Hz)
PUBLISH_RATE = 50


HELP_TEXT = """
╔════════════════════════════════════════════════════════╗
║   Forward Cartesian Controller - Keyboard Teleop       ║
║   (Pinocchio DLS IK, no MoveIt Servo required)         ║
╠════════════════════════════════════════════════════════╣
║  --- Translation ---                                   ║
║  W / S     : X forward / backward                      ║
║  A / D     : Y left / right                            ║
║  Q / E     : Z up / down                               ║
║                                                        ║
║  --- Rotation ---                                      ║
║  U / O     : Roll  (RX) +/-                            ║
║  I / K     : Pitch (RY) +/-                            ║
║  J / L     : Yaw   (RZ) +/-                            ║
║                                                        ║
║  --- Control ---                                       ║
║  + / =     : Increase speed                            ║
║  -         : Decrease speed                            ║
║  f         : Toggle frame (base_link / tool0)          ║
║  p         : Print EE pose (FK)                        ║
║  Space     : Stop                                      ║
║  Esc / x   : Quit                                      ║
╚════════════════════════════════════════════════════════╝
"""

# Key → (linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
KEY_MAP = {
    'w': (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
    'd': (0.0, -1.0, 0.0, 0.0, 0.0, 0.0),
    'q': (0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
    'e': (0.0, 0.0, -1.0, 0.0, 0.0, 0.0),
    'u': (0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
    'o': (0.0, 0.0, 0.0, -1.0, 0.0, 0.0),
    'i': (0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    'k': (0.0, 0.0, 0.0, 0.0, -1.0, 0.0),
    'j': (0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    'l': (0.0, 0.0, 0.0, 0.0, 0.0, -1.0),
}


class KeyboardCartesianNode(Node):
    def __init__(self):
        super().__init__('keyboard_cartesian_teleop')

        # State
        self.running = True
        self.speed_idx = DEFAULT_SPEED_IDX
        self.use_local_frame = False  # False = base_link, True = tool0
        self.current_twist = np.zeros(6)
        self.current_positions = None  # From /joint_states

        # Pinocchio IK
        self.ik = PinocchioIK()
        self.get_logger().info(
            f'Pinocchio loaded: {self.ik.nq} joints, '
            f'EE frame_id={self.ik.ee_frame_id}'
        )

        # Publisher — must match controller's subscription QoS
        cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            f'/{FORWARD_POSITION_CONTROLLER}/commands',
            cmd_qos,
        )

        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10,
        )

        # Controller switcher
        self.switcher = ControllerSwitcher(self)

        # Terminal settings
        self._old_settings = None

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self.speed_idx]

    @property
    def frame_name(self) -> str:
        return EE_FRAME if self.use_local_frame else BASE_FRAME

    def _joint_state_cb(self, msg: JointState):
        """Update current joint positions from /joint_states."""
        if len(msg.position) < 6:
            return
        positions = [0.0] * 6
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                positions[i] = msg.position[idx]
        self.current_positions = np.array(positions)

    def setup_terminal(self):
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def restore_terminal(self):
        if self._old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)

    def get_key(self, timeout: float = 0.02) -> str | None:
        if select.select([sys.stdin], [], [], timeout)[0]:
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        if select.select([sys.stdin], [], [], 0.01)[0]:
                            sys.stdin.read(1)  # consume arrow key code
                return 'ESC'
            return ch
        return None

    def publish_position(self, positions: np.ndarray):
        """Publish joint positions to forward_position_controller."""
        msg = Float64MultiArray()
        msg.data = positions.tolist()
        self.cmd_pub.publish(msg)

    def print_ee_pose(self):
        """Print current EE pose from FK."""
        if self.current_positions is None:
            print('\r  [No joint_states received yet]')
            return

        pos, _ = self.ik.get_ee_pose(self.current_positions)
        rpy = self.ik.get_ee_rpy(self.current_positions)

        print('\r' + ' ' * 80, end='')
        print(f'\r  EE Position: x={pos[0]:.4f}  y={pos[1]:.4f}  z={pos[2]:.4f} [m]')
        print(f'  EE Rotation: R={math.degrees(rpy[0]):.1f}°  '
              f'P={math.degrees(rpy[1]):.1f}°  Y={math.degrees(rpy[2]):.1f}°')
        print(f'  Joints: [{", ".join(f"{math.degrees(j):.1f}" for j in self.current_positions)}]°')

    def process_key(self, key: str):
        """Process a keypress and update state."""
        if key in ('x', 'ESC'):
            self.running = False
            return

        # Movement keys
        if key in KEY_MAP:
            self.current_twist = np.array(KEY_MAP[key])
            return

        # Speed adjustment
        if key in ('+', '='):
            self.speed_idx = min(self.speed_idx + 1, len(SPEED_SCALES) - 1)
            print(f'\r  Speed: {self.speed_scale:.1f}          ')
            return
        if key == '-':
            self.speed_idx = max(self.speed_idx - 1, 0)
            print(f'\r  Speed: {self.speed_scale:.1f}          ')
            return

        # Toggle frame
        if key == 'f':
            self.use_local_frame = not self.use_local_frame
            print(f'\r  Frame: {self.frame_name}          ')
            return

        # Print EE pose
        if key == 'p':
            self.print_ee_pose()
            return

        # Space = stop
        if key == ' ':
            self.current_twist = np.zeros(6)
            print('\r  >>> STOP                    ')
            return

    def run(self):
        """Main loop."""
        # Wait for services
        if not self.switcher.wait_for_services():
            self.get_logger().error('Controller manager not available. Exiting.')
            return

        # Show initial controller status
        self.switcher.print_status()

        # Switch to forward_position_controller
        if not self.switcher.activate_forward_position():
            self.get_logger().error('Failed to activate forward_position_controller. Exiting.')
            return

        # Wait for first joint_states
        self.get_logger().info('Waiting for /joint_states...')
        while self.current_positions is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_positions is None:
            self.get_logger().error('No joint_states received. Exiting.')
            return

        self.get_logger().info('Joint states received. Starting keyboard Cartesian teleop.')
        print(HELP_TEXT)
        print(f'  Speed: {self.speed_scale:.1f}  |  Frame: {self.frame_name}')
        print('  Ready! Press keys to move the robot.\n')

        # Set up terminal
        self.setup_terminal()
        dt = 1.0 / PUBLISH_RATE

        try:
            while self.running and rclpy.ok():
                # Process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0.0)

                # Read keyboard
                key = self.get_key(timeout=dt)
                if key:
                    self.process_key(key)

                if self.current_positions is None:
                    continue

                # Compute and publish
                if np.any(self.current_twist != 0.0):
                    twist = self.current_twist * self.speed_scale
                    dq = self.ik.compute_joint_delta(
                        self.current_positions, twist, dt,
                        damping=DAMPING, local=self.use_local_frame,
                    )
                    target = self.ik.clamp_positions(self.current_positions + dq)
                    self.publish_position(target)
                    # Reset twist (need to keep pressing key)
                    self.current_twist = np.zeros(6)
                else:
                    # Hold current position
                    self.publish_position(self.current_positions)

        except KeyboardInterrupt:
            pass
        finally:
            self.restore_terminal()
            print('\n\nRestoring original controller...')
            self.switcher.restore_original()
            self.switcher.print_status()
            print('Done.')


def main():
    rclpy.init()
    node = KeyboardCartesianNode()

    def signal_handler(sig, frame):
        node.running = False
    signal.signal(signal.SIGINT, signal_handler)

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
