#!/usr/bin/env python3
"""Keyboard teleop for forward_position_controller.

Directly controls UR10e joints via forward_position_controller
by publishing Float64MultiArray to /forward_position_controller/commands.

Key mappings:
  1-6     : Select joint 1~6
  w / ↑   : Increase selected joint position
  s / ↓   : Decrease selected joint position
  +/=     : Increase step size
  -       : Decrease step size
  h       : Go to home position (all zeros)
  p       : Print current joint states
  Space   : Stop (hold current position)
  Esc / q : Quit (restore original controller)

Usage:
  python3 keyboard_forward.py
"""

import sys
import termios
import tty
import select
import signal
import math

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

# Step sizes (radians)
STEP_SIZES = [0.001, 0.005, 0.01, 0.02, 0.05, 0.1]
DEFAULT_STEP_IDX = 2  # 0.01 rad

# Home position (all joints at 0)
HOME_POSITION = [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]

# Publish rate (Hz)
PUBLISH_RATE = 50


HELP_TEXT = """
╔═══════════════════════════════════════════════════╗
║     Forward Position Controller - Keyboard Teleop  ║
╠═══════════════════════════════════════════════════╣
║  1-6       : Select joint                          ║
║  w / ↑     : Increase selected joint (+step)       ║
║  s / ↓     : Decrease selected joint (-step)       ║
║  + / =     : Increase step size                    ║
║  -         : Decrease step size                    ║
║  h         : Go to home position                   ║
║  p         : Print current joint states            ║
║  Space     : Stop (hold current position)          ║
║  Esc / q   : Quit                                  ║
╚═══════════════════════════════════════════════════╝
"""


class KeyboardForwardNode(Node):
    def __init__(self):
        super().__init__('keyboard_forward_position')

        # State
        self.current_positions = None  # Will be set from /joint_states
        self.target_positions = None
        self.selected_joint = 0  # 0-indexed
        self.step_idx = DEFAULT_STEP_IDX
        self.running = True

        # Publisher — must match controller's subscription QoS (RELIABLE + TRANSIENT_LOCAL)
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
            10
        )

        # Controller switcher
        self.switcher = ControllerSwitcher(self)

        # Terminal settings
        self._old_settings = None

    def _joint_state_cb(self, msg: JointState):
        """Update current joint positions from /joint_states."""
        if len(msg.position) < 6:
            return

        # Map joint names to positions in correct order
        positions = [0.0] * 6
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                positions[i] = msg.position[idx]
        self.current_positions = positions

        # Initialize target if not set
        if self.target_positions is None:
            self.target_positions = list(positions)

    @property
    def step_size(self) -> float:
        return STEP_SIZES[self.step_idx]

    def setup_terminal(self):
        """Set terminal to raw mode for non-blocking key reading."""
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def restore_terminal(self):
        """Restore terminal to original settings."""
        if self._old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)

    def get_key(self, timeout: float = 0.02) -> str | None:
        """Read a single keypress with timeout."""
        if select.select([sys.stdin], [], [], timeout)[0]:
            ch = sys.stdin.read(1)
            # Handle escape sequences (arrow keys)
            if ch == '\x1b':
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        if select.select([sys.stdin], [], [], 0.01)[0]:
                            ch3 = sys.stdin.read(1)
                            if ch3 == 'A':
                                return 'UP'
                            elif ch3 == 'B':
                                return 'DOWN'
                            elif ch3 == 'C':
                                return 'RIGHT'
                            elif ch3 == 'D':
                                return 'LEFT'
                return 'ESC'
            return ch
        return None

    def publish_position(self):
        """Publish current target positions."""
        if self.target_positions is None:
            return
        msg = Float64MultiArray()
        msg.data = self.target_positions
        self.cmd_pub.publish(msg)

    def print_status(self):
        """Print current joint status to terminal."""
        if self.current_positions is None:
            print('\r  [No joint_states received yet]')
            return

        print('\r' + ' ' * 80, end='')  # Clear line
        print(f'\r  Joint {self.selected_joint + 1} ({JOINT_NAMES[self.selected_joint]})'
              f'  Step: {self.step_size:.3f} rad ({math.degrees(self.step_size):.1f}°)')

        for i in range(6):
            marker = '>>>' if i == self.selected_joint else '   '
            cur = self.current_positions[i] if self.current_positions else 0.0
            tgt = self.target_positions[i] if self.target_positions else 0.0
            print(f'  {marker} J{i+1} ({JOINT_NAMES[i]:>22s}): '
                  f'cur={math.degrees(cur):7.2f}°  tgt={math.degrees(tgt):7.2f}°')

    def process_key(self, key: str):
        """Process a keypress and update state."""
        if key in ('q', 'ESC'):
            self.running = False
            return

        if self.target_positions is None:
            return

        # Joint selection
        if key in '123456':
            self.selected_joint = int(key) - 1
            self.print_status()
            return

        # Position adjustment
        if key in ('w', 'UP'):
            self.target_positions[self.selected_joint] += self.step_size
            self.print_status()
            return
        if key in ('s', 'DOWN'):
            self.target_positions[self.selected_joint] -= self.step_size
            self.print_status()
            return

        # Step size adjustment
        if key in ('+', '='):
            self.step_idx = min(self.step_idx + 1, len(STEP_SIZES) - 1)
            print(f'\r  Step size: {self.step_size:.3f} rad ({math.degrees(self.step_size):.1f}°)')
            return
        if key == '-':
            self.step_idx = max(self.step_idx - 1, 0)
            print(f'\r  Step size: {self.step_size:.3f} rad ({math.degrees(self.step_size):.1f}°)')
            return

        # Home position
        if key == 'h':
            self.target_positions = list(HOME_POSITION)
            print('\r  >>> Moving to HOME position')
            return

        # Print status
        if key == 'p':
            self.print_status()
            return

        # Space = hold current
        if key == ' ':
            if self.current_positions:
                self.target_positions = list(self.current_positions)
                print('\r  >>> Holding current position')
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

        self.get_logger().info('Joint states received. Starting keyboard teleop.')
        print(HELP_TEXT)
        self.print_status()

        # Set up terminal
        self.setup_terminal()

        try:
            while self.running and rclpy.ok():
                # Process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0.0)

                # Read keyboard
                key = self.get_key(timeout=1.0 / PUBLISH_RATE)
                if key:
                    self.process_key(key)

                # Publish position
                self.publish_position()

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
    node = KeyboardForwardNode()

    # Handle SIGINT gracefully
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
