#!/usr/bin/env python3
"""Keyboard teleop for MoveIt Servo (Cartesian control).

Publishes TwistStamped to MoveIt Servo's delta_twist_cmds topic
for real-time Cartesian end-effector control.

Requires: MoveIt Servo node running (launch_servo:=true)

Key mappings:
  W/S       : X forward/backward
  A/D       : Y left/right
  Q/E       : Z up/down
  I/K       : Pitch (RY) +/-
  J/L       : Yaw (RZ) +/-
  U/O       : Roll (RX) +/-
  +/=       : Increase speed scale
  -         : Decrease speed scale
  f         : Toggle frame (base_link / tool0)
  Space     : Stop (zero twist)
  Esc / x   : Quit (restore original controller)

Usage:
  python3 keyboard_servo.py
"""
from __future__ import annotations

import sys
import termios
import tty
import select
import signal

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

from standalone.servo.controller_utils import ControllerSwitcher

# Topics (matching ur_servo.yaml: ~/delta_twist_cmds → /servo_node/delta_twist_cmds)
TWIST_TOPIC = '/servo_node/delta_twist_cmds'

# Frames
BASE_FRAME = 'base_link'
EE_FRAME = 'tool0'

# Speed scales
SPEED_SCALES = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0]
DEFAULT_SPEED_IDX = 2  # 0.3

# Publish rate
PUBLISH_RATE = 30  # Hz (servo runs at 250Hz, we publish slower)


HELP_TEXT = """
╔════════════════════════════════════════════════════╗
║     MoveIt Servo - Keyboard Cartesian Teleop       ║
╠════════════════════════════════════════════════════╣
║  --- Translation ---                               ║
║  W / S     : X forward / backward                  ║
║  A / D     : Y left / right                        ║
║  Q / E     : Z up / down                           ║
║                                                    ║
║  --- Rotation ---                                  ║
║  U / O     : Roll  (RX) +/-                        ║
║  I / K     : Pitch (RY) +/-                        ║
║  J / L     : Yaw   (RZ) +/-                        ║
║                                                    ║
║  --- Control ---                                   ║
║  + / =     : Increase speed                        ║
║  -         : Decrease speed                        ║
║  f         : Toggle frame (base_link / tool0)      ║
║  Space     : Stop                                  ║
║  Esc / x   : Quit                                  ║
╚════════════════════════════════════════════════════╝
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


class KeyboardServoNode(Node):
    def __init__(self):
        super().__init__('keyboard_servo_teleop')

        # State
        self.running = True
        self.speed_idx = DEFAULT_SPEED_IDX
        self.command_frame = BASE_FRAME  # Start with base frame for intuitive control
        self.current_twist = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # Publisher
        self.twist_pub = self.create_publisher(
            TwistStamped,
            TWIST_TOPIC,
            10
        )

        # Controller switcher
        self.switcher = ControllerSwitcher(self)

        # Terminal settings
        self._old_settings = None

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self.speed_idx]

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
                return 'ESC'
            return ch
        return None

    def publish_twist(self, twist_values: tuple):
        """Publish TwistStamped message."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame

        scale = self.speed_scale
        msg.twist.linear.x = twist_values[0] * scale
        msg.twist.linear.y = twist_values[1] * scale
        msg.twist.linear.z = twist_values[2] * scale
        msg.twist.angular.x = twist_values[3] * scale
        msg.twist.angular.y = twist_values[4] * scale
        msg.twist.angular.z = twist_values[5] * scale

        self.twist_pub.publish(msg)

    def publish_zero(self):
        """Publish zero twist to stop movement."""
        self.current_twist = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.publish_twist(self.current_twist)

    def process_key(self, key: str):
        """Process a keypress."""
        if key in ('x', 'ESC'):
            self.running = False
            return

        # Movement keys
        if key in KEY_MAP:
            self.current_twist = KEY_MAP[key]
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
            if self.command_frame == BASE_FRAME:
                self.command_frame = EE_FRAME
            else:
                self.command_frame = BASE_FRAME
            print(f'\r  Frame: {self.command_frame}          ')
            return

        # Space = stop
        if key == ' ':
            self.publish_zero()
            print('\r  >>> STOP                    ')
            return

    def run(self):
        """Main loop."""
        # Wait for controller_manager
        if not self.switcher.wait_for_services():
            self.get_logger().error('Controller manager not available. Exiting.')
            return

        # Switch to forward_position_controller (required by servo)
        self.switcher.print_status()
        if not self.switcher.activate_forward_position():
            self.get_logger().error('Failed to activate forward_position_controller. Exiting.')
            return

        print(HELP_TEXT)
        print(f'  Speed: {self.speed_scale:.1f}  |  Frame: {self.command_frame}')
        print('  Ready! Press keys to move the robot.\n')

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

                # Publish current twist (keep sending while key held)
                if any(v != 0.0 for v in self.current_twist):
                    self.publish_twist(self.current_twist)
                    # Reset after one publish (need to keep pressing)
                    self.current_twist = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        except KeyboardInterrupt:
            pass
        finally:
            self.restore_terminal()
            # Send stop command
            self.publish_zero()
            print('\n\nRestoring original controller...')
            self.switcher.restore_original()
            self.switcher.print_status()
            print('Done.')


def main():
    rclpy.init()
    node = KeyboardServoNode()

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
