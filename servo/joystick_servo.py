#!/usr/bin/env python3
"""Xbox joystick teleop for MoveIt Servo (Cartesian control).

Subscribes to /joy (sensor_msgs/Joy) from ros-jazzy-joy node and
converts Xbox controller inputs to TwistStamped for MoveIt Servo.

Requires:
  - MoveIt Servo node running (launch_servo:=true)
  - joy_node running: ros2 run joy joy_node

Xbox controller mapping:
  Left Stick X/Y   : Y/X translation
  Right Stick X/Y   : Yaw/Pitch rotation
  LT / RT (triggers): Z down / up
  LB / RB (bumpers) : Roll -/+
  A button          : Decrease speed
  B button          : Increase speed
  X button          : Toggle frame (base_link / tool0)
  Start button      : Quit

Usage:
  # Terminal 1: Start joy node
  ros2 run joy joy_node

  # Terminal 2: Start this teleop
  python3 joystick_servo.py
"""

import signal

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from moveit_msgs.srv import ServoCommandType

from controller_utils import ControllerSwitcher

# Topics
TWIST_TOPIC = '/servo_node/delta_twist_cmds'
JOY_TOPIC = '/joy'
SWITCH_CMD_TYPE_SERVICE = '/servo_node/switch_command_type'

# Frames
BASE_FRAME = 'base_link'
EE_FRAME = 'tool0'

# Speed scales
SPEED_SCALES = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0]
DEFAULT_SPEED_IDX = 2  # 0.3

# Xbox controller button/axis indices (may vary by driver)
# Axes
AXIS_LEFT_STICK_X = 0   # Y translation
AXIS_LEFT_STICK_Y = 1   # X translation
AXIS_RIGHT_STICK_X = 3  # Yaw rotation
AXIS_RIGHT_STICK_Y = 4  # Pitch rotation
AXIS_LT = 2             # Left trigger (1.0 released, -1.0 pressed)
AXIS_RT = 5             # Right trigger (1.0 released, -1.0 pressed)

# Buttons
BTN_A = 0        # Speed down
BTN_B = 1        # Speed up
BTN_X = 2        # Toggle frame
BTN_LB = 4       # Roll -
BTN_RB = 5       # Roll +
BTN_START = 7    # Quit

# Deadzone for analog sticks
DEADZONE = 0.1

# Publish rate
PUBLISH_RATE = 30  # Hz


class JoystickServoNode(Node):
    def __init__(self):
        super().__init__('joystick_servo_teleop')

        # State
        self.running = True
        self.speed_idx = DEFAULT_SPEED_IDX
        self.command_frame = BASE_FRAME
        self.latest_joy = None

        # Track button states for edge detection
        self._prev_buttons = []

        # Publisher
        self.twist_pub = self.create_publisher(
            TwistStamped,
            TWIST_TOPIC,
            10
        )

        # Subscriber
        self.joy_sub = self.create_subscription(
            Joy,
            JOY_TOPIC,
            self._joy_callback,
            10
        )

        # Service client
        self.switch_cmd_client = self.create_client(
            ServoCommandType,
            SWITCH_CMD_TYPE_SERVICE
        )

        # Controller switcher
        self.switcher = ControllerSwitcher(self)

        # Timer for publishing
        self.create_timer(1.0 / PUBLISH_RATE, self._timer_callback)

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self.speed_idx]

    def _apply_deadzone(self, value: float) -> float:
        """Apply deadzone to analog input."""
        if abs(value) < DEADZONE:
            return 0.0
        return value

    def _button_pressed(self, msg: Joy, btn_idx: int) -> bool:
        """Check if a button was just pressed (rising edge)."""
        if btn_idx >= len(msg.buttons):
            return False
        current = msg.buttons[btn_idx]
        prev = self._prev_buttons[btn_idx] if btn_idx < len(self._prev_buttons) else 0
        return current == 1 and prev == 0

    def _joy_callback(self, msg: Joy):
        """Process incoming Joy messages."""
        # Handle button presses (edge detection)
        if self._button_pressed(msg, BTN_A):
            self.speed_idx = max(self.speed_idx - 1, 0)
            self.get_logger().info(f'Speed: {self.speed_scale:.1f}')

        if self._button_pressed(msg, BTN_B):
            self.speed_idx = min(self.speed_idx + 1, len(SPEED_SCALES) - 1)
            self.get_logger().info(f'Speed: {self.speed_scale:.1f}')

        if self._button_pressed(msg, BTN_X):
            if self.command_frame == BASE_FRAME:
                self.command_frame = EE_FRAME
            else:
                self.command_frame = BASE_FRAME
            self.get_logger().info(f'Frame: {self.command_frame}')

        if self._button_pressed(msg, BTN_START):
            self.running = False

        # Save for next edge detection
        self._prev_buttons = list(msg.buttons)
        self.latest_joy = msg

    def _timer_callback(self):
        """Periodically publish twist from latest joystick state."""
        if self.latest_joy is None:
            return

        msg = self.latest_joy
        axes = msg.axes
        buttons = msg.buttons

        if len(axes) < 6:
            return

        # Compute twist from joystick
        lx = self._apply_deadzone(axes[AXIS_LEFT_STICK_X])   # Y
        ly = self._apply_deadzone(axes[AXIS_LEFT_STICK_Y])   # X
        rx = self._apply_deadzone(axes[AXIS_RIGHT_STICK_X])  # Yaw
        ry = self._apply_deadzone(axes[AXIS_RIGHT_STICK_Y])  # Pitch

        # Triggers: convert from [1.0, -1.0] to [0.0, 1.0]
        lt = (1.0 - axes[AXIS_LT]) / 2.0  # Z down
        rt = (1.0 - axes[AXIS_RT]) / 2.0  # Z up

        # Bumpers for roll
        roll = 0.0
        if BTN_RB < len(buttons) and buttons[BTN_RB]:
            roll = 1.0
        if BTN_LB < len(buttons) and buttons[BTN_LB]:
            roll = -1.0

        # Check if any input is active
        linear_x = ly
        linear_y = lx
        linear_z = rt - lt
        angular_x = roll
        angular_y = ry
        angular_z = rx

        has_input = any(abs(v) > 0.01 for v in [
            linear_x, linear_y, linear_z,
            angular_x, angular_y, angular_z
        ])

        if not has_input:
            return

        # Build and publish TwistStamped
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.command_frame

        scale = self.speed_scale
        twist_msg.twist.linear.x = linear_x * scale
        twist_msg.twist.linear.y = linear_y * scale
        twist_msg.twist.linear.z = linear_z * scale
        twist_msg.twist.angular.x = angular_x * scale
        twist_msg.twist.angular.y = angular_y * scale
        twist_msg.twist.angular.z = angular_z * scale

        self.twist_pub.publish(twist_msg)

    def switch_to_twist_mode(self):
        """Request servo to accept Twist commands."""
        if not self.switch_cmd_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                'switch_command_type service not available. '
                'Servo may already be in twist mode.'
            )
            return

        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST
        future = self.switch_cmd_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            self.get_logger().info('Servo switched to TWIST mode.')
        else:
            self.get_logger().warn('Failed to switch servo to TWIST mode.')

    def run(self):
        """Main loop."""
        # Wait for controller_manager
        if not self.switcher.wait_for_services():
            self.get_logger().error('Controller manager not available. Exiting.')
            return

        # Switch to forward_position_controller
        self.switcher.print_status()
        if not self.switcher.activate_forward_position():
            self.get_logger().error('Failed to activate forward_position_controller. Exiting.')
            return

        # Switch servo to twist mode
        self.get_logger().info('Switching servo to TWIST mode...')
        self.switch_to_twist_mode()

        self.get_logger().info(
            f'Joystick teleop ready. Speed: {self.speed_scale:.1f}  Frame: {self.command_frame}'
        )
        self.get_logger().info('Waiting for /joy messages (run: ros2 run joy joy_node)...')

        print('\n  Xbox Controller Mapping:')
        print('  Left Stick    : X/Y translation')
        print('  Right Stick   : Yaw/Pitch rotation')
        print('  LT/RT         : Z down/up')
        print('  LB/RB         : Roll -/+')
        print('  A/B           : Speed -/+')
        print('  X             : Toggle frame')
        print('  Start         : Quit\n')

        try:
            while self.running and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            # Send zero twist
            zero = TwistStamped()
            zero.header.stamp = self.get_clock().now().to_msg()
            zero.header.frame_id = self.command_frame
            self.twist_pub.publish(zero)

            self.get_logger().info('Restoring original controller...')
            self.switcher.restore_original()
            self.switcher.print_status()
            self.get_logger().info('Done.')


def main():
    rclpy.init()
    node = JoystickServoNode()

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
