#!/usr/bin/env python3
"""Xbox joystick teleop for Cartesian control via forward_position_controller.

Uses Pinocchio for FK/Jacobian and Damped Least Squares (DLS) to convert
Xbox controller inputs to joint position commands, published directly to
forward_position_controller. No MoveIt Servo required.

Requires:
  - joy_node running: ros2 run joy joy_node

Xbox controller mapping:
  Left Stick X/Y    : Y/X translation
  Right Stick X/Y   : Yaw/Pitch rotation
  LT / RT (triggers): Z down / up
  LB / RB (bumpers) : Roll -/+
  A button          : Decrease speed
  B button          : Increase speed
  X button          : Toggle frame (base_link / tool0)
  Y button          : Print EE pose (FK)
  Start button      : Quit

Usage:
  # Terminal 1: Start joy node
  ros2 run joy joy_node

  # Terminal 2: Start this teleop
  python3 joystick_cartesian.py
"""

import signal

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Joy

from controller_utils import (
    ControllerSwitcher,
    JOINT_NAMES,
    FORWARD_POSITION_CONTROLLER,
)
from pinocchio_utils import PinocchioIK

# Frames
BASE_FRAME = 'base_link'
EE_FRAME = 'tool0'

# Speed scales
SPEED_SCALES = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0]
DEFAULT_SPEED_IDX = 2  # 0.3

# DLS damping factor
DAMPING = 0.05

# Publish rate (Hz)
PUBLISH_RATE = 50

# Xbox controller axis/button indices
AXIS_LEFT_STICK_X = 0   # Y translation
AXIS_LEFT_STICK_Y = 1   # X translation
AXIS_RIGHT_STICK_X = 3  # Yaw rotation
AXIS_RIGHT_STICK_Y = 4  # Pitch rotation
AXIS_LT = 2             # Left trigger (1.0 released, -1.0 pressed)
AXIS_RT = 5             # Right trigger (1.0 released, -1.0 pressed)

BTN_A = 0        # Speed down
BTN_B = 1        # Speed up
BTN_X = 2        # Toggle frame
BTN_Y = 3        # Print EE pose
BTN_LB = 4       # Roll -
BTN_RB = 5       # Roll +
BTN_START = 7    # Quit

# Deadzone for analog sticks
DEADZONE = 0.1


class JoystickCartesianNode(Node):
    def __init__(self):
        super().__init__('joystick_cartesian_teleop')

        # State
        self.running = True
        self.speed_idx = DEFAULT_SPEED_IDX
        self.use_local_frame = False
        self.current_positions = None
        self.current_twist = np.zeros(6)

        # Track button states for edge detection
        self._prev_buttons = []

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

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10,
        )
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self._joy_callback, 10,
        )

        # Controller switcher
        self.switcher = ControllerSwitcher(self)

        # Timer for publishing
        self.create_timer(1.0 / PUBLISH_RATE, self._timer_callback)

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

    def _apply_deadzone(self, value: float) -> float:
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
        import math

        # Button edge detection
        if self._button_pressed(msg, BTN_A):
            self.speed_idx = max(self.speed_idx - 1, 0)
            self.get_logger().info(f'Speed: {self.speed_scale:.1f}')

        if self._button_pressed(msg, BTN_B):
            self.speed_idx = min(self.speed_idx + 1, len(SPEED_SCALES) - 1)
            self.get_logger().info(f'Speed: {self.speed_scale:.1f}')

        if self._button_pressed(msg, BTN_X):
            self.use_local_frame = not self.use_local_frame
            self.get_logger().info(f'Frame: {self.frame_name}')

        if self._button_pressed(msg, BTN_Y):
            if self.current_positions is not None:
                pos, _ = self.ik.get_ee_pose(self.current_positions)
                rpy = self.ik.get_ee_rpy(self.current_positions)
                self.get_logger().info(
                    f'EE: x={pos[0]:.4f} y={pos[1]:.4f} z={pos[2]:.4f} | '
                    f'R={math.degrees(rpy[0]):.1f} P={math.degrees(rpy[1]):.1f} '
                    f'Y={math.degrees(rpy[2]):.1f}'
                )

        if self._button_pressed(msg, BTN_START):
            self.running = False

        # Save for next edge detection
        self._prev_buttons = list(msg.buttons)

        # Compute twist from analog inputs
        axes = msg.axes
        buttons = msg.buttons
        if len(axes) < 6:
            return

        lx = self._apply_deadzone(axes[AXIS_LEFT_STICK_X])
        ly = self._apply_deadzone(axes[AXIS_LEFT_STICK_Y])
        rx = self._apply_deadzone(axes[AXIS_RIGHT_STICK_X])
        ry = self._apply_deadzone(axes[AXIS_RIGHT_STICK_Y])

        # Triggers: convert from [1.0, -1.0] to [0.0, 1.0]
        lt = (1.0 - axes[AXIS_LT]) / 2.0
        rt = (1.0 - axes[AXIS_RT]) / 2.0

        # Bumpers for roll
        roll = 0.0
        if BTN_RB < len(buttons) and buttons[BTN_RB]:
            roll = 1.0
        if BTN_LB < len(buttons) and buttons[BTN_LB]:
            roll = -1.0

        self.current_twist = np.array([
            ly,         # X (forward/backward)
            lx,         # Y (left/right)
            rt - lt,    # Z (up/down)
            roll,       # Roll (RX)
            ry,         # Pitch (RY)
            rx,         # Yaw (RZ)
        ])

    def _timer_callback(self):
        """Periodically compute IK and publish joint positions."""
        if self.current_positions is None:
            return

        has_input = np.any(np.abs(self.current_twist) > 0.01)

        if has_input:
            twist = self.current_twist * self.speed_scale
            dt = 1.0 / PUBLISH_RATE
            dq = self.ik.compute_joint_delta(
                self.current_positions, twist, dt,
                damping=DAMPING, local=self.use_local_frame,
            )
            target = self.ik.clamp_positions(self.current_positions + dq)
            msg = Float64MultiArray()
            msg.data = target.tolist()
            self.cmd_pub.publish(msg)
        else:
            # Hold current position
            msg = Float64MultiArray()
            msg.data = self.current_positions.tolist()
            self.cmd_pub.publish(msg)

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

        # Wait for first joint_states
        self.get_logger().info('Waiting for /joint_states...')
        while self.current_positions is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_positions is None:
            self.get_logger().error('No joint_states received. Exiting.')
            return

        self.get_logger().info(
            f'Joystick Cartesian teleop ready. Speed: {self.speed_scale:.1f}  '
            f'Frame: {self.frame_name}'
        )
        self.get_logger().info('Waiting for /joy messages (run: ros2 run joy joy_node)...')

        print('\n  Xbox Controller Mapping (Forward Cartesian):')
        print('  Left Stick    : X/Y translation')
        print('  Right Stick   : Yaw/Pitch rotation')
        print('  LT/RT         : Z down/up')
        print('  LB/RB         : Roll -/+')
        print('  A/B           : Speed -/+')
        print('  X             : Toggle frame')
        print('  Y             : Print EE pose')
        print('  Start         : Quit\n')

        try:
            while self.running and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.get_logger().info('Restoring original controller...')
            self.switcher.restore_original()
            self.switcher.print_status()
            self.get_logger().info('Done.')


def main():
    rclpy.init()
    node = JoystickCartesianNode()

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
