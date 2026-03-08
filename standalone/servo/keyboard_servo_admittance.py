#!/usr/bin/env python3
"""Keyboard teleop with Admittance control via forward_position_controller.

Combines Pinocchio DLS Cartesian teleop (like keyboard_cartesian.py) with
a direct admittance controller that reacts to F/T sensor data.

Pipeline:
  Keyboard → twist → Pinocchio DLS → dq_teleop ──┐
                                                    ├→ target_q → fwd_pos_ctrl → HW
  F/T sensor → deadzone → Admittance → J_pinv ───┘

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

  --- Admittance ---
  z         : Zero F/T sensor
  t         : Toggle admittance ON/OFF
  1 / 2 / 3 : Stiff / Medium / Soft compliance preset

Usage:
  python3 keyboard_servo_admittance.py
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
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger

from standalone.servo.controller_utils import (
    ControllerSwitcher,
    JOINT_NAMES,
    FORWARD_POSITION_CONTROLLER,
)
from standalone.servo.pinocchio_utils import PinocchioIK

# ──────────────────────────── Frames ────────────────────────────
BASE_FRAME = 'base_link'
EE_FRAME = 'tool0'

# ──────────────────────────── Speed ─────────────────────────────
SPEED_SCALES = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0]
DEFAULT_SPEED_IDX = 2  # 0.3

# ──────────────────────────── Control ───────────────────────────
DAMPING = 0.05
PUBLISH_RATE = 50  # Hz

# ──────────────────────────── F/T Topics ────────────────────────
FT_TOPIC = '/ft_data'
ZERO_FT_SERVICE = '/io_and_status_controller/zero_ftsensor'

# ──────────────────────── Admittance Presets ────────────────────
# Each preset: (M, D, K) as 6D arrays [fx,fy,fz,tx,ty,tz]
# Stability: D >= 2*sqrt(M*K), dt < 2*M/D (at 50Hz, dt=0.02)
PRESETS = {
    'STIFF': {
        'M': np.array([10.0, 10.0, 10.0, 1.0, 1.0, 1.0]),
        'D': np.array([200.0, 200.0, 200.0, 20.0, 20.0, 20.0]),
        'K': np.array([500.0, 500.0, 500.0, 50.0, 50.0, 50.0]),
    },
    'MEDIUM': {
        'M': np.array([5.0, 5.0, 5.0, 0.5, 0.5, 0.5]),
        'D': np.array([100.0, 100.0, 100.0, 10.0, 10.0, 10.0]),
        'K': np.array([200.0, 200.0, 200.0, 20.0, 20.0, 20.0]),
    },
    'SOFT': {
        'M': np.array([2.0, 2.0, 2.0, 0.2, 0.2, 0.2]),
        'D': np.array([40.0, 40.0, 40.0, 4.0, 4.0, 4.0]),
        'K': np.array([50.0, 50.0, 50.0, 5.0, 5.0, 5.0]),
    },
}
DEFAULT_PRESET = 'MEDIUM'

# ──────────────────────── Safety Limits ─────────────────────────
FORCE_DEADZONE = np.array([3.0, 3.0, 3.0, 0.3, 0.3, 0.3])  # N, Nm
MAX_CART_DISP = 0.05     # 5 cm max translation offset
MAX_CART_ROT = 0.15      # ~8.6 deg max rotation offset
FORCE_SATURATION = 100.0  # N — disable admittance above this
TORQUE_SATURATION = 10.0  # Nm

# ──────────────────────── Key Mappings ──────────────────────────
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

HELP_TEXT = """
╔══════════════════════════════════════════════════════════╗
║   Admittance Cartesian Controller - Keyboard Teleop      ║
║   (Pinocchio DLS + F/T Admittance, no MoveIt Servo)      ║
╠══════════════════════════════════════════════════════════╣
║  --- Translation ---                                     ║
║  W / S     : X forward / backward                        ║
║  A / D     : Y left / right                              ║
║  Q / E     : Z up / down                                 ║
║                                                          ║
║  --- Rotation ---                                        ║
║  U / O     : Roll  (RX) +/-                              ║
║  I / K     : Pitch (RY) +/-                              ║
║  J / L     : Yaw   (RZ) +/-                              ║
║                                                          ║
║  --- Control ---                                         ║
║  + / =     : Increase speed                              ║
║  -         : Decrease speed                              ║
║  f         : Toggle frame (base_link / tool0)            ║
║  p         : Print EE pose (FK)                          ║
║  Space     : Stop                                        ║
║  Esc / x   : Quit                                        ║
║                                                          ║
║  --- Admittance ---                                      ║
║  z         : Zero F/T sensor                             ║
║  t         : Toggle admittance ON/OFF                    ║
║  1 / 2 / 3 : Stiff / Medium / Soft compliance            ║
╚══════════════════════════════════════════════════════════╝
"""


class KeyboardAdmittanceNode(Node):
    def __init__(self):
        super().__init__('keyboard_admittance_teleop')

        # ── State ──
        self.running = True
        self.speed_idx = DEFAULT_SPEED_IDX
        self.use_local_frame = False  # False = base_link, True = tool0
        self.current_twist = np.zeros(6)
        self.current_positions = None  # From /joint_states

        # ── Pinocchio IK ──
        self.ik = PinocchioIK()
        self.get_logger().info(
            f'Pinocchio loaded: {self.ik.nq} joints, '
            f'EE frame_id={self.ik.ee_frame_id}'
        )

        # ── Publisher (forward_position_controller) ──
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

        # ── Joint state subscriber ──
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10,
        )

        # ── F/T sensor subscriber ──
        self.ft_wrench = np.zeros(6)
        self.ft_bias = np.zeros(6)
        self.ft_sub = self.create_subscription(
            WrenchStamped, FT_TOPIC, self._ft_callback, 10,
        )

        # ── Zero F/T service client ──
        self.zero_ft_client = self.create_client(Trigger, ZERO_FT_SERVICE)

        # ── Admittance state ──
        self.admittance_enabled = True
        self.preset_name = DEFAULT_PRESET
        self._load_preset(DEFAULT_PRESET)
        self.admittance_x = np.zeros(6)     # Cartesian displacement offset
        self.admittance_xdot = np.zeros(6)  # Cartesian velocity offset

        # ── Controller switcher ──
        self.switcher = ControllerSwitcher(self)

        # ── Terminal ──
        self._old_settings = None

    # ─────────────────── Properties ─────────────────────

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self.speed_idx]

    @property
    def frame_name(self) -> str:
        return EE_FRAME if self.use_local_frame else BASE_FRAME

    # ─────────────────── Callbacks ──────────────────────

    def _joint_state_cb(self, msg: JointState):
        if len(msg.position) < 6:
            return
        positions = [0.0] * 6
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                positions[i] = msg.position[idx]
        self.current_positions = np.array(positions)

    def _ft_callback(self, msg: WrenchStamped):
        raw = np.array([
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
        ])
        self.ft_wrench = raw - self.ft_bias

    # ─────────────────── Admittance ─────────────────────

    def _load_preset(self, name: str):
        p = PRESETS[name]
        self.M = p['M'].copy()
        self.D = p['D'].copy()
        self.K = p['K'].copy()
        self.preset_name = name

    def _transform_wrench_to_base(self, wrench_tool: np.ndarray,
                                   R: np.ndarray) -> np.ndarray:
        """Rotate wrench from tool0 frame to base_link frame."""
        f_base = R @ wrench_tool[:3]
        t_base = R @ wrench_tool[3:]
        return np.concatenate([f_base, t_base])

    def _apply_deadzone(self, wrench: np.ndarray) -> np.ndarray:
        result = wrench.copy()
        mask = np.abs(result) < FORCE_DEADZONE
        # Zero out values within dead-zone
        result[mask] = 0.0
        # Smooth transition: subtract dead-zone from remaining values
        result[~mask] -= np.sign(result[~mask]) * FORCE_DEADZONE[~mask]
        return result

    def _check_saturation(self, wrench: np.ndarray) -> bool:
        """Return True if F/T exceeds saturation threshold (unsafe)."""
        force_mag = np.linalg.norm(wrench[:3])
        torque_mag = np.linalg.norm(wrench[3:])
        return force_mag > FORCE_SATURATION or torque_mag > TORQUE_SATURATION

    def _update_admittance(self, dt: float):
        """Update admittance dynamics: Mẍ + Dẋ + Kx = F_ext."""
        if not self.admittance_enabled or self.current_positions is None:
            self.admittance_x[:] = 0.0
            self.admittance_xdot[:] = 0.0
            return

        # Transform F/T from tool frame to base frame
        _, R = self.ik.get_ee_pose(self.current_positions)
        f_ext = self._transform_wrench_to_base(self.ft_wrench, R)

        # Saturation check
        if self._check_saturation(f_ext):
            self.get_logger().warn(
                f'F/T saturation! F={np.linalg.norm(f_ext[:3]):.1f}N '
                f'T={np.linalg.norm(f_ext[3:]):.1f}Nm — holding position'
            )
            self.admittance_x[:] = 0.0
            self.admittance_xdot[:] = 0.0
            return

        # Apply dead-zone
        f_ext = self._apply_deadzone(f_ext)

        # Admittance dynamics: ẍ = M⁻¹(F_ext - D·ẋ - K·x)
        xddot = (f_ext - self.D * self.admittance_xdot
                 - self.K * self.admittance_x) / self.M

        # Semi-implicit Euler integration
        self.admittance_xdot += xddot * dt
        self.admittance_x += self.admittance_xdot * dt

        # Clamp translation displacement
        disp_norm = np.linalg.norm(self.admittance_x[:3])
        if disp_norm > MAX_CART_DISP:
            self.admittance_x[:3] *= MAX_CART_DISP / disp_norm
            self.admittance_xdot[:3] *= 0.5

        # Clamp rotation displacement
        rot_norm = np.linalg.norm(self.admittance_x[3:])
        if rot_norm > MAX_CART_ROT:
            self.admittance_x[3:] *= MAX_CART_ROT / rot_norm
            self.admittance_xdot[3:] *= 0.5

    def _compute_admittance_dq(self, q: np.ndarray, dt: float) -> np.ndarray:
        """Convert admittance velocity to joint delta via Jacobian pseudo-inverse."""
        if not self.admittance_enabled:
            return np.zeros(6)
        J = self.ik.get_jacobian(q, local=False)
        JJt = J @ J.T + (DAMPING ** 2) * np.eye(6)
        return J.T @ np.linalg.solve(JJt, self.admittance_xdot * dt)

    # ─────────────────── Terminal ───────────────────────

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
                            sys.stdin.read(1)
                return 'ESC'
            return ch
        return None

    # ─────────────────── Publishing ─────────────────────

    def publish_position(self, positions: np.ndarray):
        msg = Float64MultiArray()
        msg.data = positions.tolist()
        self.cmd_pub.publish(msg)

    # ─────────────────── Display ────────────────────────

    def print_ee_pose(self):
        if self.current_positions is None:
            print('\r  [No joint_states received yet]')
            return
        pos, _ = self.ik.get_ee_pose(self.current_positions)
        rpy = self.ik.get_ee_rpy(self.current_positions)
        print('\r' + ' ' * 80, end='')
        print(f'\r  EE Position: x={pos[0]:.4f}  y={pos[1]:.4f}  z={pos[2]:.4f} [m]')
        print(f'  EE Rotation: R={math.degrees(rpy[0]):.1f}  '
              f'P={math.degrees(rpy[1]):.1f}  Y={math.degrees(rpy[2]):.1f}')
        print(f'  Joints: [{", ".join(f"{math.degrees(j):.1f}" for j in self.current_positions)}]')

    def print_admittance_status(self):
        state = 'ON' if self.admittance_enabled else 'OFF'
        f = self.ft_wrench
        dx = self.admittance_x
        print(
            f'\r  Admittance: {state} [{self.preset_name}] | '
            f'F: [{f[0]:.1f}, {f[1]:.1f}, {f[2]:.1f}]N | '
            f'dx: [{dx[0]*1000:.1f}, {dx[1]*1000:.1f}, {dx[2]*1000:.1f}]mm'
            '          ', end='', flush=True
        )

    # ─────────────────── Key Processing ─────────────────

    def process_key(self, key: str):
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
            self.admittance_x[:] = 0.0
            self.admittance_xdot[:] = 0.0
            print('\r  >>> STOP                    ')
            return

        # ── Admittance controls ──

        # Zero F/T sensor
        if key == 'z':
            if self.zero_ft_client.service_is_ready():
                future = self.zero_ft_client.call_async(Trigger.Request())
                future.add_done_callback(
                    lambda f: self.get_logger().info(
                        f'Zero F/T: {f.result().message}'
                        if f.result() else 'Zero F/T: call failed'
                    )
                )
                # Also zero software bias with current reading
                self.ft_bias = self.ft_wrench + self.ft_bias
                self.admittance_x[:] = 0.0
                self.admittance_xdot[:] = 0.0
                print('\r  >>> F/T sensor zeroed          ')
            else:
                # Service not available (mock hardware) — software zero only
                self.ft_bias = self.ft_wrench + self.ft_bias
                self.admittance_x[:] = 0.0
                self.admittance_xdot[:] = 0.0
                print('\r  >>> F/T software zeroed (service N/A)          ')
            return

        # Toggle admittance
        if key == 't':
            self.admittance_enabled = not self.admittance_enabled
            self.admittance_x[:] = 0.0
            self.admittance_xdot[:] = 0.0
            state = 'ON' if self.admittance_enabled else 'OFF'
            print(f'\r  Admittance: {state} [{self.preset_name}]          ')
            return

        # Compliance presets
        if key == '1':
            self._load_preset('STIFF')
            self.admittance_x[:] = 0.0
            self.admittance_xdot[:] = 0.0
            print(f'\r  Preset: STIFF          ')
            return
        if key == '2':
            self._load_preset('MEDIUM')
            self.admittance_x[:] = 0.0
            self.admittance_xdot[:] = 0.0
            print(f'\r  Preset: MEDIUM          ')
            return
        if key == '3':
            self._load_preset('SOFT')
            self.admittance_x[:] = 0.0
            self.admittance_xdot[:] = 0.0
            print(f'\r  Preset: SOFT          ')
            return

    # ─────────────────── Main Loop ──────────────────────

    def run(self):
        # Wait for services
        if not self.switcher.wait_for_services():
            self.get_logger().error('Controller manager not available. Exiting.')
            return

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

        self.get_logger().info('Joint states received. Starting keyboard admittance teleop.')
        print(HELP_TEXT)
        print(f'  Speed: {self.speed_scale:.1f}  |  Frame: {self.frame_name}  |  '
              f'Admittance: ON [{self.preset_name}]')
        print('  Ready! Press keys to move the robot.\n')

        # Set up terminal
        self.setup_terminal()
        dt = 1.0 / PUBLISH_RATE
        status_counter = 0

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

                current_q = self.current_positions

                # 1) Keyboard teleop → DLS → joint delta
                if np.any(self.current_twist != 0.0):
                    twist = self.current_twist * self.speed_scale
                    dq_teleop = self.ik.compute_joint_delta(
                        current_q, twist, dt,
                        damping=DAMPING, local=self.use_local_frame,
                    )
                    # Reset twist (need to keep pressing key)
                    self.current_twist = np.zeros(6)
                else:
                    dq_teleop = np.zeros(6)

                # 2) Admittance dynamics update
                self._update_admittance(dt)

                # 3) Convert admittance velocity to joint delta
                dq_admittance = self._compute_admittance_dq(current_q, dt)

                # 4) Combine and publish
                target = self.ik.clamp_positions(
                    current_q + dq_teleop + dq_admittance
                )
                self.publish_position(target)

                # Periodic status display (~2Hz)
                status_counter += 1
                if status_counter >= PUBLISH_RATE // 2:
                    status_counter = 0
                    self.print_admittance_status()

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
    node = KeyboardAdmittanceNode()

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
