#!/usr/bin/env python3
"""Joystick (gamepad) teleop sender — run on the operator PC.

Reads joystick axes/buttons via pygame, accumulates velocity deltas into
a virtual target pose, and sends absolute pose via the unified teleop protocol.

Compatible with Xbox, Logitech F710/F310, and similar gamepads.

Requirements: numpy, pygame
    pip install numpy pygame

Usage:
    python3 -m vive.joystick_sender --target-ip <ROBOT_PC_IP>
    python3 -m vive.joystick_sender --target-ip 192.168.0.10 --hz 50
"""

import argparse
import time

import numpy as np

from vive.teleop_sender import InputResult, TeleopSenderBase

try:
    from standalone.core.teleop_protocol import ButtonState
except ImportError:
    from vive.teleop_protocol import ButtonState  # type: ignore[no-redef]

try:
    import pygame
except ImportError:
    pygame = None

# Speed scale presets
SPEED_SCALES = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0]
DEFAULT_SPEED_IDX = 2  # 0.3x


class JoystickSender(TeleopSenderBase):
    """Gamepad-based teleop sender using pygame.

    Axis mapping (Xbox / Logitech layout):
        L-stick X → X translation (left/right)
        L-stick Y → Y translation (forward/backward)
        LT / RT   → Z translation (down / up)
        R-stick X → Yaw
        R-stick Y → Pitch
        LB / RB   → Roll -/+

    Button mapping:
        A (0) → Speed down
        B (1) → Speed up
        X (2) → F/T zero
        Y (3) → Admittance cycle
        Start (7) → Quit
        Back (6)  → Reset
        L-stick press (8) → E-Stop
    """

    def __init__(self, target_ip: str, port: int = 9871, hz: int = 50,
                 linear_scale: float = 0.01, angular_scale: float = 0.05,
                 deadzone: float = 0.1):
        super().__init__(target_ip, port, hz)
        self._linear_scale = linear_scale
        self._angular_scale = angular_scale
        self._deadzone = deadzone
        self._speed_idx = DEFAULT_SPEED_IDX
        self._joy = None
        self._prev_buttons = []

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self._speed_idx]

    def _setup_device(self):
        if pygame is None:
            raise ImportError("pygame not installed. Run: pip install pygame")
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No joystick/gamepad detected")

        self._joy = pygame.joystick.Joystick(0)
        self._joy.init()
        self._prev_buttons = [0] * self._joy.get_numbuttons()

        print(f"[JoystickSender] Connected: {self._joy.get_name()}")
        print(f"  Axes: {self._joy.get_numaxes()}  Buttons: {self._joy.get_numbuttons()}")
        print("[JoystickSender] Controls:")
        print("  L-stick=XY  LT/RT=Z  R-stick=Pitch/Yaw  LB/RB=Roll")
        print("  A=Speed-  B=Speed+  X=FT-Zero  Y=Adm-Cycle")
        print("  Start=Quit  Back=Reset  L-stick-press=E-Stop")
        print(f"  Speed: {self.speed_scale:.1f}x")

    def _cleanup_device(self):
        if pygame is not None:
            pygame.quit()
        self._joy = None

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self._deadzone:
            return 0.0
        # Rescale so output starts from 0 at deadzone edge
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self._deadzone) / (1.0 - self._deadzone)

    def _button_pressed(self, btn_idx: int) -> bool:
        """Rising-edge detection for a button."""
        current = self._joy.get_button(btn_idx)
        prev = self._prev_buttons[btn_idx] if btn_idx < len(self._prev_buttons) else 0
        return current == 1 and prev == 0

    def _read_input(self) -> InputResult:
        result = InputResult()

        pygame.event.pump()

        num_axes = self._joy.get_numaxes()
        num_buttons = self._joy.get_numbuttons()

        # Read axes with deadzone
        def axis(i):
            return self._apply_deadzone(self._joy.get_axis(i)) if i < num_axes else 0.0

        lx = axis(0)   # L-stick X
        ly = -axis(1)  # L-stick Y (inverted)
        rx = axis(3)    # R-stick X (some controllers: axis 2 or 3)
        ry = -axis(4)   # R-stick Y (inverted)

        # Triggers: axis 2 = LT (-1 to 1), axis 5 = RT (-1 to 1)
        # Normalize from [-1, 1] to [0, 1]
        lt = (axis(2) + 1.0) / 2.0 if num_axes > 2 else 0.0
        rt = (axis(5) + 1.0) / 2.0 if num_axes > 5 else 0.0

        # Bumpers (buttons 4, 5)
        lb = self._joy.get_button(4) if num_buttons > 4 else 0
        rb = self._joy.get_button(5) if num_buttons > 5 else 0

        # Translation (L-stick + triggers)
        s = self.speed_scale
        result.delta_pos = np.array([
            lx * self._linear_scale * s,    # X
            ly * self._linear_scale * s,    # Y
            (rt - lt) * self._linear_scale * s,  # Z (RT up, LT down)
        ])

        # Rotation (R-stick + bumpers)
        roll = float(rb - lb)
        result.delta_rot_axis_angle = np.array([
            roll * self._angular_scale * s,    # Roll
            ry * self._angular_scale * s,      # Pitch
            rx * self._angular_scale * s,      # Yaw
        ])

        # Buttons (edge-triggered)
        buttons = ButtonState()

        if self._button_pressed(7):  # Start → Quit
            buttons.quit = True
        if self._button_pressed(6):  # Back → Reset
            buttons.reset = True
        if num_buttons > 8 and self._button_pressed(8):  # L-stick press → E-Stop
            buttons.estop = True
        if self._button_pressed(0):  # A → Speed down
            self._speed_idx = max(self._speed_idx - 1, 0)
            buttons.speed_down = True
            print(f"[JoystickSender] Speed: {self.speed_scale:.1f}x")
        if self._button_pressed(1):  # B → Speed up
            self._speed_idx = min(self._speed_idx + 1, len(SPEED_SCALES) - 1)
            buttons.speed_up = True
            print(f"[JoystickSender] Speed: {self.speed_scale:.1f}x")
        if self._button_pressed(2):  # X → F/T zero
            buttons.ft_zero = True
        if self._button_pressed(3):  # Y → Admittance cycle
            buttons.admittance_cycle = True

        result.buttons = buttons

        # Save button state for edge detection
        self._prev_buttons = [self._joy.get_button(i) for i in range(num_buttons)]

        return result


def main():
    parser = argparse.ArgumentParser(description="Joystick teleop sender (unified protocol)")
    parser.add_argument("--target-ip", required=True,
                        help="Robot PC IP address")
    parser.add_argument("--port", type=int, default=9871,
                        help="UDP port (default: 9871)")
    parser.add_argument("--hz", type=int, default=50,
                        help="Send rate in Hz (default: 50)")
    parser.add_argument("--linear-scale", type=float, default=0.01,
                        help="Linear velocity scale (m/tick, default: 0.01)")
    parser.add_argument("--angular-scale", type=float, default=0.05,
                        help="Angular velocity scale (rad/tick, default: 0.05)")
    parser.add_argument("--deadzone", type=float, default=0.1,
                        help="Axis deadzone (default: 0.1)")
    args = parser.parse_args()

    sender = JoystickSender(
        target_ip=args.target_ip,
        port=args.port,
        hz=args.hz,
        linear_scale=args.linear_scale,
        angular_scale=args.angular_scale,
        deadzone=args.deadzone,
    )
    sender.run()


if __name__ == "__main__":
    main()
