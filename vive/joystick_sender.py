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

    Matches XboxInput (standalone/core/input_handler.py) mappings exactly.

    Axis mapping (Xbox / Logitech layout):
        L-stick X  → X translation (좌우)
        L-stick Y  → Z translation (상하)
        LT / RT    → Y translation (앞뒤)
        R-stick X  → -Yaw
        R-stick Y  → -Roll
        LB / RB    → Pitch

    Button mapping:
        B (1)      → Admittance cycle
        Y (3)      → F/T zero
        Back (6)   → Quit
        Start (7)  → Reset
        Logitech (8) → E-Stop
        D-pad Y    → Speed ±
    """

    def __init__(self, target_ip: str, port: int = 9871, hz: int = 50,
                 linear_scale: float = 0.02, angular_scale: float = 0.05,
                 deadzone: float = 0.1):
        super().__init__(target_ip, port, hz)
        self._linear_scale = linear_scale
        self._angular_scale = angular_scale
        self._deadzone = deadzone
        self._speed_idx = DEFAULT_SPEED_IDX
        self._joy = None
        self._prev_buttons = []
        self._prev_b = False       # B button edge detect
        self._prev_hat_y = 0       # D-pad Y edge detect

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
        print(f"  Hats: {self._joy.get_numhats()}")
        print("[JoystickSender] Controls:")
        print("  L-stick X=좌우  L-stick Y=상하  LT/RT=앞뒤")
        print("  R-stick X=Yaw  R-stick Y=Roll  LB/RB=Pitch")
        print("  B=Adm-Cycle  Y=FT-Zero  Back=Quit  Start=Reset  Logitech=E-Stop")
        print("  D-pad Up/Down=Speed ±")
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

        js = self._joy
        num_axes = js.get_numaxes()
        num_buttons = js.get_numbuttons()

        # Read axes with deadzone
        def dz(val, threshold=None):
            t = threshold if threshold is not None else self._deadzone
            return self._apply_deadzone(val) if threshold is None else (val if abs(val) > t else 0.0)

        lx = self._apply_deadzone(js.get_axis(0)) if num_axes > 0 else 0.0
        ly = self._apply_deadzone(-js.get_axis(1)) if num_axes > 1 else 0.0  # inverted
        rx = self._apply_deadzone(js.get_axis(3)) if num_axes > 3 else 0.0
        ry = self._apply_deadzone(-js.get_axis(4)) if num_axes > 4 else 0.0  # inverted

        # Triggers: axis 2 = LT, axis 5 = RT, normalized [-1,1] → [0,1]
        lt = (js.get_axis(2) + 1.0) / 2.0 if num_axes > 2 else 0.0
        rt = (js.get_axis(5) + 1.0) / 2.0 if num_axes > 5 else 0.0
        vy = rt - lt
        if abs(vy) < 0.05:
            vy = 0.0

        # Bumpers → Yaw
        lb = 1.0 if (num_buttons > 4 and js.get_button(4)) else 0.0
        rb = 1.0 if (num_buttons > 5 and js.get_button(5)) else 0.0
        wyaw = rb - lb

        # Translation: L-stick X→X, LT/RT→Y, L-stick Y→Z (matches XboxInput)
        s = self.speed_scale
        result.delta_pos = np.array([
            lx * self._linear_scale * s,     # L-Stick X → X (좌우)
            vy * self._linear_scale * s,     # LT/RT    → Y (앞뒤)
            ly * self._linear_scale * s,     # L-Stick Y → Z (상하)
        ])

        # Rotation: R-stick Y→-Roll, LB/RB→Pitch, R-stick X→-Yaw
        result.delta_rot_axis_angle = np.array([
            ry * self._angular_scale * s,   # R-Stick Y → -Roll  (X축)
            wyaw * self._angular_scale * s,  # RB - LB   → Pitch  (Y축)
            -rx * self._angular_scale * s,   # R-Stick X → -Yaw   (Z축)
        ])

        # Buttons (edge-triggered, matches XboxInput)
        buttons = ButtonState()

        if num_buttons > 8 and self._button_pressed(8):  # Logitech → E-Stop
            buttons.estop = True
        if self._button_pressed(7):  # Start → Reset
            buttons.reset = True
        if self._button_pressed(6):  # Back → Quit
            buttons.quit = True
        if self._button_pressed(3):  # Y → F/T zero
            buttons.ft_zero = True

        # B button edge-detection → admittance cycle
        b_now = bool(js.get_button(1)) if num_buttons > 1 else False
        if b_now and not self._prev_b:
            buttons.admittance_cycle = True
        self._prev_b = b_now

        # D-pad for speed control (edge-triggered)
        if js.get_numhats() > 0:
            _hx, hy = js.get_hat(0)
            if hy > 0 and self._prev_hat_y <= 0:  # rising edge
                self._speed_idx = min(self._speed_idx + 1, len(SPEED_SCALES) - 1)
                buttons.speed_up = True
                print(f"\n[JoystickSender] Speed: {self.speed_scale:.1f}x")
            elif hy < 0 and self._prev_hat_y >= 0:  # falling edge
                self._speed_idx = max(self._speed_idx - 1, 0)
                buttons.speed_down = True
                print(f"\n[JoystickSender] Speed: {self.speed_scale:.1f}x")
            self._prev_hat_y = hy

        result.buttons = buttons

        # Save button state for edge detection
        self._prev_buttons = [js.get_button(i) for i in range(num_buttons)]

        return result


def main():
    parser = argparse.ArgumentParser(description="Joystick teleop sender (unified protocol)")
    parser.add_argument("--target-ip", required=True,
                        help="Robot PC IP address")
    parser.add_argument("--port", type=int, default=9871,
                        help="UDP port (default: 9871)")
    parser.add_argument("--hz", type=int, default=50,
                        help="Send rate in Hz (default: 50)")
    parser.add_argument("--linear-scale", type=float, default=0.02,
                        help="Linear velocity scale (m/tick, default: 0.02)")
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
