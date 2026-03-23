#!/usr/bin/env python3
"""Keyboard teleop sender — run on the operator PC.

Reads keyboard input via pynput, accumulates velocity deltas into a virtual
target pose, and sends absolute pose via the unified teleop protocol.

Requirements: numpy, pynput
    pip install numpy pynput

Usage:
    python3 -m vive.keyboard_sender --target-ip <ROBOT_PC_IP>
    python3 -m vive.keyboard_sender --target-ip 192.168.0.10 --hz 50
"""

import argparse
import math
import threading
import time

import numpy as np

from vive.teleop_sender import InputResult, TeleopSenderBase

try:
    from standalone.core.teleop_protocol import ButtonState
except ImportError:
    from vive.teleop_protocol import ButtonState  # type: ignore[no-redef]

try:
    from pynput import keyboard
except ImportError:
    keyboard = None


# Key → direction mapping (matches standalone/core/input_handler.py KEY_MAP)
# [dx, dy, dz, drx, dry, drz] — unit direction
KEY_MAP = {
    "w": [0, 1, 0, 0, 0, 0],    # +Y (forward)
    "s": [0, -1, 0, 0, 0, 0],   # -Y (backward)
    "a": [-1, 0, 0, 0, 0, 0],   # -X (left)
    "d": [1, 0, 0, 0, 0, 0],    # +X (right)
    "q": [0, 0, 1, 0, 0, 0],    # +Z (up)
    "e": [0, 0, -1, 0, 0, 0],   # -Z (down)
    "u": [0, 0, 0, 1, 0, 0],    # +Roll
    "o": [0, 0, 0, -1, 0, 0],   # -Roll
    "i": [0, 0, 0, 0, 1, 0],    # +Pitch
    "k": [0, 0, 0, 0, -1, 0],   # -Pitch
    "j": [0, 0, 0, 0, 0, 1],    # +Yaw
    "l": [0, 0, 0, 0, 0, -1],   # -Yaw
}

# Speed scale presets
SPEED_SCALES = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0]
DEFAULT_SPEED_IDX = 2  # 0.3x


class KeyboardSender(TeleopSenderBase):
    """Keyboard-based teleop sender using pynput.

    Accumulates key-press velocity deltas into a virtual target pose.
    The pose is sent via the unified teleop protocol.
    """

    def __init__(self, target_ip: str, port: int = 9871, hz: int = 50,
                 cartesian_step: float = 0.005,
                 rotation_step: float = 0.05):
        super().__init__(target_ip, port, hz)
        self._cart_step = cartesian_step    # metres per tick
        self._rot_step = rotation_step      # radians per tick
        self._speed_idx = DEFAULT_SPEED_IDX

        # Thread-safe key state
        self._lock = threading.Lock()
        self._pressed_keys: set = set()
        self._estop = False
        self._reset = False
        self._quit_flag = False
        self._speed_up = False
        self._speed_down = False

        self._listener = None

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self._speed_idx]

    def _setup_device(self):
        if keyboard is None:
            raise ImportError("pynput not installed. Run: pip install pynput")
        self._listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.daemon = True
        self._listener.start()
        print("[KeyboardSender] Controls:")
        print("  W/S=Y  A/D=X  Q/E=Z  U/O=Roll  I/K=Pitch  J/L=Yaw")
        print("  Space=E-Stop  R=Reset  X/Esc=Quit  +/-=Speed")
        print(f"  Speed: {self.speed_scale:.1f}x")

    def _cleanup_device(self):
        if self._listener:
            self._listener.stop()
            self._listener = None

    def _on_press(self, key):
        with self._lock:
            try:
                if key == keyboard.Key.space:
                    self._estop = True
                elif key == keyboard.Key.esc:
                    self._quit_flag = True
                elif hasattr(key, "char") and key.char is not None:
                    ch = key.char.lower()
                    if ch in KEY_MAP:
                        self._pressed_keys.add(ch)
                    elif ch == "r":
                        self._reset = True
                    elif ch == "x":
                        self._quit_flag = True
                    elif ch in ("+", "="):
                        self._speed_up = True
                    elif ch == "-":
                        self._speed_down = True
            except AttributeError:
                pass

    def _on_release(self, key):
        with self._lock:
            try:
                if hasattr(key, "char") and key.char is not None:
                    ch = key.char.lower()
                    self._pressed_keys.discard(ch)
            except AttributeError:
                pass

    def _read_input(self) -> InputResult:
        result = InputResult()

        with self._lock:
            buttons = ButtonState(
                estop=self._estop,
                reset=self._reset,
                quit=self._quit_flag,
                speed_up=self._speed_up,
                speed_down=self._speed_down,
            )
            self._estop = False
            self._reset = False
            self._quit_flag = False

            # Speed adjustment
            if self._speed_up:
                self._speed_idx = min(self._speed_idx + 1, len(SPEED_SCALES) - 1)
                print(f"[KeyboardSender] Speed: {self.speed_scale:.1f}x")
                self._speed_up = False
            if self._speed_down:
                self._speed_idx = max(self._speed_idx - 1, 0)
                print(f"[KeyboardSender] Speed: {self.speed_scale:.1f}x")
                self._speed_down = False

            pressed = set(self._pressed_keys)

        result.buttons = buttons

        # Accumulate direction from all pressed keys
        direction = np.zeros(6)
        for key in pressed:
            if key in KEY_MAP:
                direction += np.array(KEY_MAP[key])

        # Scale by step size and speed
        s = self.speed_scale
        result.delta_pos = direction[:3] * self._cart_step * s
        result.delta_rot_axis_angle = direction[3:] * self._rot_step * s

        return result


def main():
    parser = argparse.ArgumentParser(description="Keyboard teleop sender (unified protocol)")
    parser.add_argument("--target-ip", required=True,
                        help="Robot PC IP address")
    parser.add_argument("--port", type=int, default=9871,
                        help="UDP port (default: 9871)")
    parser.add_argument("--hz", type=int, default=50,
                        help="Send rate in Hz (default: 50)")
    parser.add_argument("--cart-step", type=float, default=0.005,
                        help="Cartesian step per tick in metres (default: 0.005)")
    parser.add_argument("--rot-step", type=float, default=0.05,
                        help="Rotation step per tick in radians (default: 0.05)")
    args = parser.parse_args()

    sender = KeyboardSender(
        target_ip=args.target_ip,
        port=args.port,
        hz=args.hz,
        cartesian_step=args.cart_step,
        rotation_step=args.rot_step,
    )
    sender.run()


if __name__ == "__main__":
    main()
