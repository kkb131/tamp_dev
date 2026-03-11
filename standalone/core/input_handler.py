"""Input handlers for teleop: keyboard, Xbox controller, and network (UDP)."""

import json
import socket
import sys
import select
import termios
import tty
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np


SPEED_SCALES = [0.5, 1.0, 2.0, 4.0, 8.0]
DEFAULT_SPEED_IDX = 1  # 1.0x


@dataclass
class TeleopCommand:
    """Output from input handler."""
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(6))
    estop: bool = False
    reset: bool = False
    quit: bool = False
    speed_scale: float = 1.0  # current speed multiplier
    # Admittance control
    admittance_toggle: bool = False
    admittance_preset: str = ""  # "STIFF", "MEDIUM", "SOFT", "FREE", or "" (no change)
    ft_zero: bool = False
    # Impedance control
    impedance_preset: str = ""  # "STIFF", "MEDIUM", "SOFT", or "" (no change)
    gain_scale_up: bool = False
    gain_scale_down: bool = False


class InputHandler(ABC):
    """Abstract base for teleop input devices."""

    @abstractmethod
    def setup(self):
        """Initialize the input device."""

    @abstractmethod
    def cleanup(self):
        """Restore terminal / release device."""

    @abstractmethod
    def get_command(self, timeout: float = 0.02) -> TeleopCommand:
        """Read input and return a teleop command."""

    @property
    def speed_scale(self) -> float:
        return 1.0

    def __enter__(self):
        self.setup()
        return self

    def __exit__(self, *args):
        self.cleanup()


# Key -> (vx, vy, vz, wx, wy, wz) unit direction
# UR10e base frame: X=right, Y=forward (away from base), Z=up
KEY_MAP = {
    "w": (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),   # forward  (+Y)
    "s": (0.0, -1.0, 0.0, 0.0, 0.0, 0.0),   # backward (-Y)
    "a": (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0),   # left     (-X)
    "d": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),    # right    (+X)
    "q": (0.0, 0.0, 1.0, 0.0, 0.0, 0.0),    # up       (+Z)
    "e": (0.0, 0.0, -1.0, 0.0, 0.0, 0.0),   # down     (-Z)
    "u": (0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
    "o": (0.0, 0.0, 0.0, -1.0, 0.0, 0.0),
    "i": (0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    "k": (0.0, 0.0, 0.0, 0.0, -1.0, 0.0),
    "j": (0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    "l": (0.0, 0.0, 0.0, 0.0, 0.0, -1.0),
}


class KeyboardInput(InputHandler):
    """Non-blocking keyboard input via termios."""

    def __init__(self, cartesian_step: float = 0.005, rotation_step: float = 0.05):
        self._base_cart_step = cartesian_step
        self._base_rot_step = rotation_step
        self._old_settings = None
        self._speed_idx = DEFAULT_SPEED_IDX

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self._speed_idx]

    def setup(self):
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def cleanup(self):
        if self._old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)

    def _read_key(self, timeout: float) -> str | None:
        if select.select([sys.stdin], [], [], timeout)[0]:
            ch = sys.stdin.read(1)
            if ch == "\x1b":
                # Consume escape sequence
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    ch2 = sys.stdin.read(1)
                    if ch2 == "[" and select.select([sys.stdin], [], [], 0.01)[0]:
                        sys.stdin.read(1)
                return "ESC"
            return ch
        return None

    def get_command(self, timeout: float = 0.02) -> TeleopCommand:
        cmd = TeleopCommand(speed_scale=self.speed_scale)
        key = self._read_key(timeout)

        if key is None:
            return cmd

        if key in ("x", "ESC"):
            cmd.quit = True
            return cmd

        if key == " ":
            cmd.estop = True
            return cmd

        if key == "r":
            cmd.reset = True
            return cmd

        # Speed adjustment
        if key in ("+", "="):
            self._speed_idx = min(self._speed_idx + 1, len(SPEED_SCALES) - 1)
            cmd.speed_scale = self.speed_scale
            return cmd
        if key == "-":
            self._speed_idx = max(self._speed_idx - 1, 0)
            cmd.speed_scale = self.speed_scale
            return cmd

        # Admittance controls
        if key == "t":
            cmd.admittance_toggle = True
            return cmd
        if key == "z":
            cmd.ft_zero = True
            return cmd
        if key in ("1", "2", "3", "4"):
            cmd.admittance_preset = {"1": "STIFF", "2": "MEDIUM", "3": "SOFT", "4": "FREE"}[key]
            return cmd

        # Impedance gain scaling
        if key == "[":
            cmd.gain_scale_down = True
            return cmd
        if key == "]":
            cmd.gain_scale_up = True
            return cmd

        if key in KEY_MAP:
            direction = np.array(KEY_MAP[key])
            scale = np.array(
                [self._base_cart_step * self.speed_scale] * 3
                + [self._base_rot_step * self.speed_scale] * 3
            )
            cmd.velocity = direction * scale

        return cmd


class XboxInput(InputHandler):
    """Xbox controller input via pygame."""

    def __init__(self, linear_scale: float = 0.02, angular_scale: float = 0.05):
        self._linear_scale = linear_scale
        self._angular_scale = angular_scale
        self._joystick = None
        self._pygame = None
        self._speed_idx = DEFAULT_SPEED_IDX

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self._speed_idx]

    def setup(self):
        import pygame
        self._pygame = pygame
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No Xbox controller found. Connect a controller and retry.")

        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()

    def cleanup(self):
        if self._pygame:
            self._pygame.quit()

    def get_command(self, timeout: float = 0.02) -> TeleopCommand:
        cmd = TeleopCommand(speed_scale=self.speed_scale)
        pg = self._pygame
        if pg is None:
            return cmd

        for event in pg.event.get():
            if event.type == pg.JOYBUTTONDOWN:
                if event.button == 1:  # B = E-Stop
                    cmd.estop = True
                    return cmd
                if event.button == 0:  # A = Reset
                    cmd.reset = True
                    return cmd
                if event.button == 7:  # Start = Quit
                    cmd.quit = True
                    return cmd

        js = self._joystick
        if js is None:
            return cmd

        def dz(val, threshold=0.1):
            return val if abs(val) > threshold else 0.0

        lx = dz(js.get_axis(0))
        ly = dz(-js.get_axis(1))
        lt = (js.get_axis(2) + 1.0) / 2.0 if js.get_numaxes() > 2 else 0.0
        rt = (js.get_axis(5) + 1.0) / 2.0 if js.get_numaxes() > 5 else 0.0
        vz = dz(rt - lt, 0.05)
        rx = dz(js.get_axis(3)) if js.get_numaxes() > 3 else 0.0
        ry = dz(-js.get_axis(4)) if js.get_numaxes() > 4 else 0.0
        lb = 1.0 if js.get_button(4) else 0.0
        rb = 1.0 if js.get_button(5) else 0.0
        wyaw = rb - lb

        s = self.speed_scale
        cmd.velocity = np.array([
            lx * self._linear_scale * s,
            ly * self._linear_scale * s,
            vz * self._linear_scale * s,
            -rx * self._angular_scale * s,
            ry * self._angular_scale * s,
            wyaw * self._angular_scale * s,
        ])

        return cmd


class NetworkInput(InputHandler):
    """Receives joystick commands via UDP from a remote joystick_sender.py.

    Expects JSON packets with raw pygame axes/buttons:
        {"axes": [a0, a1, ...], "buttons": [b0, b1, ...]}
    Axis/button mapping matches XboxInput (XInput mode).
    """

    def __init__(self, port: int = 9870, linear_scale: float = 0.02,
                 angular_scale: float = 0.05):
        self._port = port
        self._linear_scale = linear_scale
        self._angular_scale = angular_scale
        self._sock: socket.socket | None = None
        self._speed_idx = DEFAULT_SPEED_IDX

    @property
    def speed_scale(self) -> float:
        return SPEED_SCALES[self._speed_idx]

    def setup(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", self._port))
        self._sock.setblocking(False)
        print(f"[NetworkInput] Listening on UDP port {self._port}")

    def cleanup(self):
        if self._sock:
            self._sock.close()
            self._sock = None

    def get_command(self, timeout: float = 0.02) -> TeleopCommand:
        cmd = TeleopCommand(speed_scale=self.speed_scale)
        if self._sock is None:
            return cmd

        # Drain buffer, keep only latest packet
        data = None
        try:
            while True:
                data, _ = self._sock.recvfrom(4096)
        except BlockingIOError:
            pass

        if data is None:
            # No packet this cycle — return zero velocity
            return cmd

        try:
            pkt = json.loads(data)
        except (json.JSONDecodeError, UnicodeDecodeError):
            return cmd

        axes = pkt.get("axes", [])
        buttons = pkt.get("buttons", [])

        def axis(idx, default=0.0):
            return axes[idx] if idx < len(axes) else default

        def btn(idx):
            return buttons[idx] if idx < len(buttons) else 0

        # Button events (same mapping as XboxInput)
        if btn(1):  # B = E-Stop
            cmd.estop = True
            return cmd
        if btn(0):  # A = Reset
            cmd.reset = True
            return cmd
        if btn(7):  # Start = Quit
            cmd.quit = True
            return cmd

        # Speed adjustment via D-pad (hat)
        hat = pkt.get("hat", [0, 0])
        hx = hat[0] if len(hat) > 0 else 0
        hy = hat[1] if len(hat) > 1 else 0
        if hy > 0:  # D-pad up = speed up
            self._speed_idx = min(self._speed_idx + 1, len(SPEED_SCALES) - 1)
            cmd.speed_scale = self.speed_scale
        elif hy < 0:  # D-pad down = speed down
            self._speed_idx = max(self._speed_idx - 1, 0)
            cmd.speed_scale = self.speed_scale

        # Admittance controls via extra buttons
        if btn(2):  # X = toggle admittance
            cmd.admittance_toggle = True
            return cmd
        if btn(3):  # Y = zero F/T
            cmd.ft_zero = True
            return cmd

        # Analog sticks → velocity (same mapping as XboxInput)
        def dz(val, threshold=0.1):
            return val if abs(val) > threshold else 0.0

        lx = dz(axis(0))
        ly = dz(-axis(1))
        lt = (axis(2) + 1.0) / 2.0
        rt = (axis(5) + 1.0) / 2.0
        vz = dz(rt - lt, 0.05)
        rx = dz(axis(3))
        ry = dz(-axis(4))
        lb = 1.0 if btn(4) else 0.0
        rb = 1.0 if btn(5) else 0.0
        wyaw = rb - lb

        s = self.speed_scale
        cmd.velocity = np.array([
            lx * self._linear_scale * s,
            ly * self._linear_scale * s,
            vz * self._linear_scale * s,
            -rx * self._angular_scale * s,
            ry * self._angular_scale * s,
            wyaw * self._angular_scale * s,
        ])

        return cmd


def create_input(input_type: str, cartesian_step: float = 0.005,
                 rotation_step: float = 0.05,
                 linear_scale: float = 0.02,
                 angular_scale: float = 0.05,
                 network_port: int = 9870) -> InputHandler:
    """Factory to create input handler by type."""
    if input_type == "keyboard":
        return KeyboardInput(cartesian_step, rotation_step)
    elif input_type == "xbox":
        return XboxInput(linear_scale, angular_scale)
    elif input_type == "network":
        return NetworkInput(port=network_port, linear_scale=linear_scale,
                            angular_scale=angular_scale)
    raise ValueError(f"Unknown input type: {input_type}")
