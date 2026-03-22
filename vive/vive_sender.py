#!/usr/bin/env python3
"""UDP Vive Tracker sender — run on the operator PC with SteamVR.

Reads Vive Tracker 3.0 pose via OpenVR and keyboard state via pynput,
then sends combined data over UDP to the robot PC.

Requirements: openvr, numpy, pynput, pyyaml
    pip install openvr numpy pynput pyyaml

Usage:
    python3 -m vive.vive_sender --target-ip <ROBOT_PC_IP>
    python3 -m vive.vive_sender --config vive/config/default.yaml
    python3 -m vive.vive_sender --config vive/config/default.yaml --target-ip 10.0.0.5
"""

import argparse
import json
import socket
import threading
import time

from vive.vive_config import ViveConfig
from vive.vive_tracker import ViveTracker

try:
    from pynput import keyboard
except ImportError:
    keyboard = None


class KeyboardState:
    """Thread-safe keyboard state tracker using pynput.

    Key mappings:
        Space   = E-Stop
        R       = Reset
        Q/Esc   = Quit
        +/=     = Speed up
        -       = Speed down
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._estop = False
        self._reset = False
        self._quit = False
        self._speed_up = False
        self._speed_down = False
        self._listener = None

    def start(self):
        if keyboard is None:
            raise ImportError(
                "pynput not installed. Run: pip install pynput"
            )
        self._listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.daemon = True
        self._listener.start()
        print("[Keyboard] Listener started")
        print("[Keyboard] Space=E-Stop, R=Reset, Q/Esc=Quit, +/-=Speed")

    def stop(self):
        if self._listener is not None:
            self._listener.stop()
            self._listener = None

    def get_and_clear(self) -> dict:
        """Get current button state and clear edge-triggered flags."""
        with self._lock:
            state = {
                "estop": self._estop,
                "reset": self._reset,
                "quit": self._quit,
                "speed_up": self._speed_up,
                "speed_down": self._speed_down,
            }
            self._estop = False
            self._reset = False
            self._quit = False
            self._speed_up = False
            self._speed_down = False
        return state

    def _on_press(self, key):
        with self._lock:
            try:
                if key == keyboard.Key.space:
                    self._estop = True
                elif key == keyboard.Key.esc:
                    self._quit = True
                elif hasattr(key, "char") and key.char is not None:
                    ch = key.char.lower()
                    if ch == "r":
                        self._reset = True
                    elif ch == "q":
                        self._quit = True
                    elif ch in ("+", "="):
                        self._speed_up = True
                    elif ch == "-":
                        self._speed_down = True
            except AttributeError:
                pass

    def _on_release(self, key):
        pass


def main():
    parser = argparse.ArgumentParser(description="UDP Vive Tracker sender")
    parser.add_argument("--config", default=None,
                        help="YAML config file (default: vive/config/default.yaml)")
    parser.add_argument("--target-ip", default=None,
                        help="Robot PC IP (overrides config)")
    parser.add_argument("--port", type=int, default=None,
                        help="UDP port (overrides config)")
    parser.add_argument("--hz", type=int, default=None,
                        help="Send rate in Hz (overrides config)")
    parser.add_argument("--tracker-serial", default=None,
                        help="Tracker serial (overrides config)")
    parser.add_argument("--list-trackers", action="store_true",
                        help="List all trackers and exit")
    args = parser.parse_args()

    # Load config
    cfg = ViveConfig.load(args.config)

    # CLI overrides
    target_ip = args.target_ip or cfg.network.target_ip
    port = args.port or cfg.network.port
    hz = args.hz or cfg.network.hz
    teleop_tracker = cfg.get_teleop_tracker()
    tracker_serial = args.tracker_serial or teleop_tracker.serial

    # Connect tracker
    tracker = ViveTracker(tracker_serial=tracker_serial)
    tracker.connect()

    if args.list_trackers:
        trackers = tracker.get_all_trackers()
        if not trackers:
            print("[Sender] No trackers found")
        for t in trackers:
            status = "TRACKING" if t["tracking"] else "NOT TRACKING"
            print(f"  index={t['index']}  serial={t['serial']}  [{status}]")
        tracker.disconnect()
        return

    if target_ip is None:
        print("[ERROR] --target-ip required (or set network.target_ip in config)")
        tracker.disconnect()
        return

    kb = KeyboardState()
    kb.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (target_ip, port)
    dt = 1.0 / hz
    send_count = 0
    lost_count = 0

    print(f"[Sender] Config: tracker={teleop_tracker.name} serial={tracker_serial}")
    print(f"[Sender] Sending to {target_ip}:{port} at {hz} Hz")
    print("[Sender] Press Ctrl+C to stop.")

    try:
        while True:
            t_start = time.perf_counter()

            result = tracker.get_pose()
            buttons = kb.get_and_clear()

            if buttons["quit"]:
                print("\n[Sender] Quit requested")
                break

            if result is not None:
                pos, quat = result
                pkt = {
                    "type": "vive",
                    "pos": pos.tolist(),
                    "quat": quat.tolist(),  # wxyz
                    "tracking": True,
                    "buttons": buttons,
                    "timestamp": time.time(),
                }
                if lost_count > 0:
                    print(f"\n[Sender] Tracking recovered (was lost for {lost_count} frames)")
                    lost_count = 0
            else:
                pkt = {
                    "type": "vive",
                    "pos": [0.0, 0.0, 0.0],
                    "quat": [1.0, 0.0, 0.0, 0.0],
                    "tracking": False,
                    "buttons": buttons,
                    "timestamp": time.time(),
                }
                lost_count += 1
                if lost_count == 1 or lost_count % (hz * 2) == 0:
                    print(f"\r[Sender] Tracker LOST ({lost_count} frames)", end="", flush=True)

            data = json.dumps(pkt).encode()
            sock.sendto(data, target)

            send_count += 1
            if send_count % (hz * 5) == 0:
                print(f"[Sender] Sent {send_count} packets")

            elapsed = time.perf_counter() - t_start
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print(f"\n[Sender] Stopped. Total packets sent: {send_count}")
    finally:
        kb.stop()
        sock.close()
        tracker.disconnect()


if __name__ == "__main__":
    main()
