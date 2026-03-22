#!/usr/bin/env python3
"""Manus Quantum Metagloves reader via C SDK (ctypes).

Reads finger joint angles and wrist pose from Manus gloves
using the Manus SDK in Integrated Mode (direct USB dongle, no Windows).

The ctypes bindings are based on the Manus SDK C API.
After downloading the SDK, verify struct layouts against ManusSDK.h
and ManusSDKTypes.h — update the ctypes definitions below if needed.

Usage (standalone test):
    python3 -m manus.manus_reader [--hand right] [--sdk-path manus/sdk/libManusSDK.so]
"""

import argparse
import ctypes
import ctypes.util
import os
import time
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path
from typing import Optional

import numpy as np


# ─────────────────────────────────────────────────────────
# SDK return codes
# ─────────────────────────────────────────────────────────

class SDKReturnCode(IntEnum):
    SUCCESS = 0
    ERROR = 1
    INVALID_ARGUMENT = 2
    ARGUMENT_SIZE_MISMATCH = 3
    UNSUPPORTED_STRETCHING_TYPE = 4
    INTERNAL_ERROR = 5
    FUNCTION_CALLED_AT_WRONG_TIME = 6
    DATA_NOT_AVAILABLE = 7
    MEMORY_ERROR = 8
    NOT_CONNECTED = 9
    UNINITIALIZED = 10


class Side(IntEnum):
    LEFT = 0
    RIGHT = 1


# ─────────────────────────────────────────────────────────
# Finger / joint enums (Manus Ergonomics layout)
#
# Each hand has 20 joint values:
#   Thumb:  CMC Spread, CMC Flexion, MCP Flexion, IP Flexion
#   Index:  MCP Spread, MCP Flexion, PIP Flexion, DIP Flexion
#   Middle: MCP Spread, MCP Flexion, PIP Flexion, DIP Flexion
#   Ring:   MCP Spread, MCP Flexion, PIP Flexion, DIP Flexion
#   Pinky:  MCP Spread, MCP Flexion, PIP Flexion, DIP Flexion
# ─────────────────────────────────────────────────────────

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
JOINT_NAMES_PER_FINGER = {
    "Thumb": ["CMC_Spread", "CMC_Flex", "MCP_Flex", "IP_Flex"],
    "Index": ["MCP_Spread", "MCP_Flex", "PIP_Flex", "DIP_Flex"],
    "Middle": ["MCP_Spread", "MCP_Flex", "PIP_Flex", "DIP_Flex"],
    "Ring": ["MCP_Spread", "MCP_Flex", "PIP_Flex", "DIP_Flex"],
    "Pinky": ["MCP_Spread", "MCP_Flex", "PIP_Flex", "DIP_Flex"],
}

NUM_FINGERS = 5
JOINTS_PER_FINGER = 4
NUM_JOINTS = NUM_FINGERS * JOINTS_PER_FINGER  # 20


# ─────────────────────────────────────────────────────────
# ctypes struct definitions
#
# NOTE: These struct layouts are based on SDK v2.4.0.
# If your SDK version differs, compare with ManusSDKTypes.h
# and update field definitions accordingly.
# ─────────────────────────────────────────────────────────

class ManusVec3(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
    ]


class ManusQuaternion(ctypes.Structure):
    _fields_ = [
        ("w", ctypes.c_float),
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
    ]


class ErgonomicsData(ctypes.Structure):
    """Per-hand ergonomics data — 20 float values for finger joints.

    The SDK populates this with normalized joint angles.
    Field order matches the Manus SDK ErgonomicsData struct.
    """
    _fields_ = [
        ("id", ctypes.c_uint32),
        ("is_user_id", ctypes.c_bool),
        ("data", ctypes.c_float * NUM_JOINTS),
    ]


class ErgonomicsStream(ctypes.Structure):
    """Ergonomics stream containing data for both hands."""
    _fields_ = [
        ("publishing", ctypes.c_bool),
        ("data_count", ctypes.c_uint32),
        ("data", ErgonomicsData * 2),  # left + right
    ]


# ─────────────────────────────────────────────────────────
# Data output
# ─────────────────────────────────────────────────────────

@dataclass
class HandData:
    """Hand tracking data from a single Manus glove.

    Attributes
    ----------
    joint_angles : ndarray[20]
        Finger joint angles in radians.
        Layout: [Thumb(4), Index(4), Middle(4), Ring(4), Pinky(4)]
    finger_spread : ndarray[5]
        Inter-finger abduction/adduction angles in radians.
        Layout: [Thumb, Index, Middle, Ring, Pinky]
    wrist_pos : ndarray[3]
        Wrist position in meters (SDK coordinate frame).
    wrist_quat : ndarray[4]
        Wrist orientation as quaternion (w, x, y, z).
    hand_side : str
        "left" or "right"
    timestamp : float
        Time of reading (time.time())
    """
    joint_angles: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS))
    finger_spread: np.ndarray = field(default_factory=lambda: np.zeros(NUM_FINGERS))
    wrist_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    wrist_quat: np.ndarray = field(default_factory=lambda: np.array([1.0, 0, 0, 0]))
    hand_side: str = "right"
    timestamp: float = 0.0


class ManusReader:
    """Reads hand data from Manus Quantum Metagloves via C SDK (ctypes).

    In integrated mode: direct USB/BLE connection on Linux.

    Parameters
    ----------
    sdk_lib_path : str
        Path to libManusSDK.so
    hand_side : str
        "left", "right", or "both"
    """

    def __init__(self, sdk_lib_path: str = "manus/sdk/libManusSDK.so",
                 hand_side: str = "right"):
        self._lib_path = sdk_lib_path
        self._hand_side = hand_side
        self._sdk = None
        self._connected = False
        self._session_id = ctypes.c_uint32(0)

    def connect(self):
        """Load SDK shared library, initialize, and connect locally."""
        lib_path = Path(self._lib_path)
        if not lib_path.exists():
            raise FileNotFoundError(
                f"Manus SDK not found at {lib_path.resolve()}\n"
                f"Download from: https://docs.manus-meta.com/2.4.0/Plugins/SDK/Linux/\n"
                f"Place libManusSDK.so in: {lib_path.parent.resolve()}"
            )

        # Load shared library
        self._sdk = ctypes.CDLL(str(lib_path.resolve()))
        self._setup_function_signatures()
        print(f"[ManusReader] SDK loaded from {lib_path}")

        # Initialize SDK
        ret = self._sdk.CoreSdk_Initialize(ctypes.c_int(1))  # SessionType::IntegratedDataSession
        if ret != SDKReturnCode.SUCCESS:
            raise RuntimeError(
                f"CoreSdk_Initialize failed with code {ret}. "
                "Check SDK installation and USB dongle."
            )
        print("[ManusReader] SDK initialized (Integrated Mode)")

        # Look for available hosts (dongles)
        ret = self._sdk.CoreSdk_LookForHosts(
            ctypes.c_uint32(2),   # search duration seconds
            ctypes.c_bool(False), # loopback
        )
        if ret != SDKReturnCode.SUCCESS:
            print(f"[ManusReader] WARN: LookForHosts returned {ret}")

        # Get available host count and connect to first one
        host_count = ctypes.c_uint32(0)
        ret = self._sdk.CoreSdk_GetNumberOfAvailableHostsFound(
            ctypes.byref(host_count)
        )
        if ret == SDKReturnCode.SUCCESS and host_count.value > 0:
            print(f"[ManusReader] Found {host_count.value} host(s)")

            # Connect to first available host
            ret = self._sdk.CoreSdk_ConnectToHost(ctypes.c_uint32(0))
            if ret == SDKReturnCode.SUCCESS:
                self._connected = True
                print("[ManusReader] Connected to Manus host")
            else:
                raise RuntimeError(
                    f"CoreSdk_ConnectToHost failed with code {ret}. "
                    "Check USB dongle and glove power."
                )
        else:
            # Try direct local connection
            ret = self._sdk.CoreSdk_ConnectLocally()
            if ret == SDKReturnCode.SUCCESS:
                self._connected = True
                print("[ManusReader] Connected locally")
            else:
                raise RuntimeError(
                    f"CoreSdk_ConnectLocally failed with code {ret}. "
                    "No Manus dongle found. Check USB connection and udev rules."
                )

    def disconnect(self):
        """Shut down SDK and release resources."""
        if self._sdk is not None and self._connected:
            self._sdk.CoreSdk_ShutDown()
            self._connected = False
            print("[ManusReader] SDK shut down")
        self._sdk = None

    def get_hand_data(self, side: Optional[str] = None) -> Optional[HandData]:
        """Read current hand state.

        Parameters
        ----------
        side : str or None
            "left" or "right". If None, uses self._hand_side.

        Returns
        -------
        HandData or None
            Hand tracking data, or None if not available.
        """
        if not self._connected or self._sdk is None:
            return None

        target_side = side or self._hand_side
        side_enum = Side.LEFT if target_side == "left" else Side.RIGHT

        # Get ergonomics data (finger joint angles)
        ergo_stream = ErgonomicsStream()
        ret = self._sdk.CoreSdk_GetRawErgonomicsData(
            ctypes.byref(ergo_stream),
            ctypes.c_uint32(side_enum),
        )

        if ret != SDKReturnCode.SUCCESS:
            return None

        # Extract joint angles
        joint_angles = np.zeros(NUM_JOINTS, dtype=np.float32)
        finger_spread = np.zeros(NUM_FINGERS, dtype=np.float32)

        if ergo_stream.data_count > 0:
            data = ergo_stream.data[0]
            for i in range(NUM_JOINTS):
                joint_angles[i] = data.data[i]

            # Extract spread angles (first joint of each finger)
            for f in range(NUM_FINGERS):
                finger_spread[f] = joint_angles[f * JOINTS_PER_FINGER]

        return HandData(
            joint_angles=joint_angles,
            finger_spread=finger_spread,
            wrist_pos=np.zeros(3),    # wrist pose requires skeleton data
            wrist_quat=np.array([1.0, 0.0, 0.0, 0.0]),
            hand_side=target_side,
            timestamp=time.time(),
        )

    def get_both_hands(self) -> dict[str, Optional[HandData]]:
        """Read data from both hands.

        Returns
        -------
        dict with keys "left" and "right", values are HandData or None.
        """
        return {
            "left": self.get_hand_data("left"),
            "right": self.get_hand_data("right"),
        }

    def get_status(self) -> dict:
        """Check dongle and glove connection status.

        Returns
        -------
        dict with status information.
        """
        return {
            "sdk_loaded": self._sdk is not None,
            "connected": self._connected,
            "hand_side": self._hand_side,
            "lib_path": self._lib_path,
        }

    def _setup_function_signatures(self):
        """Define ctypes function signatures for SDK functions.

        NOTE: These signatures are based on Manus SDK v2.4.0.
        If your SDK version differs, update the argtypes/restype
        definitions below based on ManusSDK.h.
        """
        sdk = self._sdk

        # CoreSdk_Initialize(SessionType)
        sdk.CoreSdk_Initialize.argtypes = [ctypes.c_int]
        sdk.CoreSdk_Initialize.restype = ctypes.c_int

        # CoreSdk_ShutDown()
        sdk.CoreSdk_ShutDown.argtypes = []
        sdk.CoreSdk_ShutDown.restype = ctypes.c_int

        # CoreSdk_LookForHosts(seconds, loopback)
        sdk.CoreSdk_LookForHosts.argtypes = [ctypes.c_uint32, ctypes.c_bool]
        sdk.CoreSdk_LookForHosts.restype = ctypes.c_int

        # CoreSdk_GetNumberOfAvailableHostsFound(out_count)
        sdk.CoreSdk_GetNumberOfAvailableHostsFound.argtypes = [
            ctypes.POINTER(ctypes.c_uint32)
        ]
        sdk.CoreSdk_GetNumberOfAvailableHostsFound.restype = ctypes.c_int

        # CoreSdk_ConnectToHost(host_index)
        sdk.CoreSdk_ConnectToHost.argtypes = [ctypes.c_uint32]
        sdk.CoreSdk_ConnectToHost.restype = ctypes.c_int

        # CoreSdk_ConnectLocally()
        sdk.CoreSdk_ConnectLocally.argtypes = []
        sdk.CoreSdk_ConnectLocally.restype = ctypes.c_int

        # CoreSdk_GetRawErgonomicsData(out_stream, hand_side)
        sdk.CoreSdk_GetRawErgonomicsData.argtypes = [
            ctypes.POINTER(ErgonomicsStream),
            ctypes.c_uint32,
        ]
        sdk.CoreSdk_GetRawErgonomicsData.restype = ctypes.c_int

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()


# ─────────────────────────────────────────────────────────
# Standalone test
# ─────────────────────────────────────────────────────────

def _print_hand_data(data: HandData):
    """Pretty-print hand data to terminal."""
    lines = []
    lines.append(f"  Hand: {data.hand_side.upper()}")

    for f_idx, fname in enumerate(FINGER_NAMES):
        joints = data.joint_angles[f_idx * JOINTS_PER_FINGER:
                                   (f_idx + 1) * JOINTS_PER_FINGER]
        jnames = JOINT_NAMES_PER_FINGER[fname]
        vals = "  ".join(f"{jnames[j]}={joints[j]:+6.3f}" for j in range(JOINTS_PER_FINGER))
        lines.append(f"  {fname:7s}: {vals}")

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="Manus Quantum Metagloves reader"
    )
    parser.add_argument("--hand", default="right",
                        choices=["left", "right", "both"],
                        help="Which hand(s) to read (default: right)")
    parser.add_argument("--sdk-path", default="manus/sdk/libManusSDK.so",
                        help="Path to libManusSDK.so")
    parser.add_argument("--hz", type=int, default=30,
                        help="Print rate in Hz (default: 30)")
    parser.add_argument("--duration", type=float, default=0,
                        help="Duration in seconds (0 = indefinite)")
    args = parser.parse_args()

    print("=" * 60)
    print("  Manus Quantum Metagloves — Data Reader")
    print("=" * 60)

    with ManusReader(sdk_lib_path=args.sdk_path, hand_side=args.hand) as reader:
        status = reader.get_status()
        print(f"  Status: {'CONNECTED' if status['connected'] else 'DISCONNECTED'}")
        print(f"  Hand:   {status['hand_side']}")
        print(f"  Rate:   {args.hz} Hz")
        print(f"  Duration: {'indefinite' if args.duration == 0 else f'{args.duration}s'}")
        print("-" * 60)
        print("  Press Ctrl+C to stop.")
        print()

        dt = 1.0 / args.hz
        count = 0
        start_time = time.time()
        null_count = 0

        try:
            while True:
                t_loop = time.perf_counter()

                if args.hand == "both":
                    hands = reader.get_both_hands()
                    for side, data in hands.items():
                        if data is not None:
                            print(f"\033[2J\033[H")  # Clear screen
                            print(_print_hand_data(data))
                        else:
                            null_count += 1
                else:
                    data = reader.get_hand_data()
                    if data is not None:
                        # Move cursor to top and overwrite
                        if count > 0:
                            print(f"\033[{NUM_FINGERS + 2}A", end="")
                        print(_print_hand_data(data))
                    else:
                        null_count += 1
                        if null_count == 1 or null_count % (args.hz * 2) == 0:
                            print(f"\r  [NO DATA] ({null_count} frames)", end="", flush=True)

                count += 1

                # Duration check
                if args.duration > 0:
                    elapsed_total = time.time() - start_time
                    if elapsed_total >= args.duration:
                        break

                elapsed = time.perf_counter() - t_loop
                remaining = dt - elapsed
                if remaining > 0:
                    time.sleep(remaining)

        except KeyboardInterrupt:
            pass

        elapsed_total = time.time() - start_time
        actual_hz = count / elapsed_total if elapsed_total > 0 else 0
        print(f"\n{'=' * 60}")
        print(f"  Stopped. Frames: {count}, Duration: {elapsed_total:.1f}s")
        print(f"  Actual rate: {actual_hz:.1f} Hz")
        print(f"  Null reads: {null_count}")
        print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
