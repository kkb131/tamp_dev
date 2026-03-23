#!/usr/bin/env python3
"""Manus Quantum Metagloves reader via C SDK v3.1.0 (ctypes).

Reads finger joint angles from Manus gloves using the SDK in
Integrated Mode (direct USB connection, no Windows Core needed).

v3.1.0 uses a callback-based model: the SDK pushes ErgonomicsStream
data via a registered callback, and get_hand_data() reads the latest
cached values.

Usage (standalone test):
    python3 -m manus.manus_reader [--hand right] [--sdk-path manus/sdk/libManusSDK.so]
"""

import argparse
import ctypes
import ctypes.util
import threading
import time
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path
from typing import Optional

import numpy as np


# ─────────────────────────────────────────────────────────
# SDK enums (v3.1.0 — ManusSDKTypes.h)
# ─────────────────────────────────────────────────────────

class SDKReturnCode(IntEnum):
    SUCCESS = 0
    ERROR = 1
    INVALID_ARGUMENT = 2
    ARGUMENT_SIZE_MISMATCH = 3
    UNSUPPORTED_STRING_SIZE = 4
    SDK_NOT_AVAILABLE = 5
    HOST_FINDER_NOT_AVAILABLE = 6
    DATA_NOT_AVAILABLE = 7
    MEMORY_ERROR = 8
    INTERNAL_ERROR = 9
    FUNCTION_CALLED_AT_WRONG_TIME = 10
    NOT_CONNECTED = 11
    CONNECTION_TIMEOUT = 12
    INVALID_ID = 13
    NULL_POINTER = 14
    INVALID_SEQUENCE = 15
    NO_COORDINATE_SYSTEM_SET = 16
    SDK_IS_TERMINATING = 17
    STUB_NULL_POINTER = 18
    SKELETON_NOT_LOADED = 19
    FUNCTION_NOT_AVAILABLE = 20


class Side(IntEnum):
    INVALID = 0
    LEFT = 1
    RIGHT = 2
    CENTER = 3


class AxisView(IntEnum):
    INVALID = 0
    Z_FROM_VIEWER = 1
    Y_FROM_VIEWER = 2
    X_FROM_VIEWER = 3
    X_TO_VIEWER = 4
    Y_TO_VIEWER = 5
    Z_TO_VIEWER = 6


class AxisPolarity(IntEnum):
    INVALID = 0
    NEG_Z = 1
    NEG_Y = 2
    NEG_X = 3
    POS_X = 4
    POS_Y = 5
    POS_Z = 6


# ─────────────────────────────────────────────────────────
# Finger / joint layout (Manus Ergonomics)
#
# v3.1.0 ErgonomicsData.data[40]:
#   [0..19]  = Left hand  (5 fingers × 4 joints)
#   [20..39] = Right hand (5 fingers × 4 joints)
#
# Per finger: MCPSpread, MCPStretch, PIPStretch, DIPStretch
# ─────────────────────────────────────────────────────────

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
JOINT_NAMES_PER_FINGER = {
    "Thumb": ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"],
    "Index": ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"],
    "Middle": ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"],
    "Ring": ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"],
    "Pinky": ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"],
}

NUM_FINGERS = 5
JOINTS_PER_FINGER = 4
NUM_JOINTS = NUM_FINGERS * JOINTS_PER_FINGER  # 20

# SDK constants
ERGONOMICS_DATA_MAX_SIZE = 40           # Left(20) + Right(20)
MAX_NUMBER_OF_ERGONOMICS_DATA = 32
MAX_NUM_CHARS_IN_HOST_NAME = 256
MAX_NUM_CHARS_IN_IP_ADDRESS = 40
MAX_NUM_CHARS_IN_VERSION = 16
MAX_NUMBER_OF_DONGLES = 16


# ─────────────────────────────────────────────────────────
# ctypes struct definitions (v3.1.0 — ManusSDKTypes.h)
# ─────────────────────────────────────────────────────────

class ManusTimestamp(ctypes.Structure):
    _fields_ = [("time", ctypes.c_uint64)]


class Version(ctypes.Structure):
    _fields_ = [
        ("major", ctypes.c_uint32),
        ("minor", ctypes.c_uint32),
        ("patch", ctypes.c_uint32),
        ("label", ctypes.c_char * MAX_NUM_CHARS_IN_VERSION),
        ("sha", ctypes.c_char * MAX_NUM_CHARS_IN_VERSION),
        ("tag", ctypes.c_char * MAX_NUM_CHARS_IN_VERSION),
    ]


class ManusHost(ctypes.Structure):
    _fields_ = [
        ("hostName", ctypes.c_char * MAX_NUM_CHARS_IN_HOST_NAME),
        ("ipAddress", ctypes.c_char * MAX_NUM_CHARS_IN_IP_ADDRESS),
        ("manusCoreVersion", Version),
    ]


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


class CoordinateSystemVUH(ctypes.Structure):
    _fields_ = [
        ("view", ctypes.c_int),
        ("up", ctypes.c_int),
        ("handedness", ctypes.c_int),
        ("unitScale", ctypes.c_float),
    ]


class ErgonomicsData(ctypes.Structure):
    """Per-device ergonomics data — 40 floats (left 20 + right 20)."""
    _fields_ = [
        ("id", ctypes.c_uint32),
        ("isUserID", ctypes.c_bool),
        ("data", ctypes.c_float * ERGONOMICS_DATA_MAX_SIZE),
    ]


class ErgonomicsStream(ctypes.Structure):
    """Ergonomics stream pushed via callback."""
    _fields_ = [
        ("publishTime", ManusTimestamp),
        ("data", ErgonomicsData * MAX_NUMBER_OF_ERGONOMICS_DATA),
        ("dataCount", ctypes.c_uint32),
    ]


# Callback function type for ergonomics stream
ErgonomicsStreamCallback_t = ctypes.CFUNCTYPE(
    None, ctypes.POINTER(ErgonomicsStream)
)


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
    """Reads hand data from Manus Quantum Metagloves via C SDK v3.1.0.

    Uses Integrated Mode (direct USB connection on Linux).
    Data is received via callback (push-based), not polling.

    Parameters
    ----------
    sdk_lib_path : str
        Path to libManusSDK.so or libManusSDK_Integrated.so
    hand_side : str
        "left", "right", or "both"
    """

    def __init__(self, sdk_lib_path: str = "manus/sdk/libManusSDK.so",
                 hand_side: str = "right"):
        self._lib_path = sdk_lib_path
        self._hand_side = hand_side
        self._sdk = None
        self._connected = False

        # Callback state (thread-safe)
        self._ergo_lock = threading.Lock()
        self._latest_ergo: dict[int, list[float]] = {}  # glove_id → data[40]
        self._ergo_callback_ref = None  # prevent GC of ctypes callback
        self._ergo_received = threading.Event()

        # Glove IDs (discovered after connection)
        self._left_glove_id: Optional[int] = None
        self._right_glove_id: Optional[int] = None

    def connect(self):
        """Load SDK, initialize in Integrated Mode, and connect."""
        lib_path = Path(self._lib_path)
        if not lib_path.exists():
            alt_path = lib_path.parent / "libManusSDK_Integrated.so"
            if alt_path.exists():
                print(f"[ManusReader] Using Integrated variant: {alt_path}")
                lib_path = alt_path
            else:
                raise FileNotFoundError(
                    f"Manus SDK not found at {lib_path.resolve()}\n"
                    f"Also checked: {alt_path}\n"
                    f"Download from: https://docs.manus-meta.com/3.1.0/Plugins/SDK/Linux/\n"
                    f"Place libManusSDK.so (or libManusSDK_Integrated.so) in: "
                    f"{lib_path.parent.resolve()}"
                )

        # Load shared library
        self._sdk = ctypes.CDLL(str(lib_path.resolve()))
        self._setup_function_signatures()
        print(f"[ManusReader] SDK loaded from {lib_path}")

        # 1. Initialize SDK (Integrated Mode)
        ret = self._sdk.CoreSdk_InitializeIntegrated()
        if ret != SDKReturnCode.SUCCESS:
            raise RuntimeError(
                f"CoreSdk_InitializeIntegrated failed with code {ret} "
                f"({SDKReturnCode(ret).name})"
            )
        print("[ManusReader] SDK initialized (Integrated Mode)")

        # 2. Register ergonomics callback (before connecting)
        self._ergo_callback_ref = ErgonomicsStreamCallback_t(self._on_ergonomics)
        ret = self._sdk.CoreSdk_RegisterCallbackForErgonomicsStream(
            self._ergo_callback_ref
        )
        if ret != SDKReturnCode.SUCCESS:
            print(f"[ManusReader] WARN: RegisterCallbackForErgonomicsStream "
                  f"returned {ret} ({SDKReturnCode(ret).name})")
        else:
            print("[ManusReader] Ergonomics callback registered")

        # 3. Set coordinate system (REQUIRED before ConnectToHost)
        vuh = CoordinateSystemVUH()
        vuh.handedness = int(Side.RIGHT)
        vuh.up = int(AxisPolarity.POS_Y)
        vuh.view = int(AxisView.Z_FROM_VIEWER)
        vuh.unitScale = 1.0  # meters
        ret = self._sdk.CoreSdk_InitializeCoordinateSystemWithVUH(
            vuh, ctypes.c_bool(True)  # use world coordinates
        )
        if ret != SDKReturnCode.SUCCESS:
            print(f"[ManusReader] WARN: InitializeCoordinateSystemWithVUH "
                  f"returned {ret} ({SDKReturnCode(ret).name})")
        else:
            print("[ManusReader] Coordinate system set (Y-up, Z-forward, meters)")

        # 4. Connect to host (empty ManusHost for Integrated mode)
        empty_host = ManusHost()
        ctypes.memset(ctypes.byref(empty_host), 0, ctypes.sizeof(ManusHost))
        ret = self._sdk.CoreSdk_ConnectToHost(empty_host)
        if ret == SDKReturnCode.SUCCESS:
            self._connected = True
            print("[ManusReader] Connected to Manus host (Integrated)")
        else:
            raise RuntimeError(
                f"CoreSdk_ConnectToHost failed with code {ret} "
                f"({SDKReturnCode(ret).name}). "
                "Check USB dongle and glove power."
            )

        # 5. Discover glove IDs
        self._discover_gloves()

    def disconnect(self):
        """Shut down SDK and release resources."""
        if self._sdk is not None and self._connected:
            self._sdk.CoreSdk_ShutDown()
            self._connected = False
            print("[ManusReader] SDK shut down")
        self._sdk = None
        self._ergo_callback_ref = None

    def get_hand_data(self, side: Optional[str] = None) -> Optional[HandData]:
        """Read latest hand data from callback cache.

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

        # Ergonomics data layout:
        #   data[0..19]  = Left hand
        #   data[20..39] = Right hand
        if target_side == "left":
            offset = 0
        else:
            offset = NUM_JOINTS  # 20

        # Find data from any device (try all cached entries)
        with self._ergo_lock:
            if not self._latest_ergo:
                return None

            # Try specific glove ID first
            target_id = (self._left_glove_id if target_side == "left"
                         else self._right_glove_id)
            if target_id is not None and target_id in self._latest_ergo:
                raw = self._latest_ergo[target_id]
            else:
                # Use first available device data
                raw = next(iter(self._latest_ergo.values()))

        joint_angles = np.array(raw[offset:offset + NUM_JOINTS], dtype=np.float32)
        finger_spread = np.zeros(NUM_FINGERS, dtype=np.float32)
        for f in range(NUM_FINGERS):
            finger_spread[f] = joint_angles[f * JOINTS_PER_FINGER]

        return HandData(
            joint_angles=joint_angles,
            finger_spread=finger_spread,
            wrist_pos=np.zeros(3),
            wrist_quat=np.array([1.0, 0.0, 0.0, 0.0]),
            hand_side=target_side,
            timestamp=time.time(),
        )

    def get_both_hands(self) -> dict[str, Optional[HandData]]:
        """Read data from both hands."""
        return {
            "left": self.get_hand_data("left"),
            "right": self.get_hand_data("right"),
        }

    def get_status(self) -> dict:
        """Check connection status."""
        return {
            "sdk_loaded": self._sdk is not None,
            "connected": self._connected,
            "hand_side": self._hand_side,
            "lib_path": self._lib_path,
            "left_glove_id": self._left_glove_id,
            "right_glove_id": self._right_glove_id,
            "ergo_devices": len(self._latest_ergo),
        }

    def wait_for_data(self, timeout: float = 5.0) -> bool:
        """Wait until at least one ergonomics callback is received."""
        return self._ergo_received.wait(timeout=timeout)

    # ── Private ──────────────────────────────────────────

    def _on_ergonomics(self, p_ergo_ptr):
        """Callback invoked by SDK when new ergonomics data arrives."""
        try:
            stream = p_ergo_ptr.contents
            with self._ergo_lock:
                for i in range(stream.dataCount):
                    d = stream.data[i]
                    if d.isUserID:
                        continue
                    self._latest_ergo[d.id] = list(d.data)
            self._ergo_received.set()
        except Exception:
            pass  # never let callback exceptions crash the SDK thread

    def _discover_gloves(self):
        """Try to discover left/right glove IDs via dongle query."""
        try:
            num_dongles = ctypes.c_uint32(0)
            ret = self._sdk.CoreSdk_GetNumberOfDongles(
                ctypes.byref(num_dongles)
            )
            if ret != SDKReturnCode.SUCCESS or num_dongles.value == 0:
                print("[ManusReader] No dongles found (will auto-detect from stream)")
                return

            dongle_ids = (ctypes.c_uint32 * MAX_NUMBER_OF_DONGLES)()
            ret = self._sdk.CoreSdk_GetDongleIds(
                dongle_ids, ctypes.c_uint32(num_dongles.value)
            )
            if ret != SDKReturnCode.SUCCESS:
                return

            for i in range(num_dongles.value):
                left_id = ctypes.c_uint32(0)
                right_id = ctypes.c_uint32(0)
                ret = self._sdk.CoreSdk_GetGlovesForDongle(
                    dongle_ids[i],
                    ctypes.byref(left_id),
                    ctypes.byref(right_id),
                )
                if ret == SDKReturnCode.SUCCESS:
                    if left_id.value != 0:
                        self._left_glove_id = left_id.value
                        print(f"[ManusReader] Left glove ID: {left_id.value}")
                    if right_id.value != 0:
                        self._right_glove_id = right_id.value
                        print(f"[ManusReader] Right glove ID: {right_id.value}")
        except Exception as e:
            print(f"[ManusReader] WARN: Glove discovery failed: {e}")

    def _setup_function_signatures(self):
        """Define ctypes function signatures for SDK v3.1.0 functions."""
        sdk = self._sdk

        # Initialize / Shutdown
        sdk.CoreSdk_InitializeIntegrated.argtypes = []
        sdk.CoreSdk_InitializeIntegrated.restype = ctypes.c_int

        sdk.CoreSdk_ShutDown.argtypes = []
        sdk.CoreSdk_ShutDown.restype = ctypes.c_int

        # Coordinate system
        sdk.CoreSdk_InitializeCoordinateSystemWithVUH.argtypes = [
            CoordinateSystemVUH, ctypes.c_bool
        ]
        sdk.CoreSdk_InitializeCoordinateSystemWithVUH.restype = ctypes.c_int

        # Callback registration
        sdk.CoreSdk_RegisterCallbackForErgonomicsStream.argtypes = [
            ErgonomicsStreamCallback_t
        ]
        sdk.CoreSdk_RegisterCallbackForErgonomicsStream.restype = ctypes.c_int

        # Host discovery & connection
        sdk.CoreSdk_LookForHosts.argtypes = [ctypes.c_uint32, ctypes.c_bool]
        sdk.CoreSdk_LookForHosts.restype = ctypes.c_int

        sdk.CoreSdk_GetNumberOfAvailableHostsFound.argtypes = [
            ctypes.POINTER(ctypes.c_uint32)
        ]
        sdk.CoreSdk_GetNumberOfAvailableHostsFound.restype = ctypes.c_int

        sdk.CoreSdk_ConnectToHost.argtypes = [ManusHost]
        sdk.CoreSdk_ConnectToHost.restype = ctypes.c_int

        # Dongle / Glove discovery
        sdk.CoreSdk_GetNumberOfDongles.argtypes = [
            ctypes.POINTER(ctypes.c_uint32)
        ]
        sdk.CoreSdk_GetNumberOfDongles.restype = ctypes.c_int

        sdk.CoreSdk_GetDongleIds.argtypes = [
            ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32
        ]
        sdk.CoreSdk_GetDongleIds.restype = ctypes.c_int

        sdk.CoreSdk_GetGlovesForDongle.argtypes = [
            ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.POINTER(ctypes.c_uint32),
        ]
        sdk.CoreSdk_GetGlovesForDongle.restype = ctypes.c_int

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
        vals = "  ".join(f"{jnames[j]}={joints[j]:+6.3f}"
                         for j in range(JOINTS_PER_FINGER))
        lines.append(f"  {fname:7s}: {vals}")

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="Manus Quantum Metagloves reader (SDK v3.1.0)"
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
    print("  Manus Quantum Metagloves — Data Reader (SDK v3.1.0)")
    print("=" * 60)

    with ManusReader(sdk_lib_path=args.sdk_path, hand_side=args.hand) as reader:
        status = reader.get_status()
        print(f"  Status: {'CONNECTED' if status['connected'] else 'DISCONNECTED'}")
        print(f"  Hand:   {status['hand_side']}")
        print(f"  Rate:   {args.hz} Hz")

        # Wait for first callback data
        print("  Waiting for glove data...", end=" ", flush=True)
        if reader.wait_for_data(timeout=5.0):
            print("OK")
        else:
            print("TIMEOUT (no data after 5s)")

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
                            print(f"\033[2J\033[H")
                            print(_print_hand_data(data))
                        else:
                            null_count += 1
                else:
                    data = reader.get_hand_data()
                    if data is not None:
                        if count > 0:
                            print(f"\033[{NUM_FINGERS + 2}A", end="")
                        print(_print_hand_data(data))
                    else:
                        null_count += 1
                        if null_count == 1 or null_count % (args.hz * 2) == 0:
                            print(f"\r  [NO DATA] ({null_count} frames)",
                                  end="", flush=True)

                count += 1

                if args.duration > 0:
                    if time.time() - start_time >= args.duration:
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
        print(f"  Actual rate: {actual_hz:.1f} Hz, Null reads: {null_count}")
        print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
