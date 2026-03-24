#!/usr/bin/env python3
"""Manus Quantum Metagloves reader via C SDK v3.1.0 (ctypes).

Faithful port of SDKClient_Linux/SDKClient.cpp connection & ergonomics flow.
Uses Integrated Mode (direct USB, no Windows Core).

Initialization sequence (mirrors SDKClient.cpp exactly):
  1. CoreSdk_InitializeIntegrated()
  2. Register ALL callbacks (OnConnect, Landscape, Ergonomics, ...)
  3. CoreSdk_InitializeCoordinateSystemWithVUH()
  4. CoreSdk_ConnectToHost(empty ManusHost)
  5. SDK fires OnConnectedCallback → SetRawSkeletonHandMotion(Auto)
  6. SDK fires OnLandscapeCallback → extract glove IDs
  7. SDK fires OnErgonomicsCallback → store per-glove finger data

Usage:
    python3 -m manus.manus_reader [--hand right] [--sdk-path manus/sdk/libManusSDK.so]
"""

import argparse
import ctypes
import ctypes.util
import sys
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


class HandMotion(IntEnum):
    NONE = 0
    IMU = 1
    TRACKER = 2
    TRACKER_ROTATION_ONLY = 3
    AUTO = 4


# ─────────────────────────────────────────────────────────
# Finger / joint layout
#
# ErgonomicsData.data[40]:
#   [0..19]  = Left hand  (5 fingers × 4 joints)
#   [20..39] = Right hand (5 fingers × 4 joints)
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
ERGONOMICS_DATA_MAX_SIZE = 40
MAX_NUMBER_OF_ERGONOMICS_DATA = 32
MAX_NUM_CHARS_IN_HOST_NAME = 256
MAX_NUM_CHARS_IN_IP_ADDRESS = 40
MAX_NUM_CHARS_IN_VERSION = 16
MAX_NUMBER_OF_DONGLES = 16


# ─────────────────────────────────────────────────────────
# ctypes struct definitions (v3.1.0)
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
    _fields_ = [
        ("id", ctypes.c_uint32),
        ("isUserID", ctypes.c_bool),
        ("data", ctypes.c_float * ERGONOMICS_DATA_MAX_SIZE),
    ]


class ErgonomicsStream(ctypes.Structure):
    _fields_ = [
        ("publishTime", ManusTimestamp),
        ("data", ErgonomicsData * MAX_NUMBER_OF_ERGONOMICS_DATA),
        ("dataCount", ctypes.c_uint32),
    ]


# Callback function types
ErgonomicsStreamCallback_t = ctypes.CFUNCTYPE(
    None, ctypes.POINTER(ErgonomicsStream)
)
ConnectedToCoreCallback_t = ctypes.CFUNCTYPE(
    None, ctypes.POINTER(ManusHost)
)
DisconnectedFromCoreCallback_t = ctypes.CFUNCTYPE(
    None, ctypes.POINTER(ManusHost)
)
# Landscape struct is huge — receive as void* (only used as signal)
LandscapeStreamCallback_t = ctypes.CFUNCTYPE(
    None, ctypes.c_void_p
)


# ─────────────────────────────────────────────────────────
# Data output
# ─────────────────────────────────────────────────────────

@dataclass
class HandData:
    """Hand tracking data from a single Manus glove."""
    joint_angles: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS))
    finger_spread: np.ndarray = field(default_factory=lambda: np.zeros(NUM_FINGERS))
    wrist_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    wrist_quat: np.ndarray = field(default_factory=lambda: np.array([1.0, 0, 0, 0]))
    hand_side: str = "right"
    timestamp: float = 0.0


class ManusReader:
    """Reads hand data from Manus Quantum Metagloves via C SDK v3.1.0.

    Connection & data flow mirrors SDKClient_Linux/SDKClient.cpp.
    """

    def __init__(self, sdk_lib_path: str = "manus/sdk/libManusSDK.so",
                 hand_side: str = "right"):
        self._lib_path = sdk_lib_path
        self._hand_side = hand_side
        self._sdk = None
        self._connected = False

        # Callback references (prevent GC)
        self._cb_connected = None
        self._cb_disconnected = None
        self._cb_landscape = None
        self._cb_ergonomics = None

        # Ergonomics data (thread-safe)
        self._ergo_lock = threading.Lock()
        self._left_ergo = ErgonomicsData()
        self._right_ergo = ErgonomicsData()
        self._first_left_glove_id: int = 0
        self._first_right_glove_id: int = 0

        # Debug counters
        self._cb_count_connected = 0
        self._cb_count_landscape = 0
        self._cb_count_ergonomics = 0
        self._ergo_received = threading.Event()
        self._landscape_received = threading.Event()
        self._last_ergo_time: float = 0.0

    # ────────────────────────────────────────────────
    # Public API
    # ────────────────────────────────────────────────

    def connect(self):
        """Load SDK, initialize, register callbacks, set coord system, connect.

        Follows SDKClient.cpp InitializeSDK() sequence exactly.
        """
        lib_path = self._resolve_sdk_path()

        self._sdk = ctypes.CDLL(str(lib_path.resolve()))
        self._setup_function_signatures()
        print(f"[ManusReader] SDK loaded: {lib_path}")

        # Step 1: Initialize (SDKClient.cpp:531)
        ret = self._sdk.CoreSdk_InitializeIntegrated()
        if ret != SDKReturnCode.SUCCESS:
            raise RuntimeError(
                f"CoreSdk_InitializeIntegrated failed: {ret} "
                f"({SDKReturnCode(ret).name})"
            )
        print("[ManusReader] SDK initialized (Integrated)")

        # Step 2: Register ALL callbacks (SDKClient.cpp:547 → RegisterAllCallbacks)
        self._register_all_callbacks()

        # Step 3: Set coordinate system (SDKClient.cpp:559-567)
        vuh = CoordinateSystemVUH()
        vuh.handedness = int(Side.RIGHT)
        vuh.up = int(AxisPolarity.POS_Y)
        vuh.view = int(AxisView.Z_FROM_VIEWER)
        vuh.unitScale = 1.0
        ret = self._sdk.CoreSdk_InitializeCoordinateSystemWithVUH(
            vuh, ctypes.c_bool(True)
        )
        if ret != SDKReturnCode.SUCCESS:
            print(f"[ManusReader] WARN: CoordinateSystem returned {ret} "
                  f"({SDKReturnCode(ret).name})")
        else:
            print("[ManusReader] Coordinate system set (Y-up, right-handed, meters)")

        # Step 4: Connect (SDKClient.cpp:855 — Integrated mode uses empty host)
        empty_host = ManusHost()
        ctypes.memset(ctypes.byref(empty_host), 0, ctypes.sizeof(ManusHost))
        ret = self._sdk.CoreSdk_ConnectToHost(empty_host)
        if ret == SDKReturnCode.SUCCESS:
            self._connected = True
            print("[ManusReader] Connected (Integrated mode)")
        else:
            raise RuntimeError(
                f"CoreSdk_ConnectToHost failed: {ret} "
                f"({SDKReturnCode(ret).name}). "
                "Check USB dongle/glove."
            )

        # Wait for landscape + ergonomics callbacks
        print("[ManusReader] Waiting for device data...", end=" ", flush=True)
        self._landscape_received.wait(timeout=5.0)
        if self._landscape_received.is_set():
            print(f"landscape OK (L={self._first_left_glove_id:#x}, "
                  f"R={self._first_right_glove_id:#x})", flush=True)
        else:
            print("landscape TIMEOUT", flush=True)
            # Still try to discover via API
            self._discover_gloves_via_api()

    def disconnect(self):
        """Shut down SDK."""
        if self._sdk is not None and self._connected:
            self._sdk.CoreSdk_ShutDown()
            self._connected = False
            print("[ManusReader] Shut down")
        self._sdk = None
        self._cb_connected = None
        self._cb_disconnected = None
        self._cb_landscape = None
        self._cb_ergonomics = None

    def get_hand_data(self, side: Optional[str] = None) -> Optional[HandData]:
        """Read latest hand data from callback cache."""
        if not self._connected or self._sdk is None:
            return None

        target_side = side or self._hand_side

        # ErgonomicsData.data[40]: left=[0:20], right=[20:40]
        if target_side == "left":
            offset = 0
        else:
            offset = NUM_JOINTS  # 20

        with self._ergo_lock:
            # Pick the correct ergo data source
            if target_side == "left" and self._left_ergo.id != 0:
                raw = list(self._left_ergo.data)
            elif target_side == "right" and self._right_ergo.id != 0:
                raw = list(self._right_ergo.data)
            else:
                return None

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
            timestamp=self._last_ergo_time or time.time(),
        )

    def get_both_hands(self) -> dict[str, Optional[HandData]]:
        return {"left": self.get_hand_data("left"),
                "right": self.get_hand_data("right")}

    def get_status(self) -> dict:
        return {
            "sdk_loaded": self._sdk is not None,
            "connected": self._connected,
            "hand_side": self._hand_side,
            "lib_path": self._lib_path,
            "left_glove_id": f"{self._first_left_glove_id:#x}",
            "right_glove_id": f"{self._first_right_glove_id:#x}",
            "cb_connected": self._cb_count_connected,
            "cb_landscape": self._cb_count_landscape,
            "cb_ergonomics": self._cb_count_ergonomics,
        }

    def wait_for_data(self, timeout: float = 5.0) -> bool:
        return self._ergo_received.wait(timeout=timeout)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    # ────────────────────────────────────────────────
    # Callbacks (mirrors SDKClient.cpp)
    # ────────────────────────────────────────────────

    def _on_connected(self, p_host_ptr):
        """SDKClient.cpp:173 — OnConnectedCallback."""
        self._cb_count_connected += 1
        try:
            print(f"[CB] OnConnect #{self._cb_count_connected}")
            # SDKClient.cpp:218 — Set hand motion mode
            ret = self._sdk.CoreSdk_SetRawSkeletonHandMotion(
                ctypes.c_int(int(HandMotion.AUTO))
            )
            if ret == SDKReturnCode.SUCCESS:
                print("[CB] HandMotion=Auto")
            else:
                print(f"[CB] WARN: SetRawSkeletonHandMotion={ret}")
        except Exception as e:
            print(f"[CB] OnConnect error: {e}", file=sys.stderr)

    def _on_disconnected(self, p_host_ptr):
        """SDKClient.cpp — OnDisconnectedCallback."""
        self._connected = False
        print("[CB] Disconnected from core")

    def _on_landscape(self, p_landscape_ptr):
        """SDKClient.cpp:390 — OnLandscapeCallback.

        Landscape struct is too large for ctypes mapping.
        Instead, use GetGlovesForDongle API to extract glove IDs.
        """
        self._cb_count_landscape += 1
        try:
            self._discover_gloves_via_api()
            if self._cb_count_landscape <= 3:
                print(f"[CB] Landscape #{self._cb_count_landscape}: "
                      f"L={self._first_left_glove_id:#x}, "
                      f"R={self._first_right_glove_id:#x}")
            self._landscape_received.set()
        except Exception as e:
            print(f"[CB] Landscape error: {e}", file=sys.stderr)

    def _on_ergonomics(self, p_ergo_ptr):
        """SDKClient.cpp:434 — OnErgonomicsCallback.

        Mirrors the example: match data by glove ID, store left/right separately.
        Fallback: if glove IDs unknown, store from any device.
        """
        self._cb_count_ergonomics += 1
        try:
            stream = p_ergo_ptr.contents
            count = min(stream.dataCount, MAX_NUMBER_OF_ERGONOMICS_DATA)

            # Debug: log first few callbacks
            if self._cb_count_ergonomics <= 3:
                ids = [stream.data[i].id for i in range(count)]
                user_flags = [stream.data[i].isUserID for i in range(count)]
                print(f"[CB] Ergonomics #{self._cb_count_ergonomics}: "
                      f"count={count}, IDs={[f'{x:#x}' for x in ids]}, "
                      f"isUser={user_flags}")

            with self._ergo_lock:
                for i in range(count):
                    d = stream.data[i]
                    if d.isUserID:
                        continue

                    # SDKClient.cpp:443-451 — Match by glove ID
                    if (self._first_left_glove_id != 0
                            and d.id == self._first_left_glove_id):
                        ctypes.memmove(
                            ctypes.byref(self._left_ergo),
                            ctypes.byref(d),
                            ctypes.sizeof(ErgonomicsData),
                        )
                    elif (self._first_right_glove_id != 0
                            and d.id == self._first_right_glove_id):
                        ctypes.memmove(
                            ctypes.byref(self._right_ergo),
                            ctypes.byref(d),
                            ctypes.sizeof(ErgonomicsData),
                        )
                    elif (self._first_left_glove_id == 0
                            and self._first_right_glove_id == 0):
                        # Fallback: glove IDs not yet known, store for both
                        ctypes.memmove(
                            ctypes.byref(self._left_ergo),
                            ctypes.byref(d),
                            ctypes.sizeof(ErgonomicsData),
                        )
                        ctypes.memmove(
                            ctypes.byref(self._right_ergo),
                            ctypes.byref(d),
                            ctypes.sizeof(ErgonomicsData),
                        )

            self._last_ergo_time = time.time()
            self._ergo_received.set()
        except Exception as e:
            print(f"[CB] Ergonomics error: {e}", file=sys.stderr)

    # ────────────────────────────────────────────────
    # Private helpers
    # ────────────────────────────────────────────────

    def _resolve_sdk_path(self) -> Path:
        lib_path = Path(self._lib_path)
        if not lib_path.exists():
            alt = lib_path.parent / "libManusSDK_Integrated.so"
            if alt.exists():
                print(f"[ManusReader] Using Integrated variant: {alt}")
                return alt
            raise FileNotFoundError(
                f"SDK not found: {lib_path.resolve()}\n"
                f"Also checked: {alt}\n"
                f"Download: https://docs.manus-meta.com/3.1.0/Plugins/SDK/Linux/"
            )
        return lib_path

    def _register_all_callbacks(self):
        """SDKClient.cpp:611 — RegisterAllCallbacks.

        Register ALL callbacks before coordinate system and connection.
        Raises RuntimeError if critical callbacks fail.
        """
        sdk = self._sdk

        # OnConnect (SDKClient.cpp:616)
        self._cb_connected = ConnectedToCoreCallback_t(self._on_connected)
        ret = sdk.CoreSdk_RegisterCallbackForOnConnect(self._cb_connected)
        self._check_cb("OnConnect", ret)

        # OnDisconnect (SDKClient.cpp:626)
        self._cb_disconnected = DisconnectedFromCoreCallback_t(self._on_disconnected)
        ret = sdk.CoreSdk_RegisterCallbackForOnDisconnect(self._cb_disconnected)
        self._check_cb("OnDisconnect", ret, critical=False)

        # Landscape (SDKClient.cpp:654)
        self._cb_landscape = LandscapeStreamCallback_t(self._on_landscape)
        ret = sdk.CoreSdk_RegisterCallbackForLandscapeStream(self._cb_landscape)
        self._check_cb("Landscape", ret)

        # Ergonomics (SDKClient.cpp:674)
        self._cb_ergonomics = ErgonomicsStreamCallback_t(self._on_ergonomics)
        ret = sdk.CoreSdk_RegisterCallbackForErgonomicsStream(self._cb_ergonomics)
        self._check_cb("Ergonomics", ret)

        print("[ManusReader] All callbacks registered")

    def _check_cb(self, name: str, ret: int, critical: bool = True):
        if ret != SDKReturnCode.SUCCESS:
            msg = (f"RegisterCallback({name}) failed: {ret} "
                   f"({SDKReturnCode(ret).name})")
            if critical:
                raise RuntimeError(msg)
            print(f"[ManusReader] WARN: {msg}")

    def _discover_gloves_via_api(self):
        """SDKClient.cpp:966-981 — Extract glove IDs from dongles."""
        try:
            num_dongles = ctypes.c_uint32(0)
            ret = self._sdk.CoreSdk_GetNumberOfDongles(
                ctypes.byref(num_dongles)
            )
            if ret != SDKReturnCode.SUCCESS or num_dongles.value == 0:
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
                    if left_id.value != 0 and self._first_left_glove_id == 0:
                        self._first_left_glove_id = left_id.value
                    if right_id.value != 0 and self._first_right_glove_id == 0:
                        self._first_right_glove_id = right_id.value
        except Exception:
            pass

    def _setup_function_signatures(self):
        """Define ctypes signatures for SDK v3.1.0."""
        sdk = self._sdk

        # Init / Shutdown
        sdk.CoreSdk_InitializeIntegrated.argtypes = []
        sdk.CoreSdk_InitializeIntegrated.restype = ctypes.c_int
        sdk.CoreSdk_ShutDown.argtypes = []
        sdk.CoreSdk_ShutDown.restype = ctypes.c_int

        # Coordinate system
        sdk.CoreSdk_InitializeCoordinateSystemWithVUH.argtypes = [
            CoordinateSystemVUH, ctypes.c_bool]
        sdk.CoreSdk_InitializeCoordinateSystemWithVUH.restype = ctypes.c_int

        # Callbacks
        sdk.CoreSdk_RegisterCallbackForOnConnect.argtypes = [
            ConnectedToCoreCallback_t]
        sdk.CoreSdk_RegisterCallbackForOnConnect.restype = ctypes.c_int
        sdk.CoreSdk_RegisterCallbackForOnDisconnect.argtypes = [
            DisconnectedFromCoreCallback_t]
        sdk.CoreSdk_RegisterCallbackForOnDisconnect.restype = ctypes.c_int
        sdk.CoreSdk_RegisterCallbackForLandscapeStream.argtypes = [
            LandscapeStreamCallback_t]
        sdk.CoreSdk_RegisterCallbackForLandscapeStream.restype = ctypes.c_int
        sdk.CoreSdk_RegisterCallbackForErgonomicsStream.argtypes = [
            ErgonomicsStreamCallback_t]
        sdk.CoreSdk_RegisterCallbackForErgonomicsStream.restype = ctypes.c_int

        # Connection
        sdk.CoreSdk_ConnectToHost.argtypes = [ManusHost]
        sdk.CoreSdk_ConnectToHost.restype = ctypes.c_int
        sdk.CoreSdk_LookForHosts.argtypes = [ctypes.c_uint32, ctypes.c_bool]
        sdk.CoreSdk_LookForHosts.restype = ctypes.c_int
        sdk.CoreSdk_GetNumberOfAvailableHostsFound.argtypes = [
            ctypes.POINTER(ctypes.c_uint32)]
        sdk.CoreSdk_GetNumberOfAvailableHostsFound.restype = ctypes.c_int

        # Hand motion
        sdk.CoreSdk_SetRawSkeletonHandMotion.argtypes = [ctypes.c_int]
        sdk.CoreSdk_SetRawSkeletonHandMotion.restype = ctypes.c_int

        # Dongle / Glove discovery
        sdk.CoreSdk_GetNumberOfDongles.argtypes = [
            ctypes.POINTER(ctypes.c_uint32)]
        sdk.CoreSdk_GetNumberOfDongles.restype = ctypes.c_int
        sdk.CoreSdk_GetDongleIds.argtypes = [
            ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32]
        sdk.CoreSdk_GetDongleIds.restype = ctypes.c_int
        sdk.CoreSdk_GetGlovesForDongle.argtypes = [
            ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.POINTER(ctypes.c_uint32)]
        sdk.CoreSdk_GetGlovesForDongle.restype = ctypes.c_int


# ─────────────────────────────────────────────────────────
# Standalone test
# ─────────────────────────────────────────────────────────

def _print_hand_data(data: HandData) -> str:
    lines = [f"  Hand: {data.hand_side.upper()}  "
             f"(t={data.timestamp:.1f})"]
    for f_idx, fname in enumerate(FINGER_NAMES):
        joints = data.joint_angles[f_idx * JOINTS_PER_FINGER:
                                   (f_idx + 1) * JOINTS_PER_FINGER]
        jnames = JOINT_NAMES_PER_FINGER[fname]
        vals = "  ".join(f"{jnames[j]}={joints[j]:+7.3f}"
                         for j in range(JOINTS_PER_FINGER))
        lines.append(f"  {fname:7s}: {vals}")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="Manus Quantum Metagloves reader (SDK v3.1.0)")
    parser.add_argument("--hand", default="right",
                        choices=["left", "right", "both"])
    parser.add_argument("--sdk-path", default="manus/sdk/libManusSDK.so")
    parser.add_argument("--hz", type=int, default=30)
    parser.add_argument("--duration", type=float, default=0)
    args = parser.parse_args()

    print("=" * 65)
    print("  Manus Quantum Metagloves — Reader (SDK v3.1.0)")
    print("=" * 65)

    with ManusReader(sdk_lib_path=args.sdk_path, hand_side=args.hand) as reader:
        status = reader.get_status()
        print(f"  Connected: {status['connected']}")
        print(f"  Left glove:  {status['left_glove_id']}")
        print(f"  Right glove: {status['right_glove_id']}")

        print("  Waiting for glove data...", end=" ", flush=True)
        if reader.wait_for_data(timeout=5.0):
            print("OK")
        else:
            print("TIMEOUT (no ergonomics data after 5s)")
            print(f"  Callback counts: connected={status['cb_connected']}, "
                  f"landscape={status['cb_landscape']}, "
                  f"ergonomics={status['cb_ergonomics']}")

        print("-" * 65)
        print("  Press Ctrl+C to stop.\n")

        dt = 1.0 / args.hz
        count = 0
        start_time = time.time()
        null_count = 0
        # Number of output lines for cursor reset
        num_lines = NUM_FINGERS + 1  # header + 5 fingers

        try:
            while True:
                t_loop = time.perf_counter()

                if args.hand == "both":
                    for side in ("left", "right"):
                        data = reader.get_hand_data(side)
                        if data is not None:
                            if count > 0:
                                print(f"\033[{num_lines * 2 + 1}A", end="")
                            print(_print_hand_data(data))
                        else:
                            null_count += 1
                else:
                    data = reader.get_hand_data()
                    if data is not None:
                        if count > 0:
                            print(f"\033[{num_lines}A", end="")
                        print(_print_hand_data(data))
                    else:
                        null_count += 1
                        if null_count == 1 or null_count % 60 == 0:
                            s = reader.get_status()
                            print(f"\r  [NO DATA] null={null_count} "
                                  f"cb_ergo={s['cb_ergonomics']} "
                                  f"L={s['left_glove_id']} "
                                  f"R={s['right_glove_id']}    ",
                                  end="", flush=True)

                count += 1

                if args.duration > 0 and time.time() - start_time >= args.duration:
                    break

                remaining = dt - (time.perf_counter() - t_loop)
                if remaining > 0:
                    time.sleep(remaining)

        except KeyboardInterrupt:
            pass

        elapsed = time.time() - start_time
        hz = count / elapsed if elapsed > 0 else 0
        s = reader.get_status()
        print(f"\n{'=' * 65}")
        print(f"  Frames: {count}, Duration: {elapsed:.1f}s, Rate: {hz:.1f}Hz")
        print(f"  Null reads: {null_count}")
        print(f"  Callbacks: connect={s['cb_connected']}, "
              f"landscape={s['cb_landscape']}, ergonomics={s['cb_ergonomics']}")
        print(f"{'=' * 65}")


if __name__ == "__main__":
    main()
