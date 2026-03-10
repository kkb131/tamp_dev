"""RTDE interface for impedance torque control via custom URScript.

Communication architecture:
  - RTDEControlInterface: script upload, initPeriod()/waitPeriod(),
                          getCoriolisAndCentrifugalTorques()
  - RTDEReceiveInterface: read robot state (actual_q, actual_qd, etc.)
  - RTDEIOInterface (lower [18-22]): write torques + mode via mixed register types

Note: ur_rtde 1.6.2's directTorque() has a bug in its internal URScript
(Int type error in torqueThread). We bypass it by uploading our own URScript
that reads torque values from RTDE registers and calls direct_torque() directly.

Note: Only ONE RTDEIOInterface can be created per range. The upper range [42-46]
is often claimed by RTDEControlInterface or industrial protocols (EtherNet/IP,
PROFINET, MODBUS). We use the lower range with mixed types (double + int registers
are separate RTDE variables at the same index) to fit all 7 values in 5 indices.

Register allocation (all on lower range [18-22]):
  Double 18..22: tau[0..4]      (joint torques, Nm)
  Int 18:        tau[5] * 1000  (wrist3 millitorque, int32)
  Int 19:        mode           (0=idle, 1=active, -1=stop)
"""

import time
from pathlib import Path
from typing import List, Optional

try:
    import rtde_control
    import rtde_receive
    import rtde_io
    RTDE_AVAILABLE = True
except ImportError:
    RTDE_AVAILABLE = False


_SCRIPT_PATH = Path(__file__).parent / "scripts" / "impedance_pd.script"
_N_JOINTS = 6

# RTDE register mapping (all lower range [18-22])
_REG_TAU_LOWER = 18       # double registers 18..22 for tau[0..4]
_REG_TAU5_INT = 18        # integer register 18 for tau[5] (millitorque)
_REG_MODE_INT = 19        # integer register 19 for mode
_MILLITORQUE_SCALE = 1000 # tau[5] * 1000 → int32 (0.001 Nm precision)

# UR10e torque limits [Nm] per joint
TORQUE_LIMITS = [150.0, 150.0, 56.0, 56.0, 28.0, 28.0]


class URScriptManager:
    """Manages RTDE communication for impedance torque control.

    PD torque is computed Python-side. Computed torques are written to RTDE
    input registers via RTDEIOInterface. A custom URScript reads these
    registers and calls direct_torque() at 500Hz on the robot controller.
    """

    def __init__(self, robot_ip: str, frequency: float = 500.0):
        if not RTDE_AVAILABLE:
            raise ImportError(
                "ur_rtde is not installed. Install with: pip install ur-rtde"
            )
        self._ip = robot_ip
        self._frequency = frequency
        self._recv: Optional[rtde_receive.RTDEReceiveInterface] = None
        self._ctrl: Optional[rtde_control.RTDEControlInterface] = None
        self._io: Optional[rtde_io.RTDEIOInterface] = None

    def connect(self):
        """Connect RTDE interfaces and upload torque relay URScript."""
        print(f"[URScriptMgr] Connecting to {self._ip} @ {self._frequency}Hz...")

        # 1. Receive interface for robot state
        self._recv = rtde_receive.RTDEReceiveInterface(self._ip)

        # 2. Control interface (uploads default script, provides timing + dynamics)
        self._ctrl = rtde_control.RTDEControlInterface(self._ip, self._frequency)

        # 3. IO interface for register writes (lower range [18-22] only)
        #    Double 18-22: tau[0..4], Int 18: tau[5]*1000, Int 19: mode
        self._io = rtde_io.RTDEIOInterface(
            self._ip, use_upper_range_registers=False
        )

        # Initialize all registers to zero
        for i in range(18, 23):
            self._io.setInputDoubleRegister(i, 0.0)
        self._io.setInputIntRegister(_REG_TAU5_INT, 0)
        self._io.setInputIntRegister(_REG_MODE_INT, 0)

        print("[URScriptMgr] RTDE connected (recv + ctrl + io)")

        # 4. Replace default script with our torque relay script
        if not _SCRIPT_PATH.exists():
            raise FileNotFoundError(f"URScript not found: {_SCRIPT_PATH}")
        self._ctrl.sendCustomScriptFile(str(_SCRIPT_PATH))
        print(f"[URScriptMgr] URScript uploaded: {_SCRIPT_PATH.name}")

    def send_torque(self, torque: List[float]):
        """Write computed torques to RTDE input registers.

        tau[0..4] → double registers 18..22
        tau[5]    → integer register 18 (millitorque: value * 1000)
        """
        for i in range(5):
            self._io.setInputDoubleRegister(_REG_TAU_LOWER + i, torque[i])
        self._io.setInputIntRegister(
            _REG_TAU5_INT, int(torque[5] * _MILLITORQUE_SCALE)
        )

    def set_mode(self, mode: int):
        """Set mode register: 0=idle, 1=active, -1=stop."""
        self._io.setInputIntRegister(_REG_MODE_INT, mode)

    def get_coriolis(self, q: List[float], qd: List[float]) -> List[float]:
        """Get Coriolis and centrifugal torques for the given state."""
        return list(self._ctrl.getCoriolisAndCentrifugalTorques(q, qd))

    def get_joint_torques(self) -> List[float]:
        """Read current external joint torques (gravity/friction compensated)."""
        return list(self._ctrl.getJointTorques())

    def init_period(self):
        """Start a servo loop period for timing control."""
        return self._ctrl.initPeriod()

    def wait_period(self, t_start):
        """Wait until the end of the current servo period."""
        self._ctrl.waitPeriod(t_start)

    def get_joint_positions(self) -> List[float]:
        """Read current joint positions via RTDE."""
        return list(self._recv.getActualQ())

    def get_joint_velocities(self) -> List[float]:
        """Read current joint velocities via RTDE."""
        return list(self._recv.getActualQd())

    def get_tcp_pose(self) -> List[float]:
        """Read current TCP pose [x,y,z,rx,ry,rz]."""
        return list(self._recv.getActualTCPPose())

    def is_connected(self) -> bool:
        """Check if control interface is still connected."""
        if self._ctrl is None:
            return False
        return self._ctrl.isConnected()

    def disconnect(self):
        """Stop URScript and disconnect all interfaces."""
        # Signal URScript to stop
        if self._io is not None:
            try:
                self.set_mode(-1)
                time.sleep(0.1)
            except Exception:
                pass

        if self._ctrl is not None:
            try:
                self._ctrl.stopScript()
                self._ctrl.disconnect()
            except Exception:
                pass
            self._ctrl = None

        if self._recv is not None:
            try:
                self._recv.disconnect()
            except Exception:
                pass
            self._recv = None

        if self._io is not None:
            try:
                self._io.disconnect()
            except Exception:
                pass
            self._io = None

        print("[URScriptMgr] Disconnected")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()
