"""URScript upload and RTDE register I/O for impedance torque control.

Communication architecture:
  - RTDEControlInterface: upload URScript + write input registers 0..23
  - RTDEReceiveInterface: read robot state (actual_q, actual_qd, etc.)

Note: RTDEIOInterface.setInputDoubleRegister() only supports registers 18-22.
      RTDEControlInterface.setInputDoubleRegister() supports the full range 0-23,
      so we use RTDEControlInterface for ALL register writes.

RTDE register allocation:
  Input  0..5:   q_desired[6]
  Input  6..11:  Kp[6]
  Input  12..17: Kd[6]
  Input  18:     mode (0=idle, 1=active, -1=stop)
  Input  19:     enable_coriolis (1=on, 0=off)
  Output 0..5:   applied_torque[6]
  Output 6:      heartbeat
"""

import time
from pathlib import Path
from typing import List, Optional

try:
    import rtde_control
    import rtde_receive
    RTDE_AVAILABLE = True
except ImportError:
    RTDE_AVAILABLE = False


_SCRIPT_PATH = Path(__file__).parent / "scripts" / "impedance_pd.script"

# Number of RTDE registers used
_N_JOINTS = 6
_REG_Q_DESIRED = 0       # input registers 0..5
_REG_KP = 6              # input registers 6..11
_REG_KD = 12             # input registers 12..17
_REG_MODE = 18            # input register 18
_REG_CORIOLIS = 19        # input register 19
_REG_TORQUE_OUT = 0       # output registers 0..5
_REG_HEARTBEAT = 6        # output register 6


class URScriptManager:
    """Manages URScript upload and RTDE register communication for impedance control."""

    def __init__(self, robot_ip: str):
        if not RTDE_AVAILABLE:
            raise ImportError(
                "ur_rtde is not installed. Install with: pip install ur-rtde"
            )
        self._ip = robot_ip
        self._recv: Optional[rtde_receive.RTDEReceiveInterface] = None
        self._ctrl: Optional[rtde_control.RTDEControlInterface] = None
        self._script_running = False
        self._last_heartbeat = 0.0

    def connect(self):
        """Connect RTDE interfaces.

        Creates RTDEControlInterface WITHOUT FLAG_CUSTOM_SCRIPT so it uploads
        the default external_control.urscript and establishes the RTDE link.
        Our custom impedance script is later sent via sendCustomScriptFile(),
        which replaces the running program while the RTDE transport stays alive.

        Note: FLAG_CUSTOM_SCRIPT requires an already-running RTDE program on
        the controller, causing a 60s timeout if nothing is running yet.
        """
        print(f"[URScriptMgr] Connecting to {self._ip}...")
        self._recv = rtde_receive.RTDEReceiveInterface(self._ip)

        # RTDEControlInterface supports setInputDoubleRegister(0..23)
        # RTDEIOInterface only supports 18-22 — so we use ctrl for everything
        # No FLAG_CUSTOM_SCRIPT: ur_rtde uploads default script first,
        # then we replace it with our impedance script via sendCustomScriptFile()
        self._ctrl = rtde_control.RTDEControlInterface(self._ip, 125.0)

        # Initialize all input registers to zero
        for i in range(20):
            self._ctrl.setInputDoubleRegister(i, 0.0)

        print("[URScriptMgr] RTDE connected (recv + ctrl)")

    def upload_and_start(self, script_path: Optional[str] = None):
        """Upload impedance URScript and start execution.

        This replaces the default external_control.urscript with our custom
        impedance PD controller. The script runs immediately after upload.
        """
        path = Path(script_path) if script_path else _SCRIPT_PATH
        if not path.exists():
            raise FileNotFoundError(f"URScript not found: {path}")

        print(f"[URScriptMgr] Uploading URScript: {path.name}")
        self._ctrl.setCustomScriptFile(str(path))
        self._ctrl.sendCustomScriptFile()

        self._script_running = True
        print("[URScriptMgr] URScript uploaded and started")

    def set_desired_position(self, q_desired: List[float]):
        """Write q_desired to RTDE input registers 0..5."""
        for i in range(_N_JOINTS):
            self._ctrl.setInputDoubleRegister(_REG_Q_DESIRED + i, q_desired[i])

    def set_gains(self, kp: List[float], kd: List[float]):
        """Write Kp/Kd to RTDE input registers 6..17."""
        for i in range(_N_JOINTS):
            self._ctrl.setInputDoubleRegister(_REG_KP + i, kp[i])
            self._ctrl.setInputDoubleRegister(_REG_KD + i, kd[i])

    def set_mode(self, mode: float):
        """Set mode register: 0=idle, 1=active, -1=stop."""
        self._ctrl.setInputDoubleRegister(_REG_MODE, mode)

    def set_coriolis_enabled(self, enabled: bool):
        """Enable/disable Coriolis compensation in URScript."""
        self._ctrl.setInputDoubleRegister(_REG_CORIOLIS, 1.0 if enabled else 0.0)

    def get_joint_positions(self) -> List[float]:
        """Read current joint positions via RTDE."""
        return list(self._recv.getActualQ())

    def get_joint_velocities(self) -> List[float]:
        """Read current joint velocities via RTDE."""
        return list(self._recv.getActualQd())

    def get_tcp_pose(self) -> List[float]:
        """Read current TCP pose [x,y,z,rx,ry,rz]."""
        return list(self._recv.getActualTCPPose())

    def get_applied_torques(self) -> List[float]:
        """Read applied torques from output registers 0..5."""
        return [
            self._recv.getOutputDoubleRegister(_REG_TORQUE_OUT + i)
            for i in range(_N_JOINTS)
        ]

    def get_heartbeat(self) -> float:
        """Read heartbeat counter from output register 6."""
        return self._recv.getOutputDoubleRegister(_REG_HEARTBEAT)

    def is_script_alive(self) -> bool:
        """Check if URScript is still running by monitoring heartbeat."""
        hb = self.get_heartbeat()
        alive = hb != self._last_heartbeat
        self._last_heartbeat = hb
        return alive

    def disconnect(self):
        """Stop URScript and disconnect all interfaces."""
        if self._ctrl is not None and self._script_running:
            try:
                self.set_mode(-1.0)
                time.sleep(0.1)  # let URScript exit gracefully
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

        self._script_running = False
        print("[URScriptMgr] Disconnected")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()
