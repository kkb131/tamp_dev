"""RTDE interface for impedance torque control via directTorque().

Communication architecture:
  - RTDEControlInterface: directTorque(), getCoriolisAndCentrifugalTorques(),
                          initPeriod()/waitPeriod() for servo loop timing
  - RTDEReceiveInterface: read robot state (actual_q, actual_qd, etc.)

Note: Previous implementation used setInputDoubleRegister() + custom URScript,
but RTDEControlInterface does NOT have setInputDoubleRegister (only RTDEIOInterface
has it, limited to registers [18-22] or [42-46]). This version computes PD torque
in Python and sends it directly via directTorque().
"""

import time
from typing import List, Optional

try:
    import rtde_control
    import rtde_receive
    RTDE_AVAILABLE = True
except ImportError:
    RTDE_AVAILABLE = False


_N_JOINTS = 6

# UR10e torque limits [Nm] per joint
TORQUE_LIMITS = [150.0, 150.0, 56.0, 56.0, 28.0, 28.0]


class URScriptManager:
    """Manages RTDE communication for impedance torque control.

    PD torque is computed Python-side and sent via directTorque().
    No URScript upload or RTDE input registers needed.
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

    def connect(self):
        """Connect RTDE interfaces for direct torque control."""
        print(f"[URScriptMgr] Connecting to {self._ip} @ {self._frequency}Hz...")
        self._recv = rtde_receive.RTDEReceiveInterface(self._ip)
        self._ctrl = rtde_control.RTDEControlInterface(self._ip, self._frequency)
        print("[URScriptMgr] RTDE connected (recv + ctrl)")

    def send_torque(self, torque: List[float], friction_comp: bool = True) -> bool:
        """Send joint torques via directTorque().

        Gravity compensation is handled automatically by the robot controller.
        """
        return self._ctrl.directTorque(torque, friction_comp)

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
        """Stop and disconnect all interfaces."""
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

        print("[URScriptMgr] Disconnected")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()
