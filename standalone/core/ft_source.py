"""Force/Torque sensor source abstraction.

Provides a common interface for reading F/T data from different backends:
  - RTDEFTSource: real UR10e via ur_rtde (with bias correction)
  - NullFTSource: returns zeros (sim mode or no sensor)
"""

from typing import Protocol

import numpy as np


class FTSource(Protocol):
    """Protocol for F/T sensor data sources."""

    def get_wrench(self) -> np.ndarray:
        """Return [fx, fy, fz, tx, ty, tz] in sensor (tool) frame."""
        ...

    def zero_sensor(self) -> None:
        """Bias-correct by subtracting the current reading."""
        ...


class RTDEFTSource:
    """F/T readings from UR10e via ur_rtde with bias correction."""

    def __init__(self, backend):
        self._backend = backend
        self._bias = np.zeros(6)

    def get_wrench(self) -> np.ndarray:
        raw = np.array(self._backend.get_tcp_force())
        return raw - self._bias

    def zero_sensor(self) -> None:
        self._bias = np.array(self._backend.get_tcp_force())


class BaseFrameFTSource:
    """Wraps an FTSource and transforms wrench from TCP frame to base frame.

    Uses the robot backend's get_tcp_pose() to get the current TCP orientation,
    then rotates the wrench accordingly. Compatible with any backend that
    provides get_tcp_pose() → [x,y,z,rx,ry,rz] (axis-angle).
    """

    def __init__(self, source, backend):
        self._source = source
        self._backend = backend

    def get_wrench(self) -> np.ndarray:
        wrench_tcp = self._source.get_wrench()
        tcp_pose = self._backend.get_tcp_pose()
        R = rotvec_to_matrix(np.array(tcp_pose[3:6]))
        f_base = R @ wrench_tcp[:3]
        t_base = R @ wrench_tcp[3:]
        return np.concatenate([f_base, t_base])

    def zero_sensor(self) -> None:
        self._source.zero_sensor()


class NullFTSource:
    """Always returns zero wrench. Used in sim mode or when no sensor."""

    def get_wrench(self) -> np.ndarray:
        return np.zeros(6)

    def zero_sensor(self) -> None:
        pass


def rotvec_to_matrix(rotvec: np.ndarray) -> np.ndarray:
    """Convert rotation vector (axis-angle) to 3x3 rotation matrix (Rodrigues)."""
    angle = np.linalg.norm(rotvec)
    if angle < 1e-10:
        return np.eye(3)
    axis = rotvec / angle
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
