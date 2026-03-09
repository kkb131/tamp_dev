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


class NullFTSource:
    """Always returns zero wrench. Used in sim mode or when no sensor."""

    def get_wrench(self) -> np.ndarray:
        return np.zeros(6)

    def zero_sensor(self) -> None:
        pass
