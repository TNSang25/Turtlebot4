# SPDX-License-Identifier: Apache-2.0
"""ROS-independent differential-drive command limits and deadman timeout."""

import math
from typing import Sequence, Tuple


def positive_finite(name: str, value: float) -> float:
    """Validate startup limits before opening command publishers."""
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f'{name} must be finite and greater than zero')
    return value


class VelocityFilter:
    """Keep the last valid planar command until its wall-time deadline."""

    def __init__(self, max_linear=0.31, max_angular=1.9, command_timeout=0.5):
        self.max_linear = positive_finite('max_linear', max_linear)
        self.max_angular = positive_finite('max_angular', max_angular)
        self.command_timeout = positive_finite('command_timeout', command_timeout)
        self.stop()

    def stop(self) -> None:
        self._command = (0.0, 0.0)
        self._last_received = None

    def receive(self, linear: Sequence[float], angular: Sequence[float], now: float) -> bool:
        """Reject a malformed Twist entirely; retain only linear.x/angular.z."""
        if (len(linear) != 3 or len(angular) != 3
                or not all(math.isfinite(v) for v in (*linear, *angular, now))):
            self.stop()
            return False
        self._command = (
            max(-self.max_linear, min(self.max_linear, linear[0])),
            max(-self.max_angular, min(self.max_angular, angular[2])),
        )
        self._last_received = now
        return True

    def sample(self, now: float) -> Tuple[float, float]:
        """Return zero before input, on timeout, or on an invalid clock value."""
        if self._last_received is None:
            return (0.0, 0.0)
        age = now - self._last_received
        if not math.isfinite(age) or age < 0.0 or age >= self.command_timeout:
            self.stop()
        return self._command
