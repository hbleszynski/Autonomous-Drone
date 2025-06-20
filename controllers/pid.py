from typing import Tuple, Optional


class PID:
    """Simple PID controller implementation."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        setpoint: float = 0.0,
        output_limits: Tuple[Optional[float], Optional[float]] = (None, None),
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._integral = 0.0
        self._prev_error = 0.0
        self._output_limits = output_limits

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0

    def update(self, measurement: float, dt: float) -> float:
        """Compute the PID output.

        Args:
            measurement: Current measurement value.
            dt: Time since last update in seconds.
        Returns:
            Control signal.
        """
        error = self.setpoint - measurement
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        low, high = self._output_limits
        if low is not None:
            output = max(low, output)
        if high is not None:
            output = min(high, output)
        return output

    # ------------------------------------------------------------------
    # Angle / wrap-safe helper
    # ------------------------------------------------------------------
    def update_with_error(self, error: float, dt: float) -> float:
        """Compute PID output when caller supplies the error directly.

        Keeps identical internal state to `update`, but skips setpoint-measurement.
        Useful for wrap-around values (e.g., headings) where caller already
        computed a minimal signed error (−180 … +180°).
        """
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        low, high = self._output_limits
        if low is not None:
            output = max(low, output)
        if high is not None:
            output = min(high, output)
        return output 