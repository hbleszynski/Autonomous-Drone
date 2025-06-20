from typing import Tuple
from math import atan2, sqrt, degrees, cos, sin, pi


class ComplementaryFilter:
    """Simple complementary filter for roll and pitch estimation."""

    def __init__(self, alpha: float = 0.98):
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update(
        self,
        accel: Tuple[float, float, float],
        gyro: Tuple[float, float, float],
        mag: Tuple[float, float, float],
        dt: float,
    ) -> Tuple[float, float, float]:
        """Update filter with new measurements.

        Args:
            accel: Acceleration (ax, ay, az) in m/s².
            gyro: Angular velocity (gx, gy, gz) in rad/s.
            mag: Magnetometer readings (mx, my, mz) in Gauss.
            dt: Time step in seconds.
        Returns:
            Tuple (roll, pitch, yaw) in degrees.
        """
        ax, ay, az = accel
        gx, gy, gz = gyro

        # Calculate roll and pitch from accelerometer (in radians)
        acc_roll = atan2(ay, az)
        acc_pitch = atan2(-ax, sqrt(ay * ay + az * az))

        # Integrate gyro rates to get angles (roll, pitch, yaw)
        self.roll += gx * dt
        self.pitch += gy * dt
        # We integrate yaw (gz) but will also correct with magnetometer
        yaw_rate = gz * dt
        try:
            self.yaw += yaw_rate
        except AttributeError:
            self.yaw = 0.0

        # Apply complementary filter on roll and pitch
        self.roll = self.alpha * self.roll + (1 - self.alpha) * acc_roll
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * acc_pitch

        # --- Yaw from magnetometer with tilt compensation ---
        mx, my, mz = mag
        # Tilt compensation
        # Reference: https://www.nxp.com/docs/en/application-note/AN4248.pdf
        # Instead compute using trig
        cos_roll = cos(self.roll)
        sin_roll = sin(self.roll)
        cos_pitch = cos(self.pitch)
        sin_pitch = sin(self.pitch)

        # Compensate
        mx_comp = mx * cos_pitch + mz * sin_pitch
        my_comp = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch
        mag_yaw = atan2(-my_comp, mx_comp)

        # Complementary blend yaw using gyro integration and magnetometer
        self.yaw = self.alpha * self.yaw + (1 - self.alpha) * mag_yaw

        # Normalise yaw to [-pi, pi] to avoid uncontrolled growth
        if self.yaw > pi:
            self.yaw -= 2 * pi
        elif self.yaw < -pi:
            self.yaw += 2 * pi

        return degrees(self.roll), degrees(self.pitch), degrees(self.yaw) 