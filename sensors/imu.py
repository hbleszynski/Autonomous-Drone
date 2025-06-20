import time
from typing import Tuple

try:
    from mpu9250_jmdev.registers import AK8963_ADDRESS, MPU9250_ADDRESS_68
    from mpu9250_jmdev.mpu_9250 import MPU9250
except ImportError:
    raise ImportError(
        "mpu9250_jmdev library is required. Install with 'pip install mpu9250-jmdev'."
    )


class IMUSensor:
    """High-level wrapper around the MPU-9250 9-DOF IMU.

    Provides fused gyroscope, accelerometer and magnetometer data in SI units.
    """

    def __init__(self, bus: int = 1, address: int = MPU9250_ADDRESS_68):
        # Power on delay to make sure the IMU is ready
        time.sleep(0.5)
        self._mpu = MPU9250(
            address=address,
            magnetometer_address=AK8963_ADDRESS,
            bus=bus,
            gfs=0x00,  # Gyro full-scale ±250 dps
            afs=0x00,  # Acc full-scale ±2 g
            mfs=0x16,  # Mag full-scale ±4900 µT
            mode=0x06,  # Continuous measurement mode 2
        )

        self._mpu.configure()  # Apply the settings.

    # ---------------------------------------------------------------------
    # Public helpers
    # ---------------------------------------------------------------------
    def read(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
        """Return (accel, gyro, mag) tuples, each with (x, y, z) in SI units.

        Acceleration is in m/s², gyro in rad/s, magnetometer in µT.
        """
        ax, ay, az = self._mpu.readAccelerometerMaster()
        gx, gy, gz = self._mpu.readGyroscopeMaster()
        mx, my, mz = self._mpu.readMagnetometerMaster()

        # Convert units: Acc in g to m/s² (1 g ≈ 9.80665 m/s²),
        # Gyro in deg/s to rad/s.
        ax, ay, az = [a * 9.80665 for a in (ax, ay, az)]
        gx, gy, gz = [g * 3.141592653589793 / 180.0 for g in (gx, gy, gz)]

        return (ax, ay, az), (gx, gy, gz), (mx, my, mz) 