import time
from typing import List

from sensors.imu import IMUSensor
from sensors.barometer import BMP280
from filters.complementary_filter import ComplementaryFilter
from controllers.pid import PID
from actuators.esc import ESC


class FlightController:
    """Combines sensor fusion and PID loops to stabilize a quadcopter."""

    def __init__(self, esc_pins: List[int]):
        if len(esc_pins) != 4:
            raise ValueError("Exactly four ESC GPIO pins required (quad configuration).")

        # Initialise hardware interfaces
        self._imu = IMUSensor()
        self._baro = BMP280()
        self._filter = ComplementaryFilter()

        # Create ESC objects
        self._escs = [ESC(pin) for pin in esc_pins]

        # PID controllers (gains need tuning!)
        self._pid_roll = PID(1.0, 0.0, 0.02, output_limits=(-0.3, 0.3))
        self._pid_pitch = PID(1.0, 0.0, 0.02, output_limits=(-0.3, 0.3))
        self._pid_yaw = PID(1.0, 0.0, 0.02, output_limits=(-0.3, 0.3))
        self._pid_altitude = PID(1.0, 0.0, 0.02, output_limits=(-0.3, 0.3))

        self.target_altitude = 1.0  # metres
        self.throttle_base = 0.5  # Base throttle (hover point) 0..1
        self.target_heading = 0.0  # degrees CW from magnetic north

    # ---------------------------------------------------------------
    # Public helpers
    # ---------------------------------------------------------------
    def arm_escs(self):
        print("Arming ESCs... Disengaged throttle for 3 seconds.")
        for esc in self._escs:
            esc.set_speed(0.0)
        time.sleep(3)
        print("ESCs armed.")

    def disarm(self):
        print("Disarming... Motors stopped.")
        for esc in self._escs:
            esc.stop()

    def update(self, dt: float):
        # 1. Read sensors
        accel, gyro, mag = self._imu.read()
        altitude = self._baro.read_altitude()

        # 2. Sensor fusion
        roll, pitch, yaw = self._filter.update(accel, gyro, mag, dt)

        # 3. PID controllers
        roll_correction = self._pid_roll.update(roll, dt)
        pitch_correction = self._pid_pitch.update(pitch, dt)
        # Heading error wrapped to [-180, 180] degrees
        heading_error = ((self.target_heading - yaw + 540) % 360) - 180
        yaw_correction = self._pid_yaw.update_with_error(heading_error, dt)
        self._pid_altitude.setpoint = self.target_altitude
        altitude_correction = self._pid_altitude.update(altitude, dt)

        # 4. Mix corrections into motor throttle
        # Standard quad X configuration (motors: FL, FR, BL, BR)
        m1 = self.throttle_base + altitude_correction + pitch_correction + yaw_correction
        m2 = self.throttle_base + altitude_correction - roll_correction - yaw_correction
        m3 = self.throttle_base + altitude_correction - pitch_correction + yaw_correction
        m4 = self.throttle_base + altitude_correction + roll_correction - yaw_correction

        throttles = [m1, m2, m3, m4]
        throttles = [max(0.0, min(1.0, t)) for t in throttles]

        # 5. Send to ESCs
        for esc, thr in zip(self._escs, throttles):
            esc.set_speed(thr)

        # 6. Debug output
        print(
            f"Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°, Alt={altitude:.2f} m | Thr={['{:.2f}'.format(t) for t in throttles]}"
        )

    def simple_takeoff(self, target_altitude: float, tolerance: float = 0.2, hold_time: float = 2.0):
        """Block until vehicle ascends to `target_altitude` and hovers for `hold_time` seconds."""
        print(f"Taking off to {target_altitude} m…")
        self.target_altitude = target_altitude
        last_time = time.time()
        stable_start = None
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now
            self.update(dt)
            alt = self._baro.read_altitude()
            if abs(alt - target_altitude) < tolerance:
                if stable_start is None:
                    stable_start = now
                elif now - stable_start >= hold_time:
                    print("Reached target altitude – take-off complete.")
                    return
            else:
                stable_start = None
            time.sleep(0.01)

    def simple_land(self, ground_threshold: float = 0.2):
        """Descend until barometer altitude < `ground_threshold` then disarm motors."""
        print("Landing…")
        self.target_altitude = 0.0
        last_time = time.time()
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now
            self.update(dt)
            alt = self._baro.read_altitude()
            if alt <= ground_threshold:
                print("Ground detected – disarming motors.")
                break
            time.sleep(0.01)
        self.disarm() 