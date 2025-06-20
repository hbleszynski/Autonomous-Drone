import pigpio
import time


class ESC:
    """Electronic speed controller abstraction.

    Sends standard PWM pulses to control brushless motor speed.
    """

    def __init__(self, gpio_pin: int, min_pulse: int = 1000, max_pulse: int = 2000):
        self.gpio_pin = gpio_pin
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("Unable to connect to pigpio daemon. Did you run 'sudo pigpiod'? ")

        # Initialize pin for servo pulses (50 Hz = 20 ms period)
        self._pi.set_servo_pulsewidth(self.gpio_pin, 0)

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------
    def set_speed(self, throttle: float):
        """Set motor speed.

        Args:
            throttle: Value within 0.0 – 1.0.
        """
        throttle = max(0.0, min(1.0, throttle))
        pulse = self.min_pulse + (self.max_pulse - self.min_pulse) * throttle
        self._pi.set_servo_pulsewidth(self.gpio_pin, pulse)

    def stop(self):
        """Cut the throttle (send 0 pulse)."""
        self._pi.set_servo_pulsewidth(self.gpio_pin, 0)

    def __del__(self):
        # Ensure the motor is stopped
        try:
            self.stop()
            self._pi.stop()
        except Exception:
            pass 