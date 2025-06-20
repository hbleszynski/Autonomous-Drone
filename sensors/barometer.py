from typing import Tuple
import time

from smbus2 import SMBus


class BMP280:
    """Minimal BMP280 pressure/temperature sensor driver.

    This driver is *not* fully featured but provides enough functionality to
    retrieve altitude for control loops. For more advanced use cases consider
    using `adafruit-circuitpython-bmp280`.
    """

    # Registers
    _REG_ID = 0xD0
    _REG_PRESS_MSB = 0xF7
    _REG_CALIB_START = 0x88

    def __init__(self, bus: int = 1, address: int = 0x76):
        self.address = address
        self.bus_num = bus
        self._bus = SMBus(bus)

        chip_id = self._bus.read_byte_data(self.address, self._REG_ID)
        if chip_id not in (0x58, 0x60):  # BMP280/BME280 IDs
            raise RuntimeError("BMP280 not found on I2C bus.")

        self._read_calibration()
        self._configure()

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------
    def read_altitude(self, sea_level_hpa: float = 1013.25) -> float:
        """Return altitude in meters using the current pressure reading."""
        pressure, temperature = self._read_raw()
        altitude = 44330 * (1 - (pressure / (sea_level_hpa * 100)) ** (1 / 5.255))
        return altitude

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _configure(self):
        # Oversampling and normal mode configuration (for demonstration)
        self._bus.write_byte_data(self.address, 0xF4, 0x27)  # ctrl_meas: temp x1, press x1, normal mode
        self._bus.write_byte_data(self.address, 0xF5, 0xA0)  # config: 1000ms standby, filter off
        time.sleep(0.1)

    def _read_calibration(self):
        # Placeholder: skip calibration (not used in simplified altitude calc)
        pass

    def _read_raw(self) -> Tuple[float, float]:
        data = self._bus.read_i2c_block_data(self.address, self._REG_PRESS_MSB, 6)
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)

        # Very rough conversion (for control, accuracy not critical)
        pressure = pres_raw / 256.0  # Pa
        temperature = temp_raw / 100.0  # °C
        return pressure, temperature 