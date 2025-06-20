import time
from typing import Tuple, Optional

try:
    import gpsd  # type: ignore
except ImportError as e:
    raise ImportError(
        "gpsd-py3 package is required for GPS support. Install with 'pip install gpsd-py3'."
    )


def connect(host: str = "localhost", port: int = 2947):
    """Connect to gpsd daemon (call once at program start)."""
    gpsd.connect(host=host, port=port)


def get_position(timeout: float = 0.5) -> Optional[Tuple[float, float, float]]:
    """Return (latitude, longitude, altitude) from GPS.

    None is returned if no fix is currently available.
    """
    packet = gpsd.get_current()
    if packet.mode >= 2:  # 2D or 3D fix
        lat, lon = packet.lat, packet.lon
        alt = packet.alt if packet.mode == 3 else 0.0
        return lat, lon, alt
    return None 