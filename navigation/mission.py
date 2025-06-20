import json
from dataclasses import dataclass
from typing import List, Optional
from threading import Lock


@dataclass
class Waypoint:
    lat: float
    lon: float
    alt: float  # metres above sea level
    hold_time: float = 0.0  # seconds to hover at waypoint
    tolerance: float = 2.0  # metres


class Mission:
    """Holds sequence of waypoints loaded from a JSON file.

    JSON format example:
    [
        {"lat": 51.501, "lon": -0.142, "alt": 20, "hold_time": 5},
        {"lat": 51.502, "lon": -0.141, "alt": 25, "hold_time": 10}
    ]
    """

    def __init__(self, waypoints: List[Waypoint]):
        self._wps = waypoints
        self._index = 0
        self._start_hold: Optional[float] = None
        self._lock = Lock()

    @classmethod
    def from_file(cls, path: str) -> "Mission":
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        wps = [Waypoint(**wp) for wp in data]
        return cls(wps)

    # ----------------------------------------------------------------
    # Public helpers
    # ----------------------------------------------------------------
    @property
    def current(self) -> Optional[Waypoint]:
        with self._lock:
            if self._index < len(self._wps):
                return self._wps[self._index]
            return None

    def advance(self):
        with self._lock:
            self._index += 1
            self._start_hold = None

    def at_final_waypoint(self) -> bool:
        return self._index >= len(self._wps)

    def add_waypoint(self, wp: Waypoint):
        """Add a new waypoint to the mission (thread-safe)."""
        with self._lock:
            self._wps.append(wp)

    # ---------------------------- hold helpers -----------------------------
    def begin_hold(self, now: float):
        """Mark start of hover hold at current waypoint."""
        with self._lock:
            self._start_hold = now

    def hold_elapsed(self, now: float) -> float:
        """Return seconds since hold began; 0 if not in hold."""
        with self._lock:
            if self._start_hold is None:
                return 0.0
            return now - self._start_hold

    # ---------------------------- mission abort ----------------------------
    def complete(self):
        """Mark mission as complete (e.g., on land command)."""
        with self._lock:
            self._index = len(self._wps)

    # ---------------------------- diagnostics ------------------------------
    @property
    def index(self) -> int:
        """Return current waypoint index (0-based)."""
        with self._lock:
            return self._index 