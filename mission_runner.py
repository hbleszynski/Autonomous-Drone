import argparse
import time
from math import radians, cos, sin, asin, sqrt, degrees, atan2
from typing import Optional

from flight_controller import FlightController
from navigation.mission import Mission, Waypoint
from navigation import gps
from navigation.task_receiver import TaskReceiver
from navigation.radio_task_receiver import RadioTaskReceiver
from queue import Queue


ESC_PINS = [17, 18, 27, 22]  # BCM numbering (same as main.py)


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return great-circle distance between two points (metres)."""
    R = 6371000  # Earth radius in metres
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat / 2) ** 2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) ** 2
    c = 2 * asin(sqrt(a))
    return R * c


def bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return initial bearing from point 1 to point 2 in degrees (0=N)."""
    lat1, lat2, dlon = radians(lat1), radians(lat2), radians(lon2 - lon1)
    x = sin(dlon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    brng = atan2(x, y)
    return (degrees(brng) + 360) % 360


# ---------------------------------------------------------------------
# Mission runner
# ---------------------------------------------------------------------

def run_mission(
    mission: Mission,
    enable_dynamic_tasks: bool = True,
    radio_device: Optional[str] = None,
    radio_baud: int = 57600,
):
    gps.connect()
    fc = FlightController(ESC_PINS)
    fc.arm_escs()
    # Take-off to altitude of first waypoint (or 2 m if no WP altitude specified)
    first_wp = mission.current
    init_alt = first_wp.alt if first_wp else 2.0
    fc.simple_takeoff(init_alt)

    task_queue = Queue()
    if enable_dynamic_tasks:
        # Start UDP listener always (for local debugging)
        TaskReceiver(task_queue).start()
        # Start radio listener if device specified
        if radio_device is not None:
            try:
                RadioTaskReceiver(task_queue, device=radio_device, baud=radio_baud).start()
            except RuntimeError as e:
                print(e)

    last_time = time.time()

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            pos = gps.get_position()
            if pos is None:
                print("Waiting for GPS fix…")
                time.sleep(1)
                continue
            lat, lon, current_alt = pos

            # Process dynamic tasks
            while not task_queue.empty():
                task = task_queue.get()
                if task.get("command") == "waypoint":
                    wp_obj = Waypoint(
                        lat=task["lat"],
                        lon=task["lon"],
                        alt=task.get("alt", fc.target_altitude),
                        hold_time=task.get("hold_time", 0),
                        tolerance=task.get("tolerance", 2),
                    )
                    mission.add_waypoint(wp_obj)
                    print("Added new waypoint to mission.")
                elif task.get("command") == "land":
                    print("Land command received – terminating mission.")
                    mission.complete()
                    break
                # Extend with more commands as needed

            wp = mission.current
            if wp is None:
                print("Mission complete – all waypoints reached.")
                break

            # Altitude target
            fc.target_altitude = wp.alt

            # Navigation to waypoint
            dist = haversine(lat, lon, wp.lat, wp.lon)
            if dist < wp.tolerance:
                # Within tolerance – begin/continue hover hold
                if mission.hold_elapsed(now) == 0:
                    mission.begin_hold(now)
                    print(f"Reached WP {mission.index}: begin hover for {wp.hold_time}s")
                elif mission.hold_elapsed(now) >= wp.hold_time:
                    print(f"Completed hold at WP {mission.index}. Advancing…")
                    mission.advance()
                    continue
            else:
                # Set heading towards waypoint
                hdg = bearing(lat, lon, wp.lat, wp.lon)
                fc.target_heading = hdg
                # TODO: forward pitch/velocity control for navigation.

            fc.update(dt)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nAbort – user interrupt.")
    finally:
        try:
            fc.simple_land()
        except Exception:
            fc.disarm()


# ---------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description="Run mission file on quadcopter.")
    parser.add_argument("mission", help="Path to JSON file containing waypoints")
    parser.add_argument("--radio", help="Serial device for radio receiver e.g. /dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=57600, help="Baud rate for radio serial port")
    args = parser.parse_args()

    mission = Mission.from_file(args.mission)
    run_mission(mission, radio_device=args.radio, radio_baud=args.baud)
    return


if __name__ == "__main__":
    main() 