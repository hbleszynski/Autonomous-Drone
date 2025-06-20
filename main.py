import time
import argparse
from flight_controller import FlightController


ESC_PINS = [17, 18, 27, 22]  # BCM numbering: FL, FR, BL, BR


def main():
    parser = argparse.ArgumentParser(description="Manual flight controller demo")
    parser.add_argument("--takeoff", type=float, help="Take off to TARGET_ALT metres, then hover")
    parser.add_argument("--land", action="store_true", help="Land and disarm after flight loop")
    args = parser.parse_args()

    fc = FlightController(ESC_PINS)
    fc.arm_escs()

    if args.takeoff:
        fc.simple_takeoff(args.takeoff)

    last_time = time.time()
    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now
            fc.update(dt)
            time.sleep(0.01)  # 100 Hz loop
    except KeyboardInterrupt:
        print("\nUser interrupt received. Shutting down…")
    finally:
        if args.land:
            fc.simple_land()
        else:
            fc.disarm()


if __name__ == "__main__":
    main() 