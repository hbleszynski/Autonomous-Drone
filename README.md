# Autonomous Quadcopter Controller (Raspberry Pi)

This repository contains a **minimal** Python implementation of an on-board flight controller that can run on a Raspberry Pi and drive a quadcopter via four ESC-controlled brushless motors.  It demonstrates the integration of:

* MPU-9250 IMU (gyroscope, accelerometer & magnetometer)
* BMP280 barometric pressure sensor (altitude)
* pigpio PWM outputs to control ESCs
* Complementary filter for attitude estimation
* PID loops for stabilisation and altitude hold

> **Warning**  
> The gains and filter settings are **place-holders**.  You **must** tune them for your air-frame **before** attempting flight.

---

## Hardware

1. Raspberry Pi 3/4 (tested on Pi 4).  
2. 4 × brushless ESCs connected to GPIO pins (default pins: 17, 18, 27, 22 – edit `main.py` if different).  
3. 9-DOF IMU (MPU-9250) wired to the I²C bus.  
4. BMP280 barometer wired to the same I²C bus.  
5. A well-regulated power source for the Pi & electronics.


## Installation

```bash
# Install pigpio daemon
sudo apt update && sudo apt install -y pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Clone / copy this repo then inside the folder:
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

## Calibrate Sensors

Calibration is outside the scope of this demo but you will want to:

* Calibrate accelerometer and magnetometer biases.
* Level-trim the accelerometer.
* Measure hover throttle and set `throttle_base` in `flight_controller.py`.

## Running

```bash
python main.py
```

* Press `Ctrl-C` to stop the controller and disarm motors.

## File Structure

```text
├── actuators/
│   └── esc.py              # PWM ESC driver
├── controllers/
│   └── pid.py              # PID loop implementation
├── filters/
│   └── complementary_filter.py
├── sensors/
│   ├── barometer.py        # BMP280 driver
│   └── imu.py              # MPU-9250 wrapper
├── flight_controller.py    # High-level control logic
├── main.py                 # Entry-point script
└── requirements.txt
```

## Next Steps

* Replace the complementary filter with a **Kalman filter** or **Mahony/Madgwick** filter for better accuracy.
* Add **tilt-compensated** heading and full yaw control.
* Integrate **mission planning** (GPS, computer-vision, …).
* Implement automatic **failsafes** (low-battery, loss-of-signal).

## Autonomous Missions (Way-points)

`mission_runner.py` turns the Pi into a tiny **autopilot**. Supply a JSON file that contains an **array of way-points**; each element supports these fields:

| key        | type   | required | description                                              |
|------------|--------|----------|----------------------------------------------------------|
| `lat`      | float  | yes      | Latitude in decimal degrees (WGS-84).                   |
| `lon`      | float  | yes      | Longitude in decimal degrees.                           |
| `alt`      | float  | yes      | Target altitude **metres AMSL**.                        |
| `hold_time`| float  | no       | Seconds to hover once inside tolerance (default `0`).   |
| `tolerance`| float  | no       | Radius in metres that counts as "reached" (default `2`).|

Example `mission.json`:
```json
[
  {"lat": 51.501, "lon": -0.142, "alt": 10, "hold_time": 5},
  {"lat": 51.502, "lon": -0.141, "alt": 15, "tolerance": 3}
]
```

Run with only GPS/UDP tasks:
```bash
python mission_runner.py mission.json
```

Run with an HC-12 / LoRa / XBee radio attached to the Pi UART:
```bash
sudo python mission_runner.py mission.json --radio /dev/ttyAMA0 --baud 57600
```

### Adding way-points while airborne

Send JSON lines to UDP port **5005** (or over the serial radio) to extend the active mission:
```bash
# Via Wi-Fi / UDP
printf '{"command":"waypoint","lat":51.503,"lon":-0.140,"alt":20,"hold_time":3}\n' | nc -u -w0 <PI_IP> 5005

# Via radio (ensure same baud & newline termination)
echo '{"command":"waypoint","lat":51.504,"lon":-0.139,"alt":30}' > /dev/ttyUSB0
```

Other in-flight commands:
```json
{"command":"land"}   # Immediate landing – mission marked complete.
```

The autopilot merges new way-points into its queue thread-safely and continues navigating.

### Manual take-off / landing

You can test the altitude controller from a shell:

```bash
# Take off to 2.5 m, hover, hit Ctrl-C to stop, then land
python main.py --takeoff 2.5 --land
```

Or in missions, the autopilot now automatically performs a vertical take-off to the altitude of the first way-point and a smooth landing at the end. 

### In-flight task injection (dynamic way-points)

A background UDP listener (port 5005) now accepts JSON messages so you can add new goals while the vehicle is flying.

Example – add a waypoint 30 m north of the current location:

```bash
printf '{"command":"waypoint","lat":51.503,"lon":-0.140,"alt":20,"hold_time":3}' | nc -u -w0 <PI_IP> 5005
```

Send a land command:

```bash
printf '{"command":"land"}' | nc -u -w0 <PI_IP> 5005
```

The mission runner incorporates the new way-points into its queue on the fly and executes them in order. 

### Radio-based task injection

If you prefer an off-board transmitter (e.g., HC-12, LoRa, XBee) connected to the Pi's UART, start the mission runner with:

```bash
sudo python mission_runner.py mission.json --radio /dev/ttyAMA0 --baud 57600
```

Then send newline-terminated JSON over the air at the same baud rate, e.g. using another radio module attached to a laptop:

```bash
echo '{"command":"waypoint","lat":51.504,"lon":-0.139,"alt":30}' > /dev/ttyUSB0
```

Any JSON line received becomes a live task just like the UDP version. Ensure both radios are configured for the same settings. 