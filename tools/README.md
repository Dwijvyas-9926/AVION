# Tools

> **Remove the propellers** before running anything that arms the motors. Close Mission Planner / QGroundControl first, because only one program can own the Pixhawk serial port.

## `bench/`: hardware bring-up

Run these in order when setting up a new Pi or after changing hardware.

| Script | What it checks | Run |
|---|---|---|
| `cam_test.py` | Treats the D455 as a plain UVC webcam through OpenCV (no librealsense needed). This confirms the camera, cable, USB 3 port and power before you spend time building librealsense | `python3 cam_test.py` → `http://<pi-ip>:8000` |
| `depth_test.py` | Shows live RGB and colourised depth side by side. Click a pixel to read its depth; the page also shows the percentage of valid depth pixels | `python3 depth_test.py` → `http://<pi-ip>:8000` |
| `arm_test.py` | Configures the vehicle, arms in GUIDED_NOGPS or STABILIZE, ramps throttle to 15 % (capped at 30 %), holds and ramps down. Ctrl+C cuts the throttle and force-disarms | `python3 arm_test.py` |
| `hover_test.py` | Arms, takes off to 2 m, hovers for 5 s and lands, using DroneKit (originally `takeoff.py`) | `python3 hover_test.py` |

## `prototypes/`: early experiments

These are the standalone scripts the team used to prove each capability before it went into `flight_software/`. They are kept for reference and are not used by the mission code.

| Script | Original name | Purpose |
|---|---|---|
| `takeoff_land.py` | `Takeoff.py` | GUIDED → arm → takeoff → land |
| `takeoff_forward_land.py` | `takefwdlnd.py` | Takeoff, forward leg under velocity control, land |
| `yaw_auto.py` | `yawauto.py` | Takeoff to 5 m, forward 5 m, yaw 180°, forward 2 m, hold, land |
| `hook_servo_test.py` | `hook.py` | Opens and closes the payload hook servo on AUX1 (channel 9) |
| `payload_delivery.py` | `paydel2.py` | Locks yaw, takes off, flies forward 4 m, descends, drops the payload, flies on 2 m, hovers, flies back 6 m and lands |
| `image_capture.py` | `imgcap.py` | Captures frames from a Pi camera (Picamera2) to build a colour dataset |
