#!/usr/bin/env python3
# ============================================================
#  TerraAvis / Avion — test_hover.py
#  Standalone hover test.
#  Arm → Takeoff 2m → Hover 5s → Land
#
#  Run: python3 test_hover.py
# ============================================================

import time
from pymavlink import mavutil
from dronekit import connect, VehicleMode

# ── CONFIG ──────────────────────────────────────────────────
PORT             = "/dev/ttyACM0"
BAUD             = 57600
TAKEOFF_ALTITUDE = 2.0      # metres
HOVER_DURATION   = 5.0      # seconds
# ────────────────────────────────────────────────────────────


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}]  {msg}")


def connect_pixhawk():
    log(f"Connecting to Pixhawk on {PORT}...")
    vehicle = connect(PORT, baud=BAUD, wait_ready=True, timeout=30)
    log(f"Connected ✅")
    log(f"  Firmware : {vehicle.version}")
    log(f"  Mode     : {vehicle.mode.name}")
    log(f"  Armed    : {vehicle.armed}")
    log(f"  GPS Fix  : {vehicle.gps_0.fix_type}  Sats: {vehicle.gps_0.satellites_visible}")
    log(f"  EKF OK   : {vehicle.ekf_ok}")
    return vehicle


def arm_and_takeoff(vehicle, altitude):
    log("Setting GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    timeout = time.time() + 10
    while vehicle.mode.name != "GUIDED":
        if time.time() > timeout:
            log("ERROR: Could not set GUIDED mode.")
            return False
        time.sleep(0.2)
    log("GUIDED mode set ✅")

    log("Arming motors...")
    vehicle.armed = True
    timeout = time.time() + 15
    while not vehicle.armed:
        if time.time() > timeout:
            log("ERROR: Could not arm.")
            return False
        log(f"  Waiting to arm...")
        time.sleep(0.5)
    log("Armed ✅")

    time.sleep(1.0)

    log(f"Taking off to {altitude}m...")
    vehicle.simple_takeoff(altitude)

    timeout = time.time() + 30
    while True:
        alt = vehicle.location.global_relative_frame.alt or 0.0
        log(f"  Altitude: {alt:.2f}m / {altitude}m")
        if alt >= altitude * 0.90:
            log(f"Target altitude reached: {alt:.2f}m ✅")
            break
        if time.time() > timeout:
            log("WARNING: Takeoff timeout — continuing.")
            break
        time.sleep(0.5)

    return True


def hold_position(vehicle, duration):
    """Send SET_POSITION_TARGET_GLOBAL_INT to lock position + altitude."""
    log(f"Hovering for {duration}s...")

    loc      = vehicle.location.global_relative_frame
    hold_lat = loc.lat
    hold_lon = loc.lon

    end = time.time() + duration
    tick = 0
    while time.time() < end:
        remaining = end - time.time()
        if tick % 5 == 0:
            alt = vehicle.location.global_relative_frame.alt or 0.0
            log(f"  Hovering... alt={alt:.2f}m  remaining={remaining:.1f}s")

        # GPS position lock — prevents altitude drift
        msg = vehicle.message_factory.set_position_target_global_int_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(hold_lat * 1e7),
            int(hold_lon * 1e7),
            TAKEOFF_ALTITUDE,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        vehicle.send_mavlink(msg)

        time.sleep(0.2)
        tick += 1

    log("Hover complete ✅")


def land(vehicle):
    log("Landing...")
    vehicle.mode = VehicleMode("LAND")

    timeout = time.time() + 60
    while vehicle.location.global_relative_frame.alt > 0.25:
        alt = vehicle.location.global_relative_frame.alt or 0.0
        log(f"  Landing... alt={alt:.2f}m")
        if time.time() > timeout:
            log("WARNING: Landing timeout.")
            break
        time.sleep(0.5)

    time.sleep(2.0)
    vehicle.armed = False
    log("Landed and disarmed ✅")


def main():
    print()
    print("=" * 50)
    print("  TerraAvis — Hover Test")
    print(f"  Takeoff: {TAKEOFF_ALTITUDE}m  Hover: {HOVER_DURATION}s")
    print("=" * 50)
    print()

    vehicle = None
    try:
        vehicle = connect_pixhawk()

        ok = arm_and_takeoff(vehicle, TAKEOFF_ALTITUDE)
        if not ok:
            log("Takeoff failed — aborting.")
            return

        hold_position(vehicle, HOVER_DURATION)
        land(vehicle)

        print()
        print("=" * 50)
        print("  ✅  HOVER TEST COMPLETE")
        print("=" * 50)

    except KeyboardInterrupt:
        print()
        log("Keyboard interrupt — landing immediately.")
        if vehicle:
            vehicle.mode = VehicleMode("LAND")
            time.sleep(3)

    except Exception as e:
        log(f"ERROR: {e}")
        if vehicle and vehicle.armed:
            log("Drone still armed — landing for safety.")
            vehicle.mode = VehicleMode("LAND")
            time.sleep(3)

    finally:
        if vehicle:
            vehicle.close()
        log("Disconnected.")


if __name__ == "__main__":
    main()