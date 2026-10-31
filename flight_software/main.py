#!/usr/bin/env python3
# ============================================================
#  TerraAvis — main.py
#  Entry point. Run this on the Raspberry Pi 4.
#
#  Usage:
#    python3 main.py
# ============================================================

import sys
import time
import os
from logger import log
from flight_controller import FlightController
from vision import VisionSystem
from mission_manager import MissionManager
from rover import RoverController
from gui import GUI


def set_cpu_governor():
    """Set CPU to powersave to reduce heat and power draw."""
    try:
        os.system("echo powersave | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null")
        log.info("CPU governor set to powersave.")
    except:
        pass

def check_temperature():
    """Log RPi CPU temperature on startup."""
    try:
        temp = os.popen("vcgencmd measure_temp").readline()
        log.info(f"CPU Temperature: {temp.strip()}")
        # Warn if already hot before starting
        val = float(temp.replace("temp=","").replace("'C\n",""))
        if val > 70:
            log.warning(f"RPi is already hot ({val}°C) — consider adding heatsink/fan!")
    except:
        pass


def main():
    log.info("=" * 50)
    log.info("  TerraAvis — Autonomous Mission System")
    log.info("  RoboFest National Competition")
    log.info("=" * 50)

    # Reduce CPU load and check thermals
    set_cpu_governor()
    check_temperature()

    fc      = FlightController()
    vision  = VisionSystem()
    mission = None

    try:
        # --- Connect Pixhawk ---
        pixhawk_ok = fc.connect()
        if not pixhawk_ok:
            log.error("Pixhawk not connected.")
            log.warning("Starting in CAMERA-ONLY mode. Mission START will be blocked.")

        # --- Start Camera ---
        if not vision.start():
            log.error("RealSense D455 failed to start. Cannot run mission.")
            sys.exit(1)

        # --- Build mission, rover + GUI ---
        mission = MissionManager(fc, vision)
        rover   = RoverController(fc)
        rover.start()
        gui     = GUI(mission, vision, fc, rover)

        log.info("-" * 50)
        log.info("System ready.")
        log.info("  Press [S] in the window to START mission.")
        log.info("  Press [A] to ABORT at any time.")
        log.info("  Press [Q] to quit.")
        log.info("-" * 50)

        # GUI loop — blocks until Q is pressed
        gui.run()

    except KeyboardInterrupt:
        log.info("Keyboard interrupt — shutting down.")

    finally:
        log.info("Shutdown sequence...")
        if mission and mission.state not in (
            __import__('mission_manager').MissionState.IDLE,
            __import__('mission_manager').MissionState.COMPLETE
        ):
            if fc.armed:
                log.warning("Drone armed on exit — landing in place.")
                fc.land()
                time.sleep(2)

        vision.stop()
        fc.disconnect()
        try:
            rover.shutdown()
        except:
            pass
        log.info("TerraAvis shutdown complete.")


if __name__ == "__main__":
    main()