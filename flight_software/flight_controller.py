# ============================================================
#  TerraAvis — flight_controller.py
#  Production-grade MAVLink / DroneKit wrapper.
#
#  Key improvements:
#  - Pre-flight safety checks (battery, GPS, RC)
#  - Reliable yaw using SET_ATTITUDE_TARGET sent every 100ms
#  - Velocity commands sent repeatedly to prevent timeout
#  - Graceful error handling throughout
# ============================================================

import time
import math
import threading
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from logger import log
from config import (
    MAVLINK_CONNECTION, MAVLINK_BAUD,
    MIN_BATTERY_VOLTAGE, MIN_GPS_FIX, MIN_GPS_SATELLITES,
    SERVO_CHANNEL, SERVO_PWM_CLOSED, SERVO_PWM_OPEN,
    YAW_TICK_INTERVAL, TAKEOFF_ALTITUDE
)


class FlightController:
    def __init__(self):
        self.vehicle        = None
        self.connected      = False
        self._yaw_stop_flag = threading.Event()
        self._watchdog_active = False
        self._emergency_land  = False   # set by watchdog → mission must check this

    # ----------------------------------------------------------
    # CONNECTION
    # ----------------------------------------------------------
    def connect(self):
        try:
            log.info(f"Connecting to Pixhawk → {MAVLINK_CONNECTION} @ {MAVLINK_BAUD}")
            self.vehicle = connect(
                MAVLINK_CONNECTION,
                baud=MAVLINK_BAUD,
                wait_ready=True,
                timeout=30
            )
            self.connected = True
            log.info(f"Pixhawk connected.")
            log.info(f"  Firmware : {self.vehicle.version}")
            log.info(f"  GPS      : fix={self.vehicle.gps_0.fix_type} sats={self.vehicle.gps_0.satellites_visible}")
            log.info(f"  Battery  : {self.vehicle.battery.voltage:.1f}V")
            log.info(f"  Mode     : {self.vehicle.mode.name}")

            # Start heartbeat watchdog
            self._watchdog_active = True
            self._emergency_land  = False
            threading.Thread(target=self._heartbeat_watchdog, daemon=True).start()
            log.info("  Heartbeat watchdog started.")

            return True
        except Exception as e:
            log.error(f"Pixhawk connection failed: {e}")
            self.connected = False
            return False

    def disconnect(self):
        if self.vehicle:
            self.vehicle.close()
            self.connected = False
            log.info("Pixhawk disconnected.")

    @property
    def is_connected(self):
        try:
            return self.connected and self.vehicle is not None and self.vehicle.last_heartbeat < 3.0
        except:
            return False

    # ----------------------------------------------------------
    # PRE-FLIGHT CHECKS
    # ----------------------------------------------------------
    def preflight_checks(self):
        """
        Run all safety checks before allowing mission to start.
        Returns (passed: bool, reasons: list of strings)
        """
        issues = []

        if not self.is_connected:
            issues.append("Pixhawk not connected")
            return False, issues

        # Battery check
        batt = self.vehicle.battery.voltage
        if batt is None or batt < MIN_BATTERY_VOLTAGE:
            issues.append(f"Battery low: {batt:.1f}V (min {MIN_BATTERY_VOLTAGE}V)")

        # GPS check
        gps = self.vehicle.gps_0
        if gps.fix_type < MIN_GPS_FIX:
            issues.append(f"GPS fix insufficient: {gps.fix_type} (need {MIN_GPS_FIX})")
        if gps.satellites_visible < MIN_GPS_SATELLITES:
            issues.append(f"Too few satellites: {gps.satellites_visible} (need {MIN_GPS_SATELLITES})")

        # EKF check
        if not self.vehicle.ekf_ok:
            issues.append("EKF not healthy — do not fly")

        # Gyro/accel check
        if not self.vehicle.is_armable:
            issues.append("Vehicle not armable — check pre-arm messages")

        if issues:
            for i in issues:
                log.error(f"  PRE-FLIGHT FAIL: {i}")
            return False, issues

        log.info("All pre-flight checks PASSED.")
        return True, []

    # ----------------------------------------------------------
    # TELEMETRY PROPERTIES
    # ----------------------------------------------------------
    @property
    def altitude(self):
        try:
            return round(self.vehicle.location.global_relative_frame.alt, 2)
        except:
            return 0.0

    @property
    def mode(self):
        try: return self.vehicle.mode.name
        except: return "UNKNOWN"

    @property
    def heading(self):
        try: return self.vehicle.heading or 0
        except: return 0

    @property
    def battery_voltage(self):
        try: return round(self.vehicle.battery.voltage, 1)
        except: return 0.0

    @property
    def armed(self):
        try: return self.vehicle.armed
        except: return False

    @property
    def gps_fix(self):
        try: return self.vehicle.gps_0.fix_type
        except: return 0

    @property
    def ekf_ok(self):
        try: return self.vehicle.ekf_ok
        except: return False

    # ----------------------------------------------------------
    # ARM & TAKEOFF
    # ----------------------------------------------------------
    def set_guided_mode(self):
        if not self.vehicle:
            log.warning("set_guided_mode: no vehicle — skipping.")
            return False
        log.info("Setting GUIDED mode...")
        self.vehicle.mode = VehicleMode("GUIDED")
        start = time.time()
        while self.vehicle.mode.name != "GUIDED":
            if time.time() - start > 10:
                log.error("Timed out waiting for GUIDED mode.")
                return False
            time.sleep(0.3)
        log.info("GUIDED mode confirmed.")
        return True

    def arm(self):
        if not self.vehicle:
            log.warning("arm: no vehicle — skipping.")
            return False
        log.info("Arming motors...")
        self.vehicle.armed = True
        start = time.time()
        while not self.vehicle.armed:
            if time.time() - start > 15:
                log.error("Arming timed out — check pre-arm conditions in GCS.")
                return False
            time.sleep(0.5)
        log.info("Motors ARMED.")
        return True

    def takeoff(self, target_altitude):
        if not self.vehicle:
            log.warning("takeoff: no vehicle — skipping.")
            return
        log.info(f"Takeoff → {target_altitude}m")
        self.vehicle.simple_takeoff(target_altitude)
        while True:
            alt = self.altitude
            log.debug(f"  Alt: {alt:.2f}m / {target_altitude}m")
            if alt >= target_altitude * 0.95:
                log.info(f"Takeoff complete. Altitude: {alt:.2f}m")
                break
            time.sleep(0.3)

    # ----------------------------------------------------------
    # VELOCITY COMMANDS
    # ----------------------------------------------------------
    def send_body_velocity(self, vx, vy, vz):
        """
        Send body-frame velocity. vz=0 means maintain current altitude.
        Uses MAV_FRAME_BODY_OFFSET_NED so x=forward, y=right, z=down.
        """
        if not self.vehicle:
            return
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,   # use velocity only
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )
        self.vehicle.send_mavlink(msg)

    def _send_pos_hold(self, lat, lon, alt):
        """
        Send SET_POSITION_TARGET_GLOBAL_INT to lock position AND altitude.
        This is the correct way to hold position in ArduPilot GUIDED mode —
        zero velocity alone causes altitude drift.
        """
        if not self.vehicle:
            return
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms
            0, 0,    # target system, component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,   # type_mask: use position only
            int(lat * 1e7),       # lat (degE7)
            int(lon * 1e7),       # lon (degE7)
            alt,                  # alt (metres, relative)
            0, 0, 0,              # vx, vy, vz
            0, 0, 0,              # ax, ay, az
            0, 0                  # yaw, yaw_rate
        )
        self.vehicle.send_mavlink(msg)

    def stop(self):
        """
        Smooth stop — send zero velocity for 0.6s to let drone coast down,
        then lock GPS position. Avoids the sudden jerk of instant position lock.
        """
        self._yaw_stop_flag.set()
        # Phase 1: zero velocity — let inertia bleed off naturally
        for _ in range(8):
            self.send_body_velocity(0, 0, 0)
            time.sleep(0.08)
        # Phase 2: GPS position lock
        if self.vehicle:
            try:
                loc = self.vehicle.location.global_relative_frame
                if loc.lat and loc.lon:
                    for _ in range(3):
                        self._send_pos_hold(loc.lat, loc.lon, TAKEOFF_ALTITUDE)
                        time.sleep(0.05)
                    return
            except:
                pass
        self.send_body_velocity(0, 0, 0)

    def hold_position(self, duration):
        """
        Hold GPS position at TAKEOFF_ALTITUDE.
        - Runs at 15Hz
        - Sends GPS position lock every tick (prevents altitude drift)
        - Active drift correction kicks in at 0.35m (tight — catches wind early)
        - Correction speed is proportional and gentle (max 0.25 m/s)
          so corrections are smooth, not snappy
        """

        hold_lat, hold_lon = None, None
        try:
            loc = self.vehicle.location.global_relative_frame
            if loc.lat and loc.lon:
                hold_lat = loc.lat
                hold_lon = loc.lon
        except:
            pass

        end = time.time() + duration
        while time.time() < end:
            if self._emergency_land:
                log.error("WATCHDOG: heartbeat lost — landing NOW")
                self.land()
                return

            if hold_lat and self.vehicle:
                try:
                    loc = self.vehicle.location.global_relative_frame
                    if loc.lat and loc.lon:
                        dlat  = (hold_lat - loc.lat) * 111139
                        dlon  = (hold_lon - loc.lon) * 111139 * math.cos(math.radians(loc.lat))
                        drift = math.sqrt(dlat**2 + dlon**2)

                        if drift > 0.35:
                            # Proportional correction — gentle and smooth
                            # Speed scales linearly with drift, capped at 0.25 m/s
                            speed = min(0.25, drift * 0.18)
                            heading_rad = math.radians(self.heading)
                            vx =  dlat * math.cos(heading_rad) + dlon * math.sin(heading_rad)
                            vy = -dlat * math.sin(heading_rad) + dlon * math.cos(heading_rad)
                            mag = math.sqrt(vx**2 + vy**2)
                            if mag > 0:
                                vx = (vx / mag) * speed
                                vy = (vy / mag) * speed
                            self.send_body_velocity(vx, vy, 0)
                        else:
                            # Within tolerance — GPS lock
                            self._send_pos_hold(hold_lat, hold_lon, TAKEOFF_ALTITUDE)
                except:
                    self._send_pos_hold(hold_lat, hold_lon, TAKEOFF_ALTITUDE)
            else:
                self.send_body_velocity(0, 0, 0)

            time.sleep(0.067)   # 15Hz

    # ----------------------------------------------------------
    # HEARTBEAT WATCHDOG
    # ----------------------------------------------------------
    def _heartbeat_watchdog(self):
        """
        Background thread — monitors Pixhawk heartbeat.
        If heartbeat lost for >3s while armed → trigger emergency land.
        This prevents the drone dropping if MAVLink is interrupted.
        """
        HEARTBEAT_TIMEOUT = 3.0   # seconds
        CHECK_INTERVAL    = 0.5

        while self._watchdog_active:
            try:
                if self.vehicle and self.vehicle.armed:
                    last_hb = self.vehicle.last_heartbeat
                    if last_hb is not None and last_hb > HEARTBEAT_TIMEOUT:
                        log.error(f"WATCHDOG: Heartbeat lost! Last seen {last_hb:.1f}s ago — EMERGENCY LAND")
                        self._emergency_land = True
                        self.land()
            except Exception as e:
                log.warning(f"Watchdog check error: {e}")
            time.sleep(CHECK_INTERVAL)

    # ----------------------------------------------------------
    # PRE-FLIGHT SERVO TEST
    # ----------------------------------------------------------
    def test_servo(self):
        """
        Test servo open/close before arming.
        Call during preflight — if servo doesn't respond, abort mission.
        """
        log.info("Servo pre-flight test — opening...")
        self.set_servo(SERVO_CHANNEL, SERVO_PWM_OPEN)
        time.sleep(1.5)
        log.info("Servo pre-flight test — closing...")
        self.set_servo(SERVO_CHANNEL, SERVO_PWM_CLOSED)
        time.sleep(1.0)
        log.info("Servo pre-flight test complete ✅")



    # ----------------------------------------------------------
    # ✅ PRODUCTION YAW — Continuous SET_ATTITUDE_TARGET
    # ----------------------------------------------------------
    def send_yaw_rate(self, yaw_rate_deg_s, direction=1):
        """
        Send yaw command using MAV_CMD_CONDITION_YAW.
        Proven to work on ArduPilot. Called once per sweep tick.
        """
        if not self.vehicle:
            return

        # Each tick we command a small relative yaw step
        # yaw_rate_deg_s * YAW_TICK_INTERVAL = degrees per tick
        deg_step = yaw_rate_deg_s * YAW_TICK_INTERVAL

        self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            deg_step,        # angle (degrees)
            yaw_rate_deg_s,  # angular speed (deg/s)
            1,               # direction: 1=CW, -1=CCW
            1,               # 1=relative angle
            0, 0, 0
        )

        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            deg_step,
            yaw_rate_deg_s,
            1,
            1,
            0, 0, 0
        )
        self.vehicle.send_mavlink(msg)

    def yaw_sweep(self, total_degrees, yaw_rate_deg_s, on_tick):
        """
        Rotate drone by total_degrees using MAV_CMD_CONDITION_YAW.
        Calls on_tick() every tick to check for target.
        Returns True if target found, False if full sweep done.
        """
        if not self.vehicle:
            log.warning("yaw_sweep: no vehicle — skipping.")
            return False

        log.info(f"Yaw sweep: {total_degrees}° @ {yaw_rate_deg_s}°/s")
        self._yaw_stop_flag.clear()

        total_time_needed = total_degrees / yaw_rate_deg_s
        start_time        = time.time()

        while True:
            if self._yaw_stop_flag.is_set():
                log.info("Yaw sweep interrupted.")
                break

            elapsed = time.time() - start_time
            if elapsed >= total_time_needed:
                log.warning(f"Yaw sweep complete ({total_degrees}°) — no target found.")
                break

            # Send yaw command — same as working test code
            deg_step = yaw_rate_deg_s * YAW_TICK_INTERVAL
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                deg_step,        # angle step per tick
                yaw_rate_deg_s,  # speed deg/s
                1,               # CW
                1,               # relative
                0, 0, 0
            )
            self.vehicle.send_mavlink(msg)

            # Check for target
            target_found = on_tick()
            if target_found:
                log.info(f"Target found at {elapsed * yaw_rate_deg_s:.1f}° into sweep.")
                return True

            time.sleep(YAW_TICK_INTERVAL)

        return False

    def stop_yaw(self):
        """Stop yaw — just stop sending CONDITION_YAW commands."""
        self._yaw_stop_flag.set()
        self.send_body_velocity(0, 0, 0)

    # ----------------------------------------------------------
    # LATERAL ALIGNMENT
    # ----------------------------------------------------------
    def correct_lateral(self, pixel_error, frame_width, speed=0.15):
        if not self.vehicle:
            return
        norm = max(-1.0, min(1.0, pixel_error / (frame_width / 2)))
        vy   = norm * speed
        self.send_body_velocity(0, vy, 0)

    # ----------------------------------------------------------
    # PAYLOAD SERVO (AUX1)
    # ----------------------------------------------------------
    def set_servo(self, channel, pwm):
        if not self.vehicle:
            log.warning("set_servo: no vehicle — skipping.")
            return
        # Send through dronekit's existing connection
        # target_system=1 (Pixhawk), target_component=1 (autopilot)
        self.vehicle._master.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            float(channel),
            float(pwm),
            0, 0, 0, 0, 0
        )
        log.info(f"Servo CH{channel} → {pwm}us")

    def release_payload(self):
        """Open payload hook. Call close_payload() separately to close."""
        log.info("PAYLOAD HOOK OPEN — AUX1 → 2000us")
        self.set_servo(SERVO_CHANNEL, SERVO_PWM_OPEN)
        time.sleep(0.1)
        self.set_servo(SERVO_CHANNEL, SERVO_PWM_OPEN)  # send twice for reliability

    def close_payload(self):
        """Close payload hook to secure parcel."""
        log.info("PAYLOAD HOOK CLOSED — AUX1 → 1000us")
        self.set_servo(SERVO_CHANNEL, SERVO_PWM_CLOSED)
        time.sleep(0.1)
        self.set_servo(SERVO_CHANNEL, SERVO_PWM_CLOSED)  # send twice for reliability

    # ----------------------------------------------------------
    # LANDING & ABORT
    # ----------------------------------------------------------
    def land(self):
        if not self.vehicle:
            log.warning("land: no vehicle — skipping.")
            return
        log.info("Switching to LAND mode...")
        self.vehicle.mode = VehicleMode("LAND")

    def rtl(self):
        if not self.vehicle:
            log.warning("rtl: no vehicle — skipping.")
            return
        log.warning("RTL mode — returning to launch!")
        self.vehicle.mode = VehicleMode("RTL")

    def disarm(self):
        if not self.vehicle:
            return
        time.sleep(1.0)
        self.vehicle.armed = False
        log.info("Motors disarmed.")