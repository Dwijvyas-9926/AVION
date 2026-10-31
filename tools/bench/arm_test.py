#!/usr/bin/env python3
"""
Raspberry Pi -> Pixhawk over USB (pymavlink)

Flow: configure -> mode -> arm -> ramp throttle 0 -> TARGET % -> hold -> ramp down -> disarm

  * STABILIZE      : throttle is sent as an RC override on channel 3 (no RC needed)
  * GUIDED_NOGPS   : throttle is sent as SET_ATTITUDE_TARGET thrust (level, current yaw)

!!! REMOVE PROPELLERS BEFORE RUNNING - MOTORS WILL SPIN !!!
Close Mission Planner / QGroundControl first.
Press Ctrl+C at any time: throttle drops to zero and the vehicle is force-disarmed.
"""

import glob
import math
import sys
import time
from pymavlink import mavutil

# ------------------------- CONFIG -------------------------
PORT = None                 # None = auto-detect, or e.g. "/dev/ttyACM0"
BAUD = 115200
TARGET_THROTTLE_PCT = 15    # final throttle in percent
RAMP_UP_TIME = 10.0         # seconds to go from 0 to target
HOLD_TIME = 3.0             # seconds to hold at target
RAMP_DOWN_TIME = 3.0        # seconds to go back to 0
MODES_TO_TRY = ["GUIDED_NOGPS", "STABILIZE"]   # first one that arms is used
FORCE_ARM = True
NO_GPS_MODE = True          # EKF3 baro+compass only (already set if you ran arm_test.py)
SEND_RATE_HZ = 20           # control command rate (must be >= ~10 Hz)
MAX_ALLOWED_PCT = 30        # safety cap
# ----------------------------------------------------------

FORCE_MAGIC = 21196
ARM_CMD = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
GUIDED_MODES = ("GUIDED", "GUIDED_NOGPS")


# ---------------------- helpers ----------------------
def find_port():
    ports = sorted(glob.glob("/dev/serial/by-id/*")) or sorted(glob.glob("/dev/ttyACM*"))
    if not ports:
        print("[!] No Pixhawk USB device found (ls /dev/ttyACM*).")
        sys.exit(1)
    return ports[0]


def send_heartbeat(master):
    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                              mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def show_status(msg):
    print(f"\n    [FC] {msg.text}")


def pump(master, duration):
    t0 = time.time()
    while time.time() - t0 < duration:
        msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=0.2)
        if msg:
            show_status(msg)


def is_autopilot_hb(master, hb):
    return (hb.get_srcSystem() == master.target_system
            and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID)


def wait_armed_state(master, want_armed, timeout=6):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = master.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=1)
        if msg is None:
            send_heartbeat(master)
            continue
        if msg.get_type() == "STATUSTEXT":
            show_status(msg)
        elif is_autopilot_hb(master, msg):
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed == want_armed:
                return True
    return False


def wait_ack(master, command, timeout=5):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = master.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            show_status(msg)
        elif msg.command == command:
            return msg.result
    return None


def _pid(msg):
    pid = msg.param_id
    if isinstance(pid, bytes):
        pid = pid.decode(errors="ignore")
    return pid.strip("\x00")


def get_param(master, name, timeout=2.0):
    master.mav.param_request_read_send(master.target_system, master.target_component,
                                       name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is not None and _pid(msg) == name:
            return msg.param_value
    return None


def set_param(master, name, value, timeout=2.0):
    """True = set & confirmed, False = mismatch, None = param not found."""
    for _ in range(3):
        master.mav.param_set_send(master.target_system, master.target_component,
                                  name.encode(), float(value),
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = master.recv_match(type=["PARAM_VALUE", "STATUSTEXT"], blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                show_status(msg)
            elif _pid(msg) == name:
                return abs(msg.param_value - float(value)) < 1e-3
    return None


def set_param_if_changed(master, name, value):
    """Returns True if the value had to be changed (reboot needed for EK3 params)."""
    cur = get_param(master, name)
    if cur is None:
        print(f"    {name}: not found (skipped)")
        return False
    if abs(cur - value) < 1e-3:
        print(f"    {name} = {value} (already set)")
        return False
    r = set_param(master, name, value)
    print(f"    {name} = {value} -> {'OK' if r else 'FAILED'}")
    return bool(r)


def configure(master):
    print("[*] Configuring parameters (no RC connected) ...")
    needs_reboot = False

    if set_param(master, "ARMING_CHECK", 0) is None:
        set_param(master, "ARMING_SKIPCHK", -1)
    for name, val in (("FS_THR_ENABLE", 0), ("FS_GCS_ENABLE", 0)):
        set_param(master, name, val)

    if NO_GPS_MODE:
        for name, val in (("EK3_SRC1_POSXY", 0), ("EK3_SRC1_VELXY", 0),
                          ("EK3_SRC1_POSZ", 1), ("EK3_SRC1_VELZ", 0),
                          ("EK3_SRC1_YAW", 1)):
            needs_reboot |= set_param_if_changed(master, name, val)
    return needs_reboot


def set_mode(master, mode):
    mapping = master.mode_mapping()
    if mode not in mapping:
        print(f"[!] Mode {mode} not available on this firmware")
        return False
    mode_id = mapping[mode]
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                 mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                 mode_id, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < 4:
        msg = master.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=1)
        if msg is None:
            send_heartbeat(master)
            continue
        if msg.get_type() == "STATUSTEXT":
            show_status(msg)
        elif is_autopilot_hb(master, msg) and msg.custom_mode == mode_id:
            print(f"[+] Mode set to {mode}")
            return True
    print(f"[!] Could not enter {mode}")
    return False


def arm(master):
    force = FORCE_MAGIC if FORCE_ARM else 0
    master.mav.command_long_send(master.target_system, master.target_component,
                                 ARM_CMD, 0, 1, force, 0, 0, 0, 0, 0)
    result = wait_ack(master, ARM_CMD)
    if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print(f"[!] Arm rejected (result={result}). Autopilot says:")
        pump(master, 2)
        return False
    if wait_armed_state(master, True):
        print("[+] ARMED")
        return True
    print("[!] Arm accepted but vehicle did not report armed")
    pump(master, 2)
    return False


def disarm(master):
    master.mav.command_long_send(master.target_system, master.target_component,
                                 ARM_CMD, 0, 0, FORCE_MAGIC, 0, 0, 0, 0, 0)
    wait_ack(master, ARM_CMD)
    if wait_armed_state(master, False):
        print("[+] DISARMED")
        return True
    print("[!] Vehicle may still be armed! Disconnect the battery.")
    return False


# ---------------- throttle control ----------------
class ThrottleController:
    def __init__(self, master, use_attitude, yaw_rad):
        self.master = master
        self.use_attitude = use_attitude
        self.yaw = yaw_rad
        # RC3 calibration range (defaults 1100..1900 when no RC is calibrated)
        self.rc_min = int(get_param(master, "RC3_MIN") or 1100)
        self.rc_max = int(get_param(master, "RC3_MAX") or 1900)
        print(f"[*] Throttle method: "
              f"{'SET_ATTITUDE_TARGET thrust' if use_attitude else f'RC override ch3 ({self.rc_min}-{self.rc_max} us)'}")

    def send(self, frac):
        """frac = throttle 0.0 .. 1.0"""
        frac = max(0.0, min(frac, MAX_ALLOWED_PCT / 100.0))
        m = self.master
        if self.use_attitude:
            q = [math.cos(self.yaw / 2), 0.0, 0.0, math.sin(self.yaw / 2)]  # level, hold yaw
            m.mav.set_attitude_target_send(
                0, m.target_system, m.target_component,
                0b00000111,          # ignore body rates, use attitude + thrust
                q, 0.0, 0.0, 0.0, frac)
        else:
            pwm = int(self.rc_min + frac * (self.rc_max - self.rc_min))
            m.mav.rc_channels_override_send(
                m.target_system, m.target_component,
                1500, 1500, pwm, 1500,          # roll, pitch, throttle, yaw
                65535, 65535, 65535, 65535)     # other channels: ignore

    def release(self):
        if not self.use_attitude:
            m = self.master
            m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                            0, 0, 0, 0, 0, 0, 0, 0)   # 0 = release override


def ramp(master, ctrl, start_frac, end_frac, duration, label):
    """Linear ramp while streaming commands. Returns False if the vehicle disarmed."""
    dt = 1.0 / SEND_RATE_HZ
    t0 = time.time()
    last_hb = 0
    while True:
        elapsed = time.time() - t0
        k = 1.0 if duration <= 0 else min(elapsed / duration, 1.0)
        frac = start_frac + (end_frac - start_frac) * k
        ctrl.send(frac)

        if time.time() - last_hb >= 1:
            send_heartbeat(master)
            last_hb = time.time()

        # drain incoming messages quickly
        while True:
            msg = master.recv_match(type=["STATUSTEXT", "HEARTBEAT"], blocking=False)
            if msg is None:
                break
            if msg.get_type() == "STATUSTEXT":
                show_status(msg)
            elif is_autopilot_hb(master, msg):
                if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    print("\n[!] Vehicle disarmed by itself")
                    return False

        print(f"    {label}: throttle {frac * 100:5.1f} %  ", end="\r")
        if k >= 1.0:
            print()
            return True
        time.sleep(dt)


# ------------------------- main -------------------------
def main():
    target = TARGET_THROTTLE_PCT
    if target > MAX_ALLOWED_PCT:
        print(f"[!] TARGET_THROTTLE_PCT {target} exceeds MAX_ALLOWED_PCT {MAX_ALLOWED_PCT}")
        sys.exit(1)

    port = PORT or find_port()
    print(f"[*] Connecting to {port} ...")
    master = mavutil.mavlink_connection(port, baud=BAUD)
    print("[*] Waiting for heartbeat ...")
    if not master.wait_heartbeat(timeout=15):
        print("[!] No heartbeat. Check cable / other apps using the port.")
        sys.exit(1)
    print(f"[+] Heartbeat from system {master.target_system}")

    if configure(master):
        print("[*] EKF parameters changed - rebooting Pixhawk. Wait ~10 s and run again.")
        master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                                     1, 0, 0, 0, 0, 0, 0)
        master.close()
        return

    # make sure ATTITUDE is streamed (used to hold current yaw in guided mode)
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                 30, 100000, 0, 0, 0, 0, 0)
    att = master.recv_match(type="ATTITUDE", blocking=True, timeout=3)
    yaw = att.yaw if att else 0.0

    ctrl = None
    tried_arming = False
    try:
        used_mode = None
        for mode in MODES_TO_TRY:
            if not set_mode(master, mode):
                continue
            use_attitude = mode in GUIDED_MODES
            ctrl = ThrottleController(master, use_attitude, yaw)
            ctrl.send(0.0)              # throttle at minimum before arming
            time.sleep(1)
            tried_arming = True
            if arm(master):
                used_mode = mode
                break
            print(f"[*] Arming failed in {mode}, trying next mode ...")

        if used_mode is None:
            print("[!] Could not arm. Read the [FC] lines above.")
            return

        print(f"[*] Ramping throttle to {target} % over {RAMP_UP_TIME:.0f} s  (Ctrl+C = abort)")
        f = target / 100.0
        if not ramp(master, ctrl, 0.0, f, RAMP_UP_TIME, "ramp up  "):
            return
        if not ramp(master, ctrl, f, f, HOLD_TIME, "holding  "):
            return
        ramp(master, ctrl, f, 0.0, RAMP_DOWN_TIME, "ramp down")

    except KeyboardInterrupt:
        print("\n[!] Interrupted - throttle to zero and disarming")
    finally:
        if ctrl:
            for _ in range(5):
                ctrl.send(0.0)
                time.sleep(0.05)
            ctrl.release()
        if tried_arming:
            disarm(master)
        master.close()


if __name__ == "__main__":
    main()