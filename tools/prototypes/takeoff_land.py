from pymavlink import mavutil
import time

# ----------------------------
# CONNECT TO PIXHAWK
# ----------------------------
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
master.wait_heartbeat()
print("[OK] Connected to Pixhawk")

# ----------------------------
# SET GUIDED MODE
# ----------------------------
master.set_mode_apm('GUIDED')
time.sleep(1)

# Wait for mode confirmation
while True:
    master.recv_match(type='HEARTBEAT', blocking=True)
    if master.flightmode == 'GUIDED':
        print("[OK] GUIDED mode confirmed")
        break

# ----------------------------
# ARM MOTORS
# ----------------------------
print("[ACTION] Arming motors")
master.arducopter_arm()
master.motors_armed_wait()
print("[OK] Motors armed")

# ----------------------------
# TAKEOFF COMMAND
# ----------------------------
TARGET_ALT = 2  # meters

print("[ACTION] Taking off")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0,
    0, 0,
    TARGET_ALT
)

# ----------------------------
# WAIT (TIME-BASED + ALT LOG)
# ----------------------------
print("[INFO] Waiting for climb (GPS altitude unreliable at low height)")

start_time = time.time()
TAKEOFF_TIMEOUT = 15 # seconds

while time.time() - start_time < TAKEOFF_TIMEOUT:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        alt = msg.relative_alt / 1000.0
        print(f"[ALT] {alt:.2f} m")
    time.sleep(0.5)

print("[OK] Takeoff phase complete")

# ----------------------------
# HOLD POSITION
# ----------------------------
HOLD_TIME = 5
print(f"[HOLD] Holding for {HOLD_TIME} seconds")

hold_start = time.time()
while time.time() - hold_start < HOLD_TIME:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        alt = msg.relative_alt / 1000.0
        print(f"[HOLD ALT] {alt:.2f} m")
    time.sleep(0.5)

# ----------------------------
# LAND
# ----------------------------
print("[ACTION] Landing")
master.set_mode_apm('LAND')

# ----------------------------
# WAIT FOR DISARM
# ----------------------------
while master.motors_armed():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        alt = msg.relative_alt / 1000.0
        print(f"[LANDING] Altitude: {alt:.2f} m")
    time.sleep(0.5)

print("[OK] Landed and disarmed safely")
