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
# TAKEOFF
# ----------------------------
TARGET_ALT = 4 # meters

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

# Wait fixed time for climb (GPS Z unreliable at low height)
time.sleep(7)
print("[OK] Takeoff phase complete")

# ----------------------------
# FLY FORWARD (VELOCITY CONTROL)
# ----------------------------
FORWARD_SPEED = 0.8   # m/s
FORWARD_TIME = 10  # seconds → ~4 meters

print("[ACTION] Flying forward")

start_time = time.time()
while time.time() - start_time < FORWARD_TIME:
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # velocity control only
        0, 0, 0,             # position (ignored)
        FORWARD_SPEED, 0, 0, # forward velocity
        0, 0, 0,             # acceleration (ignored)
        0, 0                 # yaw (ignored)
    )
    time.sleep(0.2)

print("[OK] Forward motion complete")

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

print("[OK] Mission complete — landed safely")
