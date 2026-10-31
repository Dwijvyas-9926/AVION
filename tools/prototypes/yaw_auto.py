from pymavlink import mavutil
import time

# ---------------------------------
# CONNECT TO PIXHAWK
# ---------------------------------
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
master.wait_heartbeat()
print("[OK] Connected to Pixhawk")

# ---------------------------------
# SET GUIDED MODE
# ---------------------------------
master.set_mode_apm('GUIDED')
time.sleep(1)

while True:
    master.recv_match(type='HEARTBEAT', blocking=True)
    if master.flightmode == 'GUIDED':
        print("[OK] GUIDED mode confirmed")
        break

# ---------------------------------
# ARM MOTORS
# ---------------------------------
print("[ACTION] Arming motors")
master.arducopter_arm()
master.motors_armed_wait()
print("[OK] Motors armed")

# ---------------------------------
# TAKEOFF TO 5 METERS
# ---------------------------------
TARGET_ALT = 2
print("[ACTION] Taking off to 5 m")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0,
    0, 0,
    TARGET_ALT
)

time.sleep(10)
print("[OK] Takeoff complete")

# ---------------------------------
# FLY FORWARD 5 METERS
# ---------------------------------
print("[ACTION] Flying forward 5 m")
start = time.time()
while time.time() - start < 5.0:
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        1.0, 0, 0,
        0, 0, 0,
        0, 0
    )
    time.sleep(0.2)

print("[OK] Forward 5 m complete")

# ---------------------------------
# YAW 180 DEG
# ---------------------------------
print("[ACTION] Yawing 180 degrees")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_CONDITION_YAW,
    0,
    180,
    30,
    1,
    1,
    0, 0, 0
)

time.sleep(7)
print("[OK] Yaw complete")

# ---------------------------------
# FLY FORWARD 2 METERS
# ---------------------------------
print("[ACTION] Flying forward 2 m")
start = time.time()
while time.time() - start < 2.0:
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        1.0, 0, 0,
        0, 0, 0,
        0, 0
    )
    time.sleep(0.2)

print("[OK] Forward 2 m complete")

# ---------------------------------
# HOLD POSITION FOR 5 SECONDS
# ---------------------------------
print("[HOLD] Holding position for 5 seconds")

hold_start = time.time()
while time.time() - hold_start < 5:
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        0, 0, 0,   # ZERO velocity = HOLD
        0, 0, 0,
        0, 0
    )
    time.sleep(0.2)

print("[OK] Hold complete")

# ---------------------------------
# STABILIZE BEFORE LAND (SMOOTH)
# ---------------------------------
print("[INFO] Stabilizing before landing")
time.sleep(2)

# ---------------------------------
# LAND
# ---------------------------------
print("[ACTION] Landing")
master.set_mode_apm('LAND')

# ---------------------------------
# WAIT FOR DISARM
# ---------------------------------
while master.motors_armed():
    time.sleep(0.5)

print("[MISSION COMPLETE] Landed safely")
