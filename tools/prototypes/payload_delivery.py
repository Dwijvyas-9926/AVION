from pymavlink import mavutil
import time
import math

# ---------------- CONNECT ----------------
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
master.wait_heartbeat()
print("✅ Connected to Pixhawk")

# ---------------- PARAMETERS ----------------
TAKEOFF_ALT = 2.0        # meters
VEL = 0.5                # m/s

SERVO_CH = 9             # AUX1
SERVO_OPEN = 1900
SERVO_CLOSE = 1100

# ---------------- BASIC HELPERS ----------------
def set_mode(mode):
    master.set_mode(mode)
    time.sleep(2)

def arm():
    master.arducopter_arm()
    master.motors_armed_wait()
    print("✅ Motors armed")

def disarm():
    print("🔻 Disarming motors")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("✅ Disarmed")

def takeoff(alt):
    print(f"🚀 Taking off to {alt} m")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        alt
    )
    time.sleep(12)   # time-based, stable

def send_velocity(vx, vy, vz, duration):
    print(f"➡️ Velocity vx={vx} vy={vy} vz={vz} for {duration}s")
    for _ in range(int(duration * 10)):
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )
        time.sleep(0.1)

def drop_payload():
    print("📦 Dropping payload")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        SERVO_CH,
        SERVO_OPEN,
        0, 0, 0, 0, 0
    )
    time.sleep(2)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        SERVO_CH,
        SERVO_CLOSE,
        0, 0, 0, 0, 0
    )

def land():
    print("🛬 Landing")
    set_mode("LAND")

# ---------------- YAW LOCK ----------------
def get_current_yaw():
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    return math.degrees(msg.yaw)

def lock_yaw(yaw_deg):
    print(f"🔒 Locking yaw at {yaw_deg:.1f}°")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        yaw_deg,    # target yaw
        0,          # yaw speed (0 = default)
        1,          # direction (1 = clockwise)
        0,          # absolute yaw
        0, 0, 0
    )
    time.sleep(1)

# ---------------- MISSION ----------------
print("🚀 Mission started")

set_mode("GUIDED")
arm()

# Lock yaw BEFORE takeoff
yaw = get_current_yaw()
lock_yaw(yaw)

# 1️⃣ Takeoff to 4 m
takeoff(TAKEOFF_ALT)

# 2️⃣ Fly forward 4 m
send_velocity(VEL, 0, 0, duration=4 / VEL)

# 3️⃣ Descend to ~2 m
print("⬇️ Descending to payload drop height")
send_velocity(0, 0, 0.3, duration=5)
time.sleep(2)

# 4️⃣ Drop payload
drop_payload()

# 5️⃣ Fly forward 2 m
send_velocity(VEL, 0, 0, duration=2 / VEL)

# 6️⃣ Hover
print("⏸ Holding position")
send_velocity(0, 0, 0, duration=5)

# 7️⃣ Fly BACKWARD 4 m
print("⬅️ Flying backward 6 meters")
send_velocity(-VEL, 0, 0, duration=6 / VEL)

# 8️⃣ Land and disarm
land()
time.sleep(6)
disarm()

print("✅ Mission completed successfully")
