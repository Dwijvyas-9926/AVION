from pymavlink import mavutil
import time

# Connect to Pixhawk
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
master.wait_heartbeat()
print("✅ Connected to Pixhawk")

SERVO_CHANNEL = 9      # AUX1
SERVO_CLOSE = 1100     # closed position
SERVO_OPEN  = 1900     # open position

def set_servo(channel, pwm):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        pwm,
        0, 0, 0, 0, 0
    )


print("🔒 Opeaning servo")
set_servo(SERVO_CHANNEL, SERVO_CLOSE)
time.sleep(2)

# OPEN servo
print("🔓 Cloasing servo")
set_servo(SERVO_CHANNEL, SERVO_OPEN)
time.sleep(2)

