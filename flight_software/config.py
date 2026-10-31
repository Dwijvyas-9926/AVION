# ============================================================
#  TerraAvis — config.py
#  All tunable parameters. Edit this before every flight.
#  Calibrate HSV values at the competition venue.
# ============================================================

# --- MAVLink Connection ---
# USB:  /dev/ttyUSB0
# UART: /dev/ttyAMA0
# SITL: tcp:127.0.0.1:5760
MAVLINK_CONNECTION  = "/dev/ttyACM0"
MAVLINK_BAUD        = 57600

# --- Pre-flight Safety Limits ---
MIN_BATTERY_VOLTAGE = 14.0      # volts — below this, refuse to arm
MIN_GPS_FIX         = 3         # minimum GPS fix type (3 = 3D fix)
MIN_GPS_SATELLITES  = 6         # minimum satellites for reliable position hold

# --- Mission Parameters ---
TAKEOFF_ALTITUDE    = 1.7      # metres AGL
HOVER_DURATION      = 2.0       # seconds to hover after reaching altitude
ALT_TOLERANCE       = 0.3       # metres — acceptable altitude error

# --- Yaw Search ---
YAW_RATE_DEG        = 18        # deg/s — slower yaw = more stable, better detection
YAW_MAX_SWEEP       = 360       # max degrees before giving up
YAW_TICK_INTERVAL   = 0.1       # seconds between yaw commands (keep at 0.1)

# --- Target Detection Stability ---
LOCK_STABLE_FRAMES  = 3         # consecutive frames needed to confirm target lock
LOCK_TICK           = 0.08      # seconds between lock confirmation frames

# --- Approach ---
APPROACH_SPEED      = 0.7       # m/s — max forward speed (tuned for smooth flight)
BLIND_CLOSE_SPEED   = 0.2       # m/s — slow blind approach when beyond depth range
RETURN_HOME_SPEED   = 0.8       # m/s — steady return speed
RELOAD_DURATION     = 8.0       # seconds total reload window
SAFE_DROP_DISTANCE  = 1.5       # metres — stop and drop at this depth
ALIGN_PIXEL_THRESH  = 40        # pixels — wider threshold, less twitchy
MAX_LOST_FRAMES     = 15        # frames before considering target lost

# --- Vision ---
CAMERA_WIDTH        = 640
CAMERA_HEIGHT       = 480
CAMERA_FPS          = 15

# ⚠️  CALIBRATE THESE AT YOUR COMPETITION VENUE
# Use calibrate_hsv.py to find correct values under actual lighting
# Red wraps around 0/180 in hue, so two ranges are needed
RED_HSV_LOW1        = (0,   120, 80)
RED_HSV_HIGH1       = (10,  255, 255)
RED_HSV_LOW2        = (165, 120, 80)
RED_HSV_HIGH2       = (179, 255, 255)

# Blue HSV — calibrate at venue using calibrate_hsv.py
BLUE_HSV_LOW        = (90,  74, 10)
BLUE_HSV_HIGH       = (140, 255, 255)

MIN_CONTOUR_AREA    = 800       # px² — ignore blobs smaller than this
MAX_CONTOUR_AREA    = 60000     # px² — ignore blobs larger than this (rejects dome/walls)
DEPTH_PATCH_SIZE    = 7         # NxN median depth patch (odd number)
MAX_VALID_DEPTH     = 8.0       # metres — ignore depth beyond this

# --- Payload Servo ---
# ⚠️  On Pixhawk, AUX pins map to MAVLink channels 9-14
# AUX1 = channel 9,  AUX2 = channel 10,  AUX3 = channel 11, etc.
SERVO_CHANNEL       = 9         # AUX1 = channel 9 (confirmed working)
SERVO_PWM_CLOSED    = 1900      # us — payload secured
SERVO_PWM_OPEN      = 1100      # us — payload released
SERVO_HOLD_TIME     = 2.0       # seconds after release before flying home

# --- GUI ---
GUI_WINDOW_NAME     = "TerraAvis Mission Control"
PANEL_WIDTH         = 300
LOG_HEIGHT          = 140

# --- Logging ---
LOG_DIR             = "logs"
LOG_LEVEL           = "DEBUG"   # DEBUG / INFO / WARNING