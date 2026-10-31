# ============================================================
#  TerraAvis — rover.py
#  Rover motor control thread.
#  Reads RC channels from existing Pixhawk MAVLink connection.
#  Motors controlled via pigpio (TB6612 driver).
#
#  Mode switching:
#  - GUI ROVER button  → activates rover mode
#  - GUI DRONE button  → deactivates rover mode
#  - CH6 < 1500 on RC  → activates rover mode (hardware override)
#  - CH6 > 1500 on RC  → deactivates rover mode
# ============================================================

import time
import threading
from logger import log

# Motor GPIO pins (TB6612)
LEFT_IN1  = 17
LEFT_IN2  = 27
LEFT_PWM  = 18
RIGHT_IN1 = 22
RIGHT_IN2 = 23
RIGHT_PWM = 24
STBY_PIN  = 25

# RC config
SWITCH_CHANNEL  = 6       # CH6 for mode switch
MODE_THRESHOLD  = 1500    # below = rover, above = drone
SPEED_MULT      = 1.0


class RoverController:
    def __init__(self, flight_controller):
        self.fc          = flight_controller
        self.active      = False    # True = rover mode on
        self._thread     = None
        self._stop_flag  = False
        self._pi         = None
        self._gpio_ready = False

        self._init_gpio()

    # ----------------------------------------------------------
    # GPIO INIT
    # ----------------------------------------------------------
    def _init_gpio(self):
        try:
            import pigpio
            self._pi = pigpio.pi()
            if not self._pi.connected:
                log.warning("Rover: pigpio daemon not running — rover disabled.")
                return

            for pin in [LEFT_IN1, LEFT_IN2, LEFT_PWM,
                        RIGHT_IN1, RIGHT_IN2, RIGHT_PWM, STBY_PIN]:
                self._pi.set_mode(pin, pigpio.OUTPUT)

            self._pi.write(STBY_PIN, 1)
            self._pi.set_PWM_frequency(LEFT_PWM,  1000)
            self._pi.set_PWM_frequency(RIGHT_PWM, 1000)

            self._gpio_ready = True
            log.info("Rover: GPIO initialised ✅")

        except ImportError:
            log.warning("Rover: pigpio not installed — rover disabled.")
        except Exception as e:
            log.warning(f"Rover: GPIO init failed: {e}")

    # ----------------------------------------------------------
    # PUBLIC API
    # ----------------------------------------------------------
    def start(self):
        """Start rover RC-reading loop."""
        if not self._gpio_ready:
            log.warning("Rover: GPIO not ready — cannot start.")
            return
        if self._thread and self._thread.is_alive():
            return
        self._stop_flag = False
        self._thread    = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("Rover control thread started.")

    def stop(self):
        """Stop rover control thread and motors."""
        self._stop_flag = True
        self.active     = False
        self._motor_stop()
        log.info("Rover stopped.")

    def set_mode(self, mode):
        """
        Called from GUI button.
        mode = 'rover' or 'drone'
        """
        if mode == 'rover':
            if not self._gpio_ready:
                log.warning("Rover: GPIO not ready.")
                return
            self.active = True
            log.info("Rover: MODE = ROVER (GUI command)")
        else:
            self.active = False
            self._motor_stop()
            log.info("Rover: MODE = DRONE (GUI command)")

    # ----------------------------------------------------------
    # MAIN LOOP
    # ----------------------------------------------------------
    def _loop(self):
        log.info("Rover loop running — reading RC channels...")
        while not self._stop_flag:
            try:
                # Read RC from existing vehicle connection
                if self.fc.vehicle:
                    ch1 = self.fc.vehicle.channels.get('1', 1500)
                    ch2 = self.fc.vehicle.channels.get('2', 1500)
                    ch6 = self.fc.vehicle.channels.get('6', 1900)

                    # Hardware RC override (CH6)
                    if ch6 < MODE_THRESHOLD and not self.active:
                        self.active = True
                        log.info("Rover: MODE = ROVER (RC CH6 switch)")
                    elif ch6 >= MODE_THRESHOLD and self.active:
                        self.active = False
                        self._motor_stop()
                        log.info("Rover: MODE = DRONE (RC CH6 switch)")

                    if self.active:
                        steer = (ch1 - 1500) / 5.0
                        drive = (ch2 - 1500) / 5.0
                        left  = drive + steer
                        right = drive - steer
                        self._drive(left, right)

            except Exception as e:
                log.debug(f"Rover loop error: {e}")

            time.sleep(0.02)   # 50Hz

    # ----------------------------------------------------------
    # MOTOR CONTROL
    # ----------------------------------------------------------
    def _drive_motor(self, in1, in2, pwm_pin, val):
        val  = max(min(val * SPEED_MULT, 100), -100)
        duty = int(abs(val) * 2.55)

        if val > 0:
            self._pi.write(in1, 1)
            self._pi.write(in2, 0)
        elif val < 0:
            self._pi.write(in1, 0)
            self._pi.write(in2, 1)
        else:
            self._pi.write(in1, 0)
            self._pi.write(in2, 0)

        self._pi.set_PWM_dutycycle(pwm_pin, duty)

    def _drive(self, left, right):
        if not self._gpio_ready:
            return
        self._drive_motor(LEFT_IN1,  LEFT_IN2,  LEFT_PWM,  left)
        self._drive_motor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM, right)

    def _motor_stop(self):
        if not self._gpio_ready:
            return
        try:
            self._drive(0, 0)
            log.info("Rover: motors stopped.")
        except:
            pass

    def shutdown(self):
        """Full shutdown — stop thread, motors, GPIO."""
        self.stop()
        if self._pi and self._gpio_ready:
            try:
                self._pi.write(STBY_PIN, 0)
                self._pi.stop()
                log.info("Rover: pigpio stopped.")
            except:
                pass