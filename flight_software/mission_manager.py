# ============================================================
#  TerraAvis — mission_manager.py
#  Full autonomous state machine.
#  Phase 1 → RED dummy.  Phase 2 → BLUE dummy.
#
#  Operator confirmation: every lock pauses in CONFIRM_TARGET.
#    APPROACH → delivery continues.
#    IGNORE   → object's ground position (bearing + depth) is
#               blacklisted; vision skips it from then on, even
#               when it shares the frame with the real dummy.
#               Yaw search resumes past it. Repeats until APPROACH.
#    If a full sweep finds nothing new, the ignore list is cleared
#    so a mistaken IGNORE can be corrected.
#
#  Approach behaviour:
#    Depth = None (target >7m): blind close at BLIND_CLOSE_SPEED
#    Depth valid: three-zone smooth speed profile
#      Far >4m  → APPROACH_SPEED (0.7 m/s)
#      Mid 2-4m → smooth linear ramp down
#      Close    → 0.18 m/s creep
#    All speed changes rate-limited — no sudden jerks.
#    Lateral correction: dead zone + rate-limited — no snapping.
#
#  Return home: simple_goto (Phase 1) + velocity precision (Phase 2)
#  No RTL — stays at TAKEOFF_ALTITUDE throughout.
# ============================================================

import time
import math
import threading
from enum import Enum, auto
from pymavlink import mavutil
from logger import log
from config import (
    TAKEOFF_ALTITUDE, HOVER_DURATION,
    YAW_RATE_DEG, YAW_MAX_SWEEP,
    APPROACH_SPEED, BLIND_CLOSE_SPEED, SAFE_DROP_DISTANCE,
    ALIGN_PIXEL_THRESH, MAX_LOST_FRAMES,
    LOCK_STABLE_FRAMES, LOCK_TICK,
    CAMERA_WIDTH,
    RETURN_HOME_SPEED, RELOAD_DURATION
)

# --- Operator target confirmation / ignore filtering ---
CAMERA_HFOV_DEG      = 90.0   # D455 RGB horizontal FOV — converts pixel offset to bearing
IGNORE_POS_TOL       = 1.5    # m   — a detection within this of an ignored object's ground position is skipped
IGNORE_BEARING_TOL   = 12.0   # deg — fallback when depth unavailable (object beyond depth range)
IGNORE_VALID_RADIUS  = 3.0    # m   — bearing fallback only valid near where the object was ignored
TRACK_GATE_PX        = 120    # px  — once locked: blob must stay near last frame's position
TRACK_POS_TOL        = 2.0    # m   — after APPROACH: blob must be near confirmed target's ground position


class MissionState(Enum):
    IDLE           = auto()
    PREFLIGHT      = auto()
    ARMING         = auto()
    TAKEOFF        = auto()
    HOVER          = auto()
    YAW_SEARCH     = auto()
    TARGET_LOCK    = auto()
    CONFIRM_TARGET = auto()   # operator confirms: approach or ignore
    ALIGN          = auto()
    APPROACH       = auto()
    DROP           = auto()
    RETURN_HOME    = auto()
    RELOAD         = auto()
    LAND           = auto()
    COMPLETE       = auto()
    ABORTED        = auto()
    ERROR          = auto()


STATE_LABELS = {
    MissionState.IDLE:           "Idle — Awaiting Start",
    MissionState.PREFLIGHT:      "Running Pre-Flight Checks",
    MissionState.ARMING:         "Arming & GUIDED Mode",
    MissionState.TAKEOFF:        "Taking Off",
    MissionState.HOVER:          "Hovering — Stabilising",
    MissionState.YAW_SEARCH:     "Yaw Searching for Target",
    MissionState.TARGET_LOCK:    "Target Confirmed — Locking",
    MissionState.CONFIRM_TARGET: "⚠ Awaiting Operator Confirmation",
    MissionState.ALIGN:          "Aligning to Target",
    MissionState.APPROACH:       "Approaching Target",
    MissionState.DROP:           "Dropping Payload",
    MissionState.RETURN_HOME:    "Returning to Home Position",
    MissionState.RELOAD:         "Reloading Payload at Home",
    MissionState.LAND:           "Landing",
    MissionState.COMPLETE:       "Mission Complete ✓",
    MissionState.ABORTED:        "Mission Aborted",
    MissionState.ERROR:          "Error — Check Logs",
}

MISSION_ORDER = [
    MissionState.IDLE, MissionState.PREFLIGHT, MissionState.ARMING,
    MissionState.TAKEOFF, MissionState.HOVER,
    MissionState.YAW_SEARCH, MissionState.TARGET_LOCK, MissionState.CONFIRM_TARGET,
    MissionState.ALIGN, MissionState.APPROACH, MissionState.DROP,
    MissionState.RETURN_HOME, MissionState.RELOAD,
    MissionState.YAW_SEARCH, MissionState.TARGET_LOCK, MissionState.CONFIRM_TARGET,
    MissionState.ALIGN, MissionState.APPROACH, MissionState.DROP,
    MissionState.RETURN_HOME, MissionState.LAND, MissionState.COMPLETE
]


class MissionManager:
    def __init__(self, flight_controller, vision_system):
        self.fc      = flight_controller
        self.vision  = vision_system
        self.state   = MissionState.IDLE
        self._abort  = False
        self._thread = None
        self.preflight_issues = []
        self.target_color = 'red'
        self.hook_state   = 'closed'

        self._home_lat = None
        self._home_lon = None

        # Operator target confirmation (used in CONFIRM_TARGET state)
        self._confirm_event  = threading.Event()
        self._confirm_choice = None   # 'approach' or 'ignore'

        # Operator reload check (used in RELOAD state)
        #   reload_stage: None | 'await_close' | 'await_done'  (read by the GUI)
        self.reload_stage         = None
        self._reload_close_event  = threading.Event()
        self._reload_done_event   = threading.Event()

        # Objects the operator rejected this phase: {'bearing', 'lat', 'lon'}
        self._ignored        = []
        self._locked_bearing = None
        self._locked_depth   = None
        self._locked_cx      = None

        # After APPROACH: stay locked on the confirmed object, not just "largest red blob"
        self._tracking    = False
        self._target_gpos = None

        # Vision skips ignored / non-confirmed objects and falls through to the next-largest blob
        self.vision.reject_fn = self._reject_candidate

    # ----------------------------------------------------------
    # PUBLIC API
    # ----------------------------------------------------------
    def start(self):
        if self.state not in (MissionState.IDLE, MissionState.COMPLETE,
                              MissionState.ABORTED, MissionState.ERROR):
            log.warning("Mission already running.")
            return
        self._abort       = False
        self.target_color = 'red'
        self.hook_state   = 'closed'
        self._ignored     = []
        self.reload_stage = None
        self._reload_close_event.clear()
        self._reload_done_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        log.info("Mission thread started.")

    def abort(self):
        log.warning("ABORT commanded — landing in place.")
        self._abort = True
        self.fc.stop_yaw()
        self.fc.stop()
        time.sleep(0.3)
        self.fc.land()
        self._set_state(MissionState.ABORTED)

    def confirm_approach(self):
        """Operator pressed YES — approach this target."""
        if self.state != MissionState.CONFIRM_TARGET:
            log.warning("APPROACH pressed but no target is awaiting confirmation — ignored.")
            return
        log.info("Operator: APPROACH confirmed ✅")
        self._confirm_choice = 'approach'
        self._confirm_event.set()

    def ignore_target(self):
        """Operator pressed IGNORE — skip this target, continue search."""
        if self.state != MissionState.CONFIRM_TARGET:
            log.warning("IGNORE pressed but no target is awaiting confirmation — ignored.")
            return
        log.info("Operator: IGNORE — continuing yaw search.")
        self._confirm_choice = 'ignore'
        self._confirm_event.set()

    def reload_close_hook(self):
        """Operator pressed CLOSE HOOK during reload."""
        if self.state != MissionState.RELOAD or self.reload_stage != 'await_close':
            log.warning("CLOSE HOOK pressed but the drone is not waiting for it — ignored.")
            return
        log.info("Operator: CLOSE HOOK ✅")
        self._reload_close_event.set()

    def reload_done(self):
        """Operator pressed RELOAD DONE — continue to the BLUE phase."""
        if self.state != MissionState.RELOAD or self.reload_stage != 'await_done':
            log.warning("RELOAD DONE pressed but the hook is not closed yet — ignored.")
            return
        log.info("Operator: RELOAD DONE ✅")
        self._reload_done_event.set()

    def reset(self):
        if self.state in (MissionState.COMPLETE, MissionState.ABORTED, MissionState.ERROR):
            self.state        = MissionState.IDLE
            self._abort       = False
            self.hook_state   = 'closed'
            self.target_color = 'red'
            log.info("Mission reset to IDLE.")

    @property
    def ignored_count(self):
        return len(self._ignored)

    @property
    def state_label(self):
        return STATE_LABELS.get(self.state, "Unknown")

    @property
    def progress(self):
        try:
            idx = MISSION_ORDER.index(self.state)
            return round(idx / (len(MISSION_ORDER) - 1), 3)
        except ValueError:
            return 0.0

    # ----------------------------------------------------------
    # MAIN STATE MACHINE
    # ----------------------------------------------------------
    def _run(self):
        try:
            # ── STARTUP ──
            self._step_preflight()
            if self._check_abort(): return

            self._step_arm()
            if self._check_abort(): return

            self._step_takeoff()
            if self._check_abort(): return

            self._record_home()

            self._step_hover()
            if self._check_abort(): return

            # ── PHASE 1 — RED ──
            log.info("=" * 45)
            log.info("  PHASE 1 — RED DUMMY")
            log.info("=" * 45)
            self.target_color = 'red'
            self.vision.set_target_color('red')

            # Search → lock → operator popup, repeated until APPROACH
            if not self._search_and_confirm(): return

            self._step_align()
            if self._check_abort(): return
            self._step_approach()
            if self._check_abort(): return
            self._step_drop()
            if self._check_abort(): return

            # ── RETURN + RELOAD ──
            self._step_return_home()
            if self._check_abort(): return
            self._step_reload()
            if self._check_abort(): return

            # ── PHASE 2 — BLUE ──
            log.info("=" * 45)
            log.info("  PHASE 2 — BLUE DUMMY")
            log.info("=" * 45)
            self.target_color = 'blue'
            self.vision.set_target_color('blue')

            if not self._search_and_confirm(): return

            self._step_align()
            if self._check_abort(): return
            self._step_approach()
            if self._check_abort(): return
            self._step_drop()
            if self._check_abort(): return

            # ── FINAL RETURN + LAND ──
            self._step_return_home()
            if self._check_abort(): return
            self._step_land()

            self._set_state(MissionState.COMPLETE)
            log.info("=" * 45)
            log.info("  ✅  MISSION COMPLETE")
            log.info("=" * 45)

        except Exception as e:
            log.error(f"Mission exception: {e}", exc_info=True)
            self._set_state(MissionState.ERROR)
            if self.fc.armed:
                log.warning("Drone armed on exception — landing for safety.")
                self.fc.land()

    # ----------------------------------------------------------
    # PREFLIGHT
    # ----------------------------------------------------------
    def _step_preflight(self):
        self._set_state(MissionState.PREFLIGHT)
        self.preflight_issues = []
        log.info("Running pre-flight checks...")

        if not self.fc.vehicle:
            log.warning("No vehicle — running servo test only.")
            self.fc.test_servo()
            return

        batt = self.fc.battery_voltage
        if batt and batt < 14.0:
            msg = f"Battery low: {batt:.1f}V"
            self.preflight_issues.append(msg)
            log.warning(f"  ⚠ {msg}")

        gps = self.fc.gps_fix
        if gps < 3:
            msg = f"GPS fix weak: type {gps}"
            self.preflight_issues.append(msg)
            log.warning(f"  ⚠ {msg}")

        if not self.fc.ekf_ok:
            msg = "EKF not healthy"
            self.preflight_issues.append(msg)
            log.warning(f"  ⚠ {msg}")

        log.info("Servo test before arming...")
        self.fc.test_servo()

        if self.preflight_issues:
            log.warning(f"Pre-flight: {len(self.preflight_issues)} issue(s) — proceeding with caution.")
        else:
            log.info("Pre-flight checks passed ✅")

    # ----------------------------------------------------------
    # ARM
    # ----------------------------------------------------------
    def _step_arm(self):
        self._set_state(MissionState.ARMING)
        if not self.fc.vehicle:
            log.warning("No vehicle — skipping arm.")
            return

        self.fc.set_guided_mode()
        self._sleep(1.0)

        armed = self.fc.arm()
        if not armed:
            log.error("Arming failed — check Pixhawk pre-arm messages.")
            raise RuntimeError("Arming failed")

        self._sleep(2.0)
        log.info("Armed and ready ✅")

    # ----------------------------------------------------------
    # TAKEOFF
    # ----------------------------------------------------------
    def _step_takeoff(self):
        self._set_state(MissionState.TAKEOFF)
        log.info(f"Taking off to {TAKEOFF_ALTITUDE}m...")
        self.fc.takeoff(TAKEOFF_ALTITUDE)

        timeout = time.time() + 35
        while True:
            if self._check_abort(): return
            if time.time() > timeout:
                log.warning("Takeoff timeout — continuing at current altitude.")
                break
            alt = self.fc.altitude
            log.debug(f"  Altitude: {alt:.2f}m / {TAKEOFF_ALTITUDE}m")
            if alt >= TAKEOFF_ALTITUDE * 0.90:
                log.info(f"Takeoff complete: {alt:.2f}m ✅")
                break
            self._sleep(0.3)

    # ----------------------------------------------------------
    # HOVER
    # ----------------------------------------------------------
    def _step_hover(self):
        self._set_state(MissionState.HOVER)
        log.info(f"Stabilising for {HOVER_DURATION}s...")
        self.fc.hold_position(HOVER_DURATION)

    # ----------------------------------------------------------
    # SEARCH + OPERATOR CONFIRMATION LOOP
    # ----------------------------------------------------------
    def _search_and_confirm(self):
        """
        Yaw search → target lock → operator popup.
        APPROACH → returns True (delivery continues).
        IGNORE   → object's bearing is blacklisted and yaw search
                   resumes past it. Repeats until APPROACH.
        Returns False only on abort / emergency.
        """
        self._ignored     = []      # fresh ignore list for each colour phase
        self._tracking    = False
        self._target_gpos = None

        while True:
            if self._check_abort(): return False

            # New search: no object is locked yet → largest un-ignored blob wins
            self._tracking    = False
            self._target_gpos = None

            if not self._step_yaw_search():
                if self._check_abort(): return False
                if self._ignored:
                    log.warning(f"Full sweep found nothing new — clearing {len(self._ignored)} "
                                f"ignored object(s) so the operator can re-decide.")
                    self._ignored = []
                else:
                    log.warning("No target in full sweep — sweeping again.")
                continue

            if not self._step_target_lock():
                if self._check_abort(): return False
                continue

            choice = self._step_confirm_target()
            if self._check_abort(): return False

            if choice == 'approach':
                self._target_gpos = self._ground_pos(self._current_pos(),
                                                     self._locked_cx, self._locked_depth)
                log.info("Tracking locked to confirmed target" +
                         (" (ground position known)" if self._target_gpos else " (pixel tracking)"))
                return True
            if choice == 'lost':
                log.warning("Target slipped out of view before confirmation — searching again.")
                continue

            self._register_ignore()
            log.info(f"Resuming yaw search ({len(self._ignored)} object(s) ignored this phase)...")

    # ----------------------------------------------------------
    # IGNORE FILTER HELPERS
    # ----------------------------------------------------------
    def _current_pos(self):
        try:
            loc = self.fc.vehicle.location.global_relative_frame
            if loc.lat and loc.lon:
                return (loc.lat, loc.lon)
        except Exception:
            pass
        return None

    @staticmethod
    def _dist_m(a, b):
        dlat = (a[0] - b[0]) * 111139
        dlon = (a[1] - b[1]) * 111139 * math.cos(math.radians(a[0]))
        return math.sqrt(dlat**2 + dlon**2)

    @staticmethod
    def _ang_diff(a, b):
        return abs((a - b + 180) % 360 - 180)

    @staticmethod
    def _pixel_offset_deg(cx):
        """Horizontal angle of pixel column cx from camera centre (pinhole model)."""
        f = (CAMERA_WIDTH / 2) / math.tan(math.radians(CAMERA_HFOV_DEG / 2))
        return math.degrees(math.atan((cx - CAMERA_WIDTH / 2) / f))

    def _bearing_of(self, cx):
        return (self.fc.heading + self._pixel_offset_deg(cx)) % 360

    def _ground_pos(self, pos, cx, depth):
        """Project a detection to lat/lon using drone position, bearing and depth."""
        if pos is None or depth is None:
            return None
        off   = math.radians(self._pixel_offset_deg(cx))
        rng   = depth / max(math.cos(off), 0.3)      # z-depth → slant range
        brg   = math.radians(self._bearing_of(cx))
        dn, de = rng * math.cos(brg), rng * math.sin(brg)
        return (pos[0] + dn / 111139,
                pos[1] + de / (111139 * math.cos(math.radians(pos[0]))))

    def _detection_bearing(self):
        c = self.vision.centroid
        return None if c is None else self._bearing_of(c[0])

    def _is_ignored_candidate(self, cx, cy=None, depth=None):
        """Called by vision for every blob. True = operator already ignored it."""
        if not self._ignored:
            return False
        pos  = self._current_pos()
        gpos = self._ground_pos(pos, cx, depth)
        brg  = self._bearing_of(cx)
        for ign in self._ignored:
            if gpos is not None and ign['gpos'] is not None:
                # Both positions known → decide by ground position only
                if self._dist_m(gpos, ign['gpos']) <= IGNORE_POS_TOL:
                    return True
                continue
            # Fallback: bearing match, only near where it was ignored
            if pos and ign['pos'] and self._dist_m(pos, ign['pos']) > IGNORE_VALID_RADIUS:
                continue
            if self._ang_diff(brg, ign['bearing']) <= IGNORE_BEARING_TOL:
                return True
        return False

    def _reject_candidate(self, cx, cy=None, depth=None):
        """Vision filter: ignored objects always rejected; after APPROACH,
        anything that isn't the confirmed target is rejected too."""
        if self._is_ignored_candidate(cx, cy, depth):
            return True
        if not self._tracking:
            return False
        # Gate 1: must be near where the target was last frame (no jumping to a neighbour)
        prev = self.vision.centroid           # previous frame's chosen target
        if prev is not None and abs(cx - prev[0]) > TRACK_GATE_PX:
            return True
        # Gate 2 (after APPROACH): must be near the confirmed target's ground position
        if self._target_gpos is not None and depth is not None:
            g = self._ground_pos(self._current_pos(), cx, depth)
            if g is not None and self._dist_m(g, self._target_gpos) > TRACK_POS_TOL:
                return True
        return False

    def _valid_detection(self):
        """Target detected AND not one the operator already ignored."""
        if not self.vision.detected or self.vision.centroid is None:
            return False
        return not self._is_ignored_candidate(self.vision.centroid[0], None, self.vision.depth_m)

    def _remember_lock(self):
        c = self.vision.centroid
        if c is not None:
            self._locked_cx      = c[0]
            self._locked_bearing = self._bearing_of(c[0])
            if self.vision.depth_m is not None:
                self._locked_depth = self.vision.depth_m

    def _register_ignore(self):
        if self._locked_bearing is None:
            log.warning("Could not compute bearing of ignored object — it may be offered again.")
            return
        pos  = self._current_pos()
        gpos = self._ground_pos(pos, self._locked_cx, self._locked_depth)
        self._ignored.append({'bearing': self._locked_bearing, 'pos': pos, 'gpos': gpos})
        where = f"{self._locked_depth:.1f}m away" if self._locked_depth else "beyond depth range"
        log.info(f"Ignored {self.target_color.upper()} object at bearing {self._locked_bearing:.0f}° ({where})")

    # ----------------------------------------------------------
    # YAW SEARCH
    # ----------------------------------------------------------
    def _step_yaw_search(self):
        """Returns True when an un-ignored target is in frame, False otherwise."""
        self._set_state(MissionState.YAW_SEARCH)
        color = self.target_color.upper()
        log.info(f"Yaw search — looking for {color} dummy...")

        # Check if already visible
        self.vision.process_frame()
        if self._valid_detection():
            log.info(f"{color} dummy already in frame — skipping sweep.")
            return True
        if self.vision.detected:
            log.info("Only an ignored object in view — rotating past it.")

        yaw_rate  = YAW_RATE_DEG
        deg_step  = yaw_rate * 0.1
        swept     = 0.0

        while swept < YAW_MAX_SWEEP:
            if self._check_abort(): return False

            self.vision.process_frame()
            if self._valid_detection():
                log.info(f"{color} dummy found after {swept:.0f}° ✅")
                return True

            if self.fc.vehicle:
                msg = self.fc.vehicle.message_factory.command_long_encode(
                    0, 0,
                    mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                    0,
                    deg_step, yaw_rate,
                    1,   # CW
                    1,   # relative
                    0, 0, 0
                )
                self.fc.vehicle.send_mavlink(msg)
                log.debug(f"  Yaw CW  step={deg_step:.1f}°  total={swept:.0f}°")

            self._sleep(0.1)
            swept += deg_step

        log.warning(f"Full 360° sweep done — {color} dummy not found. Holding position.")
        self.fc.stop()
        return False

    # ----------------------------------------------------------
    # TARGET LOCK
    # ----------------------------------------------------------
    def _step_target_lock(self):
        """Returns True once locked, False on timeout (caller resumes search)."""
        self._set_state(MissionState.TARGET_LOCK)
        log.info(f"Confirming lock — need {LOCK_STABLE_FRAMES} stable frames...")

        stable       = 0
        bad          = 0
        attempts     = 0
        max_attempts = 150
        # From here on vision sticks to THIS object (no flicker between similar blobs),
        # so the popup and the approach are always about the same thing.
        self._tracking       = True
        self._locked_bearing = None
        self._locked_depth   = None
        self._locked_cx      = None

        while stable < LOCK_STABLE_FRAMES:
            if self._check_abort(): return False

            self.vision.process_frame()

            if self._valid_detection():
                stable += 1
                bad     = 0
                self._remember_lock()
                log.debug(f"  Lock {stable}/{LOCK_STABLE_FRAMES} — depth={self.vision.depth_m}m")
            else:
                bad += 1
                if bad >= 3:
                    stable = 0
                log.debug(f"  Unstable frame {bad}")

            attempts += 1
            if attempts > max_attempts:
                log.warning("Lock timeout — back to yaw search.")
                return False

            self._sleep(LOCK_TICK)

        depth_str = f"{self.vision.depth_m:.2f}m" if self.vision.depth_m else "beyond depth range"
        log.info(f"TARGET LOCKED ✅  depth={depth_str}  area={self.vision.contour_area:.0f}px²")
        return True

    # ----------------------------------------------------------
    # CONFIRM TARGET (operator decision)
    # ----------------------------------------------------------
    def _yaw_toward(self, error_px, step_s=0.05):
        """One gentle proportional yaw nudge toward the target (same law as ALIGN)."""
        if not self.fc.vehicle:
            return
        norm      = abs(error_px) / (CAMERA_WIDTH / 2)
        yaw_rate  = max(3.0, min(15.0, 3 + norm * 12))
        direction = 1 if error_px > 0 else -1
        msg = self.fc.vehicle.message_factory.command_long_encode(
            0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
            yaw_rate * step_s, yaw_rate, direction, 1, 0, 0, 0)
        self.fc.vehicle.send_mavlink(msg)

    def _center_target(self, timeout=8.0):
        """Yaw until the locked target sits in the centre of the frame,
        so the operator sees it clearly in the popup. False if lost."""
        settled, lost = 0, 0
        end = time.time() + timeout
        while time.time() < end:
            if self._check_abort(): return False
            self.vision.process_frame()
            if not self._valid_detection():
                lost += 1
                if lost >= MAX_LOST_FRAMES:
                    return False
                self._sleep(0.05)
                continue
            lost  = 0
            error = self.vision.lateral_error_pixels()
            if error is None:
                continue
            if abs(error) <= ALIGN_PIXEL_THRESH:
                settled += 1
                if settled >= 3:
                    self._remember_lock()
                    return True
            else:
                settled = 0
                self._yaw_toward(error)
            self._sleep(0.05)
        log.warning("Centering timeout — showing popup with target off-centre.")
        return self._valid_detection()

    def _step_confirm_target(self):
        """
        Centre the target, stop and hold one fixed GPS point, then wait
        for the GUI popup answer: APPROACH or IGNORE. No timeout.
        While waiting, small yaw nudges keep the target centred on screen.
        Returns 'approach', 'ignore', or 'lost' (target gone before popup).
        """
        log.info("Centering target for operator view...")
        if not self._center_target():
            if self._check_abort(): return 'ignore'
            return 'lost'

        # Lock position once (GUIDED position target persists while we wait)
        self.fc.stop()

        # Clear any stale click BEFORE the popup is shown
        self._confirm_event.clear()
        self._confirm_choice = None

        self._set_state(MissionState.CONFIRM_TARGET)

        color    = self.target_color.upper()
        depth    = self.vision.depth_m
        area     = self.vision.contour_area
        dist_str = f"{depth:.1f}m" if depth else "beyond depth range"
        log.info(f"TARGET CONFIRMATION REQUIRED — {color} object  dist={dist_str}  area={area:.0f}px²")
        log.info("Waiting for operator: APPROACH or IGNORE...")

        while not self._confirm_event.is_set():
            if self._check_abort():
                return 'ignore'
            self.vision.process_frame()          # keeps live feed + depth fresh
            if self._valid_detection():
                self._remember_lock()
                err = self.vision.lateral_error_pixels()
                if err is not None and abs(err) > 2 * ALIGN_PIXEL_THRESH:
                    self._yaw_toward(err)        # keep it on screen for the operator
            self._sleep(0.1)

        choice = self._confirm_choice or 'ignore'
        log.info(f"Operator chose: {choice.upper()}")
        return choice

    # ----------------------------------------------------------
    # ALIGN
    # ----------------------------------------------------------
    def _step_align(self):
        self._set_state(MissionState.ALIGN)
        log.info("Aligning to target via yaw...")

        settled = 0
        required = 5

        while settled < required:
            if self._check_abort(): return

            self.vision.process_frame()

            if not self.vision.detected:
                log.debug("  Target lost during align — holding.")
                self.fc.stop()
                settled = 0
                self._sleep(0.1)
                continue

            error = self.vision.lateral_error_pixels()
            if error is None:
                self._sleep(0.05)
                continue

            if abs(error) <= ALIGN_PIXEL_THRESH:
                settled += 1
                log.debug(f"  Aligned {settled}/{required} — error={error}px")
                self.fc.stop_yaw()
                self._sleep(0.05)
                continue

            settled = 0

            # Proportional yaw: 3–15 deg/s — gentle, no snapping
            norm      = abs(error) / (CAMERA_WIDTH / 2)
            yaw_rate  = max(3.0, min(15.0, 3 + norm * 12))
            direction = 1 if error > 0 else -1

            if self.fc.vehicle:
                msg = self.fc.vehicle.message_factory.command_long_encode(
                    0, 0,
                    mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                    0,
                    yaw_rate * 0.05,   # small step per 50ms tick
                    yaw_rate,
                    direction,
                    1,                  # relative
                    0, 0, 0
                )
                self.fc.vehicle.send_mavlink(msg)
                log.debug(f"  Yaw {'CW' if direction==1 else 'CCW'} {yaw_rate:.1f}°/s  err={error}px")

            self._sleep(0.05)

        self.fc.stop_yaw()
        log.info("Aligned ✅")

    # ----------------------------------------------------------
    # APPROACH
    # ----------------------------------------------------------
    def _step_approach(self):
        self._set_state(MissionState.APPROACH)
        log.info(f"Approaching  blind={BLIND_CLOSE_SPEED}m/s  "
                 f"max={APPROACH_SPEED}m/s  stop at {SAFE_DROP_DISTANCE}m")

        lost_frames    = 0
        last_depth     = None
        filtered_depth = None
        current_vx     = 0.0
        current_vy     = 0.0

        # Rate limits per 50ms tick
        MAX_VX_ACCEL  = 0.04    # gentle acceleration
        MAX_VX_DECEL  = 0.06    # slightly faster deceleration
        MAX_VY_CHANGE = 0.025   # lateral rate limit — no sudden snaps
        DEPTH_ALPHA   = 0.35    # low-pass filter: 0=frozen, 1=raw
        LATERAL_DEAD  = 25      # pixel dead zone — ignore tiny errors

        while True:
            if self._check_abort(): return

            self.vision.process_frame()

            # --- Target lost ---
            if not self.vision.detected:
                lost_frames += 1
                log.debug(f"  Not visible ({lost_frames}/{MAX_LOST_FRAMES})")
                if lost_frames >= MAX_LOST_FRAMES:
                    log.warning("Target lost — stopping and recovering...")
                    self.fc.stop()
                    self._recover_target()
                    if self._check_abort(): return
                    lost_frames = 0
                    current_vx  = 0.0
                    current_vy  = 0.0
                # Smoothly decelerate, don't cut instantly
                current_vx = max(0.0, current_vx - MAX_VX_DECEL * 2)
                self.fc.send_body_velocity(current_vx, 0, 0)
                self._sleep(0.05)
                continue

            lost_frames = 0
            raw_depth   = self.vision.depth_m

            # Low-pass depth filter — smooths noisy sensor readings
            if raw_depth is not None:
                last_depth = raw_depth
                filtered_depth = (raw_depth if filtered_depth is None
                                  else DEPTH_ALPHA * raw_depth + (1 - DEPTH_ALPHA) * filtered_depth)

            # Lateral correction: dead zone + rate limiting
            error     = self.vision.lateral_error_pixels()
            target_vy = 0.0
            if error is not None and abs(error) > LATERAL_DEAD:
                norm      = max(-1.0, min(1.0, error / (CAMERA_WIDTH / 2)))
                target_vy = norm * 0.15

            vy_diff    = max(-MAX_VY_CHANGE, min(MAX_VY_CHANGE, target_vy - current_vy))
            current_vy += vy_diff

            # --- Phase 1: beyond depth range — blind approach ---
            if filtered_depth is None and last_depth is None:
                log.debug("  Blind approach (depth out of range)")
                target_vx  = BLIND_CLOSE_SPEED
                vx_diff    = max(-MAX_VX_DECEL, min(MAX_VX_ACCEL, target_vx - current_vx))
                current_vx += vx_diff
                self.fc.send_body_velocity(current_vx, current_vy, 0)
                self._sleep(0.05)
                continue

            depth = filtered_depth if filtered_depth is not None else last_depth

            log.debug(f"  depth={depth:.2f}m  err={error}px  vx={current_vx:.2f}  vy={current_vy:.2f}")

            # --- Stop at safe drop distance ---
            if depth <= SAFE_DROP_DISTANCE:
                log.info(f"Drop distance reached: {depth:.2f}m — stopping ✅")
                self.fc.stop()
                return

            # --- Phase 2: smooth three-zone speed profile ---
            if depth > 4.0:
                target_vx = APPROACH_SPEED
            elif depth > SAFE_DROP_DISTANCE + 0.5:
                t         = (depth - (SAFE_DROP_DISTANCE + 0.5)) / (4.0 - (SAFE_DROP_DISTANCE + 0.5))
                target_vx = 0.18 + t * (APPROACH_SPEED - 0.18)
            else:
                target_vx = 0.18   # creep for final approach

            # Rate-limit vx change
            vx_diff    = target_vx - current_vx
            vx_diff    = (min(vx_diff, MAX_VX_ACCEL) if vx_diff > 0
                          else max(vx_diff, -MAX_VX_DECEL))
            current_vx += vx_diff

            self.fc.send_body_velocity(current_vx, current_vy, 0)
            self._sleep(0.05)   # 20Hz

    # ----------------------------------------------------------
    # RECOVERY
    # ----------------------------------------------------------
    def _recover_target(self):
        log.info("Recovery: sweeping to re-find target...")

        def on_tick():
            if self._check_abort(): return True
            self.vision.process_frame()
            return self._valid_detection()

        if self.fc.yaw_sweep(45, YAW_RATE_DEG, on_tick):
            log.info("Target re-found ✅")
            self._step_target_lock()
            return

        if self.fc.yaw_sweep(90, YAW_RATE_DEG, on_tick):
            log.info("Target re-found ✅")
            self._step_target_lock()
            return

        log.warning("Short recovery failed — full 360° sweep.")
        while not self._check_abort():
            if self._step_yaw_search() and self._step_target_lock():
                return

    # ----------------------------------------------------------
    # DROP
    # ----------------------------------------------------------
    def _step_drop(self):
        self._set_state(MissionState.DROP)
        log.info("DROP SEQUENCE starting...")

        # Stabilise
        self.fc.stop()
        log.info("Stabilising 2s before drop...")
        self.fc.hold_position(2.0)

        # Open hook
        self.hook_state = 'opening'
        log.info("Opening hook...")
        self.fc.release_payload()
        self.hook_state = 'open'
        log.info("HOOK OPEN ✓ — payload released")

        # Hold while payload clears
        self.fc.hold_position(2.0)
        self._tracking    = False
        self._target_gpos = None
        log.info("Drop complete.")

    # ----------------------------------------------------------
    # RECORD HOME
    # ----------------------------------------------------------
    def _record_home(self):
        if self.fc.vehicle:
            try:
                loc = self.fc.vehicle.location.global_relative_frame
                if loc.lat and loc.lon:
                    self._home_lat = loc.lat
                    self._home_lon = loc.lon
                    log.info(f"Home recorded: {self._home_lat:.6f}, {self._home_lon:.6f}")
                    return
            except:
                pass
        log.warning("Could not record home position — GPS not ready.")

    def _dist_to_home(self, lat, lon):
        dlat = (lat - self._home_lat) * 111139
        dlon = (lon - self._home_lon) * 111139 * math.cos(math.radians(lat))
        return math.sqrt(dlat**2 + dlon**2)

    # ----------------------------------------------------------
    # RETURN HOME
    # ----------------------------------------------------------
    def _step_return_home(self):
        self._set_state(MissionState.RETURN_HOME)
        if self._home_lat is None:
            log.warning("No home position — landing here.")
            return

        log.info(f"Returning home: {self._home_lat:.6f}, {self._home_lon:.6f}")

        from dronekit import LocationGlobalRelative

        # Phase 1: simple_goto until within 3m
        log.info("Return Phase 1 — simple_goto...")
        home_loc = LocationGlobalRelative(self._home_lat, self._home_lon, TAKEOFF_ALTITUDE)
        self.fc.vehicle.simple_goto(home_loc, groundspeed=RETURN_HOME_SPEED)

        timeout = time.time() + 90
        while True:
            if self._check_abort(): return
            if time.time() > timeout:
                log.warning("Return Phase 1 timeout.")
                break
            loc = self.fc.vehicle.location.global_relative_frame
            if loc.lat and loc.lon:
                dist = self._dist_to_home(loc.lat, loc.lon)
                log.debug(f"  Phase 1 dist: {dist:.2f}m")
                if dist < 3.0:
                    log.info("Within 3m — switching to precision.")
                    break
            self._sleep(0.3)

        self.fc.stop()
        self._sleep(0.5)

        # Phase 2: velocity-based precision correction
        log.info("Return Phase 2 — precision correction...")
        timeout2 = time.time() + 25

        while True:
            if self._check_abort(): return
            if time.time() > timeout2:
                log.warning("Phase 2 timeout — close enough.")
                break

            loc = self.fc.vehicle.location.global_relative_frame
            if not (loc.lat and loc.lon):
                self._sleep(0.1)
                continue

            dlat = (self._home_lat - loc.lat) * 111139
            dlon = (self._home_lon - loc.lon) * 111139 * math.cos(math.radians(loc.lat))
            dist = math.sqrt(dlat**2 + dlon**2)
            log.debug(f"  Phase 2 dist: {dist:.2f}m")

            if dist < 0.6:
                log.info(f"Home reached ✅  error={dist:.2f}m")
                break

            speed = min(0.4, max(0.12, dist * 0.25))
            heading_rad = math.radians(self.fc.heading)
            vx  =  dlat * math.cos(heading_rad) + dlon * math.sin(heading_rad)
            vy  = -dlat * math.sin(heading_rad) + dlon * math.cos(heading_rad)
            mag = math.sqrt(vx**2 + vy**2)
            if mag > 0:
                vx = (vx / mag) * speed
                vy = (vy / mag) * speed

            self.fc.send_body_velocity(vx, vy, 0)
            self._sleep(0.12)

        self.fc.stop()
        self._sleep(0.5)
        log.info("Return home complete.")

    # ----------------------------------------------------------
    # RELOAD
    # ----------------------------------------------------------
    def _step_reload(self):
        """
        Operator-checked reload at home (replaces the old fixed-timer reload).
          1. Hover and stabilise, then open the hook.
          2. Hold position until the operator presses CLOSE HOOK  -> hook closes.
          3. Hold position until the operator presses RELOAD DONE -> blue phase starts.
        No timeout: the drone holds a GPS position lock while it waits. ABORT and
        the heartbeat watchdog still work at every moment.
        """
        self._set_state(MissionState.RELOAD)
        self.reload_stage = None
        self._reload_close_event.clear()
        self._reload_done_event.clear()
        log.info("RELOAD — stabilising, then opening hook...")

        self.fc.hold_position(4.0)                  # settle over home (same 4 s as before)
        if self._check_abort(): return

        self.hook_state = 'opening'
        self.fc.release_payload()
        self.hook_state = 'open'
        log.info("HOOK OPEN ✓ — load the parcel, then press CLOSE HOOK")

        # Wait for CLOSE HOOK
        self.reload_stage = 'await_close'
        if not self._hold_until(self._reload_close_event):
            self.reload_stage = None
            return

        self.hook_state = 'closing'
        self.fc.close_payload()
        self.hook_state = 'closed'
        log.info("HOOK CLOSED ✓ — check the parcel, then press RELOAD DONE")

        # Wait for RELOAD DONE
        self.reload_stage = 'await_done'
        if not self._hold_until(self._reload_done_event):
            self.reload_stage = None
            return

        self.reload_stage = None
        self.fc.hold_position(2.0)                  # settle before the next search (same 2 s as before)
        log.info("Reload complete.")

    def _hold_until(self, event):
        """
        Hold a fixed GPS point at TAKEOFF_ALTITUDE (same position lock that
        hold_position() uses) until `event` is set. Returns False on abort/emergency.
        """
        anchor = self._current_pos()
        if anchor is None and self._home_lat is not None:
            anchor = (self._home_lat, self._home_lon)
        while not event.is_set():
            if self._check_abort():
                return False
            if anchor is not None and self.fc.vehicle:
                try:
                    self.fc._send_pos_hold(anchor[0], anchor[1], TAKEOFF_ALTITUDE)
                except Exception as e:
                    log.debug(f"Reload hold error: {e}")
            self._sleep(0.1)
        return True

    # ----------------------------------------------------------
    # LAND
    # ----------------------------------------------------------
    def _step_land(self):
        self._set_state(MissionState.LAND)
        log.info("Landing...")
        self.fc.stop()
        self._sleep(0.5)
        self.fc.land()

        timeout = time.time() + 60
        while self.fc.altitude > 0.25:
            if time.time() > timeout:
                log.warning("Landing timeout — assuming grounded.")
                break
            log.debug(f"  Landing... {self.fc.altitude:.2f}m")
            self._sleep(0.4)

        self._sleep(2.0)
        self.hook_state = 'closed'
        self.fc.disarm()
        log.info("Landed and disarmed ✅")

    # ----------------------------------------------------------
    # HELPERS
    # ----------------------------------------------------------
    def _set_state(self, s):
        self.state = s
        log.info(f"▶ {s.name} — {STATE_LABELS[s]}")

    def _check_abort(self):
        if self.fc._emergency_land:
            log.error("Watchdog triggered — EMERGENCY LAND")
            self._set_state(MissionState.ERROR)
            return True
        if self._abort:
            log.warning("Abort flag — stopping mission.")
            self._set_state(MissionState.ABORTED)
            return True
        return False

    def _sleep(self, seconds):
        """Interruptible sleep — checks abort + emergency every 50ms."""
        end = time.time() + seconds
        while time.time() < end:
            if self._abort or self.fc._emergency_land:
                return
            time.sleep(0.05)