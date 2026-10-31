# ============================================================
#  AVION — gui.py   (Ground Control Station + Digital Twin)
#  Browser-based GUI using Flask + Flask-SocketIO.
#
#  Opens a web server on port 5000.
#  Open in browser: http://<RPi-IP>:5000
#
#  DROP-IN REPLACEMENT for the previous gui.py.
#  Nothing in mission_manager / flight_controller / vision /
#  config / main is changed. This file only READS state from
#  those objects and forwards the SAME operator commands:
#     start_mission    -> mission_manager.start()
#     abort_mission    -> mission_manager.abort()
#     confirm_approach -> mission_manager.confirm_approach()
#     ignore_target    -> mission_manager.ignore_target()
#     rover_mode       -> rover.set_mode('rover')
#     drone_mode       -> rover.set_mode('drone')
#     reload_close_hook-> mission_manager.reload_close_hook()
#     reload_done      -> mission_manager.reload_done()
#
#  The page (gui_assets/gcs.html) adds:
#   - Live 3D digital twin of the Avion airframe (from the team's
#     CAD assembly) driven by real Pixhawk attitude / position
#   - Flight instruments (attitude, heading, altitude, speed,
#     battery, motor outputs)
#   - Two-phase mission tracker, live pre-flight checklist,
#     D455 depth / alignment gauges, filterable log
#   - Pixhawk STATUSTEXT (pre-arm messages) in the log
#
#  All page assets are served locally from gui_assets/ so the
#  GCS works at a venue with no internet.
# ============================================================

import cv2
import os
import math
import threading
import time
import logging
from collections import deque
from flask import Flask, Response, send_from_directory, jsonify
from flask_socketio import SocketIO, emit as _sio_emit
from mission_manager import MissionState, STATE_LABELS
from logger import log
import config as _cfg

app      = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

_HERE       = os.path.dirname(os.path.abspath(__file__))
_ASSET_DIR  = os.path.join(_HERE, "gui_assets")
_PAGE_FILE  = os.path.join(_ASSET_DIR, "gcs.html")

TELEMETRY_PERIOD = 0.1     # seconds (10 Hz) — smooth twin; GUI-only, read-only


def _safe(fn, default=None):
    """Evaluate fn() and swallow any error — GUI must never break the flight code."""
    try:
        v = fn()
        return default if v is None else v
    except Exception:
        return default


def _num(v, nd=2):
    try:
        if v is None or (isinstance(v, float) and math.isnan(v)):
            return None
        return round(float(v), nd)
    except Exception:
        return None


class GUI:
    def __init__(self, mission_manager, vision_system, flight_controller, rover=None):
        self.mm      = mission_manager
        self.vision  = vision_system
        self.fc      = flight_controller
        self.rover   = rover
        self._running = True

        # Recent log lines, replayed to a browser when it (re)connects
        self._log_buf   = deque(maxlen=300)
        self._log_lock  = threading.Lock()

        # Extra read-only MAVLink data collected by passive listeners
        self._servo     = {}       # SERVO_OUTPUT_RAW  servoN_raw -> pwm
        self._throttle  = None     # VFR_HUD throttle %
        self._climb     = None     # VFR_HUD climb m/s
        self._cpu_temp  = None
        self._cpu_t_ts  = 0.0

        # Hook Python logger into SocketIO log panel
        _handler = _GUILogHandler(self._emit_log)
        logging.getLogger("terraavis").addHandler(_handler)

        # Passive MAVLink listeners (read-only; do not send anything)
        self._attach_listeners()

        # Register routes
        app.add_url_rule("/",                   "index",       self._index)
        app.add_url_rule("/video_feed",         "video_feed",  self._video_feed)
        app.add_url_rule("/assets/<path:fname>", "assets",     self._asset)
        app.add_url_rule("/gcs_config.json",    "gcs_config",  self._gcs_config)

        # Register SocketIO events (same names as before)
        socketio.on_event("start_mission",    self._on_start)
        socketio.on_event("abort_mission",    self._on_abort)
        socketio.on_event("confirm_approach", self._on_confirm_approach)
        socketio.on_event("ignore_target",    self._on_ignore_target)
        socketio.on_event("rover_mode",       self._on_rover_mode)
        socketio.on_event("drone_mode",       self._on_drone_mode)
        socketio.on_event("reload_close_hook", self._on_reload_close_hook)
        socketio.on_event("reload_done",      self._on_reload_done)
        socketio.on_event("connect",          self._on_client_connect)

        # Telemetry broadcast thread
        threading.Thread(target=self._broadcast_telemetry, daemon=True).start()

    # ----------------------------------------------------------
    # PASSIVE MAVLINK LISTENERS
    # ----------------------------------------------------------
    def _attach_listeners(self):
        v = getattr(self.fc, "vehicle", None)
        if v is None:
            return
        try:
            @v.on_message("SERVO_OUTPUT_RAW")
            def _servo_listener(_self, _name, msg):
                d = {}
                for i in range(1, 17):
                    val = getattr(msg, f"servo{i}_raw", None)
                    if val:
                        d[i] = int(val)
                self._servo = d

            @v.on_message("VFR_HUD")
            def _hud_listener(_self, _name, msg):
                self._throttle = getattr(msg, "throttle", None)
                self._climb    = getattr(msg, "climb", None)

            @v.on_message("STATUSTEXT")
            def _status_listener(_self, _name, msg):
                try:
                    text = msg.text
                    if isinstance(text, bytes):
                        text = text.decode("utf-8", "ignore")
                    text = str(text).strip("\x00 ").strip()
                    sev = int(getattr(msg, "severity", 6))
                    lvl = "ERROR" if sev <= 3 else "WARNING" if sev == 4 else "INFO"
                    if text:
                        self._push_log({"t": time.strftime("%H:%M:%S"), "level": lvl,
                                        "src": "FC", "msg": text})
                except Exception:
                    pass
        except Exception as e:
            log.debug(f"GUI: MAVLink listeners not attached: {e}")

    # ----------------------------------------------------------
    # ROUTES
    # ----------------------------------------------------------
    def _index(self):
        try:
            with open(_PAGE_FILE, "r", encoding="utf-8") as f:
                html = f.read()
        except Exception as e:
            html = (f"<h1>AVION GCS</h1><p>gui_assets/gcs.html not found next to gui.py "
                    f"({e}).</p>")
        resp = Response(html, mimetype="text/html")
        resp.headers["Cache-Control"] = "no-store"
        return resp

    def _asset(self, fname):
        return send_from_directory(_ASSET_DIR, fname, max_age=3600)

    def _gcs_config(self):
        """Read-only mission parameters for the page (display + twin geometry)."""
        hfov = 90.0
        try:
            import mission_manager as _mm
            hfov = getattr(_mm, "CAMERA_HFOV_DEG", 90.0)
        except Exception:
            pass
        keys = ["TAKEOFF_ALTITUDE", "HOVER_DURATION", "ALT_TOLERANCE", "YAW_RATE_DEG",
                "YAW_MAX_SWEEP", "LOCK_STABLE_FRAMES", "APPROACH_SPEED", "BLIND_CLOSE_SPEED",
                "RETURN_HOME_SPEED", "RELOAD_DURATION", "SAFE_DROP_DISTANCE",
                "ALIGN_PIXEL_THRESH", "MAX_LOST_FRAMES", "CAMERA_WIDTH", "CAMERA_HEIGHT",
                "CAMERA_FPS", "MAX_VALID_DEPTH", "MIN_CONTOUR_AREA", "MAX_CONTOUR_AREA",
                "SERVO_CHANNEL", "SERVO_PWM_CLOSED", "SERVO_PWM_OPEN", "MIN_BATTERY_VOLTAGE",
                "MIN_GPS_FIX", "MIN_GPS_SATELLITES", "MAVLINK_CONNECTION"]
        data = {k: getattr(_cfg, k, None) for k in keys}
        data["CAMERA_HFOV_DEG"] = hfov
        return jsonify(data)

    def _video_feed(self):
        return Response(
            self._gen_frames(),
            mimetype="multipart/x-mixed-replace; boundary=frame"
        )

    def _gen_frames(self):
        """MJPEG stream — yields annotated frames continuously (unchanged behaviour)."""
        while self._running:
            frame = self.vision.process_frame()
            if frame is None:
                frame = self._no_feed_frame()
            _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" +
                   buf.tobytes() + b"\r\n")
            time.sleep(0.1)   # ~10fps

    def _no_feed_frame(self):
        import numpy as np
        f = np.zeros((480, 640, 3), dtype="uint8")
        f[:] = (34, 20, 28)
        cv2.putText(f, "NO CAMERA FEED", (170, 230),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (232, 166, 185), 2, cv2.LINE_AA)
        cv2.putText(f, "Check RealSense D455 on USB 3.0", (170, 265),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 150, 170), 1, cv2.LINE_AA)
        return f

    # ----------------------------------------------------------
    # SOCKETIO EVENTS (same behaviour as before)
    # ----------------------------------------------------------
    def _on_client_connect(self, *args):
        with self._log_lock:
            history = list(self._log_buf)
        try:
            _sio_emit("log_history", history)      # only to the browser that just connected
        except Exception:
            pass

    def _on_start(self):
        log.info("START commanded from browser.")
        self.mm.start()

    def _on_abort(self):
        log.warning("ABORT commanded from browser.")
        self.mm.abort()

    def _on_confirm_approach(self):
        log.info("GUI: operator confirmed APPROACH.")
        self.mm.confirm_approach()

    def _on_ignore_target(self):
        log.info("GUI: operator chose IGNORE — continuing search.")
        self.mm.ignore_target()

    def _on_reload_close_hook(self):
        log.info("GUI: operator pressed CLOSE HOOK.")
        fn = getattr(self.mm, "reload_close_hook", None)
        if fn: fn()

    def _on_reload_done(self):
        log.info("GUI: operator pressed RELOAD DONE.")
        fn = getattr(self.mm, "reload_done", None)
        if fn: fn()

    def _on_rover_mode(self):
        log.info("ROVER MODE commanded from browser.")
        if self.rover:
            self.rover.set_mode('rover')

    def _on_drone_mode(self):
        log.info("DRONE MODE commanded from browser.")
        if self.rover:
            self.rover.set_mode('drone')

    # ----------------------------------------------------------
    # TELEMETRY BROADCAST
    # ----------------------------------------------------------
    def _read_cpu_temp(self):
        now = time.time()
        if now - self._cpu_t_ts < 5.0:
            return self._cpu_temp
        self._cpu_t_ts = now
        try:
            with open("/sys/class/thermal/thermal_zone0/temp") as f:
                self._cpu_temp = round(int(f.read().strip()) / 1000.0, 1)
        except Exception:
            self._cpu_temp = None
        return self._cpu_temp

    def _vehicle_block(self):
        """Extra read-only vehicle telemetry for instruments and the twin."""
        v = getattr(self.fc, "vehicle", None)
        if v is None:
            return {}
        att  = _safe(lambda: v.attitude)
        loc  = _safe(lambda: v.location.global_relative_frame)
        vel  = _safe(lambda: v.velocity, [None, None, None]) or [None, None, None]
        gps  = _safe(lambda: v.gps_0)
        bat  = _safe(lambda: v.battery)
        return {
            "hb_age":      _num(_safe(lambda: v.last_heartbeat), 1),
            "armed":       bool(_safe(lambda: v.armed, False)),
            "is_armable":  bool(_safe(lambda: v.is_armable, False)),
            "sys_status":  _safe(lambda: v.system_status.state, None),
            "roll":        _num(math.degrees(att.roll), 2) if att and att.roll is not None else None,
            "pitch":       _num(math.degrees(att.pitch), 2) if att and att.pitch is not None else None,
            "yaw":         _num(math.degrees(att.yaw), 2) if att and att.yaw is not None else None,
            "heading":     _num(_safe(lambda: v.heading), 1),
            "lat":         _num(loc.lat, 8) if loc else None,
            "lon":         _num(loc.lon, 8) if loc else None,
            "alt_rel":     _num(loc.alt, 2) if loc else None,
            "groundspeed": _num(_safe(lambda: v.groundspeed), 2),
            "vn":          _num(vel[0], 2), "ve": _num(vel[1], 2), "vd": _num(vel[2], 2),
            "climb":       _num(self._climb, 2),
            "throttle":    self._throttle,
            "gps_sats":    _safe(lambda: gps.satellites_visible, None) if gps else None,
            "gps_eph":     _safe(lambda: gps.eph, None) if gps else None,
            "batt_level":  _safe(lambda: bat.level, None) if bat else None,
            "batt_current": _num(_safe(lambda: bat.current, None), 1) if bat else None,
            "servo":       dict(self._servo),
        }

    def _mission_block(self):
        """Read-only view of mission_manager internals for the twin overlay."""
        mm = self.mm
        ign = []
        for o in list(_safe(lambda: mm._ignored, []) or []):
            try:
                g = o.get("gpos")
                ign.append({"bearing": _num(o.get("bearing"), 1),
                            "gpos": [g[0], g[1]] if g else None})
            except Exception:
                pass
        tg = _safe(lambda: mm._target_gpos)
        home_lat = _safe(lambda: mm._home_lat)
        home_lon = _safe(lambda: mm._home_lon)
        return {
            "state_label":      _safe(lambda: mm.state_label, ""),
            "reload_stage":     _safe(lambda: mm.reload_stage, None),
            "preflight_issues": list(_safe(lambda: mm.preflight_issues, []) or []),
            "home":             [home_lat, home_lon] if home_lat is not None and home_lon is not None else None,
            "ignored":          ign,
            "tracking":         bool(_safe(lambda: mm._tracking, False)),
            "target_gpos":      [tg[0], tg[1]] if tg else None,
            "locked_bearing":   _num(_safe(lambda: mm._locked_bearing), 1),
            "locked_depth":     _num(_safe(lambda: mm._locked_depth), 2),
        }

    def _vision_block(self):
        vs = self.vision
        bbox = _safe(lambda: vs.bbox)
        cen  = _safe(lambda: vs.centroid)
        return {
            "vision_running": bool(_safe(lambda: vs.running, False)),
            "bbox":     [int(x) for x in bbox] if bbox else None,
            "centroid": [int(cen[0]), int(cen[1])] if cen else None,
            "area":     _num(_safe(lambda: vs.contour_area, 0), 0),
        }

    def _broadcast_telemetry(self):
        """Push telemetry to all connected browsers (10 Hz)."""
        SERVO_RELEASED_STATES = (
            MissionState.DROP, MissionState.LAND, MissionState.COMPLETE
        )
        while self._running:
            try:
                data = {
                    # --- original fields (unchanged meaning) ---
                    "pixhawk":        self.fc.connected,
                    "battery":        self.fc.battery_voltage,
                    "gps":            self.fc.gps_fix,
                    "ekf":            self.fc.ekf_ok,
                    "altitude":       self.fc.altitude,
                    "mode":           self.fc.mode,
                    "detected":       self.vision.detected,
                    "aligned":        self.vision.aligned,
                    "depth":          round(self.vision.depth_m, 2) if self.vision.depth_m else None,
                    "state":          self.mm.state.name,
                    "progress":       self.mm.progress,
                    "mission_running": self.mm.state not in (
                        MissionState.IDLE, MissionState.COMPLETE,
                        MissionState.ABORTED, MissionState.ERROR
                    ),
                    "servo_released": self.mm.state in SERVO_RELEASED_STATES,
                    "target_color":   self.mm.target_color,
                    "hook_state":     self.mm.hook_state,
                    "ignored_count":  self.mm.ignored_count,
                    "rover_active":   self.rover.active if self.rover else False,
                    # --- new read-only fields ---
                    "ts":             time.time(),
                    "link_ok":        bool(_safe(lambda: self.fc.is_connected, False)),
                    "cpu_temp":       self._read_cpu_temp(),
                }
                data.update(self._vehicle_block())
                data.update(self._mission_block())
                data.update(self._vision_block())
                socketio.emit("telemetry", data)
            except Exception as e:
                log.debug(f"Telemetry broadcast error: {e}")
            time.sleep(TELEMETRY_PERIOD)

    # ----------------------------------------------------------
    # LOG FORWARDING
    # ----------------------------------------------------------
    def _push_log(self, entry):
        with self._log_lock:
            self._log_buf.append(entry)
        try:
            socketio.emit("log", entry)
        except Exception:
            pass

    def _emit_log(self, record_dict):
        self._push_log(record_dict)

    # ----------------------------------------------------------
    # RUN
    # ----------------------------------------------------------
    def run(self):
        log.info("=" * 50)
        log.info("  Browser GUI starting...")
        log.info("  Open in browser: http://<RPi-IP>:5000")
        log.info("=" * 50)
        try:
            # Flask-SocketIO >= 5.3 refuses to start Werkzeug without this flag
            socketio.run(app, host="0.0.0.0", port=5000, debug=False, use_reloader=False,
                         allow_unsafe_werkzeug=True)
        except TypeError:
            # Older Flask-SocketIO (the version this project was built on)
            socketio.run(app, host="0.0.0.0", port=5000, debug=False, use_reloader=False)
        self._running = False


# ----------------------------------------------------------
# Log handler
# ----------------------------------------------------------
class _GUILogHandler(logging.Handler):
    def __init__(self, cb):
        super().__init__()
        self.cb = cb

    def emit(self, record):
        try:
            self.cb({
                "t":     time.strftime("%H:%M:%S", time.localtime(record.created)),
                "level": record.levelname,
                "src":   "GCS",
                "msg":   record.getMessage(),
            })
        except Exception:
            pass
