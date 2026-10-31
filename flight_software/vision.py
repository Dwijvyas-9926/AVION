# ============================================================
#  TerraAvis — vision.py
#  Intel RealSense D455 — RGB + depth.
#
#  Supports RED and BLUE target detection.
#  Switch target using set_target_color('red') or ('blue').
#
#  Camera feed divided into 3 vertical alignment zones:
#  ┌────────────┬──────────────┬────────────┐
#  │  LEFT ZONE │ CENTRE ZONE  │ RIGHT ZONE │
#  │  misaligned│   ALIGNED    │ misaligned │
#  └────────────┴──────────────┴────────────┘
#
#  is_detection_stable() — visual detection alone, no depth needed.
#  Depth out of range (>7m) handled by blind close in approach.
# ============================================================

import cv2
import numpy as np
import threading
import pyrealsense2 as rs
from logger import log
from config import (
    CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS,
    RED_HSV_LOW1, RED_HSV_HIGH1,
    RED_HSV_LOW2, RED_HSV_HIGH2,
    BLUE_HSV_LOW, BLUE_HSV_HIGH,
    MIN_CONTOUR_AREA, MAX_CONTOUR_AREA, DEPTH_PATCH_SIZE, MAX_VALID_DEPTH,
    ALIGN_PIXEL_THRESH
)

ZONE_LEFT_EDGE  = CAMERA_WIDTH // 3
ZONE_RIGHT_EDGE = (CAMERA_WIDTH * 2) // 3


class VisionSystem:
    def __init__(self):
        self.pipeline  = rs.pipeline()
        self.align     = None
        self.running   = False
        self._lock     = threading.Lock()

        # Detection results — read by mission_manager
        self.frame_display = None
        self.detected      = False
        self.bbox          = None
        self.centroid      = None
        self.depth_m       = None
        self.contour_area  = 0
        self.aligned       = False

        # Current target colour — switched by mission_manager
        self.target_color  = 'red'

    def set_target_color(self, color):
        """Switch detection between 'red' and 'blue'."""
        self.target_color = color
        log.info(f"Vision target → {color.upper()}")

    # ----------------------------------------------------------
    # START / STOP
    # ----------------------------------------------------------
    def start(self):
        try:
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, CAMERA_WIDTH, CAMERA_HEIGHT,
                              rs.format.bgr8, CAMERA_FPS)
            cfg.enable_stream(rs.stream.depth, CAMERA_WIDTH, CAMERA_HEIGHT,
                              rs.format.z16,  CAMERA_FPS)

            profile = self.pipeline.start(cfg)
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_sensor.set_option(rs.option.enable_auto_exposure, 1)

            self.align   = rs.align(rs.stream.color)
            self.running = True
            log.info("RealSense D455 started.")
            log.info(f"  Resolution : {CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS}fps")
            log.info(f"  Zone edges : Left={ZONE_LEFT_EDGE}px  Right={ZONE_RIGHT_EDGE}px")
            return True

        except Exception as e:
            log.error(f"RealSense start failed: {e}")
            return False

    def stop(self):
        if self.running:
            self.pipeline.stop()
            self.running = False
            log.info("RealSense pipeline stopped.")

    # ----------------------------------------------------------
    # MAIN PROCESS FRAME
    # ----------------------------------------------------------
    def process_frame(self):
        if not self.running:
            return self.frame_display

        try:
            frames       = self.pipeline.wait_for_frames(timeout_ms=500)
            aligned      = self.align.process(frames)
            colour_frame = aligned.get_color_frame()
            depth_frame  = aligned.get_depth_frame()

            if not colour_frame or not depth_frame:
                return self.frame_display

            colour_img = np.asanyarray(colour_frame.get_data())
            depth_img  = np.asanyarray(depth_frame.get_data())

            # --- Detect based on current target colour ---
            if self.target_color == 'blue':
                detected, bbox, centroid, area = self._detect_blue(colour_img)
            else:
                detected, bbox, centroid, area = self._detect_red(colour_img)

            # --- Depth (internal only) ---
            depth = None
            if detected and centroid:
                depth = self._sample_depth(depth_img, centroid[0], centroid[1])
                if depth is not None and (depth <= 0.3 or depth > MAX_VALID_DEPTH):
                    depth = None

            # --- Alignment zone check ---
            is_aligned = False
            if detected and centroid:
                is_aligned = ZONE_LEFT_EDGE <= centroid[0] <= ZONE_RIGHT_EDGE

            # --- Update shared state ---
            with self._lock:
                self.detected     = detected
                self.bbox         = bbox
                self.centroid     = centroid
                self.depth_m      = depth
                self.contour_area = area
                self.aligned      = is_aligned

            # --- Build display frame ---
            display = self._draw_zones(colour_img)
            if detected and bbox and centroid:
                display = self._draw_bbox(display, bbox, centroid, is_aligned)
            display = self._draw_hud(display, is_aligned)

            with self._lock:
                self.frame_display = display

            return display

        except Exception as e:
            log.error(f"Vision frame error: {e}")
            return self.frame_display

    # ----------------------------------------------------------
    # 3-ZONE OVERLAY
    # ----------------------------------------------------------
    def _draw_zones(self, img):
        overlay = img.copy()
        H = CAMERA_HEIGHT

        cv2.rectangle(overlay, (0, 0),              (ZONE_LEFT_EDGE,  H), (0, 0, 60),  -1)
        cv2.rectangle(overlay, (ZONE_LEFT_EDGE, 0), (ZONE_RIGHT_EDGE, H), (0, 40, 0),  -1)
        cv2.rectangle(overlay, (ZONE_RIGHT_EDGE, 0),(CAMERA_WIDTH,    H), (0, 0, 60),  -1)

        display = cv2.addWeighted(overlay, 0.18, img, 0.82, 0)

        cv2.line(display, (ZONE_LEFT_EDGE,  0), (ZONE_LEFT_EDGE,  H), (80, 80, 80), 1)
        cv2.line(display, (ZONE_RIGHT_EDGE, 0), (ZONE_RIGHT_EDGE, H), (80, 80, 80), 1)

        cv2.putText(display, "LEFT",    (ZONE_LEFT_EDGE // 2 - 18, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (80, 80, 200), 1, cv2.LINE_AA)
        cv2.putText(display, "ALIGNED", (ZONE_LEFT_EDGE + 20, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (80, 180, 80), 1, cv2.LINE_AA)
        cv2.putText(display, "RIGHT",   (ZONE_RIGHT_EDGE + 18, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (80, 80, 200), 1, cv2.LINE_AA)

        return display

    # ----------------------------------------------------------
    # BOUNDING BOX
    # ----------------------------------------------------------
    def _draw_bbox(self, img, bbox, centroid, is_aligned):
        x, y, w, h = bbox
        cx, cy     = centroid

        # Colour scheme: red target = red/amber box, blue target = blue box
        if self.target_color == 'blue':
            box_col  = (200, 200, 0) if is_aligned else (255, 100, 0)   # cyan / orange
            text_col = box_col
        else:
            box_col  = (0, 220, 50)  if is_aligned else (0, 165, 255)   # green / amber
            text_col = box_col

        cv2.rectangle(img, (x, y), (x+w, y+h), box_col, 2)

        L = 14
        corners = [(x, y), (x+w, y), (x, y+h), (x+w, y+h)]
        for i, (px, py) in enumerate(corners):
            dx = L if i in (0, 2) else -L
            dy = L if i in (0, 1) else -L
            cv2.line(img, (px, py), (px+dx, py), box_col, 3)
            cv2.line(img, (px, py), (px, py+dy), box_col, 3)

        cv2.line(img, (cx-18, cy), (cx+18, cy), (0, 0, 220), 2)
        cv2.line(img, (cx, cy-18), (cx, cy+18), (0, 0, 220), 2)
        cv2.circle(img, (cx, cy), 5, (0, 0, 220), -1)

        # Label — shows dummy colour + depth status
        dummy_name = "BLUE DUMMY" if self.target_color == 'blue' else "RED DUMMY"
        if self.depth_m is not None:
            label = f"{dummy_name}  ✓ ALIGNED  {self.depth_m:.1f}m" if is_aligned else f"{dummy_name}  {self.depth_m:.1f}m"
        else:
            label = f"{dummy_name}  ✓ ALIGNED  (closing...)" if is_aligned else f"{dummy_name}  (beyond depth range)"
        cv2.putText(img, label, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, text_col, 2, cv2.LINE_AA)

        frame_cx = CAMERA_WIDTH // 2
        cv2.line(img, (frame_cx, 0), (frame_cx, CAMERA_HEIGHT), (50, 50, 50), 1)
        cv2.line(img, (cx, y - 5), (cx, 0), (100, 100, 220), 1)

        return img

    # ----------------------------------------------------------
    # HUD
    # ----------------------------------------------------------
    def _draw_hud(self, img, is_aligned):
        H, W = img.shape[:2]

        dummy_name = "BLUE DUMMY" if self.target_color == 'blue' else "RED DUMMY"

        if self.detected:
            if self.depth_m is not None:
                status = f"{dummy_name} ALIGNED  {self.depth_m:.1f}m" if is_aligned else f"{dummy_name} DETECTED  {self.depth_m:.1f}m"
            else:
                status = f"{dummy_name} ALIGNED — CLOSING" if is_aligned else f"{dummy_name} — DEPTH OUT OF RANGE"
            col = (0, 220, 50) if is_aligned else (0, 165, 255)
        else:
            status = f"SCANNING FOR {dummy_name}..."
            col    = (80, 80, 200)

        cv2.putText(img, status, (10, H - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, col, 2, cv2.LINE_AA)

        # Target colour indicator — top left
        indicator_col = (255, 80, 0) if self.target_color == 'blue' else (0, 0, 220)
        cv2.circle(img, (18, 18), 8, indicator_col, -1)
        cv2.putText(img, self.target_color.upper(), (32, 23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, indicator_col, 2, cv2.LINE_AA)

        cv2.circle(img, (W - 18, 30), 5, (0, 0, 200), -1)
        cv2.putText(img, "REC", (W - 36, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1)
        cv2.putText(img, "REALSENSE D455", (W - 140, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 100), 1)

        return img

    # ----------------------------------------------------------
    # RED DETECTION
    # ----------------------------------------------------------
    def _detect_red(self, bgr_img):
        hsv   = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array(RED_HSV_LOW1),  np.array(RED_HSV_HIGH1))
        mask2 = cv2.inRange(hsv, np.array(RED_HSV_LOW2),  np.array(RED_HSV_HIGH2))
        mask  = cv2.bitwise_or(mask1, mask2)

        k_s  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        k_l  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.erode(mask,  k_s, iterations=2)
        mask = cv2.dilate(mask, k_l, iterations=3)
        mask = cv2.erode(mask,  k_s, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, None, None, 0

        largest = max(contours, key=cv2.contourArea)
        area    = cv2.contourArea(largest)
        if area < MIN_CONTOUR_AREA:
            return False, None, None, 0
        if area > MAX_CONTOUR_AREA:
            log.debug(f"Red detection rejected — area {area:.0f}px² too large (dome/background)")
            return False, None, None, 0

        x, y, w, h = cv2.boundingRect(largest)
        return True, (x, y, w, h), (x + w//2, y + h//2), area

    # ----------------------------------------------------------
    # BLUE DETECTION
    # ----------------------------------------------------------
    def _detect_blue(self, bgr_img):
        hsv  = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(BLUE_HSV_LOW), np.array(BLUE_HSV_HIGH))

        k_s  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        k_l  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.erode(mask,  k_s, iterations=2)
        mask = cv2.dilate(mask, k_l, iterations=3)
        mask = cv2.erode(mask,  k_s, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, None, None, 0

        largest = max(contours, key=cv2.contourArea)
        area    = cv2.contourArea(largest)
        if area < MIN_CONTOUR_AREA:
            return False, None, None, 0
        if area > MAX_CONTOUR_AREA:
            log.debug(f"Blue detection rejected — area {area:.0f}px² too large (dome/background)")
            return False, None, None, 0

        x, y, w, h = cv2.boundingRect(largest)
        return True, (x, y, w, h), (x + w//2, y + h//2), area

    # ----------------------------------------------------------
    # DEPTH (internal only)
    # ----------------------------------------------------------
    def _sample_depth(self, depth_img, cx, cy):
        half = DEPTH_PATCH_SIZE // 2
        H, W = depth_img.shape
        x1, x2 = max(0, cx - half), min(W, cx + half + 1)
        y1, y2 = max(0, cy - half), min(H, cy + half + 1)
        patch  = depth_img[y1:y2, x1:x2].flatten().astype(float)
        valid  = patch[patch > 0]
        if len(valid) < 3:
            return None
        return round(float(np.median(valid)) / 1000.0, 3)

    # ----------------------------------------------------------
    # HELPERS FOR MISSION MANAGER
    # ----------------------------------------------------------
    def lateral_error_pixels(self):
        with self._lock:
            if not self.detected or self.centroid is None:
                return None
            return self.centroid[0] - (CAMERA_WIDTH // 2)

    def is_detection_stable(self):
        """
        Visual detection alone — depth NOT required.
        Depth may be None when target is beyond D455 range (~7m).
        Approach closes in blind until depth is acquired.
        """
        with self._lock:
            return self.detected