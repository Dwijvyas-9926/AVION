#!/usr/bin/env python3
# ============================================================
#  cam_test.py — live RGB feed, NO librealsense needed.
#
#    python3 cam_test.py
#    then open http://<pi-ip>:8000  in your browser
#
#  Uses the D455 as a plain USB webcam through OpenCV. This proves the
#  camera, cable, USB 3 port and power are all good — which is what you
#  want to know BEFORE spending two hours building librealsense.
#
#  It does NOT give you depth. Depth needs librealsense. See depth_test.py.
# ============================================================

import sys
import time

import cv2
from flask import Flask, Response

PORT = 8000
WIDTH, HEIGHT, FPS = 640, 480, 30

app = Flask(__name__)
cap = None
frames = 0
t_start = time.time()


def find_camera():
    """The D455 registers several /dev/video nodes. Only some give colour."""
    print("Scanning for a usable colour stream...")
    for idx in range(0, 12):
        c = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not c.isOpened():
            c.release()
            continue
        c.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        c.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        c.set(cv2.CAP_PROP_FPS, FPS)
        ok, frame = c.read()
        if ok and frame is not None and frame.ndim == 3:
            h, w = frame.shape[:2]
            print(f"  /dev/video{idx}  ->  {w}x{h} colour   USING THIS ONE")
            return c, idx
        print(f"  /dev/video{idx}  ->  opened, no colour frame")
        c.release()
    return None, None


def stream():
    global frames
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        frames += 1
        fps = frames / max(1e-6, time.time() - t_start)

        cv2.putText(frame, f"{frame.shape[1]}x{frame.shape[0]}  {fps:.1f} fps",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, "RGB only - depth needs librealsense",
                    (12, frame.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (180, 220, 255), 1)

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            yield (b"--f\r\nContent-Type: image/jpeg\r\n\r\n"
                   + buf.tobytes() + b"\r\n")


@app.route("/video")
def video():
    return Response(stream(),
                    mimetype="multipart/x-mixed-replace; boundary=f")


@app.route("/")
def index():
    return """<!doctype html><html><head><title>D455 RGB test</title>
    <style>body{background:#16181c;color:#d8d4cc;font-family:system-ui;
    text-align:center;padding:24px}img{max-width:96%;border-radius:8px;
    border:1px solid #333}h2{font-weight:600;letter-spacing:.02em}
    p{color:#8a857c;font-size:14px}</style></head><body>
    <h2>Intel RealSense D455 &mdash; RGB</h2>
    <img src="/video"><p>If this is smooth, the camera, cable, USB 3 port
    and power are all good.</p></body></html>"""


if __name__ == "__main__":
    cap, idx = find_camera()
    if cap is None:
        print("\nNo colour stream found.")
        print("  - is the camera on a BLUE USB 3 port?")
        print("  - check:  lsusb | grep -i intel")
        print("  - check:  lsusb -t | grep -i intel     (want 5000M, not 480M)")
        sys.exit(1)

    print(f"\n  Open  http://<pi-ip>:{PORT}  in your browser")
    print("  Ctrl+C to stop\n")
    try:
        app.run(host="0.0.0.0", port=PORT, threaded=True)
    finally:
        cap.release()