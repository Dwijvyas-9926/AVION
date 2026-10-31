#!/usr/bin/env python3
# ============================================================
#  depth_test.py — live RGB + DEPTH side by side.
#
#  Needs librealsense built and pyrealsense2 importable.
#
#    python3 depth_test.py
#    then open http://<pi-ip>:8000
#
#  Click anywhere on the image to read the depth at that pixel.
#  Shows the colourised depth map, the depth at the centre, and the
#  percentage of pixels returning a valid reading — which is the number
#  that actually tells you whether depth is usable in your conditions.
# ============================================================

import sys
import time

import cv2
import numpy as np
from flask import Flask, Response, request, jsonify

PORT = 8000
W, H, FPS = 640, 480, 15
MAX_DEPTH = 8.0            # metres, for the colour scale

app = Flask(__name__)
pipeline = None
align = None
depth_scale = 0.001
latest = {"depth": None}
frames = 0
t0 = time.time()


def start_camera():
    global pipeline, align, depth_scale
    import pyrealsense2 as rs

    ctx = rs.context()
    devs = ctx.query_devices()
    if len(devs) == 0:
        print("No RealSense device found.")
        print("  check:  rs-enumerate-devices")
        return False
    d = devs[0]
    print(f"Found: {d.get_info(rs.camera_info.name)}")
    print(f"  serial   {d.get_info(rs.camera_info.serial_number)}")
    print(f"  firmware {d.get_info(rs.camera_info.firmware_version)}")
    print(f"  USB      {d.get_info(rs.camera_info.usb_type_descriptor)}   "
          f"(want 3.x — 2.1 means a USB 2 port and depth will struggle)")

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    profile = pipeline.start(cfg)

    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    align = rs.align(rs.stream.color)
    print(f"  depth scale {depth_scale}  (metres per raw unit)")
    return True


def stream():
    global frames
    while True:
        try:
            f = pipeline.wait_for_frames(timeout_ms=2000)
        except Exception as e:
            print(f"frame error: {e}")
            time.sleep(0.2)
            continue

        f = align.process(f)
        cf, df = f.get_color_frame(), f.get_depth_frame()
        if not cf or not df:
            continue

        colour = np.asanyarray(cf.get_data())
        depth_raw = np.asanyarray(df.get_data())
        depth_m = depth_raw.astype(np.float32) * depth_scale
        latest["depth"] = depth_m
        frames += 1

        # how much of the frame is actually returning depth
        valid = float(np.count_nonzero((depth_m > 0.2) & (depth_m < MAX_DEPTH)))
        valid_pct = valid / depth_m.size * 100

        # colourise
        clipped = np.clip(depth_m, 0, MAX_DEPTH)
        norm = (clipped / MAX_DEPTH * 255).astype(np.uint8)
        vis = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
        vis[depth_m <= 0.01] = (32, 30, 34)          # no reading = dark

        # centre reading — median of a patch, never a single pixel
        cy, cx = H // 2, W // 2
        patch = depth_m[cy - 4:cy + 5, cx - 4:cx + 5].ravel()
        good = patch[patch > 0.2]
        centre = float(np.median(good)) if good.size >= 3 else None

        for img in (colour, vis):
            cv2.drawMarker(img, (cx, cy), (255, 255, 255),
                           cv2.MARKER_CROSS, 18, 1)

        fps = frames / max(1e-6, time.time() - t0)
        cv2.putText(colour, "RGB", (12, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)
        cv2.putText(vis, "DEPTH", (12, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)

        txt = f"centre {centre:.3f} m" if centre else "centre: no reading"
        cv2.putText(vis, txt, (12, H - 40), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2)
        cv2.putText(vis, f"valid {valid_pct:.0f}%   {fps:.1f} fps",
                    (12, H - 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 1)

        both = np.hstack([colour, vis])
        ok, buf = cv2.imencode(".jpg", both, [cv2.IMWRITE_JPEG_QUALITY, 78])
        if ok:
            yield (b"--f\r\nContent-Type: image/jpeg\r\n\r\n"
                   + buf.tobytes() + b"\r\n")


@app.route("/video")
def video():
    return Response(stream(), mimetype="multipart/x-mixed-replace; boundary=f")


@app.route("/at")
def at():
    """Depth at a pixel, median of a 9x9 patch."""
    d = latest["depth"]
    if d is None:
        return jsonify({"m": None})
    x = int(request.args.get("x", 0)) % W
    y = int(request.args.get("y", 0))
    y = max(0, min(H - 1, y))
    p = d[max(0, y - 4):y + 5, max(0, x - 4):x + 5].ravel()
    g = p[p > 0.2]
    return jsonify({"m": round(float(np.median(g)), 3) if g.size >= 3 else None,
                    "x": x, "y": y})


@app.route("/")
def index():
    return """<!doctype html><html><head><title>D455 RGB + Depth</title>
<style>body{background:#16181c;color:#d8d4cc;font-family:system-ui;
text-align:center;padding:20px}img{max-width:98%;border-radius:8px;
border:1px solid #333;cursor:crosshair}h2{font-weight:600}
#r{font-size:20px;margin-top:14px;font-variant-numeric:tabular-nums}
p{color:#8a857c;font-size:14px}</style></head><body>
<h2>Intel RealSense D455 &mdash; RGB and Depth</h2>
<img id="v" src="/video">
<div id="r">click the image to read depth at that point</div>
<p>Turbo colour map, 0 to 8 m. Dark areas return no depth &mdash; too far,
too shiny, or washed out by sunlight.</p>
<script>
document.getElementById('v').onclick = function(e){
  const r = this.getBoundingClientRect();
  const sx = this.naturalWidth / r.width, sy = this.naturalHeight / r.height;
  const x = Math.round((e.clientX - r.left) * sx);
  const y = Math.round((e.clientY - r.top) * sy);
  fetch(`/at?x=${x}&y=${y}`).then(r=>r.json()).then(d=>{
    document.getElementById('r').textContent =
      d.m ? `${d.m.toFixed(3)} m  at (${d.x}, ${d.y})`
          : `no depth reading at (${d.x}, ${d.y})`;
  });
};
</script></body></html>"""


if __name__ == "__main__":
    try:
        __import__('pyrealsense2')
    except ImportError:
        print("pyrealsense2 not importable.")
        print("  librealsense must be built first, and the .so files copied")
        print("  into the venv. See the setup guide.")
        sys.exit(1)

    if not start_camera():
        sys.exit(1)
    print(f"\n  Open  http://<pi-ip>:{PORT}\n  Ctrl+C to stop\n")
    try:
        app.run(host="0.0.0.0", port=PORT, threaded=True)
    finally:
        pipeline.stop()