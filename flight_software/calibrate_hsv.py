#!/usr/bin/env python3
# ============================================================
#  TerraAvis — calibrate_hsv.py
#  Web-based HSV calibration — RED and BLUE tabs.
#
#  Run:  python3 calibrate_hsv.py
#  Open: http://192.168.1.29:5001
#
#  Switch between RED and BLUE tabs.
#  Adjust sliders until dummy is solid WHITE in mask.
#  Click SAVE to write to config.py.
# ============================================================

import cv2
import numpy as np
import pyrealsense2 as rs
import time
import re
from flask import Flask, Response, render_template_string, request, jsonify

app = Flask(__name__)

hsv_red = {
    "h_low1": 0,   "h_high1": 10,
    "h_low2": 165, "h_high2": 179,
    "s_low":  120, "v_low":   80
}
hsv_blue = {
    "h_low":  100, "h_high": 130,
    "s_low":  120, "v_low":  80
}

active_color = "red"
pipeline     = None
running      = False

HTML = """
<!DOCTYPE html>
<html>
<head>
  <title>TerraAvis — HSV Calibrator</title>
  <meta charset="UTF-8">
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { background: #040a05; color: #00ee44; font-family: 'Courier New', monospace; padding: 16px; }
    h2 { font-size: 16px; letter-spacing: 3px; margin-bottom: 4px; }
    .sub { color: #446644; font-size: 11px; margin-bottom: 16px; }
    .layout { display: flex; gap: 16px; flex-wrap: wrap; }
    .feeds { display: flex; gap: 10px; flex-wrap: wrap; margin-bottom: 10px; }
    .feed-box { border: 1px solid #00ee4430; border-radius: 4px; overflow: hidden; }
    .feed-label { background: #001500; padding: 4px 10px; font-size: 10px; color: #009922; border-bottom: 1px solid #00ee4415; }
    .feed-box img { display: block; width: 380px; }
    .tabs { display: flex; gap: 4px; margin-bottom: 0; }
    .tab { padding: 8px 24px; font-family: 'Courier New'; font-size: 12px; font-weight: bold; letter-spacing: 2px; cursor: pointer; border: 2px solid; border-bottom: none; background: #010d02; border-radius: 4px 4px 0 0; }
    .tab.red-tab  { border-color: #662222; color: #664444; }
    .tab.blue-tab { border-color: #222266; color: #444466; }
    .tab.active.red-tab  { background: #1a0505; color: #ee4444; border-color: #ee4444; }
    .tab.active.blue-tab { background: #05051a; color: #4488ff; border-color: #4488ff; }
    .panel { background: #010d02; border: 2px solid; border-radius: 0 4px 4px 4px; padding: 16px; min-width: 260px; }
    .panel.red-panel  { border-color: #662222; }
    .panel.blue-panel { border-color: #222266; }
    .panel.hidden { display: none; }
    .section { font-size: 10px; color: #009922; margin: 12px 0 8px; }
    .slider-row { margin-bottom: 10px; }
    .slider-row label { display: flex; justify-content: space-between; font-size: 11px; color: #446644; margin-bottom: 3px; }
    .red-val  { color: #ee6666 !important; font-weight: bold; }
    .blue-val { color: #6688ff !important; font-weight: bold; }
    input[type=range] { width: 100%; }
    input.red-slider  { accent-color: #ee4444; }
    input.blue-slider { accent-color: #4488ff; }
    .btn { width: 100%; padding: 10px; margin-top: 8px; border-radius: 3px; font-family: 'Courier New'; font-weight: bold; font-size: 12px; letter-spacing: 2px; cursor: pointer; border: 2px solid; }
    .btn-save-red  { background:#1a0505; border-color:#ee4444; color:#ee4444; }
    .btn-save-blue { background:#05051a; border-color:#4488ff; color:#4488ff; }
    .status { margin-top: 12px; padding: 8px; border-radius: 3px; font-size: 11px; text-align: center; display: none; }
    .status.ok  { background:#001500; border:1px solid #00ee44; color:#00ee44; display:block; }
    .status.err { background:#1a0000; border:1px solid #dd2222; color:#dd2222; display:block; }
    .hint { margin-top: 14px; padding: 10px; background: #010d02; border: 1px solid #00ee4415; border-radius: 3px; font-size: 10px; color: #446644; line-height: 1.8; }
  </style>
</head>
<body>
  <h2>◈ TERRAAVIS — HSV CALIBRATOR</h2>
  <p class="sub">Switch tabs for RED / BLUE. Adjust until dummy = solid WHITE in mask. Click SAVE.</p>

  <div class="layout">
    <div>
      <div class="feeds">
        <div class="feed-box">
          <div class="feed-label">📷 RGB FEED</div>
          <img src="/rgb_feed" alt="RGB">
        </div>
        <div class="feed-box">
          <div class="feed-label" id="mask-label">🎭 RED MASK</div>
          <img src="/mask_feed" id="mask-img" alt="Mask">
        </div>
      </div>
    </div>

    <div>
      <div class="tabs">
        <div class="tab red-tab active" id="tab-red" onclick="switchTab('red')">🔴 RED</div>
        <div class="tab blue-tab" id="tab-blue" onclick="switchTab('blue')">🔵 BLUE</div>
      </div>

      <!-- RED PANEL -->
      <div class="panel red-panel" id="panel-red">
        <div class="section">◈ RED RANGE 1 (Hue 0–30)</div>
        <div class="slider-row">
          <label>H LOW 1  <span class="red-val" id="r-h1l">0</span></label>
          <input type="range" class="red-slider" min="0" max="30" value="0" id="r_h_low1" oninput="updateRed()">
        </div>
        <div class="slider-row">
          <label>H HIGH 1 <span class="red-val" id="r-h1h">10</span></label>
          <input type="range" class="red-slider" min="0" max="30" value="10" id="r_h_high1" oninput="updateRed()">
        </div>
        <div class="section">◈ RED RANGE 2 (Hue 150–179)</div>
        <div class="slider-row">
          <label>H LOW 2  <span class="red-val" id="r-h2l">165</span></label>
          <input type="range" class="red-slider" min="150" max="179" value="165" id="r_h_low2" oninput="updateRed()">
        </div>
        <div class="slider-row">
          <label>H HIGH 2 <span class="red-val" id="r-h2h">179</span></label>
          <input type="range" class="red-slider" min="150" max="179" value="179" id="r_h_high2" oninput="updateRed()">
        </div>
        <div class="section">◈ SATURATION & VALUE</div>
        <div class="slider-row">
          <label>S LOW <span class="red-val" id="r-sl">120</span></label>
          <input type="range" class="red-slider" min="0" max="255" value="120" id="r_s_low" oninput="updateRed()">
        </div>
        <div class="slider-row">
          <label>V LOW <span class="red-val" id="r-vl">80</span></label>
          <input type="range" class="red-slider" min="0" max="255" value="80" id="r_v_low" oninput="updateRed()">
        </div>
        <button class="btn btn-save-red" onclick="saveRed()">💾 SAVE RED TO CONFIG.PY</button>
        <div class="status" id="status-red"></div>
        <div class="hint">🔴 Point camera at RED dummy.<br>Aim: dummy = solid WHITE, background = BLACK.<br>No scattered white noise.</div>
      </div>

      <!-- BLUE PANEL -->
      <div class="panel blue-panel hidden" id="panel-blue">
        <div class="section">◈ BLUE RANGE (Hue 80–140)</div>
        <div class="slider-row">
          <label>H LOW  <span class="blue-val" id="b-hl">100</span></label>
          <input type="range" class="blue-slider" min="80" max="140" value="100" id="b_h_low" oninput="updateBlue()">
        </div>
        <div class="slider-row">
          <label>H HIGH <span class="blue-val" id="b-hh">130</span></label>
          <input type="range" class="blue-slider" min="80" max="140" value="130" id="b_h_high" oninput="updateBlue()">
        </div>
        <div class="section">◈ SATURATION & VALUE</div>
        <div class="slider-row">
          <label>S LOW <span class="blue-val" id="b-sl">120</span></label>
          <input type="range" class="blue-slider" min="0" max="255" value="120" id="b_s_low" oninput="updateBlue()">
        </div>
        <div class="slider-row">
          <label>V LOW <span class="blue-val" id="b-vl">80</span></label>
          <input type="range" class="blue-slider" min="0" max="255" value="80" id="b_v_low" oninput="updateBlue()">
        </div>
        <button class="btn btn-save-blue" onclick="saveBlue()">💾 SAVE BLUE TO CONFIG.PY</button>
        <div class="status" id="status-blue"></div>
        <div class="hint">🔵 Point camera at BLUE dummy.<br>Aim: dummy = solid WHITE, background = BLACK.<br>No scattered white noise.</div>
      </div>
    </div>
  </div>

  <script>
    function switchTab(color) {
      document.getElementById('tab-red').classList.toggle('active', color === 'red');
      document.getElementById('tab-blue').classList.toggle('active', color === 'blue');
      document.getElementById('panel-red').classList.toggle('hidden', color !== 'red');
      document.getElementById('panel-blue').classList.toggle('hidden', color !== 'blue');
      document.getElementById('mask-label').textContent = color === 'red' ? '🎭 RED MASK' : '🎭 BLUE MASK';
      fetch('/set_active', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({color}) });
    }

    function updateRed() {
      const v = {
        h_low1:  +document.getElementById("r_h_low1").value,
        h_high1: +document.getElementById("r_h_high1").value,
        h_low2:  +document.getElementById("r_h_low2").value,
        h_high2: +document.getElementById("r_h_high2").value,
        s_low:   +document.getElementById("r_s_low").value,
        v_low:   +document.getElementById("r_v_low").value,
      };
      document.getElementById("r-h1l").textContent = v.h_low1;
      document.getElementById("r-h1h").textContent = v.h_high1;
      document.getElementById("r-h2l").textContent = v.h_low2;
      document.getElementById("r-h2h").textContent = v.h_high2;
      document.getElementById("r-sl").textContent  = v.s_low;
      document.getElementById("r-vl").textContent  = v.v_low;
      fetch('/update_red', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(v) });
    }

    function updateBlue() {
      const v = {
        h_low:  +document.getElementById("b_h_low").value,
        h_high: +document.getElementById("b_h_high").value,
        s_low:  +document.getElementById("b_s_low").value,
        v_low:  +document.getElementById("b_v_low").value,
      };
      document.getElementById("b-hl").textContent = v.h_low;
      document.getElementById("b-hh").textContent = v.h_high;
      document.getElementById("b-sl").textContent = v.s_low;
      document.getElementById("b-vl").textContent = v.v_low;
      fetch('/update_blue', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(v) });
    }

    function saveRed() {
      const v = {
        h_low1:  +document.getElementById("r_h_low1").value,
        h_high1: +document.getElementById("r_h_high1").value,
        h_low2:  +document.getElementById("r_h_low2").value,
        h_high2: +document.getElementById("r_h_high2").value,
        s_low:   +document.getElementById("r_s_low").value,
        v_low:   +document.getElementById("r_v_low").value,
      };
      fetch('/save_red', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(v) })
      .then(r => r.json()).then(d => {
        const el = document.getElementById("status-red");
        el.className = "status " + (d.ok ? "ok" : "err");
        el.textContent = d.ok ? "✅ Red saved to config.py!" : "❌ " + d.error;
      });
    }

    function saveBlue() {
      const v = {
        h_low:  +document.getElementById("b_h_low").value,
        h_high: +document.getElementById("b_h_high").value,
        s_low:  +document.getElementById("b_s_low").value,
        v_low:  +document.getElementById("b_v_low").value,
      };
      fetch('/save_blue', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(v) })
      .then(r => r.json()).then(d => {
        const el = document.getElementById("status-blue");
        el.className = "status " + (d.ok ? "ok" : "err");
        el.textContent = d.ok ? "✅ Blue saved to config.py!" : "❌ " + d.error;
      });
    }
  </script>
</body>
</html>
"""

def get_frame():
    global pipeline
    try:
        frames = pipeline.wait_for_frames(timeout_ms=500)
        cf = frames.get_color_frame()
        if not cf: return None
        return np.asanyarray(cf.get_data())
    except:
        return None

def apply_red_mask(img):
    hsv   = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv,
        np.array([hsv_red["h_low1"], hsv_red["s_low"], hsv_red["v_low"]]),
        np.array([hsv_red["h_high1"], 255, 255]))
    mask2 = cv2.inRange(hsv,
        np.array([hsv_red["h_low2"], hsv_red["s_low"], hsv_red["v_low"]]),
        np.array([hsv_red["h_high2"], 255, 255]))
    mask = cv2.bitwise_or(mask1, mask2)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    return cv2.dilate(cv2.erode(mask, k, iterations=1), k, iterations=2)

def apply_blue_mask(img):
    hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,
        np.array([hsv_blue["h_low"],  hsv_blue["s_low"], hsv_blue["v_low"]]),
        np.array([hsv_blue["h_high"], 255, 255]))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    return cv2.dilate(cv2.erode(mask, k, iterations=1), k, iterations=2)

def gen_rgb():
    while running:
        img = get_frame()
        if img is None:
            time.sleep(0.05)
            continue
        mask = apply_red_mask(img) if active_color == 'red' else apply_blue_mask(img)
        display = img.copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 300:
                x, y, w, h = cv2.boundingRect(largest)
                col = (255, 80, 0) if active_color == 'blue' else (0, 200, 60)
                cv2.rectangle(display, (x, y), (x+w, y+h), col, 2)
        H, W = display.shape[:2]
        cv2.line(display, (W//3, 0), (W//3, H), (60, 60, 60), 1)
        cv2.line(display, (W*2//3, 0), (W*2//3, H), (60, 60, 60), 1)
        _, buf = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 60])
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n")
        time.sleep(0.07)

def gen_mask():
    while running:
        img = get_frame()
        if img is None:
            time.sleep(0.05)
            continue
        mask = apply_red_mask(img) if active_color == 'red' else apply_blue_mask(img)
        _, buf = cv2.imencode(".jpg", cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), [cv2.IMWRITE_JPEG_QUALITY, 60])
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n")
        time.sleep(0.07)

@app.route("/")
def index():
    return render_template_string(HTML)

@app.route("/rgb_feed")
def rgb_feed():
    return Response(gen_rgb(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/mask_feed")
def mask_feed():
    return Response(gen_mask(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/set_active", methods=["POST"])
def set_active():
    global active_color
    active_color = request.json.get("color", "red")
    print(f"[CAL] Switched to: {active_color.upper()}")
    return jsonify(ok=True)

@app.route("/update_red", methods=["POST"])
def update_red():
    global hsv_red
    hsv_red.update(request.json)
    print(f"[RED] H1({hsv_red['h_low1']}-{hsv_red['h_high1']}) H2({hsv_red['h_low2']}-{hsv_red['h_high2']}) S>{hsv_red['s_low']} V>{hsv_red['v_low']}")
    return jsonify(ok=True)

@app.route("/update_blue", methods=["POST"])
def update_blue():
    global hsv_blue
    hsv_blue.update(request.json)
    print(f"[BLUE] H({hsv_blue['h_low']}-{hsv_blue['h_high']}) S>{hsv_blue['s_low']} V>{hsv_blue['v_low']}")
    return jsonify(ok=True)

@app.route("/save_red", methods=["POST"])
def save_red():
    global hsv_red
    hsv_red.update(request.json)
    try:
        with open("config.py", "r") as f: content = f.read()
        content = re.sub(r'RED_HSV_LOW1\s*=.*',  f'RED_HSV_LOW1        = ({hsv_red["h_low1"]},  {hsv_red["s_low"]}, {hsv_red["v_low"]})', content)
        content = re.sub(r'RED_HSV_HIGH1\s*=.*', f'RED_HSV_HIGH1       = ({hsv_red["h_high1"]}, 255, 255)', content)
        content = re.sub(r'RED_HSV_LOW2\s*=.*',  f'RED_HSV_LOW2        = ({hsv_red["h_low2"]},  {hsv_red["s_low"]}, {hsv_red["v_low"]})', content)
        content = re.sub(r'RED_HSV_HIGH2\s*=.*', f'RED_HSV_HIGH2       = ({hsv_red["h_high2"]}, 255, 255)', content)
        with open("config.py", "w") as f: f.write(content)
        print(f"✅ RED saved to config.py")
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e))

@app.route("/save_blue", methods=["POST"])
def save_blue():
    global hsv_blue
    hsv_blue.update(request.json)
    try:
        with open("config.py", "r") as f: content = f.read()
        content = re.sub(r'BLUE_HSV_LOW\s*=.*',  f'BLUE_HSV_LOW        = ({hsv_blue["h_low"]},  {hsv_blue["s_low"]}, {hsv_blue["v_low"]})', content)
        content = re.sub(r'BLUE_HSV_HIGH\s*=.*', f'BLUE_HSV_HIGH       = ({hsv_blue["h_high"]}, 255, 255)', content)
        with open("config.py", "w") as f: f.write(content)
        print(f"✅ BLUE saved to config.py")
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e))

def main():
    global pipeline, running
    pipeline = rs.pipeline()
    cfg      = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    try:
        pipeline.start(cfg)
        print("RealSense D455 started.")
    except Exception as e:
        print(f"RealSense failed: {e}")
        return
    running = True
    print("=" * 50)
    print("  TerraAvis HSV Calibrator — RED + BLUE")
    print("  Open: http://192.168.1.29:5001")
    print("=" * 50)
    try:
        app.run(host="0.0.0.0", port=5001, debug=False, threaded=True)
    finally:
        running = False
        pipeline.stop()
        print("Calibrator stopped.")

if __name__ == "__main__":
    main()