#!/usr/bin/env python3
import sys, os, time, logging, threading
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(BASE_DIR)                    # logs/ and models/ paths are relative
sys.path.insert(0, BASE_DIR)
import cv2
from flask import Flask, Response, render_template, jsonify, request

import config
from utils.esp32_link import ESP32Link
from utils.arduino_link import ArduinoLink
from utils.pi_motors import PiMotors
from utils.yolo_detector import HumanDetector
from utils.thermal_view import render as render_thermal
from utils.mission import Mission

os.makedirs("logs", exist_ok=True)
logging.basicConfig(
    level=getattr(logging, config.LOG_LEVEL),
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[logging.FileHandler("logs/mission.log"),
              logging.StreamHandler()]
)
log = logging.getLogger("main")

app = Flask(__name__)

if not os.path.isdir(config.YOLO_MODEL_DIR):
    sys.exit(f"YOLO model not found at {config.YOLO_MODEL_DIR}.\n"
             "Run once:  python scripts/export_yolo.py")

esp = ESP32Link(config.ESP32_PORT_CANDIDATES,
                config.ESP32_BAUD,
                config.ESP32_HEARTBEAT_MS)
esp.start()

ard = ArduinoLink(config.ARDUINO_PORT, config.ARDUINO_BAUD)
ard.start()

motors = PiMotors(config.PI_LEFT_RPWM, config.PI_LEFT_LPWM,
                  config.PI_RIGHT_RPWM, config.PI_RIGHT_LPWM,
                  config.PI_PWM_FREQ)

yolo = HumanDetector(config.YOLO_MODEL_DIR,
                     config.YOLO_INPUT_SIZE,
                     config.YOLO_CONF_THRESHOLD)
yolo.start(send_fn=esp.send,
           confirm_cm=config.HUMAN_CONFIRM_DISTANCE_CM,
           clear_cm=config.HUMAN_CLEAR_DISTANCE_CM,
           hysteresis=config.HUMAN_HYSTERESIS,
           focal_px=config.CAMERA_FOCAL_PX,
           human_height_m=config.HUMAN_HEIGHT_M,
           hz=config.HUMAN_POLL_HZ)

mission = Mission(esp, ard, yolo, motors, config)

_last_manual_cmd_ts = 0.0
_manual_lock = threading.Lock()

def _manual_touch():
    global _last_manual_cmd_ts
    with _manual_lock:
        _last_manual_cmd_ts = time.monotonic()

def _manual_watchdog():
    while True:
        with _manual_lock:
            idle_for = (time.monotonic() - _last_manual_cmd_ts) * 1000.0
        if mission.state in ("IDLE", "DONE") and motors.status != "STOP" \
                and idle_for > config.MANUAL_TIMEOUT_MS:
            motors.stop()
        time.sleep(0.05)

threading.Thread(target=_manual_watchdog, daemon=True).start()

def thermal_stream():
    while True:
        img = render_thermal(size=(640, 480), thermal_msg=esp.get_thermal())
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                   + buf.tobytes() + b"\r\n")
        time.sleep(0.15)

def camera_stream():
    while True:
        frame = yolo.get_frame()
        if frame is not None:
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + buf.tobytes() + b"\r\n")
        time.sleep(0.12)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video/thermal")
def video_thermal():
    return Response(thermal_stream(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/video/camera")
def video_camera():
    return Response(camera_stream(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/status")
def status():
    tele  = esp.get_telemetry()
    human = yolo.get_latest_status()
    return jsonify({
        "esp":         tele,
        "thermal":     esp.get_thermal().get("thermal", {}),
        "pan":         esp.get_thermal().get("pan", 90),
        "flame":       ard.get_status(),
        "mission":     mission.state,
        "motor":       motors.status,
        "human_near":  tele.get("human_near", False),
        "human_seen":  human["detected"],
        "human_cm":    human["distance_cm"],
        "human_h_px":  human["height_px"],
        "esp_ok":      esp.connected and esp.telemetry_age() < 2.0,
        "shaft_state": tele.get("shaft_state", "IDLE"),
    })

@app.route("/cmd/<name>")
def cmd(name):
    if name == "stop":
        motors.stop()
        _manual_touch()
        return jsonify({"ok": True, "cmd": "stop"})

    if name == "pan":
        try:
            a = int(request.args.get("angle", 90))
        except ValueError:
            return jsonify({"ok": False, "reason": "bad angle"}), 400
        esp.send({"cmd": "PAN", "angle": a})
        return jsonify({"ok": True, "cmd": "pan"})

    if mission.state not in ("IDLE", "DONE"):
        return jsonify({"ok": False, "reason": "mission_running"}), 409

    shaft_state = esp.get_telemetry().get("shaft_state", "IDLE")
    if shaft_state != "IDLE":
        motors.stop()
        return jsonify({"ok": False, "reason": "shafts_active"}), 409

    spd = config.MANUAL_SPEED
    if name == "forward":    motors.forward(spd)
    elif name == "backward": motors.backward(spd)
    elif name == "left":     motors.turn_left(spd)
    elif name == "right":    motors.turn_right(spd)
    else:
        return jsonify({"ok": False, "reason": "unknown"}), 400
    _manual_touch()
    return jsonify({"ok": True, "cmd": name})

@app.route("/mission/<name>")
def mission_ctrl(name):
    if name == "start":
        if esp.get_telemetry().get("shaft_state", "IDLE") != "IDLE":
            return jsonify({"ok": False, "reason": "shafts_active"}), 409
        mission.start()
    elif name == "stop":
        mission.stop()
        motors.stop()
    return jsonify({"ok": True, "state": mission.state})

if __name__ == "__main__":
    try:
        app.run(host=config.WEB_HOST, port=config.WEB_PORT, threaded=True)
    finally:
        motors.release()
        esp.stop()
        ard.stop()
        yolo.release()
