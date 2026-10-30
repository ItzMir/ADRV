# ~/Documents/adrv/config.py

# ---- ESP32 UART ----
# Tried in order; first one that exists AND yields JSON wins.
ESP32_PORT_CANDIDATES = ["/dev/ttyAMA0", "/dev/serial0", "/dev/ttyAMA10"]
ESP32_BAUD = 115200
ESP32_HEARTBEAT_MS = 200

# ---- Arduino ----
ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 115200

# ---- Pi motor driver (BCM pin numbers) ----
PI_LEFT_RPWM  = 26
PI_LEFT_LPWM  = 25
PI_RIGHT_RPWM = 12
PI_RIGHT_LPWM = 13
PI_PWM_FREQ   = 1000

# ---- Manual control ----
MANUAL_SPEED = 150
MANUAL_TIMEOUT_MS = 700

# ---- Heat-seeking mission ----
SEEK_SPEED       = 140
SEEK_INTERVAL_S  = 0.4
TURN_BIAS        = 4
STOP_DISTANCE_CM = 10.0        # must match ESP32's OBSTACLE_TRIGGER_CM
PAN_CENTER       = 90

# ---- YOLO ----
YOLO_MODEL_DIR  = "models/yolov8n_ncnn_model"
YOLO_INPUT_SIZE = 320
YOLO_CONF_THRESHOLD = 0.55

# ---- Human detection / distance ----
HUMAN_POLL_HZ       = 8

# Mission goes CONFIRM/DONE only when the ESTIMATED human distance (cm)
# is <= HUMAN_CONFIRM_DISTANCE_CM. Use the readings from the dashboard to
# calibrate this to your real-world 200 cm.
HUMAN_CONFIRM_DISTANCE_CM = 200.0
HUMAN_CLEAR_DISTANCE_CM   = 230.0
HUMAN_HYSTERESIS          = 3

HUMAN_HEIGHT_M      = 1.70
CAMERA_FOCAL_PX     = 1066.0   # <-- still being calibrated. See notes.

# ---- Server ----
WEB_HOST = "0.0.0.0"
WEB_PORT = 5000
LOG_LEVEL = "INFO"
