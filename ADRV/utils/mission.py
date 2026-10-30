# ~/Documents/adrv/utils/mission.py
import time, threading, logging

logger = logging.getLogger(__name__)

STALE_S = 2.0   # ESP32 data older than this => do not drive


class State:
    IDLE     = "IDLE"
    SEEK     = "SEEK"
    OBSTACLE = "OBSTACLE"
    SPRAY    = "SPRAY"
    CONFIRM  = "CONFIRM"
    DONE     = "DONE"


class Mission:
    """
    Priority: FLAME > stale data > shafts_active > human_near > obstacle > heat-seek.
    """

    def __init__(self, esp, arduino, yolo, motors, cfg):
        self.esp    = esp
        self.ard    = arduino
        self.yolo   = yolo
        self.motors = motors
        self.cfg    = cfg

        self.state       = State.IDLE
        self.stop_flag   = False
        self.thread      = None
        self._stale_warn = False

    def start(self):
        if self.thread and self.thread.is_alive():
            if self.state == State.DONE:
                logger.info("Restarting mission from DONE")
                self.state = State.SEEK
            else:
                logger.info("Mission already running")
            return
        self.stop_flag = False
        self.state     = State.SEEK
        try:
            self.esp.send({"cmd": "PAN", "angle": self.cfg.PAN_CENTER})
        except Exception:
            pass
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_flag = True
        if self.thread:
            self.thread.join(timeout=3.0)
        self.motors.stop()
        self.state = State.IDLE

    def _loop(self):
        while not self.stop_flag:
            try:
                self._tick()
            except Exception as e:
                logger.exception(f"Mission tick error: {e}")
                self.motors.stop()
            time.sleep(self.cfg.SEEK_INTERVAL_S)
        self.motors.stop()

    def _tick(self):
        flame = self.ard.get_status()

        if flame.get("flame1") or flame.get("flame2"):
            if self.state != State.SPRAY:
                logger.warning("FLAME detected — stopping and spraying")
            self.motors.stop()
            self.state = State.SPRAY
            return

        if self.state == State.SPRAY:
            logger.info("Flame cleared — resuming heat seek")
            self.state = State.SEEK

        # Never drive on stale / missing ESP32 data.
        if self.esp.telemetry_age() > STALE_S:
            self.motors.stop()
            if not self._stale_warn:
                logger.warning("ESP32 telemetry stale/missing — holding still")
                self._stale_warn = True
            return
        if self._stale_warn:
            logger.info("ESP32 telemetry restored")
            self._stale_warn = False

        tele    = self.esp.get_telemetry()
        human   = self.yolo.get_latest_status()

        if tele.get("shaft_state", "IDLE") != "IDLE":
            self.motors.stop()
            return

        if human["detected"] and human["near"]:
            self.motors.stop()
            self.state = State.CONFIRM
            dist = human.get("distance_cm")
            logger.info(f"Human near (dist={dist if dist is None else round(dist,1)} cm) — CONFIRM")
            if self.yolo.detect():
                logger.info("YOLO confirms person — DONE")
                self.state = State.DONE
            return

        if self.state == State.DONE:
            self.motors.stop()
            return

        front = tele.get("front_cm", 200)
        if front is not None and front <= self.cfg.STOP_DISTANCE_CM:
            self.motors.stop()
            if self.state != State.OBSTACLE:
                logger.info(f"Obstacle at {front} cm — waiting for shafts")
            self.state = State.OBSTACLE
            return
        if self.state == State.OBSTACLE:
            self.state = State.SEEK

        self._seek()

    def _seek(self):
        if self.esp.thermal_age() > STALE_S:
            self.motors.stop()
            return
        t = self.esp.get_thermal().get("thermal", {})
        if not t:
            self.motors.stop()
            return
        max_c  = t.get("max_c", 16)
        offset = max_c - 16
        if offset > self.cfg.TURN_BIAS:
            self.motors.turn_right(self.cfg.SEEK_SPEED)
        elif offset < -self.cfg.TURN_BIAS:
            self.motors.turn_left(self.cfg.SEEK_SPEED)
        else:
            self.motors.forward(self.cfg.SEEK_SPEED)
