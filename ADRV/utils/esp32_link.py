# ~/Documents/adrv/utils/esp32_link.py
import serial, json, time, threading, logging, os

logger = logging.getLogger(__name__)


class ESP32Link:
    """
    UART link to the ESP32.

    * Tries each port candidate in order; the first one that yields a valid
      JSON *object* wins.
    * Never raises if the ESP32 is missing at start-up: the RX thread keeps
      retrying every 2 s, and the mission refuses to move on stale data.
    * Keeps the HUMAN_NEAR state and re-sends it every heartbeat so the
      ESP32's 2 s HUMAN_TIMEOUT never expires while a human is in range.
    * Telemetry / thermal timestamps let callers detect stale data.
    """

    RETRY_S = 2.0

    def __init__(self, port_candidates, baud=115200, heartbeat_ms=200):
        if isinstance(port_candidates, str):
            port_candidates = [port_candidates]
        self.candidates   = list(port_candidates)
        self.baud         = baud
        self.heartbeat_ms = heartbeat_ms

        self.ser  = None
        self.port = None

        self.lock       = threading.Lock()   # serial writes
        self.data_lock  = threading.Lock()   # telemetry / thermal
        self.human_near = False
        self.last_tx    = 0.0
        self.running    = False
        self._hb        = None
        self._rx        = None

        self.telemetry    = {}
        self.thermal      = {}
        self.telemetry_ts = 0.0
        self.thermal_ts   = 0.0

        self._open_any()

    # ---------- connection ----------
    def _probe(self, p):
        try:
            ser = serial.Serial(p, self.baud, timeout=0.5)
        except Exception as e:
            logger.warning(f"ESP32 UART: cannot open {p}: {e}")
            return None
        try:
            ser.reset_input_buffer()
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                raw = ser.readline()
                if not raw:
                    continue
                try:
                    obj = json.loads(raw.decode(errors="ignore").strip())
                except Exception:
                    continue
                if isinstance(obj, dict):
                    return ser
        except Exception as e:
            logger.warning(f"ESP32 UART: probe {p} failed: {e}")
        try:
            ser.close()
        except Exception:
            pass
        logger.warning(f"ESP32 UART: opened {p} but no valid JSON — trying next")
        return None

    def _open_any(self):
        for p in self.candidates:
            if not os.path.exists(p):
                logger.info(f"ESP32 UART: {p} does not exist — skipping")
                continue
            ser = self._probe(p)
            if ser is not None:
                self.ser  = ser
                self.port = p
                logger.info(f"ESP32 UART: opened {p} (data seen)")
                return True
        logger.warning("ESP32 UART: no working port yet — will keep retrying")
        return False

    @property
    def connected(self):
        return self.ser is not None

    # ---------- lifecycle ----------
    def start(self):
        self.running = True
        self._hb = threading.Thread(target=self._heartbeat, daemon=True)
        self._rx = threading.Thread(target=self._rx_loop,  daemon=True)
        self._hb.start()
        self._rx.start()

    def stop(self):
        self.running = False
        for t in (self._hb, self._rx):
            if t:
                t.join(timeout=1.5)
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass

    # ---------- TX ----------
    def send(self, cmd_dict):
        if cmd_dict.get("cmd") == "HUMAN_NEAR":
            self.human_near = bool(cmd_dict.get("value"))
        self._write(cmd_dict)

    def _write(self, cmd_dict):
        with self.lock:
            ser = self.ser
            if ser is None:
                return
            try:
                ser.write((json.dumps(cmd_dict) + "\n").encode())
                self.last_tx = time.monotonic()
            except Exception as e:
                logger.error(f"ESP32 write: {e}")

    def _heartbeat(self):
        while self.running:
            if time.monotonic() - self.last_tx > self.heartbeat_ms / 1000.0:
                self._write({"cmd": "HUMAN_NEAR", "value": self.human_near})
            time.sleep(0.02)

    # ---------- RX ----------
    def _rx_loop(self):
        while self.running:
            ser = self.ser
            if ser is None:
                time.sleep(self.RETRY_S)
                if self.running:
                    self._open_any()
                continue
            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="ignore").strip()
                if not line:
                    continue
                try:
                    d = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not isinstance(d, dict):
                    continue
                now = time.monotonic()
                with self.data_lock:
                    if "thermal" in d:
                        self.thermal = d
                        self.thermal_ts = now
                    elif "front_cm" in d:
                        self.telemetry = d
                        self.telemetry_ts = now
            except Exception as e:
                logger.warning(f"ESP32 RX: {e} — reconnecting")
                with self.lock:
                    try:
                        ser.close()
                    except Exception:
                        pass
                    self.ser = None
                time.sleep(0.5)

    # ---------- accessors ----------
    def get_telemetry(self):
        with self.data_lock:
            return dict(self.telemetry)

    def get_thermal(self):
        with self.data_lock:
            return dict(self.thermal)

    def telemetry_age(self):
        with self.data_lock:
            return float("inf") if self.telemetry_ts == 0 else time.monotonic() - self.telemetry_ts

    def thermal_age(self):
        with self.data_lock:
            return float("inf") if self.thermal_ts == 0 else time.monotonic() - self.thermal_ts
