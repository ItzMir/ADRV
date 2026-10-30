# ~/Documents/adrv/utils/arduino_link.py
import serial, json, time, threading, logging, os

logger = logging.getLogger(__name__)


class ArduinoLink:
    def __init__(self, port, baud=115200):
        self.port    = port
        self.baud    = baud
        self.ser     = None
        self.lock    = threading.Lock()
        self.running = False
        self._thread = None
        self.status  = {"flame1": False, "flame2": False, "pump": False}

    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._rx, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass

    def _open(self):
        if not os.path.exists(self.port):
            return False
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
            logger.info(f"Arduino: opened {self.port}")
            return True
        except Exception as e:
            logger.warning(f"Arduino: open {self.port} failed: {e}")
            self.ser = None
            return False

    def _rx(self):
        while self.running:
            if self.ser is None:
                if not self._open():
                    time.sleep(2.0)
                    continue
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="ignore").strip()
                if not line or not line.startswith("{"):
                    continue
                try:
                    j = json.loads(line)
                except json.JSONDecodeError:
                    continue
                with self.lock:
                    self.status = j
            except Exception as e:
                logger.warning(f"Arduino RX: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(1.0)

    def get_status(self):
        with self.lock:
            return dict(self.status)
