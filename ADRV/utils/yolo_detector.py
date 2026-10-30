# ~/Documents/adrv/utils/yolo_detector.py
import cv2, time, logging, threading
from picamera2 import Picamera2
from ultralytics import YOLO

logger = logging.getLogger(__name__)


class HumanDetector:
    """
    Threaded YOLO worker.
    'near' decided by DISTANCE IN CM (pinhole model), not by bbox ratio.
    """

    def __init__(self, model_dir, input_size=320, conf=0.55):
        self.input_size = input_size
        self.conf       = conf
        self.model      = YOLO(model_dir, task="detect")

        self.picam = Picamera2()
        cfg = self.picam.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            buffer_count=2, queue=False)
        self.picam.configure(cfg)
        self.picam.start()

        self._frame       = None
        self._detected    = False
        self._near        = False
        self._distance_cm = None
        self._bbox        = None
        self._height_px   = None
        self._lock        = threading.Lock()

        self._running = False
        self._thread  = None
        self._send_fn = None

        self._confirm_cm  = 200.0
        self._clear_cm    = 230.0
        self._hysteresis  = 3
        self._near_count  = 0
        self._far_count   = 0

        self._focal_px       = 1066.0
        self._human_height_m = 1.70
        self._poll_period    = 1.0 / 8.0

    def start(self, send_fn=None, confirm_cm=200.0, clear_cm=230.0,
              hysteresis=3, focal_px=1066.0, human_height_m=1.70, hz=8):
        self._send_fn        = send_fn
        self._confirm_cm     = confirm_cm
        self._clear_cm       = clear_cm
        self._hysteresis     = hysteresis
        self._focal_px       = focal_px
        self._human_height_m = human_height_m
        self._poll_period    = 1.0 / max(1, hz)
        self._running = True
        self._thread  = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _worker(self):
        while self._running:
            try:
                self._step()
            except Exception as e:
                logger.exception(f"YOLO worker error: {e}")
            time.sleep(self._poll_period)

    def _step(self):
        # Picamera2 "RGB888" is laid out as B,G,R in memory — which is exactly
        # what OpenCV drawing and Ultralytics expect. Do NOT convert colours.
        frame = self.picam.capture_array()
        results = self.model.predict(frame, imgsz=self.input_size,
                                     conf=self.conf, device="cpu", verbose=False)

        best_h   = 0
        best_box = None
        for r in results:
            for box in r.boxes:
                if int(box.cls[0]) != 0:
                    continue
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                h = int(xyxy[3] - xyxy[1])
                if h > best_h:
                    best_h   = h
                    best_box = xyxy
                cv2.rectangle(frame, (xyxy[0], xyxy[1]),
                              (xyxy[2], xyxy[3]), (0, 255, 0), 2)
                cv2.putText(frame, f"person {float(box.conf[0]):.2f}",
                            (xyxy[0], max(20, xyxy[1] - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        detected    = best_box is not None
        distance_cm = None

        if detected and best_h > 0:
            distance_m  = (self._human_height_m * self._focal_px) / best_h
            distance_cm = distance_m * 100.0
            cv2.putText(frame, f"{distance_cm:.0f} cm",
                        (best_box[0], min(frame.shape[0] - 10, best_box[3] + 22)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

        if not detected or distance_cm is None:
            self._far_count  += 1
            self._near_count  = 0
        elif distance_cm <= self._confirm_cm:
            self._near_count += 1
            self._far_count   = 0
        elif distance_cm > self._clear_cm:
            self._far_count  += 1
            self._near_count  = 0

        was_near = self._near
        if self._near_count >= self._hysteresis:
            self._near = True
        elif self._far_count >= self._hysteresis:
            self._near = False

        if self._send_fn and self._near != was_near:
            try:
                self._send_fn({"cmd": "HUMAN_NEAR", "value": self._near})
            except Exception as e:
                logger.warning(f"HUMAN_NEAR send failed: {e}")
            logger.info(f"HUMAN_NEAR -> {self._near} (dist={distance_cm} cm)")

        with self._lock:
            self._frame       = frame
            self._height_px   = int(best_h) if detected else None
            self._detected    = detected
            self._distance_cm = distance_cm
            self._bbox        = best_box.tolist() if best_box is not None else None

    def get_frame(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def get_latest_status(self):
        with self._lock:
            return {
                "detected":    self._detected,
                "near":        self._near,
                "distance_cm": self._distance_cm,
                "bbox":        self._bbox,
                "height_px":   self._height_px,
            }

    def detect(self):
        with self._lock:
            return self._detected

    def release(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        try:
            self.picam.stop()
        except Exception:
            pass
