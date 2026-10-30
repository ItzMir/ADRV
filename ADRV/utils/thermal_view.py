# ~/Documents/adrv/utils/thermal_view.py
import cv2, numpy as np


def render(size=(640, 480), thermal_msg=None):
    img = np.zeros((size[1], size[0], 3), dtype=np.uint8)
    if not thermal_msg:
        cv2.putText(img, "waiting for thermal data...",
                    (30, size[1] // 2), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (200, 200, 200), 2)
        return img

    t = thermal_msg.get("thermal", {})
    if not t:
        return img

    maxT  = t.get("max", 30)
    minT  = t.get("min", 25)
    maxR  = t.get("max_r", 0)
    maxC  = t.get("max_c", 0)
    centR = t.get("centroid_r", maxR)
    centC = t.get("centroid_c", maxC)

    grid_w = size[0] // 32
    grid_h = size[1] // 24
    for r in range(24):
        for c in range(32):
            d = ((r - centR) ** 2 + (c - centC) ** 2) ** 0.5
            v = max(0, 255 - int(d * 20))
            colour = (0, v // 2, v)
            cv2.rectangle(img,
                          (c * grid_w, r * grid_h),
                          ((c + 1) * grid_w, (r + 1) * grid_h),
                          colour, -1)

    cx = int((maxC + 0.5) * grid_w)
    cy = int((maxR + 0.5) * grid_h)
    cv2.circle(img, (cx, cy), 22, (0, 255, 0), 2)
    cv2.putText(img, f"{maxT:.1f}C", (cx + 26, cy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(img, f"min {minT:.1f}  max {maxT:.1f}",
                (10, size[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return img
