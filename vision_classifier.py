"""
vision_classifier.py — Perception / Classification layer.
Detects a box in the camera frame and extracts attributes:
  - color (HSV bucket)
  - size (bounding-box area -> small/medium/large)
  - barcode/QR payload (destination code), if present
Install: pip install opencv-python pyzbar
  (pyzbar also needs the system zbar library: `apt install libzbar0` on Linux)
"""

import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional
from pyzbar import pyzbar

# HSV ranges — tune to your lighting / boxes
COLOR_RANGES = {
    "red":    [(0, 120, 70), (10, 255, 255)],
    "blue":   [(100, 120, 70), (130, 255, 255)],
    "green":  [(40, 70, 70), (80, 255, 255)],
    "yellow": [(20, 120, 70), (35, 255, 255)],
}

SIZE_THRESHOLDS_PX = {  # bounding-box area in pixels, tune per camera setup
    "small": 15_000,
    "medium": 40_000,
}


@dataclass
class BoxAttributes:
    detected: bool
    color: Optional[str] = None
    size: Optional[str] = None
    barcode: Optional[str] = None
    bbox: Optional[tuple] = None


class BoxClassifier:
    def __init__(self, camera_index: int = 0):
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open camera")

    def _detect_color(self, hsv_roi) -> Optional[str]:
        best_color, best_count = None, 0
        for name, (lo, hi) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv_roi, np.array(lo), np.array(hi))
            count = cv2.countNonZero(mask)
            if count > best_count:
                best_color, best_count = name, count
        return best_color if best_count > 500 else None

    def _classify_size(self, area_px: float) -> str:
        if area_px < SIZE_THRESHOLDS_PX["small"]:
            return "small"
        elif area_px < SIZE_THRESHOLDS_PX["medium"]:
            return "medium"
        return "large"

    def capture_and_classify(self) -> BoxAttributes:
        ok, frame = self.cap.read()
        if not ok:
            return BoxAttributes(detected=False)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return BoxAttributes(detected=False)

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 3000:  # too small — likely noise, not a real box
            return BoxAttributes(detected=False)

        x, y, w, h = cv2.boundingRect(largest)
        roi = frame[y:y + h, x:x + w]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        color = self._detect_color(hsv_roi)
        size = self._classify_size(area)

        barcode_value = None
        for code in pyzbar.decode(frame):
            barcode_value = code.data.decode("utf-8")
            break

        return BoxAttributes(
            detected=True,
            color=color,
            size=size,
            barcode=barcode_value,
            bbox=(x, y, w, h),
        )

    def release(self):
        self.cap.release()