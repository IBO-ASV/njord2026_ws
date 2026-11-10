"""Small, ROS-independent helpers used by the YOLO inference worker."""

import threading

import numpy as np


class LatestFrameBuffer:
    """Single-slot blocking buffer that replaces stale, unprocessed frames."""

    def __init__(self):
        self._condition = threading.Condition()
        self._latest = None
        self._stopping = False

    def put(self, frame):
        with self._condition:
            if self._stopping:
                return False
            self._latest = frame
            self._condition.notify()
            return True

    def take(self):
        with self._condition:
            self._condition.wait_for(
                lambda: self._stopping or self._latest is not None
            )
            if self._stopping:
                return None
            frame = self._latest
            self._latest = None
            return frame

    def stop(self):
        with self._condition:
            self._stopping = True
            self._latest = None
            self._condition.notify_all()


def crop_image_roi(image, x_min_ratio, x_max_ratio, y_min_ratio, y_max_ratio):
    """Return a normalized image ROI and its origin in the full image."""
    height, width = image.shape[:2]
    x_min = max(0.0, min(1.0, float(x_min_ratio)))
    x_max = max(0.0, min(1.0, float(x_max_ratio)))
    y_min = max(0.0, min(1.0, float(y_min_ratio)))
    y_max = max(0.0, min(1.0, float(y_max_ratio)))

    x1 = int(round(width * min(x_min, x_max)))
    x2 = int(round(width * max(x_min, x_max)))
    y1 = int(round(height * min(y_min, y_max)))
    y2 = int(round(height * max(y_min, y_max)))

    x1 = max(0, min(width - 1, x1))
    x2 = max(x1 + 1, min(width, x2))
    y1 = max(0, min(height - 1, y1))
    y2 = max(y1 + 1, min(height, y2))
    return image[y1:y2, x1:x2], x1, y1, (x1, y1, x2 - 1, y2 - 1)


def shift_and_clip_bbox(xyxy, offset_x, offset_y, image_width, image_height):
    """Map a crop-local bounding box back to full-image pixel coordinates."""
    shifted = np.asarray(xyxy, dtype=float).copy()
    shifted[[0, 2]] += offset_x
    shifted[[1, 3]] += offset_y
    shifted[[0, 2]] = np.clip(shifted[[0, 2]], 0.0, float(image_width))
    shifted[[1, 3]] = np.clip(shifted[[1, 3]], 0.0, float(image_height))
    return shifted
