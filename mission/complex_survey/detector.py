import os

# Force CPU-only before torch/ultralytics get imported anywhere in this
# process. Without this, torch still probes for a CUDA device on import
# and prints a driver-mismatch UserWarning even though we always pass
# device="cpu" explicitly below. Setting this here (module level) means
# it's in place before Detector() is ever constructed.
os.environ.setdefault("CUDA_VISIBLE_DEVICES", "")

from typing import Optional


# COCO car class id for Ultralytics YOLOv8 COCO models
CAR_CLASS_ID = 2

DEFAULT_MODEL_PATH = "yolov8s.pt"
DEFAULT_CONF_THR    = 0.6


class Detector:
    """
    Standalone YOLOv8 vehicle detector.

    Loads the model once on construction, then runs inference on single
    image files. No MAVLink, no Gazebo dependency -- completely separate
    from drone.py and camera.py.
    """

    def __init__(
        self,
        model_path: str = DEFAULT_MODEL_PATH,
        conf_threshold: float = DEFAULT_CONF_THR,
        class_id: int = CAR_CLASS_ID,
        device: str = "cpu",
    ):
        from ultralytics import YOLO

        print("[DETECTOR] Loading YOLOv8 model from {0}...".format(model_path))
        self._model          = YOLO(model_path)
        self._conf_threshold = conf_threshold
        self._class_id       = class_id
        self._device         = device
        print("[DETECTOR] Model loaded.")

    def detect(self, image_path: str) -> Optional[float]:
        """
        Run detection on a single image file.

        Returns the highest confidence score among detections of the
        target class (e.g. car), or None if no such detection was found.
        """
        import cv2
        from collections import Counter

        frame = cv2.imread(image_path)
        if frame is None:
            print("[DETECTOR] Could not read image: {0}".format(image_path))
            return None

        results = self._model(
            frame, conf=self._conf_threshold, verbose=False, device=self._device
        )
        r = results[0]

        if r.boxes is None or len(r.boxes) == 0:
            return None

        class_ids   = r.boxes.cls.cpu().numpy().astype(int).tolist()
        confidences = r.boxes.conf.cpu().numpy().tolist()

        best_conf: Optional[float] = None
        for class_id, conf in zip(class_ids, confidences):
            if class_id == self._class_id:
                if best_conf is None or conf > best_conf:
                    best_conf = conf

        return best_conf

    def detect_all(self, image_path: str, min_conf: float = 0.05):
        """
        Run detection and return EVERY detection regardless of class,
        for debugging/calibration purposes. Uses a much lower confidence
        floor than the main detect() method so low-confidence guesses
        are still visible.

        Returns a list of (class_id, class_name, confidence) tuples,
        sorted by confidence descending. Empty list if nothing detected.
        """
        import cv2

        frame = cv2.imread(image_path)
        if frame is None:
            print("[DETECTOR] Could not read image: {0}".format(image_path))
            return []

        results = self._model(frame, conf=min_conf, verbose=False, device=self._device)
        r = results[0]

        if r.boxes is None or len(r.boxes) == 0:
            return []

        class_ids   = r.boxes.cls.cpu().numpy().astype(int).tolist()
        confidences = r.boxes.conf.cpu().numpy().tolist()

        detections = [
            (class_id, self._model.names[class_id], conf)
            for class_id, conf in zip(class_ids, confidences)
        ]
        detections.sort(key=lambda d: d[2], reverse=True)
        return detections

    def detect_anomaly(self, image_path: str, threshold: float = 0.15):
        """
        Returns the single highest-confidence detection above threshold,
        regardless of class -- used for the "is there anything here at
        all" stage. None if nothing exceeds the threshold.

        Returns (class_id, class_name, confidence) or None.
        """
        detections = self.detect_all(image_path, min_conf=threshold)
        if not detections:
            return None
        return detections[0]   # already sorted by confidence descending