"""
ML Obstacle Detector Module
Uses a machine learning model (e.g., YOLOv5) to detect various obstacles.
"""

import torch
import numpy as np
from typing import List, Dict, Tuple
import os

# COCO class names for a standard YOLOv5 model
COCO_CLASS_NAMES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
    'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
    'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush'
]


class MLObstacle:
    """Represents an obstacle detected by the ML model"""

    def __init__(self, bbox: Tuple[int, int, int, int], confidence: float, class_id: int, class_name: str):
        self.bbox = bbox  # (x1, y1, x2, y2)
        self.confidence = confidence
        self.class_id = class_id
        self.class_name = class_name

    def __repr__(self):
        return f"MLObstacle(class='{self.class_name}', conf={self.confidence:.2f}, bbox={self.bbox})"


class MLObstacleDetector:
    """
    ML-based obstacle detector using a pre-trained model like YOLOv5
    """

    def __init__(self, config: Dict):
        """
        Initialize the ML obstacle detector

        Args:
            config: Configuration dictionary for the detector
        """
        model_path = config.get("model_path", "models/yolov5s.pt")
        self.confidence_threshold = config.get("confidence_threshold", 0.4)
        self.device = config.get("device", "cpu")
        self.classes_to_detect = config.get("classes_to_detect", None)

        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found at {model_path}")

        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
        self.model.to(self.device)

        if self.classes_to_detect:
            self.class_indices = [COCO_CLASS_NAMES.index(name) for name in self.classes_to_detect if name in COCO_CLASS_NAMES]
        else:
            self.class_indices = None

    def detect(self, image: np.ndarray) -> List[MLObstacle]:
        """
        Detect obstacles in an image

        Args:
            image: Image in numpy array format (H, W, C)

        Returns:
            List of detected MLObstacle objects
        """
        image_tensor = torch.from_numpy(image).to(self.device).float() / 255.0
        if image_tensor.ndim == 3:
            image_tensor = image_tensor.permute(2, 0, 1).unsqueeze(0)

        results = self.model(image_tensor)

        predictions = results.xyxy[0]

        if self.class_indices:
            predictions = predictions[np.isin(predictions[:, -1].cpu(), self.class_indices)]

        # Filter by confidence
        predictions = predictions[predictions[:, 4] >= self.confidence_threshold]

        detected_obstacles = []
        for pred in predictions:
            bbox = pred[:4].cpu().numpy().astype(int)
            confidence = pred[4].cpu().item()
            class_id = int(pred[5].cpu().item())
            class_name = COCO_CLASS_NAMES[class_id]

            obstacle = MLObstacle(
                bbox=tuple(bbox),
                confidence=confidence,
                class_id=class_id,
                class_name=class_name
            )
            detected_obstacles.append(obstacle)

        return detected_obstacles
