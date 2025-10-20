"""
Person Detection Module
Uses YOLOv5 to detect people in images
"""

import torch
import numpy as np
from typing import List, Tuple

class PersonDetector:
    """
    Person detector using YOLOv5
    """

    def __init__(self, config: dict):
        """
        Initialize the person detector

        Args:
            config: Configuration dictionary for person detection
        """
        model_path = config.get("model_path")
        self.confidence_threshold = config.get("confidence_threshold", 0.5)
        self.device = config.get("device", "cpu")

        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
        self.model.to(self.device)

    def detect(self, image: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """
        Detect people in an image

        Args:
            image: Image in numpy array format (H, W, C)

        Returns:
            List of bounding boxes (x1, y1, x2, y2)
        """
        image_tensor = torch.from_numpy(image).to(self.device).float() / 255.0
        if image_tensor.ndim == 3:
            image_tensor = image_tensor.permute(2, 0, 1).unsqueeze(0)

        results = self.model(image_tensor)

        # Get bounding boxes for 'person' class (class 0)
        persons = results.xyxy[0][results.xyxy[0][:, -1] == 0]

        # Filter by confidence
        persons = persons[persons[:, 4] >= self.confidence_threshold]

        return persons[:, :4].cpu().numpy().astype(int).tolist()
