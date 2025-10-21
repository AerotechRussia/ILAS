"""
Semantic Segmentation Module
Uses a pre-trained model to classify pixels in an image
"""

import torch
import numpy as np
from torchvision import models
from torchvision.transforms import functional as F
from PIL import Image
from typing import Dict

class SemanticDetector:
    """
    Semantic detector using a pre-trained model
    """

    def __init__(self, config: dict):
        """
        Initialize the semantic detector

        Args:
            config: Configuration dictionary for semantic detection
        """
        self.device = config.get("device", "cpu")
        self.model = models.segmentation.deeplabv3_resnet101(pretrained=True)
        self.model.to(self.device)
        self.model.eval()

    def detect(self, image: np.ndarray) -> np.ndarray:
        """
        Perform semantic segmentation on an image

        Args:
            image: Image in numpy array format (H, W, C)

        Returns:
            Segmentation mask (H, W)
        """
        input_image = Image.fromarray(image)
        input_tensor = F.to_tensor(input_image).unsqueeze(0).to(self.device)

        with torch.no_grad():
            output = self.model(input_tensor)['out'][0]

        output_predictions = output.argmax(0).byte().cpu().numpy()
        return output_predictions

    def get_legend(self) -> Dict[int, str]:
        """
        Get the legend for the segmentation classes

        Returns:
            Dictionary mapping class index to class name
        """
        return {
            0: 'background', 1: 'aeroplane', 2: 'bicycle', 3: 'bird', 4: 'boat',
            5: 'bottle', 6: 'bus', 7: 'car', 8: 'cat', 9: 'chair', 10: 'cow',
            11: 'diningtable', 12: 'dog', 13: 'horse', 14: 'motorbike', 15: 'person',
            16: 'pottedplant', 17: 'sheep', 18: 'sofa', 19: 'train', 20: 'tvmonitor'
        }
