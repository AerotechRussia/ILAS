"""
Unit tests for the MLObstacleDetector
"""

import unittest
from unittest.mock import MagicMock, patch
import numpy as np
import torch
from ilas.detection.ml_obstacle_detector import MLObstacleDetector, COCO_CLASS_NAMES

class TestMLObstacleDetector(unittest.TestCase):
    """Test suite for the ML obstacle detector"""

    def setUp(self):
        """Set up the test environment"""
        self.config = {
            "model_path": "yolov5s",
            "confidence_threshold": 0.5,
            "device": "cpu"
        }

    @patch('torch.hub.load')
    def test_detector_initialization(self, mock_torch_load):
        """Test that the detector initializes correctly"""
        mock_model = MagicMock()
        mock_torch_load.return_value = mock_model

        detector = MLObstacleDetector(self.config)

        mock_torch_load.assert_called_with('ultralytics/yolov5', 'custom', path='yolov5s')
        mock_model.to.assert_called_with('cpu')
        self.assertEqual(detector.confidence_threshold, 0.5)

    @patch('torch.hub.load')
    def test_object_detection(self, mock_torch_load):
        """Test the object detection functionality"""
        # Create a mock model that returns predefined predictions
        mock_model = MagicMock()

        # Mock predictions (x1, y1, x2, y2, conf, class_id)
        mock_predictions = np.array([
            [10, 10, 100, 100, 0.9, 0],  # person
            [150, 50, 250, 150, 0.8, 2], # car
            [200, 200, 300, 300, 0.4, 1] # bicycle (below threshold)
        ])

        # The model output is a list containing a tensor
        mock_results = MagicMock()
        mock_results.xyxy = [torch.from_numpy(mock_predictions)]
        mock_model.return_value = mock_results

        mock_torch_load.return_value = mock_model

        detector = MLObstacleDetector(self.config)

        # Create a dummy image
        dummy_image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Perform detection
        obstacles = detector.detect(dummy_image)

        # Check the results
        self.assertEqual(len(obstacles), 2)

        # Check the first obstacle (person)
        self.assertEqual(obstacles[0].class_name, 'person')
        self.assertAlmostEqual(obstacles[0].confidence, 0.9)
        self.assertEqual(obstacles[0].bbox, (10, 10, 100, 100))

        # Check the second obstacle (car)
        self.assertEqual(obstacles[1].class_name, 'car')
        self.assertAlmostEqual(obstacles[1].confidence, 0.8)
        self.assertEqual(obstacles[1].bbox, (150, 50, 250, 150))

    @patch('torch.hub.load')
    def test_class_filtering(self, mock_torch_load):
        """Test filtering by class"""
        mock_model = MagicMock()

        mock_predictions = np.array([
            [10, 10, 100, 100, 0.9, 0],  # person
            [150, 50, 250, 150, 0.8, 2], # car
        ])

        mock_results = MagicMock()
        mock_results.xyxy = [torch.from_numpy(mock_predictions)]
        mock_model.return_value = mock_results

        mock_torch_load.return_value = mock_model

        # Configure to detect only cars
        config = self.config.copy()
        config['classes_to_detect'] = ['car']

        detector = MLObstacleDetector(config)

        dummy_image = np.zeros((480, 640, 3), dtype=np.uint8)
        obstacles = detector.detect(dummy_image)

        self.assertEqual(len(obstacles), 1)
        self.assertEqual(obstacles[0].class_name, 'car')


if __name__ == '__main__':
    unittest.main()
