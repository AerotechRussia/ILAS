import unittest
import numpy as np
from ilas.detection.obstacle_detector import ObstacleDetector

class TestObstacleDetector(unittest.TestCase):
    """Unit tests for the ObstacleDetector class."""

    def setUp(self):
        """Set up a default obstacle detector for testing."""
        config = {
            'sensors': [{'type': 'lidar', 'enabled': True}],
            'min_confidence': 0.2
        }
        self.detector = ObstacleDetector(config)

    def test_detect_obstacles(self):
        """Test the detection of obstacles from sensor data."""
        sensor_data = {'lidar': np.array([
            [1.0, 0.0, 0.0], [1.1, 0.1, 0.0], [0.9, -0.1, 0.0],
            [1.0, 0.1, 0.0], [1.1, 0.2, 0.0], [0.9, 0.0, 0.0],
            [4.0, 1.0, 0.0], [4.1, 1.1, 0.0], [3.9, 0.9, 0.0],
            [4.0, 1.1, 0.0], [4.1, 1.2, 0.0], [3.9, 1.0, 0.0]
        ])}
        obstacles = self.detector.detect_obstacles(sensor_data)
        self.assertEqual(len(obstacles), 2)

    def test_get_critical_obstacles(self):
        """Test the identification of critical obstacles."""
        current_position = np.array([0.0, 0.0, 0.0])
        heading = np.array([1.0, 0.0, 0.0])
        sensor_data = {'lidar': np.array([
            [2.0, 0.0, 0.0], [2.1, 0.1, 0.0], [1.9, -0.1, 0.0],
            [2.0, 0.1, 0.0], [2.1, 0.2, 0.0], [1.9, 0.0, 0.0],
            [8.0, 0.0, 0.0], [8.1, 0.1, 0.0], [7.9, -0.1, 0.0],
            [8.0, 0.1, 0.0], [8.1, 0.2, 0.0], [7.9, 0.0, 0.0]
        ])}
        self.detector.detect_obstacles(sensor_data)
        critical_obstacles = self.detector.get_critical_obstacles(current_position, heading, 5.0)
        self.assertEqual(len(critical_obstacles), 1)

if __name__ == '__main__':
    unittest.main()
