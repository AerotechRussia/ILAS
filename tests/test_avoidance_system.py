import unittest
import numpy as np
from ilas.avoidance.avoidance_system import AvoidanceSystem
from ilas.detection.obstacle_detector import Obstacle

class TestAvoidanceSystem(unittest.TestCase):
    """Unit tests for the AvoidanceSystem class."""

    def setUp(self):
        """Set up a default avoidance system for testing."""
        config = {'strategy': 'potential_field'}
        self.avoidance = AvoidanceSystem(config)

    def test_calculate_avoidance_vector(self):
        """Test the calculation of an avoidance vector."""
        current_position = np.array([0.0, 0.0, 0.0])
        target_position = np.array([10.0, 0.0, 0.0])
        current_velocity = np.array([1.0, 0.0, 0.0])
        obstacles = [Obstacle(np.array([2.0, 0.0, 0.0]), np.array([1.0, 1.0, 1.0]), 2.0, 1.0)]
        avoidance_vector, is_safe = self.avoidance.calculate_avoidance_vector(
            current_position, target_position, current_velocity, obstacles
        )
        self.assertFalse(is_safe)
        self.assertGreater(np.linalg.norm(avoidance_vector), 0)

if __name__ == '__main__':
    unittest.main()
