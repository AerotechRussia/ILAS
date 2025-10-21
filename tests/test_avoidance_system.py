import unittest
import numpy as np

from ilas.avoidance.avoidance_system import AvoidanceSystem
from ilas.detection.obstacle_detector import Obstacle

class TestAvoidanceSystem(unittest.TestCase):

    def test_potential_field_strategy(self):
        config = {'strategy': 'potential_field'}
        avoidance_system = AvoidanceSystem(config)

        drone_pos = np.array([0, 0, 0])
        target_pos = np.array([10, 0, 0])
        obstacles = [Obstacle(np.array([5, 0, 0]), np.array([1, 1, 1]), 5, 0.9)]

        vector, is_safe = avoidance_system.calculate_avoidance_vector(drone_pos, target_pos, None, obstacles)

        # Obstacle is slightly to the right (positive y)
        obstacles = [Obstacle(np.array([5, 0.5, 0]), np.array([1, 1, 1]), 5, 0.9)]

        vector, is_safe = avoidance_system.calculate_avoidance_vector(drone_pos, target_pos, None, obstacles)

        # The repulsive force from the obstacle at (5, 0.5, 0) should create a negative y component
        # to push the drone to the left.
        self.assertTrue(vector[1] < 0.0)

    def test_rrt_strategy(self):
        config = {'strategy': 'rrt'}
        avoidance_system = AvoidanceSystem(config)

        drone_pos = np.array([0, 0, 0])
        target_pos = np.array([10, 0, 0])
        obstacles = [Obstacle(np.array([5, 0, 0]), np.array([1, 1, 1]), 5, 0.9)]

        vector, is_safe = avoidance_system.calculate_avoidance_vector(drone_pos, target_pos, None, obstacles)

        # RRT should find a path around the obstacle
        self.assertTrue(is_safe)
        self.assertIsNotNone(vector)

if __name__ == '__main__':
    unittest.main()
