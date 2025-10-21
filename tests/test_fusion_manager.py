import unittest
import numpy as np

from ilas.fusion.fusion_manager import FusionManager
from ilas.detection.obstacle_detector import Obstacle

class TestFusionManager(unittest.TestCase):

    def setUp(self):
        config = {
            'cluster_threshold': 1.5,
            'sensor_weights': {
                'lidar': 0.9,
                'camera': 0.7,
            },
            'min_cluster_confidence': 0.6
        }
        self.fusion_manager = FusionManager(config)

    def test_fuse_obstacles(self):
        # Create some dummy obstacles
        obs1 = Obstacle(position=np.array([1, 1, 1]), size=np.array([0.5, 0.5, 0.5]), distance=1.73, confidence=0.8)
        obs2 = Obstacle(position=np.array([1.2, 1.2, 1.2]), size=np.array([0.5, 0.5, 0.5]), distance=2.08, confidence=0.9)
        obs3 = Obstacle(position=np.array([5, 5, 5]), size=np.array([1, 1, 1]), distance=8.66, confidence=0.95)

        sensor_obstacles = {
            'lidar': [obs1],
            'camera': [obs2, obs3]
        }

        fused_obstacles = self.fusion_manager.fuse_obstacles(sensor_obstacles)

        # We expect obs1 and obs2 to be fused, and obs3 to remain separate
        self.assertEqual(len(fused_obstacles), 2)

        # The first obstacle should be the fused one, with higher confidence
        self.assertTrue(fused_obstacles[0].confidence > 0.8)

if __name__ == '__main__':
    unittest.main()
