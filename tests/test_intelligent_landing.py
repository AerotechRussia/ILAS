import unittest
import numpy as np

from ilas.landing.intelligent_landing import IntelligentLanding
from ilas.detection.obstacle_detector import Obstacle

class TestIntelligentLanding(unittest.TestCase):

    def setUp(self):
        config = {
            'max_slope': 15.0,
            'max_roughness': 0.3,
        }
        self.landing_system = IntelligentLanding(config)

    def test_landing_site_scoring(self):
        site_pos = np.array([5, 5, 0])
        target_pos = np.array([0, 0, 0])

        # Test a good landing site
        good_score = self.landing_system._calculate_landing_score(
            site_pos, target_pos, slope=5.0, roughness=0.1, semantic_safety_score=1.0
        )

        # Test a bad landing site
        bad_score = self.landing_system._calculate_landing_score(
            site_pos, target_pos, slope=20.0, roughness=0.5, semantic_safety_score=0.2
        )

        self.assertTrue(good_score > bad_score)
        self.assertTrue(0.0 <= good_score <= 1.0)
        self.assertTrue(0.0 <= bad_score <= 1.0)

if __name__ == '__main__':
    unittest.main()
