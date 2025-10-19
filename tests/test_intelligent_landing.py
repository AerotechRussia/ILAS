import unittest
import numpy as np
from ilas.landing.intelligent_landing import IntelligentLanding
from ilas.detection.obstacle_detector import Obstacle

class TestIntelligentLanding(unittest.TestCase):
    """Unit tests for the IntelligentLanding class."""

    def setUp(self):
        """Set up a default intelligent landing system for testing."""
        config = {'landing_mode': 'terrain'}
        self.landing = IntelligentLanding(config)

    def test_analyze_landing_area(self):
        """Test the analysis of a landing area."""
        landing_area_center = np.array([0.0, 0.0, 0.0])
        search_radius = 10.0
        terrain_data = np.zeros((20, 20))
        obstacles = [Obstacle(np.array([5.0, 0.0, 0.0]), np.array([1.0, 1.0, 1.0]), 5.0, 1.0)]
        landing_sites = self.landing.analyze_landing_area(
            landing_area_center, search_radius, terrain_data, obstacles
        )
        self.assertGreater(len(landing_sites), 0)

if __name__ == '__main__':
    unittest.main()
