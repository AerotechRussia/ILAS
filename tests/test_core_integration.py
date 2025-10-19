import unittest
import numpy as np
from ilas.core import ILASCore
from ilas.utils.config import get_default_config

class TestCoreIntegration(unittest.TestCase):
    """Integration tests for the ILASCore class."""

    def setUp(self):
        """Set up a default ILASCore instance for testing."""
        config = get_default_config()
        self.ilas = ILASCore(config)
        self.ilas.start()

    def tearDown(self):
        """Stop the ILASCore instance."""
        self.ilas.stop()

    def test_navigate_and_land(self):
        """Test the full navigation and landing sequence."""
        target_position = np.array([10.0, 0.0, 5.0])
        sensor_data = {'lidar': np.array([
            [2.0, 0.0, 0.0], [2.1, 0.1, 0.0], [1.9, -0.1, 0.0]
        ])}

        # Initial position
        self.ilas.controller.set_telemetry({'position': np.array([0.0, 0.0, 5.0]), 'velocity': np.array([0.0, 0.0, 0.0])})

        nav_command, is_safe = self.ilas.navigate_to_target(target_position, sensor_data)
        self.assertTrue(is_safe)

        # Simulate moving the drone
        self.ilas.controller.set_telemetry({'position': np.array([5.0, 0.0, 5.0]), 'velocity': np.array([0.0, 0.0, 0.0])})

        landing_area_center = np.array([10.0, 0.0, 0.0])
        search_radius = 10.0
        success = self.ilas.execute_landing(landing_area_center, search_radius, sensor_data)
        self.assertTrue(success)

if __name__ == '__main__':
    unittest.main()
