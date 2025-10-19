# tests/test_slam_system.py

import unittest
import numpy as np

from ilas.slam.slam_system import SLAMSystem

class TestSLAMSystem(unittest.TestCase):

    def test_slam_system_init(self):
        config = {}
        slam = SLAMSystem(config)
        self.assertIsNotNone(slam)

    def test_slam_system_update(self):
        config = {}
        slam = SLAMSystem(config)

        # Mock lidar data (360 degrees, 10m range)
        lidar_data = [10000] * 360
        sensor_data = {'lidar': lidar_data}

        try:
            slam.update(sensor_data)
        except Exception as e:
            self.fail(f"slam.update() raised an exception: {e}")

if __name__ == '__main__':
    unittest.main()
