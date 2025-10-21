"""
Unit tests for the ILASCore class
"""

import unittest
from unittest.mock import MagicMock, patch, ANY
import numpy as np
import collections
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping
from ilas.core import ILASCore

class TestILASCore(unittest.TestCase):
    """Test suite for the ILAS core system"""

    def setUp(self):
        """Set up the test environment"""
        self.config = {
            'controller': {'controller_type': 'simulation'},
            'detection': {},
            'avoidance': {},
            'landing': {},
            'slam': {},
            'terrain_mapping': {},
            'sensor_fusion': {},
            'mission_planning': {},
            'geofence': {},
            'failsafe': {},
            'wind_estimation': {},
            'person_detection': {},
        }

    @patch('ilas.core.connect')
    @patch('ilas.core.ObstacleDetector')
    @patch('ilas.core.AvoidanceSystem')
    @patch('ilas.core.IntelligentLanding')
    @patch('ilas.core.ControllerManager')
    @patch('ilas.core.SLAMSystem')
    @patch('ilas.core.TerrainMapper')
    @patch('ilas.core.SensorFusionEKF')
    @patch('ilas.core.MissionPlanner')
    @patch('ilas.core.GeofenceManager')
    @patch('ilas.core.FailsafeManager')
    @patch('ilas.core.WindEstimator')
    @patch('ilas.core.VersatileLanding')
    @patch('ilas.core.ImprovedAvoidance')
    def test_slam_integration(self, *args):
        """Test that SLAM is initialized and updated"""

        # Mock the SLAMSystem instance
        mock_slam_instance = MagicMock()

        # Find the SLAMSystem mock in the args and replace its return value
        for arg in args:
            if 'SLAMSystem' in str(arg):
                arg.return_value = mock_slam_instance
                break

        # Mock controller telemetry
        mock_controller = MagicMock()
        mock_controller.get_telemetry.return_value = {
            'position': np.array([0, 0, 0]),
            'velocity': np.array([0, 0, 0]),
            'attitude': np.array([0, 0, 0]),
            'acceleration': np.array([0, 0, 0]),
        }

        # Find the ControllerManager mock and replace its return value
        for arg in args:
            if 'ControllerManager' in str(arg):
                arg.return_value = mock_controller
                break

        core = ILASCore(self.config)
        core.start()

        # Let the main loop run a few times
        import time
        time.sleep(0.5)

        # Check that SLAM update was called
        self.assertTrue(mock_slam_instance.update.called)

        core.stop()

if __name__ == '__main__':
    unittest.main()
