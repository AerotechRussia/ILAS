import unittest
import numpy as np

from ilas.safety.failsafe_manager import FailsafeManager

class TestFailsafeManager(unittest.TestCase):

    def test_geofence_inside(self):
        config = {
            'check_geofence': True,
            'geofence': [
                [0, 0],
                [100, 0],
                [100, 100],
                [0, 100]
            ]
        }
        failsafe_manager = FailsafeManager(config)

        position_inside = np.array([50, 50, 10])
        telemetry = {'position': position_inside}

        events = failsafe_manager.check_failsafes(telemetry)
        self.assertEqual(len(events), 0)

    def test_geofence_outside(self):
        config = {
            'check_geofence': True,
            'geofence': [
                [0, 0],
                [100, 0],
                [100, 100],
                [0, 100]
            ]
        }
        failsafe_manager = FailsafeManager(config)

        position_outside = np.array([150, 50, 10])
        telemetry = {'position': position_outside}

        events = failsafe_manager.check_failsafes(telemetry)
        self.assertIn('geofence_breached', events)

    def test_geofence_on_edge(self):
        config = {
            'check_geofence': True,
            'geofence': [
                [0, 0],
                [100, 0],
                [100, 100],
                [0, 100]
            ]
        }
        failsafe_manager = FailsafeManager(config)

        position_on_edge = np.array([100, 50, 10])
        telemetry = {'position': position_on_edge}

        events = failsafe_manager.check_failsafes(telemetry)
        self.assertEqual(len(events), 0)

    def test_geofence_disabled(self):
        config = {
            'check_geofence': False,
            'geofence': [
                [0, 0],
                [100, 0],
                [100, 100],
                [0, 100]
            ]
        }
        failsafe_manager = FailsafeManager(config)

        position_outside = np.array([150, 50, 10])
        telemetry = {'position': position_outside}

        events = failsafe_manager.check_failsafes(telemetry)
        self.assertEqual(len(events), 0)

if __name__ == '__main__':
    unittest.main()
