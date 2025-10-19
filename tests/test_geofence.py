import unittest
import numpy as np
from ilas.geofence.geofence_manager import GeofenceManager

class TestGeofenceManager(unittest.TestCase):

    def test_geofence_disabled(self):
        config = {'enabled': False}
        manager = GeofenceManager(config)
        position = np.array([0, 0, 10])
        self.assertTrue(manager.is_within_geofence(position))

    def test_single_inclusion_zone(self):
        config = {
            'enabled': True,
            'inclusion_zones': [
                {
                    'polygon_points': [[0, 0], [100, 0], [100, 100], [0, 100]],
                    'min_altitude': 5,
                    'max_altitude': 50
                }
            ]
        }
        manager = GeofenceManager(config)

        # Inside the zone
        self.assertTrue(manager.is_within_geofence(np.array([50, 50, 25])))

        # Outside the zone (horizontally)
        self.assertFalse(manager.is_within_geofence(np.array([150, 50, 25])))

        # Outside the zone (below min altitude)
        self.assertFalse(manager.is_within_geofence(np.array([50, 50, 4])))

        # Outside the zone (above max altitude)
        self.assertFalse(manager.is_within_geofence(np.array([50, 50, 51])))

    def test_single_exclusion_zone(self):
        config = {
            'enabled': True,
            'exclusion_zones': [
                {
                    'polygon_points': [[10, 10], [20, 10], [20, 20], [10, 20]],
                    'min_altitude': 5,
                    'max_altitude': 50
                }
            ]
        }
        manager = GeofenceManager(config)

        # Outside the exclusion zone
        self.assertTrue(manager.is_within_geofence(np.array([5, 5, 25])))

        # Inside the exclusion zone
        self.assertFalse(manager.is_within_geofence(np.array([15, 15, 25])))

    def test_inclusion_and_exclusion_zones(self):
        config = {
            'enabled': True,
            'inclusion_zones': [
                {
                    'polygon_points': [[0, 0], [100, 0], [100, 100], [0, 100]],
                    'min_altitude': 5,
                    'max_altitude': 50
                }
            ],
            'exclusion_zones': [
                {
                    'polygon_points': [[10, 10], [20, 10], [20, 20], [10, 20]],
                    'min_altitude': 5,
                    'max_altitude': 50
                }
            ]
        }
        manager = GeofenceManager(config)

        # In inclusion, outside exclusion
        self.assertTrue(manager.is_within_geofence(np.array([5, 5, 25])))

        # In inclusion, but also in exclusion
        self.assertFalse(manager.is_within_geofence(np.array([15, 15, 25])))

        # Outside inclusion
        self.assertFalse(manager.is_within_geofence(np.array([150, 150, 25])))

    def test_multiple_inclusion_zones(self):
        config = {
            'enabled': True,
            'inclusion_zones': [
                {
                    'polygon_points': [[0, 0], [10, 0], [10, 10], [0, 10]],
                    'min_altitude': 5,
                    'max_altitude': 50
                },
                {
                    'polygon_points': [[20, 20], [30, 20], [30, 30], [20, 30]],
                    'min_altitude': 5,
                    'max_altitude': 50
                }
            ]
        }
        manager = GeofenceManager(config)

        # In the first inclusion zone
        self.assertTrue(manager.is_within_geofence(np.array([5, 5, 25])))

        # In the second inclusion zone
        self.assertTrue(manager.is_within_geofence(np.array([25, 25, 25])))

        # Outside both inclusion zones
        self.assertFalse(manager.is_within_geofence(np.array([15, 15, 25])))

    def test_no_inclusion_zones(self):
        config = {
            'enabled': True,
            'exclusion_zones': [
                {
                    'polygon_points': [[10, 10], [20, 10], [20, 20], [10, 20]],
                    'min_altitude': 5,
                    'max_altitude': 50
                }
            ]
        }
        manager = GeofenceManager(config)

        # Outside the exclusion zone, so it's valid
        self.assertTrue(manager.is_within_geofence(np.array([5, 5, 25])))

        # Inside the exclusion zone
        self.assertFalse(manager.is_within_geofence(np.array([15, 15, 25])))

if __name__ == '__main__':
    unittest.main()
