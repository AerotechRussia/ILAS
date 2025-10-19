
import numpy as np
from typing import List, Dict, Any
from shapely.geometry import Point, Polygon

class GeofenceManager:
    """
    Manages multiple geofence zones, including inclusion and exclusion zones.
    """

    def __init__(self, config: Dict[str, Any]):
        """
        Initializes the GeofenceManager.

        Args:
            config: Configuration dictionary containing geofence settings.
        """
        self.enabled = config.get('enabled', False)
        self.inclusion_zones = []
        self.exclusion_zones = []

        if self.enabled:
            self._create_zones(config.get('inclusion_zones', []), self.inclusion_zones)
            self._create_zones(config.get('exclusion_zones', []), self.exclusion_zones)

    def _create_zones(self, zone_configs: List[Dict[str, Any]], zone_list: List[Dict[str, Any]]):
        """
        Creates geofence zones from configuration.

        Args:
            zone_configs: A list of zone configuration dictionaries.
            zone_list: The list to append the created zones to.
        """
        for zone_config in zone_configs:
            polygon_points = zone_config.get('polygon_points', [])
            if len(polygon_points) >= 3:
                zone = {
                    'polygon': Polygon(polygon_points),
                    'min_altitude': zone_config.get('min_altitude', -np.inf),
                    'max_altitude': zone_config.get('max_altitude', np.inf)
                }
                zone_list.append(zone)
            else:
                print("Warning: A geofence zone has fewer than 3 points and will be ignored.")

    def is_within_geofence(self, position: np.ndarray) -> bool:
        """
        Checks if the given position is within the defined geofence.
        - Must be within at least one inclusion zone.
        - Must not be within any exclusion zones.

        Args:
            position: The current position [x, y, z] of the drone.

        Returns:
            True if the position is within the geofence, False otherwise.
        """
        if not self.enabled:
            return True

        point = Point(position[0], position[1])
        altitude = position[2]

        # Check exclusion zones first
        for zone in self.exclusion_zones:
            if zone['polygon'].contains(point) and zone['min_altitude'] <= altitude <= zone['max_altitude']:
                return False

        # Check inclusion zones
        for zone in self.inclusion_zones:
            if zone['polygon'].contains(point) and zone['min_altitude'] <= altitude <= zone['max_altitude']:
                return True

        # If there are inclusion zones, the drone must be in at least one.
        # If there are no inclusion zones, then everywhere is implicitly an inclusion zone (except for exclusion zones).
        return not self.inclusion_zones
