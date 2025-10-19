"""
Geofence Manager for ILAS
Handles loading, management, and checking of geofence boundaries.
"""

import numpy as np
from typing import List, Dict, Optional
from shapely.geometry import Point, Polygon

class GeofenceManager:
    """
    Manages the drone's operational geofence.
    """

    def __init__(self, config: Dict):
        """
        Initialize GeofenceManager.

        Args:
            config: Configuration dictionary containing geofence settings.
        """
        self.enabled = config.get('enabled', False)
        self.polygon_points = np.array(config.get('polygon_points', []))
        self.max_altitude = config.get('max_altitude', 100.0)
        self.min_altitude = config.get('min_altitude', 5.0)
        self.polygon = None

        if self.enabled and len(self.polygon_points) >= 3:
            self.polygon = Polygon(self.polygon_points)
        elif self.enabled:
            print("Warning: Geofence is enabled but the polygon has fewer than 3 points. Disabling.")
            self.enabled = False

    def is_within_geofence(self, position: np.ndarray) -> bool:
        """
        Check if the given position is within the defined geofence.

        Args:
            position: The current position [x, y, z] of the drone.

        Returns:
            True if the position is within the geofence, False otherwise.
        """
        if not self.enabled:
            return True

        # Check altitude first
        if not (self.min_altitude <= position[2] <= self.max_altitude):
            return False

        # Check horizontal boundaries using shapely
        point = Point(position[0], position[1])
        return self.polygon.intersects(point)
