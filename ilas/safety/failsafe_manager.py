"""
Failsafe Manager for ILAS
Monitors system health and triggers failsafe procedures
"""

from typing import Dict, List
import numpy as np

class FailsafeManager:
    """
    Monitors system health and triggers failsafe procedures
    """

    def __init__(self, config: Dict):
        """
        Initialize FailsafeManager

        Args:
            config: Failsafe configuration
        """
        self.config = config
        self.geofence = self._parse_geofence(config.get('geofence', []))

    def check_failsafes(self, telemetry: Dict) -> List[str]:
        """
        Check all failsafe conditions

        Args:
            telemetry: Current telemetry data

        Returns:
            List of triggered failsafe events
        """
        triggered_events = []

        if self.config.get('check_geofence', False):
            if not self._is_within_geofence(telemetry.get('position', np.zeros(3))):
                triggered_events.append('geofence_breached')

        return triggered_events

    def _parse_geofence(self, geofence_points: List[List[float]]) -> np.ndarray:
        """
        Parse geofence points from configuration

        Args:
            geofence_points: List of [lat, lon] points

        Returns:
            Numpy array of geofence points
        """
        return np.array(geofence_points)

    def _is_within_geofence(self, position: np.ndarray) -> bool:
        """
        Check if position is within the defined geofence using the Ray Casting algorithm.

        Args:
            position: Current position [x, y, z] (only x and y are used)

        Returns:
            True if within geofence
        """
        if self.geofence.shape[0] < 3:
            # A polygon needs at least 3 vertices
            return True # No valid geofence defined

        x, y = position[0], position[1]
        n = len(self.geofence)
        inside = False

        p1x, p1y = self.geofence[0]
        for i in range(n + 1):
            p2x, p2y = self.geofence[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside
