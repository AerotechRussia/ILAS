"""
Versatile Landing System for ILAS
Handles landing for different vehicle configurations (e.g., plane, multicopter)
"""

import numpy as np
from typing import Dict, List

from .intelligent_landing import LandingSite

class VersatileLanding:
    """
    Manages versatile landing procedures
    """

    def __init__(self, config: Dict):
        """
        Initialize VersatileLanding

        Args:
            config: Landing configuration
        """
        self.config = config

    def plan_landing(self, vehicle_type: str, landing_site: LandingSite, current_position: np.ndarray) -> List[np.ndarray]:
        """
        Plan landing trajectory based on vehicle type

        Args:
            vehicle_type: 'plane' or 'multicopter'
            landing_site: Selected landing site
            current_position: Current drone position

        Returns:
            List of waypoints for landing trajectory
        """
        if vehicle_type == 'plane':
            return self._plan_plane_landing(landing_site, current_position)
        else: # multicopter
            return self._plan_multicopter_landing(landing_site, current_position)

    def _plan_plane_landing(self, landing_site: LandingSite, current_position: np.ndarray) -> List[np.ndarray]:
        """
        Plan landing trajectory for a fixed-wing aircraft

        Args:
            landing_site: Selected landing site
            current_position: Current drone position

        Returns:
            List of waypoints for landing trajectory
        """
        # Standard landing pattern for fixed-wing aircraft
        landing_altitude = self.config.get('landing_altitude', 50) # Altitude for the pattern
        pattern_width = self.config.get('pattern_width', 100) # Width of the downwind leg
        final_approach_distance = self.config.get('final_approach_distance', 200)

        # Assuming runway heading is along the x-axis
        landing_point = landing_site.position

        # 1. Downwind leg
        downwind_start = landing_point + np.array([-final_approach_distance, pattern_width, landing_altitude])
        downwind_end = landing_point + np.array([0, pattern_width, landing_altitude])

        # 2. Base leg
        base_leg_end = landing_point + np.array([0, 0, landing_altitude])

        # 3. Final approach
        final_approach_start = landing_point + np.array([-final_approach_distance, 0, landing_altitude])

        return [downwind_start, downwind_end, base_leg_end, final_approach_start, landing_point]

    def _plan_multicopter_landing(self, landing_site: LandingSite, current_position: np.ndarray) -> List[np.ndarray]:
        """
        Plan landing trajectory for a multicopter

        Args:
            landing_site: Selected landing site
            current_position: Current drone position

        Returns:
            List of waypoints for landing trajectory
        """
        # For multicopter, we can descend directly
        hover_point = np.array([landing_site.position[0], landing_site.position[1], current_position[2]])
        return [hover_point, landing_site.position]
