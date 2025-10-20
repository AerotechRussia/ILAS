"""
Improved Avoidance System for ILAS
Implements advanced collision avoidance algorithms
"""

import numpy as np
from typing import Dict, List, Tuple

from ..detection.obstacle_detector import Obstacle

class ImprovedAvoidance:
    """
    Manages improved obstacle avoidance
    """

    def __init__(self, config: Dict):
        """
        Initialize ImprovedAvoidance

        Args:
            config: Avoidance configuration
        """
        self.config = config

    def navigate(self, current_position: np.ndarray, target_position: np.ndarray, obstacles: List[Obstacle]) -> Tuple[np.ndarray, bool]:
        """
        Navigate with improved avoidance

        Args:
            current_position: Current drone position
            target_position: Target position
            obstacles: List of detected obstacles

        Returns:
            Tuple of (navigation_command, is_safe)
        """
        # Vector Field Histogram (VFH) implementation

        # 1. Create polar histogram
        polar_histogram = self._create_polar_histogram(current_position, obstacles)

        # 2. Find safe corridors
        safe_corridors = self._find_safe_corridors(polar_histogram)

        # 3. Choose best corridor
        best_corridor = self._choose_best_corridor(safe_corridors, current_position, target_position)

        if best_corridor is None:
            # No safe corridor found, try to stop
            return np.zeros(3), False

        # 4. Generate navigation command
        heading_angle = np.deg2rad(best_corridor)
        navigation_vector = np.array([np.cos(heading_angle), np.sin(heading_angle), 0])

        return navigation_vector, True

    def _create_polar_histogram(self, current_position: np.ndarray, obstacles: List[Obstacle]) -> np.ndarray:
        # Implementation of VFH polar histogram
        num_sectors = self.config.get('vfh_num_sectors', 72)
        histogram = np.zeros(num_sectors)

        for obstacle in obstacles:
            vector_to_obstacle = obstacle.position - current_position
            distance = np.linalg.norm(vector_to_obstacle)
            angle = np.rad2deg(np.arctan2(vector_to_obstacle[1], vector_to_obstacle[0]))

            if angle < 0:
                angle += 360

            sector = int(angle / (360 / num_sectors))

            # Magnitude based on distance (closer obstacles have higher magnitude)
            magnitude = 1.0 / (1.0 + distance)
            histogram[sector] += magnitude

        return histogram

    def _find_safe_corridors(self, polar_histogram: np.ndarray) -> List[int]:
        # Find corridors with low obstacle density
        threshold = self.config.get('vfh_threshold', 0.1)
        safe_corridors = []
        for i, density in enumerate(polar_histogram):
            if density < threshold:
                safe_corridors.append(i * (360 / len(polar_histogram)))
        return safe_corridors

    def _choose_best_corridor(self, safe_corridors: List[int], current_position: np.ndarray, target_position: np.ndarray) -> int:
        # Choose the corridor that is closest to the target direction
        if not safe_corridors:
            return None

        vector_to_target = target_position - current_position
        target_angle = np.rad2deg(np.arctan2(vector_to_target[1], vector_to_target[0]))

        if target_angle < 0:
            target_angle += 360

        best_corridor = min(safe_corridors, key=lambda c: abs(c - target_angle))
        return best_corridor
