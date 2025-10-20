"""
Search Grid Generator
Generates a grid of waypoints for a search and rescue mission
"""

import numpy as np
from typing import List

def generate_search_grid(
    area_bounds: List[float],
    altitude: float,
    overlap: float
) -> List[np.ndarray]:
    """
    Generate a grid of waypoints for a search and rescue mission.

    Args:
        area_bounds: [min_x, min_y, max_x, max_y]
        altitude: Flight altitude
        overlap: Overlap between camera footprints (0.0 to 1.0)

    Returns:
        List of waypoints
    """
    min_x, min_y, max_x, max_y = area_bounds

    # Assuming a camera with a 45-degree field of view
    fov = np.deg2rad(45)
    footprint_width = 2 * altitude * np.tan(fov / 2)
    step_size = footprint_width * (1 - overlap)

    waypoints = []

    x = min_x
    y = min_y
    direction = 1  # 1 for positive y, -1 for negative y

    while min_x <= x <= max_x:
        while (direction == 1 and y <= max_y) or (direction == -1 and y >= min_y):
            waypoints.append(np.array([x, y, altitude]))
            y += direction * step_size

        # Reverse direction and move to next column
        direction *= -1
        y = max_y if direction == -1 else min_y
        x += step_size

    return waypoints
