"""
Search and Rescue Mapping Module
Creates a map of the search area, drone's path, and detected persons
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import List, Dict

class SearchMapping:
    """
    Creates and updates a map of the search area
    """

    def __init__(self, area_bounds: List[float]):
        """
        Initialize the search map

        Args:
            area_bounds: [min_x, min_y, max_x, max_y]
        """
        self.area_bounds = area_bounds
        self.flight_path = []
        self.detected_persons = []

    def update_path(self, position: np.ndarray):
        """
        Update the drone's flight path

        Args:
            position: Current position of the drone
        """
        self.flight_path.append(position)

    def add_detection(self, detection: Dict):
        """
        Add a detected person to the map

        Args:
            detection: Dictionary with person detection info
        """
        self.detected_persons.append(detection)

    def generate_map(self, output_path: str):
        """
        Generate and save the search map

        Args:
            output_path: Path to save the map image
        """
        min_x, min_y, max_x, max_y = self.area_bounds

        plt.figure(figsize=(10, 10))
        plt.xlim(min_x, max_x)
        plt.ylim(min_y, max_y)

        # Plot flight path
        if self.flight_path:
            path = np.array(self.flight_path)
            plt.plot(path[:, 0], path[:, 1], 'b-', label='Flight Path')

        # Plot detected persons
        if self.detected_persons:
            persons_pos = np.array([p['position'] for p in self.detected_persons])
            plt.scatter(persons_pos[:, 0], persons_pos[:, 1], c='r', marker='o', label='Detected Persons')

        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.title("Search and Rescue Map")
        plt.legend()
        plt.grid(True)

        plt.savefig(output_path)
        plt.close()
