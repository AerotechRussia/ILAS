# ilas/mission/mission_planner.py

import numpy as np
import heapq
from typing import List, Dict, Optional, Tuple

class MissionPlanner:
    def __init__(self, config):
        self.config = config
        self.map_resolution = config.get('map_resolution', 0.5) # meters per grid cell

    def generate_plan(self, start: np.ndarray, goal: np.ndarray, map_data: Dict) -> Optional[List[np.ndarray]]:
        """
        Generate a mission plan from a start to a goal using A*.
        """

        # Discretize start and goal positions
        start_grid = (start / self.map_resolution).astype(int)
        goal_grid = (goal / self.map_resolution).astype(int)

        # A* implementation
        open_set = [(0, tuple(start_grid))]
        came_from = {}
        g_score = {tuple(start_grid): 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == tuple(goal_grid):
                return self._reconstruct_path(came_from, current)

            for neighbor in self._get_neighbors(current):
                if not self._is_cell_clear(neighbor, map_data):
                    continue

                tentative_g_score = g_score[current] + 1 # Assuming cost of 1 for each step

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + self._heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None # No path found

    def adapt_plan(self, current_plan: List[np.ndarray], current_position: np.ndarray, map_data: Dict) -> Optional[List[np.ndarray]]:
        """
        Adapt the current mission plan based on new information.
        """
        if len(current_plan) > 0:
            next_waypoint = current_plan[0]
            if not self._is_path_clear(current_position, next_waypoint, map_data):
                return self.generate_plan(current_position, current_plan[-1], map_data)

        return None

    def _is_path_clear(self, start: np.ndarray, end: np.ndarray, map_data: Dict) -> bool:
        """
        Check if the path between two points is clear of obstacles by sampling points along the line.
        """
        if map_data['map'] is None:
            return True # No map data, assume clear

        start_grid = (start / self.map_resolution)
        end_grid = (end / self.map_resolution)

        num_samples = int(np.linalg.norm(end_grid - start_grid) / 0.5) + 2

        for i in range(num_samples + 1):
            alpha = i / num_samples
            point = (1 - alpha) * start_grid + alpha * end_grid
            cell = tuple(point.astype(int))
            if not self._is_cell_clear(cell, map_data):
                return False

        return True

    def _is_cell_clear(self, cell: Tuple[int, int, int], map_data: Dict) -> bool:
        """
        Check if a cell in the 3D grid is clear of obstacles.
        """
        # This is a placeholder for the actual obstacle check in the SLAM map.
        # It assumes that the map is a 3D numpy array where 0 is free and 1 is occupied.
        slam_map = map_data.get('map')
        if slam_map is None:
            return True

        x, y, z = cell
        if 0 <= x < slam_map.shape[0] and 0 <= y < slam_map.shape[1] and 0 <= z < slam_map.shape[2]:
            return slam_map[x, y, z] == 0

        return True # Assume cells outside the map are clear

    def _get_neighbors(self, node: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
        """
        Get the neighbors of a node in the 3D grid.
        """
        neighbors = []
        for i, j, k in [(1,0,0), (-1,0,0), (0,1,0), (0,-1,0), (0,0,1), (0,0,-1)]:
            neighbors.append((node[0] + i, node[1] + j, node[2] + k))
        return neighbors

    def _heuristic(self, a: Tuple[int, int, int], b: Tuple[int, int, int]) -> float:
        """
        Heuristic function for A* (Manhattan distance).
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])

    def _reconstruct_path(self, came_from: Dict, current: Tuple[int, int, int]) -> List[np.ndarray]:
        """
        Reconstruct the path from the came_from map.
        """
        path = [np.array(current) * self.map_resolution]
        while current in came_from:
            current = came_from[current]
            path.append(np.array(current) * self.map_resolution)
        return path[::-1]
