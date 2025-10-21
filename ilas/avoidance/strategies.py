"""
Avoidance Strategies for ILAS
Implements different obstacle avoidance algorithms as separate classes.
"""

import numpy as np
from typing import List, Tuple, Dict
from abc import ABC, abstractmethod

from ..detection.obstacle_detector import Obstacle

class BaseAvoidanceStrategy(ABC):
    """Abstract base class for all avoidance strategies."""
    def __init__(self, config: Dict):
        self.config = config
        self.safety_margin = config.get('safety_margin', 2.0)
        self.max_avoidance_distance = config.get('max_avoidance_distance', 10.0)

    @abstractmethod
    def calculate_avoidance_vector(self,
                                  drone_position: np.ndarray,
                                  target_position: np.ndarray,
                                  drone_velocity: np.ndarray,
                                  obstacles: List[Obstacle]) -> Tuple[np.ndarray, bool]:
        """
        Calculate the avoidance vector.

        Returns:
            Tuple of (avoidance_vector, is_safe)
        """
        pass

class PotentialFieldStrategy(BaseAvoidanceStrategy):
    """Implements the Artificial Potential Field (APF) algorithm."""
    def calculate_avoidance_vector(self, drone_position, target_position, drone_velocity, obstacles):
        avoidance_gain = self.config.get('avoidance_gain', 1.0)
        # Attractive force towards target
        to_target = target_position - drone_position
        distance_to_target = np.linalg.norm(to_target)

        if distance_to_target > 0:
            attractive_force = to_target / distance_to_target
        else:
            attractive_force = np.zeros(3)

        # Repulsive forces from obstacles
        repulsive_force = np.zeros(3)
        is_safe = True

        if obstacles:
            for obs in obstacles:
                to_obstacle = obs.position - drone_position
                distance = np.linalg.norm(to_obstacle)

                if distance < self.safety_margin:
                    is_safe = False

                if 0 < distance < self.max_avoidance_distance:
                    magnitude = avoidance_gain * (
                        1.0 / distance - 1.0 / self.max_avoidance_distance
                    ) / (distance ** 2)
                    direction = -to_obstacle / distance
                    repulsive_force += magnitude * direction

        total_force = attractive_force + repulsive_force

        force_magnitude = np.linalg.norm(total_force)
        if force_magnitude > 0:
            avoidance_vector = total_force / force_magnitude
        else:
            avoidance_vector = attractive_force

        return avoidance_vector, is_safe

class VFHStrategy(BaseAvoidanceStrategy):
    """Implements the Vector Field Histogram (VFH) algorithm."""
    def calculate_avoidance_vector(self, drone_position, target_position, drone_velocity, obstacles):
        num_sectors = 72
        histogram = np.zeros(num_sectors)
        is_safe = True

        for obs in obstacles:
            to_obstacle = obs.position - drone_position
            distance = np.linalg.norm(to_obstacle)

            if distance < self.safety_margin:
                is_safe = False

            if 0 < distance < self.max_avoidance_distance:
                angle = np.arctan2(to_obstacle[1], to_obstacle[0])
                sector = int((angle + np.pi) / (2 * np.pi) * num_sectors) % num_sectors
                magnitude = obs.confidence * (1.0 - distance / self.max_avoidance_distance)
                histogram[sector] += magnitude

        to_target = target_position - drone_position
        target_angle = np.arctan2(to_target[1], to_target[0])
        target_sector = int((target_angle + np.pi) / (2 * np.pi) * num_sectors) % num_sectors

        search_range = 20
        best_sector = target_sector
        min_cost = float('inf')

        for i in range(-search_range, search_range + 1):
            sector = (target_sector + i) % num_sectors
            cost = histogram[sector] + abs(i) * 0.1
            if cost < min_cost:
                min_cost = cost
                best_sector = sector

        angle = (best_sector / num_sectors) * 2 * np.pi - np.pi
        avoidance_vector = np.array([np.cos(angle), np.sin(angle), to_target[2] / (np.linalg.norm(to_target[:2]) + 1e-6)])

        return avoidance_vector / (np.linalg.norm(avoidance_vector) + 1e-6), is_safe

class DWAStrategy(BaseAvoidanceStrategy):
    """Implements the Dynamic Window Approach (DWA) algorithm."""
    def calculate_avoidance_vector(self, drone_position, target_position, drone_velocity, obstacles):
        max_vel = self.config.get('max_velocity', 5.0)
        max_accel = self.config.get('max_acceleration', 2.0)
        dt = self.config.get('prediction_time', 1.0)
        num_samples = 10
        best_velocity = drone_velocity
        best_score = float('-inf')
        is_safe = True

        to_target = target_position - drone_position
        target_dir = to_target / (np.linalg.norm(to_target) + 1e-6)

        vx_samples = np.linspace(drone_velocity[0] - max_accel * dt, drone_velocity[0] + max_accel * dt, num_samples)
        vy_samples = np.linspace(drone_velocity[1] - max_accel * dt, drone_velocity[1] + max_accel * dt, num_samples)
        vz_samples = np.linspace(drone_velocity[2] - max_accel * dt, drone_velocity[2] + max_accel * dt, num_samples // 2)

        for vx in vx_samples:
            for vy in vy_samples:
                for vz in vz_samples:
                    velocity = np.array([vx, vy, vz])

                    if np.linalg.norm(velocity - drone_velocity) > max_accel * dt:
                        continue

                    predicted_pos = drone_position + velocity * dt

                    min_distance = float('inf')
                    if obstacles:
                        for obs in obstacles:
                            dist = np.linalg.norm(predicted_pos - obs.position)
                            if dist < min_distance:
                                min_distance = dist

                    if min_distance < self.safety_margin:
                        continue

                    to_target_pred = target_position - predicted_pos
                    heading_score = np.dot(velocity, to_target_pred) / (np.linalg.norm(velocity) * np.linalg.norm(to_target_pred) + 1e-6)
                    clearance_score = min(min_distance / self.max_avoidance_distance, 1.0)

                    total_score = heading_score + 0.5 * clearance_score

                    if total_score > best_score:
                        best_score = total_score
                        best_velocity = velocity

        vel_magnitude = np.linalg.norm(best_velocity)
        if vel_magnitude > 0:
            avoidance_vector = best_velocity / vel_magnitude
        else:
            avoidance_vector = target_dir

        return avoidance_vector, is_safe

class RRTStrategy(BaseAvoidanceStrategy):
    """Implements the Rapidly-exploring Random Tree (RRT) algorithm."""
    def calculate_avoidance_vector(self, drone_position, target_position, drone_velocity, obstacles):
        class Node:
            def __init__(self, position):
                self.position = position
                self.parent = None

        start_node = Node(drone_position)
        end_node = Node(target_position)

        node_list = [start_node]

        max_iter = self.config.get('rrt_max_iter', 200)
        step_size = self.config.get('rrt_step_size', 1.0)

        for _ in range(max_iter):
            rnd_point = self._get_random_point(target_position)
            nearest_node = self._get_nearest_node(node_list, rnd_point)
            new_node = self._steer(nearest_node, rnd_point, step_size)

            if not self._is_collision(new_node, obstacles):
                node_list.append(new_node)

                if np.linalg.norm(new_node.position - target_position) <= step_size:
                    end_node.parent = new_node
                    path = self._get_final_path(end_node)

                    if len(path) > 1:
                        next_waypoint = path[1]
                        avoidance_vector = next_waypoint - drone_position
                        return avoidance_vector / np.linalg.norm(avoidance_vector), True
                    else:
                        to_target = target_position - drone_position
                        return to_target / np.linalg.norm(to_target), True

        # Fallback to a simple strategy if no path is found
        pf_strategy = PotentialFieldStrategy(self.config)
        return pf_strategy.calculate_avoidance_vector(drone_position, target_position, drone_velocity, obstacles)

    def _get_random_point(self, target_position):
        if np.random.rand() > 0.1:
            return np.random.uniform(-15, 15, 3)
        else:
            return target_position

    def _get_nearest_node(self, node_list, rnd_point):
        min_dist = float('inf')
        nearest_node = None
        for node in node_list:
            dist = np.linalg.norm(node.position - rnd_point)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def _steer(self, from_node, to_point, step_size):
        direction = to_point - from_node.position
        dist = np.linalg.norm(direction)

        if dist < step_size:
            new_position = to_point
        else:
            new_position = from_node.position + (direction / dist) * step_size

        Node = type("Node", (), {"__init__": lambda self, pos: setattr(self, "position", pos), "parent": None})
        new_node = Node(new_position)
        new_node.parent = from_node
        return new_node

    def _is_collision(self, node, obstacles):
        for obs in obstacles:
            if np.linalg.norm(node.position - obs.position) < self.safety_margin + obs.size[0]/2:
                return True
        return False

    def _get_final_path(self, end_node):
        path = [end_node.position]
        node = end_node
        while node.parent is not None:
            path.append(node.parent.position)
            node = node.parent
        path.reverse()
        return path
