"""
Obstacle Avoidance System
Implements various avoidance algorithms for VTOL drones
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
from enum import Enum

from ..detection.obstacle_detector import Obstacle
from .strategies import PotentialFieldStrategy, VFHStrategy, DWAStrategy, RRTStrategy


class AvoidanceStrategy(Enum):
    """Available avoidance strategies"""
    POTENTIAL_FIELD = "potential_field"
    VECTOR_FIELD = "vector_field"
    DYNAMIC_WINDOW = "dynamic_window"
    RRT = "rrt"
    HYBRID = "hybrid"


class AvoidanceState(Enum):
    """Finite states for the avoidance system"""
    DIRECT = "direct"
    AVOIDING = "avoiding"
    CIRCLING = "circling"


class AvoidanceSystem:
    """
    Main obstacle avoidance system
    Calculates safe trajectories around obstacles
    """
    
    def __init__(self, config: Dict):
        """
        Initialize avoidance system
        
        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.strategy_name = config.get('strategy', 'potential_field')
        self.safety_margin = config.get('safety_margin', 2.0)

        self.strategies = {
            'potential_field': PotentialFieldStrategy(config),
            'vector_field': VFHStrategy(config),
            'dynamic_window': DWAStrategy(config),
            'rrt': RRTStrategy(config)
        }

        self.state = AvoidanceState.DIRECT
        self.circling_obstacle = None
        self.circling_direction = 1
        
    def calculate_avoidance_vector(self, 
                                  drone_position: np.ndarray,
                                  target_position: np.ndarray,
                                  drone_velocity: np.ndarray,
                                  obstacles: List[Obstacle]) -> Tuple[np.ndarray, bool]:
        """
        Calculate avoidance vector to avoid obstacles
        
        Args:
            drone_position: Current drone position
            target_position: Desired target position
            drone_velocity: Current velocity
            obstacles: List of detected obstacles
            
        Returns:
            Tuple of (avoidance_vector, is_safe)
        """
        if self.strategy_name == 'hybrid':
            return self._hybrid_avoidance(drone_position, target_position, drone_velocity, obstacles)

        strategy = self.strategies.get(self.strategy_name)
        if not strategy:
            raise ValueError(f"Unknown avoidance strategy: {self.strategy_name}")

        return strategy.calculate_avoidance_vector(
            drone_position, target_position, drone_velocity, obstacles
        )

    def _hybrid_avoidance(self,
                          drone_position: np.ndarray,
                          target_position: np.ndarray,
                          drone_velocity: np.ndarray,
                          obstacles: List[Obstacle]) -> Tuple[np.ndarray, bool]:
        """
        State-machine based hybrid avoidance strategy.
        """
        # Find closest obstacle
        closest_obstacle = None
        min_dist = float('inf')
        if obstacles:
            for obs in obstacles:
                dist = np.linalg.norm(drone_position - obs.position)
                if dist < min_dist:
                    min_dist = dist
                    closest_obstacle = obs

        # --- State Machine Logic ---
        path_is_clear = self.check_path_clear(drone_position, target_position, obstacles)

        # State: DIRECT
        if self.state == AvoidanceState.DIRECT:
            if closest_obstacle and min_dist < self.config.get('max_avoidance_distance', 10.0):
                self.state = AvoidanceState.AVOIDING
            else:
                to_target = target_position - drone_position
                return to_target / (np.linalg.norm(to_target) + 1e-6), True

        # State: AVOIDING
        if self.state == AvoidanceState.AVOIDING:
            if path_is_clear and min_dist > self.config.get('max_avoidance_distance', 10.0):
                self.state = AvoidanceState.DIRECT
                to_target = target_position - drone_position
                return to_target / (np.linalg.norm(to_target) + 1e-6), True
            elif not path_is_clear and closest_obstacle:
                to_target_dir = (target_position - drone_position) / (np.linalg.norm(target_position - drone_position) + 1e-6)
                avoid_vec, _ = self.strategies['potential_field'].calculate_avoidance_vector(drone_position, target_position, drone_velocity, obstacles)
                if np.dot(to_target_dir, avoid_vec) < 0.3:
                    self.state = AvoidanceState.CIRCLING
                    self.circling_obstacle = closest_obstacle
                    to_obs = closest_obstacle.position - drone_position
                    cross_product_z = np.cross(to_target_dir[:2], to_obs[:2])
                    self.circling_direction = 1 if cross_product_z > 0 else -1

        # State: CIRCLING
        if self.state == AvoidanceState.CIRCLING:
            if path_is_clear:
                self.state = AvoidanceState.DIRECT
                self.circling_obstacle = None
                to_target = target_position - drone_position
                return to_target / (np.linalg.norm(to_target) + 1e-6), True

            return self._circling_avoidance(drone_position, target_position, obstacles)

        return self.strategies['potential_field'].calculate_avoidance_vector(
            drone_position, target_position, drone_velocity, obstacles
        )

    def _circling_avoidance(self,
                              drone_position: np.ndarray,
                              target_position: np.ndarray,
                              obstacles: List[Obstacle]) -> Tuple[np.ndarray, bool]:
        """
        Follows the boundary of an obstacle to navigate around it.
        """
        if self.circling_obstacle is None:
            return self.strategies['potential_field'].calculate_avoidance_vector(drone_position, target_position, None, obstacles)

        obstacle_pos = self.circling_obstacle.position
        to_obstacle = obstacle_pos - drone_position
        dist_to_obstacle = np.linalg.norm(to_obstacle)

        radial_vector = -to_obstacle / (dist_to_obstacle + 1e-6)
        tangential_vector_2d = np.array([-radial_vector[1], radial_vector[0]]) * self.circling_direction
        tangential_vector = np.append(tangential_vector_2d, 0)

        distance_error = dist_to_obstacle - (self.safety_margin + self.circling_obstacle.size[0] / 2)
        correction_force = radial_vector * distance_error * 0.5

        to_target = target_position - drone_position
        attractive_force = to_target / (np.linalg.norm(to_target) + 1e-6)

        combined_vector = tangential_vector + correction_force + attractive_force * 0.3
        avoidance_vector = combined_vector / (np.linalg.norm(combined_vector) + 1e-6)
        is_safe = dist_to_obstacle > self.safety_margin

        return avoidance_vector, is_safe
    
    def check_path_clear(self,
                        start_position: np.ndarray,
                        end_position: np.ndarray,
                        obstacles: List[Obstacle]) -> bool:
        """
        Check if path between two points is clear of obstacles
        
        Args:
            start_position: Start point
            end_position: End point
            obstacles: List of obstacles
            
        Returns:
            True if path is clear
        """
        # Check line segment against obstacles
        direction = end_position - start_position
        distance = np.linalg.norm(direction)
        
        if distance < 0.01:
            return True
        
        direction = direction / distance
        
        # Sample along path
        num_samples = int(distance / 0.5) + 1
        
        for i in range(num_samples):
            point = start_position + direction * (i * distance / num_samples)
            
            for obstacle in obstacles:
                dist_to_obstacle = np.linalg.norm(point - obstacle.position)
                if dist_to_obstacle < self.safety_margin:
                    return False
        
        return True
    
    def calculate_safe_altitude(self,
                               drone_position: np.ndarray,
                               obstacles: List[Obstacle]) -> float:
        """
        Calculate safe altitude to fly over obstacles
        
        Args:
            drone_position: Current position
            obstacles: List of obstacles
            
        Returns:
            Recommended safe altitude
        """
        max_obstacle_height = 0.0
        
        for obstacle in obstacles:
            # Check if obstacle is nearby in XY plane
            xy_distance = np.linalg.norm(
                drone_position[:2] - obstacle.position[:2]
            )
            
            if xy_distance < self.max_avoidance_distance:
                obstacle_top = obstacle.position[2] + obstacle.size[2] / 2
                max_obstacle_height = max(max_obstacle_height, obstacle_top)
        
        # Add safety margin
        safe_altitude = max_obstacle_height + self.safety_margin + 1.0
        
        return safe_altitude
