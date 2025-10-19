"""
Obstacle Avoidance System
Implements various avoidance algorithms for VTOL drones
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
from enum import Enum

from ..detection.obstacle_detector import Obstacle


class AvoidanceStrategy(Enum):
    """Available avoidance strategies"""
    POTENTIAL_FIELD = "potential_field"
    VECTOR_FIELD = "vector_field"
    DYNAMIC_WINDOW = "dynamic_window"
    RRT = "rrt"  # Rapidly-exploring Random Tree
    BEBOP = "bebop"  # Best Effort Bug-based Obstacle Planner


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
        self.strategy = AvoidanceStrategy(config.get('strategy', 'potential_field'))
        self.safety_margin = config.get('safety_margin', 2.0)  # meters
        self.max_avoidance_distance = config.get('max_avoidance_distance', 10.0)
        self.avoidance_gain = config.get('avoidance_gain', 1.0)
        
    def calculate_avoidance_vector(self, 
                                  drone_position: np.ndarray,
                                  target_position: np.ndarray,
                                  drone_velocity: np.ndarray,
                                  obstacles: List[Obstacle]) -> Tuple[np.ndarray, bool]:
        """
        Calculate avoidance vector to avoid obstacles
        
        Args:
            drone_position: Current drone position [x, y, z]
            target_position: Desired target position [x, y, z]
            drone_velocity: Current velocity [vx, vy, vz]
            obstacles: List of detected obstacles
            
        Returns:
            Tuple of (avoidance_vector, is_safe)
        """
        if self.strategy == AvoidanceStrategy.POTENTIAL_FIELD:
            return self._potential_field_avoidance(
                drone_position, target_position, obstacles
            )
        elif self.strategy == AvoidanceStrategy.VECTOR_FIELD:
            return self._vector_field_avoidance(
                drone_position, target_position, drone_velocity, obstacles
            )
        elif self.strategy == AvoidanceStrategy.DYNAMIC_WINDOW:
            return self._dynamic_window_avoidance(
                drone_position, target_position, drone_velocity, obstacles
            )
        elif self.strategy == AvoidanceStrategy.RRT:
            return self._rrt_avoidance(
                drone_position, target_position, obstacles
            )
        else:
            return self._potential_field_avoidance(
                drone_position, target_position, obstacles
            )
    
    def _potential_field_avoidance(self,
                                   drone_position: np.ndarray,
                                   target_position: np.ndarray,
                                   obstacles: List[Obstacle]) -> Tuple[np.ndarray, bool]:
        """
        Artificial Potential Field method
        Attractive force to target, repulsive force from obstacles
        """
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
            obstacle_positions = np.array([o.position for o in obstacles])
            to_obstacles = obstacle_positions - drone_position
            distances = np.linalg.norm(to_obstacles, axis=1)

            # Check safety
            if np.any(distances < self.safety_margin):
                is_safe = False

            # Calculate repulsive forces for obstacles within range
            in_range = (distances < self.max_avoidance_distance) & (distances > 0)
            if np.any(in_range):
                distances_in_range = distances[in_range]
                to_obstacles_in_range = to_obstacles[in_range]

                magnitude = self.avoidance_gain * (
                    1.0 / distances_in_range - 1.0 / self.max_avoidance_distance
                ) / (distances_in_range ** 2)
                
                direction = -to_obstacles_in_range / distances_in_range[:, np.newaxis]
                repulsive_force = np.sum(magnitude[:, np.newaxis] * direction, axis=0)
        
        # Combine forces
        total_force = attractive_force + repulsive_force
        
        # If the repulsive force is not strong enough to overcome the attractive force,
        # then the path is not safe.
        if np.linalg.norm(attractive_force) > np.linalg.norm(repulsive_force):
            is_safe = False

        # Normalize
        force_magnitude = np.linalg.norm(total_force)
        if force_magnitude > 0:
            avoidance_vector = total_force / force_magnitude
        else:
            avoidance_vector = attractive_force
            
        return avoidance_vector, is_safe
    
    def _vector_field_avoidance(self,
                               drone_position: np.ndarray,
                               target_position: np.ndarray,
                               drone_velocity: np.ndarray,
                               obstacles: List[Obstacle]) -> Tuple[np.ndarray, bool]:
        """
        Vector Field Histogram (VFH) method
        Creates a polar histogram of obstacles and finds safe direction
        """
        # Create angular histogram
        num_sectors = 72  # 5-degree resolution
        histogram = np.zeros(num_sectors)
        
        is_safe = True

        if obstacles:
            obstacle_positions = np.array([o.position for o in obstacles])
            obstacle_confidences = np.array([o.confidence for o in obstacles])
            to_obstacles = obstacle_positions - drone_position
            distances = np.linalg.norm(to_obstacles, axis=1)

            # Check safety
            if np.any(distances < self.safety_margin):
                is_safe = False

            # Populate histogram with obstacle densities
            in_range = (distances > 0) & (distances < self.max_avoidance_distance)
            if np.any(in_range):
                to_obstacles_in_range = to_obstacles[in_range]
                distances_in_range = distances[in_range]
                confidences_in_range = obstacle_confidences[in_range]

                angles = np.arctan2(to_obstacles_in_range[:, 1], to_obstacles_in_range[:, 0])
                sectors = ((angles + np.pi) / (2 * np.pi) * num_sectors).astype(int) % num_sectors
                
                magnitudes = confidences_in_range * (1.0 - distances_in_range / self.max_avoidance_distance)
                np.add.at(histogram, sectors, magnitudes)
        
        # Find direction to target
        to_target = target_position - drone_position
        target_angle = np.arctan2(to_target[1], to_target[0])
        target_sector = int((target_angle + np.pi) / (2 * np.pi) * num_sectors) % num_sectors
        
        # Find lowest cost direction near target direction
        search_range = 20  # sectors to search on each side
        best_sector = target_sector
        min_cost = float('inf')
        
        for i in range(-search_range, search_range + 1):
            sector = (target_sector + i) % num_sectors
            # Cost is obstacle density + angular deviation from target
            cost = histogram[sector] + abs(i) * 0.1
            if cost < min_cost:
                min_cost = cost
                best_sector = sector
        
        # Convert best sector to direction vector
        angle = (best_sector / num_sectors) * 2 * np.pi - np.pi
        avoidance_vector = np.array([
            np.cos(angle),
            np.sin(angle),
            to_target[2] / max(np.linalg.norm(to_target[:2]), 0.01)
        ])
        
        # Normalize
        magnitude = np.linalg.norm(avoidance_vector)
        if magnitude > 0:
            avoidance_vector = avoidance_vector / magnitude
            
        return avoidance_vector, is_safe
    
    def _dynamic_window_avoidance(self,
                                 drone_position: np.ndarray,
                                 target_position: np.ndarray,
                                 drone_velocity: np.ndarray,
                                 obstacles: List[Obstacle]) -> Tuple[np.ndarray, bool]:
        """
        Dynamic Window Approach (DWA)
        Considers drone dynamics and finds optimal velocity
        """
        # Simplified DWA - sample velocity space
        max_vel = self.config.get('max_velocity', 5.0)
        max_accel = self.config.get('max_acceleration', 2.0)
        dt = self.config.get('prediction_time', 1.0)
        
        # Sample velocities within dynamic window
        num_samples = 10 # Reduced number of samples
        best_velocity = drone_velocity
        best_score = float('-inf')
        is_safe = True
        
        # Intelligent sampling around current velocity and target direction
        to_target = target_position - drone_position
        target_dir = to_target / (np.linalg.norm(to_target) + 1e-6)

        vx_samples = np.linspace(drone_velocity[0] - max_accel * dt, drone_velocity[0] + max_accel * dt, num_samples)
        vy_samples = np.linspace(drone_velocity[1] - max_accel * dt, drone_velocity[1] + max_accel * dt, num_samples)
        vz_samples = np.linspace(drone_velocity[2] - max_accel * dt, drone_velocity[2] + max_accel * dt, num_samples // 2)

        # Add target direction to samples
        vx_samples = np.append(vx_samples, drone_velocity[0] + target_dir[0] * max_accel * dt)
        vy_samples = np.append(vy_samples, drone_velocity[1] + target_dir[1] * max_accel * dt)
        vz_samples = np.append(vz_samples, drone_velocity[2] + target_dir[2] * max_accel * dt)

        for vx in vx_samples:
            for vy in vy_samples:
                for vz in vz_samples:
                    velocity = np.array([vx, vy, vz])
                    
                    # Check if achievable given acceleration limits
                    dv = velocity - drone_velocity
                    if np.linalg.norm(dv) > max_accel * dt:
                        continue
                    
                    # Predict trajectory
                    predicted_pos = drone_position + velocity * dt
                    
                    # Vectorized collision check
                    if obstacles:
                        obstacle_positions = np.array([o.position for o in obstacles])
                        distances = np.linalg.norm(predicted_pos - obstacle_positions, axis=1)
                        min_distance = np.min(distances)
                        if min_distance < self.safety_margin:
                            continue
                    else:
                        min_distance = float('inf')
                    
                    # Score this velocity
                    # Prefer: heading to target, high clearance, smooth motion
                    to_target = target_position - predicted_pos
                    heading_score = np.dot(velocity, to_target) / (
                        np.linalg.norm(velocity) * np.linalg.norm(to_target) + 0.01
                    )
                    clearance_score = min(min_distance / self.max_avoidance_distance, 1.0)
                    smoothness_score = -np.linalg.norm(dv)
                    
                    total_score = heading_score + 0.5 * clearance_score + 0.2 * smoothness_score
                    
                    if total_score > best_score:
                        best_score = total_score
                        best_velocity = velocity
        
        # Convert velocity to direction
        vel_magnitude = np.linalg.norm(best_velocity)
        if vel_magnitude > 0:
            avoidance_vector = best_velocity / vel_magnitude
        else:
            to_target = target_position - drone_position
            avoidance_vector = to_target / (np.linalg.norm(to_target) + 0.01)
        
        # Check if current position is safe
        for obstacle in obstacles:
            if np.linalg.norm(drone_position - obstacle.position) < self.safety_margin:
                is_safe = False
                break
                
        return avoidance_vector, is_safe
    
    def _rrt_avoidance(self,
                      drone_position: np.ndarray,
                      target_position: np.ndarray,
                      obstacles: List[Obstacle]) -> Tuple[np.ndarray, bool]:
        """
        Rapidly-exploring Random Tree (RRT) path planning
        Simplified version for real-time use
        """
        # Simplified RRT - just return potential field for now
        # Full RRT implementation would be too slow for real-time
        return self._potential_field_avoidance(drone_position, target_position, obstacles)
    
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
