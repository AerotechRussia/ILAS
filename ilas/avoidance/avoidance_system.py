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
        self.strategy = AvoidanceStrategy(config.get('strategy', 'potential_field'))
        self.safety_margin = config.get('safety_margin', 2.0)  # meters
        self.max_avoidance_distance = config.get('max_avoidance_distance', 10.0)
        self.avoidance_gain = config.get('avoidance_gain', 1.0)

        # State machine
        self.state = AvoidanceState.DIRECT
        self.circling_start_position = None
        self.circling_obstacle = None
        self.circling_direction = 1  # 1 for clockwise, -1 for counter-clockwise
        
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
        elif self.strategy == AvoidanceStrategy.HYBRID:
            return self._hybrid_avoidance(
                drone_position, target_position, obstacles
            )
        else:
            return self._potential_field_avoidance(
                drone_position, target_position, obstacles
            )

    def _hybrid_avoidance(self,
                          drone_position: np.ndarray,
                          target_position: np.ndarray,
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
            if closest_obstacle and min_dist < self.max_avoidance_distance:
                self.state = AvoidanceState.AVOIDING
            else:
                to_target = target_position - drone_position
                return to_target / (np.linalg.norm(to_target) + 1e-6), True

        # State: AVOIDING
        if self.state == AvoidanceState.AVOIDING:
            if path_is_clear and min_dist > self.max_avoidance_distance:
                self.state = AvoidanceState.DIRECT
                to_target = target_position - drone_position
                return to_target / (np.linalg.norm(to_target) + 1e-6), True
            elif not path_is_clear and closest_obstacle:
                # If we are not making progress, start circling
                to_target_dir = (target_position - drone_position) / (np.linalg.norm(target_position - drone_position) + 1e-6)
                avoid_vec, _ = self._potential_field_avoidance(drone_position, target_position, obstacles)
                if np.dot(to_target_dir, avoid_vec) < 0.3: # Check if avoidance is pushing us away from target
                    self.state = AvoidanceState.CIRCLING
                    self.circling_obstacle = closest_obstacle
                    self.circling_start_position = drone_position
                    # Determine circling direction
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

            # Check if we've been circling for too long or returned to start
            if self.circling_start_position is not None and np.linalg.norm(drone_position - self.circling_start_position) < 1.0:
                # Fallback: reverse direction to try and unstick
                self.circling_direction *= -1
                self.circling_start_position = None # Avoid rapid flipping

            return self._circling_avoidance(drone_position, target_position, obstacles)

        # Default behavior for AVOIDING state
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

    def _circling_avoidance(self,
                              drone_position: np.ndarray,
                              target_position: np.ndarray,
                              obstacles: List[Obstacle]) -> Tuple[np.ndarray, bool]:
        """
        Follows the boundary of an obstacle to navigate around it.
        """
        if self.circling_obstacle is None:
            # Fallback to potential field if no obstacle is being circled
            return self._potential_field_avoidance(drone_position, target_position, obstacles)

        obstacle_pos = self.circling_obstacle.position
        to_obstacle = obstacle_pos - drone_position
        dist_to_obstacle = np.linalg.norm(to_obstacle)

        # Vector pointing from obstacle to drone
        radial_vector = -to_obstacle / (dist_to_obstacle + 1e-6)

        # Tangential vector for circling (2D for now)
        tangential_vector_2d = np.array([-radial_vector[1], radial_vector[0]]) * self.circling_direction
        tangential_vector = np.append(tangential_vector_2d, 0)

        # Force to maintain safe distance
        distance_error = dist_to_obstacle - (self.safety_margin + self.circling_obstacle.size[0] / 2)
        correction_force = radial_vector * distance_error * 0.5

        # Attraction to target
        to_target = target_position - drone_position
        attractive_force = to_target / (np.linalg.norm(to_target) + 1e-6)

        # Combine forces: tangential, correction, and attraction
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
