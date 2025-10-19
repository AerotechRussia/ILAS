"""
Intelligent Landing System
Analyzes terrain and selects optimal landing locations for VTOL drones
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
from enum import Enum

from ..detection.obstacle_detector import Obstacle


class LandingMode(Enum):
    """Supported landing modes"""
    PRECISION = "precision"  # Precise landing on marked pad
    TERRAIN = "terrain"  # Landing on analyzed terrain
    EMERGENCY = "emergency"  # Emergency landing
    VERTICAL = "vertical"  # Vertical descent
    SLOPE = "slope"  # Landing on sloped surface


class LandingSite:
    """Represents a potential landing site"""
    
    def __init__(self, position: np.ndarray, size: np.ndarray,
                 slope: float, roughness: float, score: float):
        self.position = position  # Center position [x, y, z]
        self.size = size  # [width, depth] in meters
        self.slope = slope  # Slope angle in degrees
        self.roughness = roughness  # 0.0 (smooth) to 1.0 (very rough)
        self.score = score  # Overall suitability score 0.0 to 1.0
        
    def __repr__(self):
        return f"LandingSite(pos={self.position}, score={self.score:.2f})"


class IntelligentLanding:
    """
    Intelligent landing system for VTOL drones
    Analyzes environment and executes safe landings
    """
    
    def __init__(self, config: Dict):
        """
        Initialize intelligent landing system
        
        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.landing_mode = LandingMode(config.get('landing_mode', 'terrain'))
        self.min_landing_size = config.get('min_landing_size', [2.0, 2.0])  # meters
        self.max_slope = config.get('max_slope', 15.0)  # degrees
        self.max_roughness = config.get('max_roughness', 0.3)
        self.descent_rate = config.get('descent_rate', 0.5)  # m/s
        self.hover_height = config.get('hover_height', 2.0)  # meters
        
    def analyze_landing_area(self,
                            center_position: np.ndarray,
                            search_radius: float,
                            terrain_mapper: 'TerrainMapper',
                            obstacles: Optional[List[Obstacle]] = None) -> List[LandingSite]:
        """
        Analyze area and find suitable landing sites
        
        Args:
            center_position: Center of search area [x, y, z]
            search_radius: Radius to search in meters
            terrain_mapper: The terrain mapper object
            obstacles: Optional list of detected obstacles
            
        Returns:
            List of potential landing sites sorted by score
        """
        landing_sites = []
        
        # Use the terrain mapper to find suitable landing sites
        terrain_map = terrain_mapper.get_terrain_map()

        # This is a placeholder for a more sophisticated analysis of the terrain map.
        # For now, we'll just create a single landing site at the center of the search area
        # and score it using the terrain mapper.

        suitability = terrain_mapper.get_landing_suitability(center_position, search_radius)

        if suitability > 0.5: # Some threshold
            site = LandingSite(
                position=center_position,
                size=np.array(self.min_landing_size),
                slope=0.0, # Placeholder
                roughness=0.0, # Placeholder
                score=suitability
            )
            landing_sites.append(site)
        
        # Sort by score
        landing_sites.sort(key=lambda x: x.score, reverse=True)
        
        return landing_sites
    
    def _check_area_clear(self,
                         position: np.ndarray,
                         size: List[float],
                         obstacles: List[Obstacle]) -> bool:
        """Check if area is clear of obstacles"""
        if not obstacles:
            return True
        
        obstacle_positions = np.array([o.position for o in obstacles])
        obstacle_sizes = np.array([o.size for o in obstacles])
        
        dx = np.abs(obstacle_positions[:, 0] - position[0])
        dy = np.abs(obstacle_positions[:, 1] - position[1])
        
        clearance_x = (size[0] + obstacle_sizes[:, 0]) / 2 + 1.0
        clearance_y = (size[1] + obstacle_sizes[:, 1]) / 2 + 1.0
        
        if np.any((dx < clearance_x) & (dy < clearance_y)):
            return False
        
        return True
    
    def _calculate_landing_score(self,
                                 site_position: np.ndarray,
                                 target_position: np.ndarray,
                                 slope: float,
                                 roughness: float) -> float:
        """Calculate overall landing site score"""
        # Distance penalty
        distance = np.linalg.norm(site_position - target_position)
        distance_score = np.exp(-distance / 10.0)  # Prefer closer sites
        
        # Slope penalty
        slope_score = 1.0 - (slope / self.max_slope)
        
        # Roughness penalty
        roughness_score = 1.0 - (roughness / self.max_roughness)
        
        # Weighted combination
        total_score = (
            0.4 * distance_score +
            0.4 * slope_score +
            0.2 * roughness_score
        )
        
        return max(0.0, min(1.0, total_score))
    
    def select_best_landing_site(self,
                                landing_sites: List[LandingSite],
                                constraints: Optional[Dict] = None) -> Optional[LandingSite]:
        """
        Select best landing site from candidates
        
        Args:
            landing_sites: List of candidate sites
            constraints: Optional additional constraints
            
        Returns:
            Best landing site or None if no suitable site
        """
        if not landing_sites:
            return None
        
        # Apply constraints if provided
        if constraints:
            min_score = constraints.get('min_score', 0.5)
            landing_sites = [s for s in landing_sites if s.score >= min_score]
        
        if not landing_sites:
            return None
        
        return landing_sites[0]  # Already sorted by score
    
    def plan_landing_trajectory(self,
                               current_position: np.ndarray,
                               landing_site: LandingSite,
                               obstacles: List[Obstacle]) -> List[np.ndarray]:
        """
        Plan trajectory from current position to landing site
        
        Args:
            current_position: Current drone position
            landing_site: Target landing site
            obstacles: List of obstacles to avoid
            
        Returns:
            List of waypoints for landing trajectory
        """
        waypoints = []
        
        # 1. Move to hover position above landing site
        hover_position = landing_site.position.copy()
        hover_position[2] = landing_site.position[2] + self.hover_height
        
        # Add intermediate waypoint if needed for obstacle avoidance
        if not self._is_path_clear(current_position, hover_position, obstacles):
            # Add waypoint at safe altitude
            safe_altitude = self._get_safe_altitude(obstacles)
            intermediate = hover_position.copy()
            intermediate[2] = max(current_position[2], safe_altitude)
            waypoints.append(intermediate)
        
        waypoints.append(hover_position)
        
        # 2. Descend vertically to landing site
        num_descent_points = 5
        for i in range(1, num_descent_points + 1):
            descent_point = hover_position.copy()
            descent_point[2] = (
                hover_position[2] - 
                (hover_position[2] - landing_site.position[2]) * i / num_descent_points
            )
            waypoints.append(descent_point)
        
        return waypoints
    
    def _is_path_clear(self,
                      start: np.ndarray,
                      end: np.ndarray,
                      obstacles: List[Obstacle]) -> bool:
        """Check if path is clear of obstacles"""
        direction = end - start
        distance = np.linalg.norm(direction)
        
        if distance < 0.01:
            return True
        
        direction = direction / distance
        
        # Sample path
        num_samples = int(distance / 0.5) + 1
        points = start + direction * np.linspace(0, distance, num_samples)[:, np.newaxis]

        if obstacles:
            obstacle_positions = np.array([o.position for o in obstacles])

            # Calculate distances from all points to all obstacles
            diffs = points[:, np.newaxis, :] - obstacle_positions[np.newaxis, :, :]
            distances = np.linalg.norm(diffs, axis=2)
            
            if np.any(distances < 2.0):
                return False
        
        return True
    
    def _get_safe_altitude(self, obstacles: List[Obstacle]) -> float:
        """Get safe altitude above all obstacles"""
        if not obstacles:
            return 10.0
        
        max_height = 0.0
        for obstacle in obstacles:
            obstacle_top = obstacle.position[2] + obstacle.size[2] / 2
            max_height = max(max_height, obstacle_top)
        
        return max_height + 3.0  # 3m clearance
    
    def execute_landing(self,
                       waypoints: List[np.ndarray],
                       current_position: np.ndarray,
                       position_tolerance: float = 0.5) -> Tuple[np.ndarray, bool]:
        """
        Execute landing by following waypoints
        
        Args:
            waypoints: List of waypoints to follow
            current_position: Current drone position
            position_tolerance: Tolerance for waypoint arrival
            
        Returns:
            Tuple of (next_target_position, is_landing_complete)
        """
        if not waypoints:
            return current_position, True
        
        # Find next waypoint to reach
        next_waypoint = waypoints[0]
        
        # Check if reached current waypoint
        distance = np.linalg.norm(current_position - next_waypoint)
        
        if distance < position_tolerance:
            # Move to next waypoint
            waypoints.pop(0)
            if not waypoints:
                return current_position, True
            next_waypoint = waypoints[0]
        
        return next_waypoint, False
    
    def emergency_landing(self,
                         current_position: np.ndarray,
                         obstacles: List[Obstacle]) -> np.ndarray:
        """
        Execute emergency landing - descend vertically avoiding obstacles
        
        Args:
            current_position: Current position
            obstacles: List of obstacles
            
        Returns:
            Target position for emergency descent
        """
        # Check if area below is clear
        target = current_position.copy()
        target[2] = 0.0  # Ground level
        
        # Check for obstacles below
        for obstacle in obstacles:
            if (abs(obstacle.position[0] - current_position[0]) < 2.0 and
                abs(obstacle.position[1] - current_position[1]) < 2.0 and
                obstacle.position[2] < current_position[2]):
                # Obstacle below, try to move horizontally
                offset = np.array([2.0, 2.0, 0.0])
                target[:2] = current_position[:2] + offset[:2]
                break
        
        return target
