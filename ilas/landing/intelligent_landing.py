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
                            terrain_data: Optional[np.ndarray] = None,
                            obstacles: Optional[List[Obstacle]] = None) -> List[LandingSite]:
        """
        Analyze area and find suitable landing sites
        
        Args:
            center_position: Center of search area [x, y, z]
            search_radius: Radius to search in meters
            terrain_data: Optional height map of terrain
            obstacles: Optional list of detected obstacles
            
        Returns:
            List of potential landing sites sorted by score
        """
        landing_sites = []
        
        if terrain_data is not None:
            landing_sites = self._analyze_terrain(
                center_position, search_radius, terrain_data, obstacles or []
            )
        else:
            # Use obstacle-free regions
            landing_sites = self._analyze_free_space(
                center_position, search_radius, obstacles or []
            )
        
        # Sort by score
        landing_sites.sort(key=lambda x: x.score, reverse=True)
        
        return landing_sites
    
    def _analyze_terrain(self,
                        center_position: np.ndarray,
                        search_radius: float,
                        terrain_data: np.ndarray,
                        obstacles: List[Obstacle]) -> List[LandingSite]:
        """Analyze terrain height map for landing sites"""
        sites = []

        grid_resolution = 0.5
        num_points = int(2 * search_radius / grid_resolution)

        for i in range(num_points):
            for j in range(num_points):
                x = center_position[0] - search_radius + i * grid_resolution
                y = center_position[1] - search_radius + j * grid_resolution

                patch_size_px = int(self.min_landing_size[0] / grid_resolution)

                if i + patch_size_px >= terrain_data.shape[0] or j + patch_size_px >= terrain_data.shape[1]:
                    continue

                patch = terrain_data[i:i+patch_size_px, j:j+patch_size_px]

                slope = self._calculate_slope(patch, grid_resolution)
                roughness = self._calculate_roughness(patch)

                if slope <= self.max_slope and roughness <= self.max_roughness:
                    site_pos = np.array([x + self.min_landing_size[0]/2, y + self.min_landing_size[1]/2, np.mean(patch)])

                    # Calculate obstacle clearance
                    clearance = 1.0
                    if obstacles:
                        min_dist = float('inf')
                        for obs in obstacles:
                            dist = np.linalg.norm(site_pos[:2] - obs.position[:2])
                            if dist < min_dist:
                                min_dist = dist
                        # Score clearance from 0 to 1 based on distance
                        clearance = min(min_dist / (self.min_landing_size[0]), 1.0)

                    score = self._calculate_landing_score(
                        site_pos,
                        center_position,
                        slope,
                        roughness,
                        clearance
                    )

                    site = LandingSite(
                        position=site_pos,
                        size=np.array(self.min_landing_size),
                        slope=slope,
                        roughness=roughness,
                        score=score
                    )
                    sites.append(site)

        return sites

    def _analyze_free_space(self,
                           center_position: np.ndarray,
                           search_radius: float,
                           obstacles: List[Obstacle]) -> List[LandingSite]:
        """Find flat obstacle-free areas for landing"""
        sites = []

        # Grid search
        grid_resolution = 1.0
        num_points = int(2 * search_radius / grid_resolution)

        for i in range(num_points):
            for j in range(num_points):
                x = center_position[0] - search_radius + i * grid_resolution
                y = center_position[1] - search_radius + j * grid_resolution
                z = center_position[2]  # Assume ground level

                candidate_pos = np.array([x, y, z])

                # Check if area is free of obstacles
                is_clear = self._check_area_clear(
                    candidate_pos,
                    self.min_landing_size,
                    obstacles
                )

                if is_clear:
                    # Calculate score based on distance from center
                    distance = np.linalg.norm(candidate_pos[:2] - center_position[:2])
                    score = 1.0 - (distance / search_radius)

                    site = LandingSite(
                        position=candidate_pos,
                        size=np.array(self.min_landing_size),
                        slope=0.0,
                        roughness=0.0,
                        score=score
                    )
                    sites.append(site)

        return sites

    def _check_area_clear(self,
                         position: np.ndarray,
                         size: List[float],
                         obstacles: List[Obstacle]) -> bool:
        """Check if area is clear of obstacles"""
        for obstacle in obstacles:
            # Check if obstacle overlaps with landing area
            dx = abs(obstacle.position[0] - position[0])
            dy = abs(obstacle.position[1] - position[1])

            clearance_x = (size[0] + obstacle.size[0]) / 2 + 1.0  # 1m safety margin
            clearance_y = (size[1] + obstacle.size[1]) / 2 + 1.0

            if dx < clearance_x and dy < clearance_y:
                return False

        return True

    def _calculate_slope(self, height_map: np.ndarray, resolution: float) -> float:
        """Calculate average slope of terrain patch in degrees"""
        if height_map.shape[0] < 2 or height_map.shape[1] < 2:
            return 0.0
        
        # Calculate gradients
        grad_x = np.gradient(height_map, axis=1) / resolution
        grad_y = np.gradient(height_map, axis=0) / resolution
        
        # Calculate slope magnitude
        slope_rad = np.arctan(np.sqrt(grad_x**2 + grad_y**2))
        slope_deg = np.degrees(np.mean(slope_rad))
        
        return slope_deg

    def _calculate_roughness(self, height_map: np.ndarray) -> float:
        """Calculate terrain roughness (0.0 = smooth, 1.0 = very rough)"""
        if height_map.size < 2:
            return 0.0
        
        # Standard deviation of heights normalized
        std = np.std(height_map)
        roughness = min(std / 0.5, 1.0)  # Normalize to 0-1
        
        return roughness
    
    def _calculate_landing_score(self,
                                 site_position: np.ndarray,
                                 target_position: np.ndarray,
                                 slope: float,
                                 roughness: float,
                                 obstacle_clearance: float) -> float:
        """Calculate overall landing site score based on multiple criteria."""
        # --- Scoring Components (0.0 to 1.0) ---

        # 1. Roughness/Flatness Score (40%)
        # Lower roughness is better.
        roughness_score = 1.0 - (roughness / self.max_roughness)

        # 2. Obstacle Clearance Score (30%)
        # Higher clearance is better.
        clearance_score = obstacle_clearance

        # 3. Slope Score (20%)
        # Lower slope is better.
        slope_score = 1.0 - (slope / self.max_slope)
        
        # 4. Distance Score (10%)
        # Closer to the target is better.
        distance = np.linalg.norm(site_position[:2] - target_position[:2])
        # Using exponential decay for distance scoring
        distance_score = np.exp(-distance / (self.config.get('landing_search_radius', 20.0) * 0.5))

        # --- Weighted Combination ---
        total_score = (
            0.4 * roughness_score +
            0.3 * clearance_score +
            0.2 * slope_score +
            0.1 * distance_score
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
        for i in range(num_samples):
            point = start + direction * (i * distance / num_samples)
            
            for obstacle in obstacles:
                if np.linalg.norm(point - obstacle.position) < 2.0:
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
