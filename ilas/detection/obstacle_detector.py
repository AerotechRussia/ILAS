"""
Obstacle Detection Module
Detects obstacles using various sensor inputs (LiDAR, cameras, ultrasonic, etc.)
"""

import numpy as np
from typing import List, Dict, Tuple, Optional
from enum import Enum


class SensorType(Enum):
    """Supported sensor types for obstacle detection"""
    LIDAR = "lidar"
    CAMERA = "camera"
    ULTRASONIC = "ultrasonic"
    RADAR = "radar"
    DEPTH_CAMERA = "depth_camera"


class Obstacle:
    """Represents a detected obstacle"""
    
    def __init__(self, position: np.ndarray, size: np.ndarray, 
                 distance: float, confidence: float):
        self.position = position  # [x, y, z] in meters
        self.size = size  # [width, height, depth] in meters
        self.distance = distance  # distance from drone in meters
        self.confidence = confidence  # 0.0 to 1.0
        
    def __repr__(self):
        return f"Obstacle(pos={self.position}, dist={self.distance:.2f}m, conf={self.confidence:.2f})"


class ObstacleDetector:
    """
    Main obstacle detection system
    Processes sensor data and identifies obstacles in the environment
    """
    
    def __init__(self, config: Dict):
        """
        Initialize obstacle detector
        
        Args:
            config: Configuration dictionary with sensor settings
        """
        self.config = config
        self.sensors = {}
        self.detection_range = config.get('detection_range', 20.0)  # meters
        self.min_confidence = config.get('min_confidence', 0.6)
        self.obstacle_buffer = []
        self._initialize_sensors()
        
    def _initialize_sensors(self):
        """Initialize configured sensors"""
        sensor_configs = self.config.get('sensors', [])
        for sensor_config in sensor_configs:
            sensor_type = SensorType(sensor_config['type'])
            self.sensors[sensor_type] = sensor_config
            
    def detect_obstacles(self, sensor_data: Dict) -> List[Obstacle]:
        """
        Detect obstacles from sensor data
        
        Args:
            sensor_data: Dictionary of sensor readings
            
        Returns:
            List of detected obstacles
        """
        obstacles = []
        
        for sensor_type, data in sensor_data.items():
            if sensor_type in self.sensors:
                detected = self._process_sensor_data(sensor_type, data)
                obstacles.extend(detected)
                
        # Filter by confidence and merge close obstacles
        obstacles = self._filter_obstacles(obstacles)
        obstacles = self._merge_close_obstacles(obstacles)
        
        # Update obstacle buffer for tracking
        self.obstacle_buffer = obstacles
        
        return obstacles
    
    def _process_sensor_data(self, sensor_type: SensorType, 
                            data: np.ndarray) -> List[Obstacle]:
        """Process data from specific sensor type"""
        obstacles = []
        
        if sensor_type == SensorType.LIDAR:
            obstacles = self._process_lidar(data)
        elif sensor_type == SensorType.CAMERA:
            obstacles = self._process_camera(data)
        elif sensor_type == SensorType.ULTRASONIC:
            obstacles = self._process_ultrasonic(data)
        elif sensor_type == SensorType.RADAR:
            obstacles = self._process_radar(data)
        elif sensor_type == SensorType.DEPTH_CAMERA:
            obstacles = self._process_depth_camera(data)
            
        return obstacles
    
    def _process_lidar(self, data: np.ndarray) -> List[Obstacle]:
        """Process LiDAR point cloud data"""
        obstacles = []
        
        # Simple clustering algorithm for point cloud
        if len(data) == 0:
            return obstacles
            
        # Group points by distance and angle
        for i in range(0, len(data), 10):
            cluster = data[i:i+10]
            if len(cluster) < 3:
                continue
                
            # Calculate cluster properties
            mean_pos = np.mean(cluster[:, :3], axis=0)
            distance = np.linalg.norm(mean_pos)
            
            if distance <= self.detection_range:
                size = np.max(cluster[:, :3], axis=0) - np.min(cluster[:, :3], axis=0)
                confidence = min(1.0, len(cluster) / 10.0)
                
                obstacle = Obstacle(mean_pos, size, distance, confidence)
                obstacles.append(obstacle)
                
        return obstacles
    
    def _process_camera(self, data: np.ndarray) -> List[Obstacle]:
        """Process camera image data with object detection"""
        obstacles = []
        
        # Placeholder for computer vision object detection
        # In real implementation, use YOLO, SSD, or similar
        # For now, simulate detection
        
        return obstacles
    
    def _process_ultrasonic(self, data: np.ndarray) -> List[Obstacle]:
        """Process ultrasonic sensor data"""
        obstacles = []
        
        # Ultrasonic gives distance measurements
        for reading in data:
            distance = reading[0]
            angle = reading[1] if len(reading) > 1 else 0
            
            if 0.1 < distance <= self.detection_range:
                position = np.array([
                    distance * np.cos(angle),
                    distance * np.sin(angle),
                    0.0
                ])
                size = np.array([0.5, 0.5, 0.5])  # Unknown size
                confidence = 0.7
                
                obstacle = Obstacle(position, size, distance, confidence)
                obstacles.append(obstacle)
                
        return obstacles
    
    def _process_radar(self, data: np.ndarray) -> List[Obstacle]:
        """Process radar data"""
        obstacles = []
        
        # Similar to ultrasonic but with better range
        for reading in data:
            distance = reading[0]
            azimuth = reading[1] if len(reading) > 1 else 0
            elevation = reading[2] if len(reading) > 2 else 0
            
            if distance <= self.detection_range:
                position = np.array([
                    distance * np.cos(elevation) * np.cos(azimuth),
                    distance * np.cos(elevation) * np.sin(azimuth),
                    distance * np.sin(elevation)
                ])
                size = np.array([1.0, 1.0, 1.0])
                confidence = 0.8
                
                obstacle = Obstacle(position, size, distance, confidence)
                obstacles.append(obstacle)
                
        return obstacles
    
    def _process_depth_camera(self, data: np.ndarray) -> List[Obstacle]:
        """Process depth camera data"""
        obstacles = []
        
        # Depth camera provides dense depth map
        # Find clusters of nearby points
        if len(data.shape) == 2:
            # Depth image format
            threshold_distance = self.detection_range
            valid_points = data < threshold_distance
            
            # Simple region growing (simplified)
            # In production, use proper segmentation
            
        return obstacles
    
    def _filter_obstacles(self, obstacles: List[Obstacle]) -> List[Obstacle]:
        """Filter obstacles by confidence threshold"""
        return [obs for obs in obstacles if obs.confidence >= self.min_confidence]
    
    def _merge_close_obstacles(self, obstacles: List[Obstacle], 
                               threshold: float = 1.0) -> List[Obstacle]:
        """Merge obstacles that are close to each other"""
        if len(obstacles) <= 1:
            return obstacles
            
        merged = []
        used = set()
        
        for i, obs1 in enumerate(obstacles):
            if i in used:
                continue
                
            cluster = [obs1]
            for j, obs2 in enumerate(obstacles[i+1:], i+1):
                if j in used:
                    continue
                    
                dist = np.linalg.norm(obs1.position - obs2.position)
                if dist < threshold:
                    cluster.append(obs2)
                    used.add(j)
                    
            # Merge cluster into single obstacle
            if len(cluster) == 1:
                merged.append(obs1)
            else:
                positions = np.array([obs.position for obs in cluster])
                mean_pos = np.mean(positions, axis=0)
                max_size = np.max([obs.size for obs in cluster], axis=0)
                mean_dist = np.mean([obs.distance for obs in cluster])
                max_conf = max([obs.confidence for obs in cluster])
                
                merged_obs = Obstacle(mean_pos, max_size, mean_dist, max_conf)
                merged.append(merged_obs)
                
        return merged
    
    def get_critical_obstacles(self, drone_position: np.ndarray, 
                              drone_heading: np.ndarray,
                              safety_distance: float = 5.0) -> List[Obstacle]:
        """
        Get obstacles that are critical for collision avoidance
        
        Args:
            drone_position: Current drone position [x, y, z]
            drone_heading: Drone heading vector
            safety_distance: Minimum safe distance in meters
            
        Returns:
            List of critical obstacles
        """
        critical = []
        
        for obstacle in self.obstacle_buffer:
            # Calculate relative position
            rel_pos = obstacle.position - drone_position
            distance = np.linalg.norm(rel_pos)
            
            # Check if in front of drone
            if distance > 0:
                direction = rel_pos / distance
                dot_product = np.dot(direction, drone_heading)
                
                # If obstacle is ahead and within safety distance
                if dot_product > 0.5 and distance < safety_distance:
                    critical.append(obstacle)
                    
        return critical
