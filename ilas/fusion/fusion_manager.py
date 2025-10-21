"""
Fusion Manager for ILAS
Fuses obstacle data from multiple sensors into a coherent 3D map of the environment.
"""

import numpy as np
from typing import List, Dict, Tuple, Optional
from ..detection.obstacle_detector import Obstacle

class FusionManager:
    """
    Manages the fusion of obstacle data from various sensors.
    """

    def __init__(self, config: Dict):
        """
        Initialize the Fusion Manager.

        Args:
            config: Configuration dictionary with fusion parameters.
        """
        self.config = config
        self.cluster_threshold = config.get('cluster_threshold', 1.5)  # meters
        self.sensor_weights = config.get('sensor_weights', {
            'lidar': 0.9,
            'camera': 0.7,
            'radar': 0.8,
            'ultrasonic': 0.5
        })
        self.min_cluster_confidence = config.get('min_cluster_confidence', 0.6)

    def fuse_obstacles(self, sensor_obstacles: Dict[str, List[Obstacle]]) -> List[Obstacle]:
        """
        Fuse obstacle data from multiple sensors.

        Args:
            sensor_obstacles: Dictionary with sensor type as key and list of obstacles as value.

        Returns:
            A list of fused and verified obstacles.
        """
        all_obstacles = []
        for sensor_type, obstacles in sensor_obstacles.items():
            for obstacle in obstacles:
                # Add sensor type information for weighting
                obstacle.sensor_type = sensor_type
                all_obstacles.append(obstacle)

        if not all_obstacles:
            return []

        clusters = self._cluster_obstacles(all_obstacles)

        fused_obstacles = []
        for cluster in clusters:
            merged_obstacle = self._merge_cluster(cluster)
            if merged_obstacle.confidence >= self.min_cluster_confidence:
                fused_obstacles.append(merged_obstacle)

        return fused_obstacles

    def _cluster_obstacles(self, obstacles: List[Obstacle]) -> List[List[Obstacle]]:
        """
        Cluster obstacles based on spatial proximity.

        Args:
            obstacles: A list of all detected obstacles.

        Returns:
            A list of clusters, where each cluster is a list of obstacles.
        """
        clusters = []
        visited = [False] * len(obstacles)

        for i in range(len(obstacles)):
            if not visited[i]:
                cluster = []
                q = [i]
                visited[i] = True

                while q:
                    current_idx = q.pop(0)
                    cluster.append(obstacles[current_idx])

                    for j in range(len(obstacles)):
                        if not visited[j]:
                            dist = np.linalg.norm(
                                obstacles[current_idx].position - obstacles[j].position
                            )
                            if dist < self.cluster_threshold:
                                visited[j] = True
                                q.append(j)
                clusters.append(cluster)

        return clusters

    def _merge_cluster(self, cluster: List[Obstacle]) -> Obstacle:
        """
        Merge a cluster of obstacles into a single obstacle.

        Args:
            cluster: A list of obstacles in the same cluster.

        Returns:
            A single, merged obstacle.
        """
        if not cluster:
            # This should not happen if called correctly
            return None

        # Use weighted average for position and confidence
        total_weight = 0
        weighted_pos = np.zeros(3)
        weighted_conf = 0

        for obstacle in cluster:
            weight = self.sensor_weights.get(obstacle.sensor_type, 0.5)
            weighted_pos += obstacle.position * weight
            weighted_conf += obstacle.confidence * weight
            total_weight += weight

        # Normalize position and confidence
        merged_pos = weighted_pos / total_weight if total_weight > 0 else np.mean([o.position for o in cluster], axis=0)
        merged_conf = weighted_conf / total_weight if total_weight > 0 else np.mean([o.confidence for o in cluster])

        # Take max size
        merged_size = np.max([o.size for o in cluster], axis=0)

        # Recalculate distance
        merged_dist = np.linalg.norm(merged_pos)

        return Obstacle(
            position=merged_pos,
            size=merged_size,
            distance=merged_dist,
            confidence=merged_conf
        )
