"""
Visualization utilities for ILAS
"""

import numpy as np
from typing import List, Optional
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from ..detection.obstacle_detector import Obstacle
from ..landing.intelligent_landing import LandingSite


def visualize_obstacles_3d(obstacles: List[Obstacle], 
                          drone_position: Optional[np.ndarray] = None,
                          target_position: Optional[np.ndarray] = None,
                          save_path: Optional[str] = None):
    """
    Visualize obstacles in 3D space
    
    Args:
        obstacles: List of detected obstacles
        drone_position: Optional drone position to display
        target_position: Optional target position to display
        save_path: Optional path to save figure
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot obstacles
    for obstacle in obstacles:
        pos = obstacle.position
        size = obstacle.size
        
        # Draw box for obstacle
        x = [pos[0] - size[0]/2, pos[0] + size[0]/2]
        y = [pos[1] - size[1]/2, pos[1] + size[1]/2]
        z = [pos[2] - size[2]/2, pos[2] + size[2]/2]
        
        # Draw edges
        for xi in x:
            for yi in y:
                ax.plot([xi, xi], [yi, yi], z, 'r-')
        for xi in x:
            for zi in z:
                ax.plot([xi, xi], y, [zi, zi], 'r-')
        for yi in y:
            for zi in z:
                ax.plot(x, [yi, yi], [zi, zi], 'r-')
    
    # Plot drone position
    if drone_position is not None:
        ax.scatter(drone_position[0], drone_position[1], drone_position[2],
                  c='blue', marker='o', s=100, label='Drone')
    
    # Plot target position
    if target_position is not None:
        ax.scatter(target_position[0], target_position[1], target_position[2],
                  c='green', marker='*', s=200, label='Target')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('ILAS - Obstacle Detection')
    ax.legend()
    
    if save_path:
        plt.savefig(save_path)
    else:
        plt.show()


def visualize_landing_sites(landing_sites: List[LandingSite],
                           drone_position: Optional[np.ndarray] = None,
                           save_path: Optional[str] = None):
    """
    Visualize landing sites on 2D map
    
    Args:
        landing_sites: List of landing sites
        drone_position: Optional drone position
        save_path: Optional path to save figure
    """
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Plot landing sites
    for site in landing_sites:
        pos = site.position
        size = site.size
        
        # Color by score
        color = plt.cm.RdYlGn(site.score)
        
        # Draw rectangle
        rect = plt.Rectangle(
            (pos[0] - size[0]/2, pos[1] - size[1]/2),
            size[0], size[1],
            facecolor=color,
            edgecolor='black',
            alpha=0.5
        )
        ax.add_patch(rect)
        
        # Add score text
        ax.text(pos[0], pos[1], f'{site.score:.2f}',
               ha='center', va='center', fontsize=8)
    
    # Plot drone position
    if drone_position is not None:
        ax.plot(drone_position[0], drone_position[1],
               'bo', markersize=10, label='Drone')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('ILAS - Landing Site Analysis')
    ax.set_aspect('equal')
    ax.legend()
    ax.grid(True)
    
    if save_path:
        plt.savefig(save_path)
    else:
        plt.show()
