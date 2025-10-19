"""
Example usage of ILAS system
"""

import numpy as np
from ilas import ObstacleDetector, AvoidanceSystem, IntelligentLanding, ControllerManager
from ilas.core import ILASCore
from ilas.utils.config import get_default_config


def example_obstacle_detection():
    """Example: Obstacle detection"""
    print("=== Obstacle Detection Example ===\n")
    
    config = {
        'detection_range': 20.0,
        'min_confidence': 0.6,
        'sensors': [
            {'type': 'lidar', 'enabled': True}
        ]
    }
    
    detector = ObstacleDetector(config)
    
    # Simulate LiDAR data (point cloud)
    lidar_data = np.array([
        [5.0, 0.0, 0.0],
        [5.1, 0.1, 0.0],
        [4.9, -0.1, 0.0],
        [10.0, 5.0, 1.0],
        [10.1, 5.1, 1.1],
        [9.9, 4.9, 0.9],
    ])
    
    sensor_data = {'lidar': lidar_data}
    
    # Detect obstacles
    obstacles = detector.detect_obstacles(sensor_data)
    
    print(f"Detected {len(obstacles)} obstacles:")
    for i, obs in enumerate(obstacles):
        print(f"  {i+1}. {obs}")
    
    # Get critical obstacles
    drone_pos = np.array([0.0, 0.0, 0.0])
    drone_heading = np.array([1.0, 0.0, 0.0])
    
    critical = detector.get_critical_obstacles(drone_pos, drone_heading, 10.0)
    print(f"\nCritical obstacles: {len(critical)}")


def example_avoidance():
    """Example: Obstacle avoidance"""
    print("\n=== Obstacle Avoidance Example ===\n")
    
    config = {
        'strategy': 'potential_field',
        'safety_margin': 2.0,
        'max_avoidance_distance': 10.0,
        'avoidance_gain': 1.0
    }
    
    avoidance = AvoidanceSystem(config)
    
    # Create sample obstacle
    from ilas.detection.obstacle_detector import Obstacle
    obstacle = Obstacle(
        position=np.array([5.0, 0.0, 0.0]),
        size=np.array([1.0, 1.0, 2.0]),
        distance=5.0,
        confidence=0.9
    )
    
    # Calculate avoidance
    drone_pos = np.array([0.0, 0.0, 0.0])
    target_pos = np.array([10.0, 0.0, 0.0])
    drone_vel = np.array([1.0, 0.0, 0.0])
    
    avoidance_vector, is_safe = avoidance.calculate_avoidance_vector(
        drone_pos, target_pos, drone_vel, [obstacle]
    )
    
    print(f"Avoidance vector: {avoidance_vector}")
    print(f"Safety status: {'SAFE' if is_safe else 'UNSAFE'}")


def example_landing():
    """Example: Intelligent landing"""
    print("\n=== Intelligent Landing Example ===\n")
    
    config = {
        'landing_mode': 'terrain',
        'min_landing_size': [2.0, 2.0],
        'max_slope': 15.0,
        'max_roughness': 0.3
    }
    
    landing = IntelligentLanding(config)
    
    # Analyze landing area
    center = np.array([10.0, 10.0, 0.0])
    search_radius = 10.0
    
    # Create sample obstacles
    from ilas.detection.obstacle_detector import Obstacle
    obstacles = [
        Obstacle(
            position=np.array([12.0, 12.0, 0.0]),
            size=np.array([2.0, 2.0, 3.0]),
            distance=2.8,
            confidence=0.9
        )
    ]
    
    # Find landing sites
    sites = landing.analyze_landing_area(center, search_radius, obstacles=obstacles)
    
    print(f"Found {len(sites)} potential landing sites:")
    for i, site in enumerate(sites[:5]):  # Show top 5
        print(f"  {i+1}. {site}")
    
    # Select best site
    best_site = landing.select_best_landing_site(sites)
    
    if best_site:
        print(f"\nBest landing site: {best_site}")
        
        # Plan trajectory
        current_pos = np.array([0.0, 0.0, 10.0])
        waypoints = landing.plan_landing_trajectory(current_pos, best_site, obstacles)
        
        print(f"\nLanding trajectory with {len(waypoints)} waypoints:")
        for i, wp in enumerate(waypoints):
            print(f"  {i+1}. {wp}")


def example_full_system():
    """Example: Full ILAS system"""
    print("\n=== Full ILAS System Example ===\n")
    
    config = get_default_config()
    
    # Initialize ILAS
    ilas = ILASCore(config)
    
    # Start system
    if ilas.start():
        print("ILAS system started successfully")
        
        # Get system status
        status = ilas.get_system_status()
        print(f"\nSystem Status:")
        print(f"  Running: {status['is_running']}")
        print(f"  Controller: {status['controller_connected']}")
        
        # Stop system
        ilas.stop()
        print("\nILAS system stopped")
    else:
        print("Failed to start ILAS system")


def main():
    """Run all examples"""
    print("="*60)
    print("ILAS Examples - Intelligent Landing and Avoidance System")
    print("="*60 + "\n")
    
    example_obstacle_detection()
    example_avoidance()
    example_landing()
    example_full_system()
    
    print("\n" + "="*60)
    print("Examples completed")
    print("="*60)


if __name__ == '__main__':
    main()
