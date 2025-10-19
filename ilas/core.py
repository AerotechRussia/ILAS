"""
ILAS Main Integration Module
Integrates all components for complete obstacle avoidance and landing system
"""

import numpy as np
from typing import Dict, List, Optional, Tuple
import time
from dronekit import connect, VehicleMode

from .detection.obstacle_detector import ObstacleDetector, Obstacle
from .avoidance.avoidance_system import AvoidanceSystem
from .landing.intelligent_landing import IntelligentLanding, LandingSite
from .controllers.controller_manager import ControllerManager
from .slam.slam_system import SLAMSystem
from .terrain.terrain_mapper import TerrainMapper
from .fusion.sensor_fusion import SensorFusionEKF
from .mission.mission_planner import MissionPlanner
from .geofence.geofence_manager import GeofenceManager


class ILASCore:
    """
    Core ILAS system integrating all components
    """
    
    def __init__(self, config: Dict):
        """
        Initialize ILAS system
        
        Args:
            config: Master configuration dictionary
        """
        self.config = config
        
        # Initialize components
        self.detector = ObstacleDetector(config.get('detection', {}))
        self.avoidance = AvoidanceSystem(config.get('avoidance', {}))
        self.landing = IntelligentLanding(config.get('landing', {}))
        self.controller = ControllerManager(config.get('controller', {}))
        self.slam = SLAMSystem(config.get('slam', {}))
        self.terrain_mapper = TerrainMapper(config.get('terrain_mapping', {}))
        self.sensor_fusion = SensorFusionEKF(config.get('sensor_fusion', {}))
        self.mission_planner = MissionPlanner(config.get('mission_planning', {}))
        self.geofence_manager = GeofenceManager(config.get('geofence', {}))
        
        # Connect to the vehicle
        connection_string = config.get('connection_string', '127.0.0.1:14550')
        self.vehicle = connect(connection_string, wait_ready=True)
        self.lidar_data = [0] * 360

        @self.vehicle.on_message('DISTANCE_SENSOR')
        def listener(self, name, message):
            self.lidar_data = message.distances

        self.is_running = False
        self.current_mission = None
        self.update_rate = config.get('update_rate', 10.0)
        
    def start(self) -> bool:
        """
        Start ILAS system
        
        Returns:
            True if started successfully
        """
        # Connect to flight controller
        if not self.controller.connect():
            print("Failed to connect to flight controller")
            return False
        
        self.is_running = True
        print("ILAS system started successfully")
        return True
    
    def stop(self):
        """Stop ILAS system"""
        self.is_running = False
        self.controller.disconnect()
        print("ILAS system stopped")
    
    def navigate_to_target(self,
                          target_position: np.ndarray,
                          sensor_data: Dict) -> Tuple[np.ndarray, bool]:
        """
        Navigate to target position while avoiding obstacles
        
        Args:
            target_position: Target position [x, y, z]
            sensor_data: Current sensor readings
            
        Returns:
            Tuple of (navigation_command, is_safe)
        """
        # Get the fused state
        fused_state = self.sensor_fusion.get_state()
        current_position = fused_state[0:3]
        current_velocity = fused_state[3:6]
        
        # Detect obstacles
        obstacles = self.detector.detect_obstacles(sensor_data)
        
        # Calculate heading to target
        to_target = target_position - current_position
        target_distance = np.linalg.norm(to_target)
        
        if target_distance < 0.5:
            # Reached target
            return np.zeros(3), True
        
        heading = to_target / target_distance
        
        # Check for critical obstacles
        critical_obstacles = self.detector.get_critical_obstacles(
            current_position, heading, safety_distance=5.0
        )
        
        # Calculate avoidance vector if needed
        if critical_obstacles:
            avoidance_vector, is_safe = self.avoidance.calculate_avoidance_vector(
                current_position,
                target_position,
                current_velocity,
                obstacles
            )
            return avoidance_vector, is_safe
        else:
            # No obstacles, proceed directly
            return heading, True
    
    def execute_landing(self,
                       landing_area_center: np.ndarray,
                       search_radius: float,
                       sensor_data: Dict) -> bool:
        """
        Execute intelligent landing procedure
        
        Args:
            landing_area_center: Center of landing area
            search_radius: Radius to search for landing sites
            sensor_data: Current sensor readings
            
        Returns:
            True if landing successful
        """
        # Get the fused state
        fused_state = self.sensor_fusion.get_state()
        current_position = fused_state[0:3]

        # Update Terrain Mapper with the latest sensor data and fused pose
        pose = (
            current_position[0], current_position[1], current_position[2],
            fused_state[9], fused_state[10], fused_state[11] # roll, pitch, yaw
        )
        self.terrain_mapper.update(sensor_data, pose)
        
        # Detect obstacles
        obstacles = self.detector.detect_obstacles(sensor_data)
        
        # Analyze landing area
        landing_sites = self.landing.analyze_landing_area(
            landing_area_center,
            search_radius,
            terrain_data=self.terrain_mapper.get_terrain_map(),
            obstacles=obstacles
        )
        
        # Select best site
        best_site = self.landing.select_best_landing_site(landing_sites)
        
        if not best_site:
            print("No suitable landing site found")
            return False
        
        print(f"Selected landing site: {best_site}")
        
        # Plan landing trajectory
        waypoints = self.landing.plan_landing_trajectory(
            current_position,
            best_site,
            obstacles
        )
        
        # Execute landing by following waypoints
        for waypoint in waypoints:
            self.controller.send_command('position', waypoint)
            # Wait for waypoint to be reached
            # In real implementation, monitor progress
            time.sleep(1.0)
        
        # Final landing command
        self.controller.send_command('land', None)
        
        return True
    
    def emergency_land(self, sensor_data: Dict):
        """
        Execute emergency landing procedure
        
        Args:
            sensor_data: Current sensor readings
        """
        # Get the fused state
        fused_state = self.sensor_fusion.get_state()
        current_position = fused_state[0:3]
        
        # Detect obstacles
        obstacles = self.detector.detect_obstacles(sensor_data)
        
        # Calculate emergency landing position
        emergency_target = self.landing.emergency_landing(
            current_position,
            obstacles
        )
        
        # Send emergency landing command
        self.controller.send_command('position', emergency_target)
        time.sleep(0.5)
        self.controller.send_command('land', None)
    
    def run_mission(self, mission_config: Dict):
        """
        Run complete mission with waypoints
        
        Args:
            mission_config: Mission configuration with waypoints
        """
        self.current_mission = mission_config
        waypoints = mission_config.get('waypoints', [])
        
        # Arm and takeoff
        self.controller.send_command('arm', None)
        takeoff_altitude = mission_config.get('takeoff_altitude', 10.0)
        self.controller.send_command('takeoff', takeoff_altitude)
        
        # Generate an initial mission plan
        fused_state = self.sensor_fusion.get_state()
        current_position = fused_state[0:3]
        goal_position = np.array(waypoints[-1])

        # Create a 3D map from the 2D SLAM map
        slam_map_2d = self.slam.get_map()
        map_size = int(np.sqrt(len(slam_map_2d)))
        slam_map_3d = np.zeros((map_size, map_size, 10)) # Assume a height of 10 for the map
        for y in range(map_size):
            for x in range(map_size):
                if slam_map_2d[y * map_size + x] != 0:
                    # Create a curtain of obstacles
                    slam_map_3d[x, y, 0:int(current_position[2] / self.mission_planner.map_resolution)] = 1

        map_data = {'map': slam_map_3d}
        mission_plan = self.mission_planner.generate_plan(current_position, goal_position, map_data)

        # Navigate through the planned waypoints
        last_update_time = time.time()
        while mission_plan:
            target = mission_plan[0]
            
            # Navigate with obstacle avoidance
            while True:
                # Calculate dt
                current_time = time.time()
                dt = current_time - last_update_time
                last_update_time = current_time

                # Predict the next state
                self.sensor_fusion.predict(dt)

                # Get sensor data
                sensor_data = self._get_sensor_data()
                telemetry = self.controller.get_telemetry()

                # Update with IMU data
                imu_data = np.concatenate([telemetry['acceleration'], telemetry['attitude']])
                self.sensor_fusion.update(imu_data, 'imu')

                # Update with GPS data
                if 'position' in telemetry:
                    self.sensor_fusion.update(telemetry['position'], 'gps')

                # Update with SLAM data
                self.slam.update(sensor_data)
                slam_pose = self.slam.get_pose()
                self.sensor_fusion.update(np.array([slam_pose[0], slam_pose[1], slam_pose[2]]), 'slam')
                
                nav_command, is_safe = self.navigate_to_target(target, sensor_data)
                
                if not is_safe:
                    print("Warning: Unsafe conditions detected")
                
                fused_state = self.sensor_fusion.get_state()
                current_position = fused_state[0:3]

                # Adapt the plan if necessary
                map_data = {'map': self.slam.get_map()}
                new_plan = self.mission_planner.adapt_plan(mission_plan, current_position, map_data)
                if new_plan:
                    mission_plan = new_plan
                    break # Restart the inner loop with the new plan

                # Check if reached waypoint
                distance = np.linalg.norm(current_position - target)
                
                if distance < 1.0:
                    mission_plan.pop(0) # Move to the next waypoint
                    if not mission_plan:
                        break # Mission complete
                
                # Check for geofence breach
                if not self.geofence_manager.is_within_geofence(current_position):
                    print("Geofence breached! Initiating emergency landing.")
                    self.emergency_land(sensor_data)
                    return

                # Send navigation command
                self.controller.send_command('velocity', nav_command)
                time.sleep(1.0 / self.update_rate)
        
        # Land at final waypoint
        landing_center = waypoints[-1] if waypoints else [0, 0, 0]
        self.execute_landing(np.array(landing_center), 10.0, self._get_sensor_data())
        
        # Disarm
        self.controller.send_command('disarm', None)

    def _get_sensor_data(self) -> Dict:
        """
        Get sensor data from the drone's sensors.
        """
        # Simulate a depth camera by generating a point cloud from the SLAM map
        slam_map = self.slam.get_map()
        map_size = int(np.sqrt(len(slam_map)))

        # Create a point cloud from the SLAM map
        points = []
        for y in range(map_size):
            for x in range(map_size):
                if slam_map[y * map_size + x] != 0:
                    points.append([x * self.mission_planner.map_resolution, y * self.mission_planner.map_resolution, 0])

        point_cloud = np.array(points)

        # Project the point cloud onto an image plane (simplified)
        depth_camera_data = np.ones((100, 100)) * 10
        if point_cloud.any():
            fused_state = self.sensor_fusion.get_state()
            current_position = fused_state[0:3]

            for point in point_cloud:
                # Transform point to camera frame (simplified)
                p_cam = point - current_position

                # Project point onto image plane (simplified)
                if p_cam[2] > 0: # Check if point is in front of the camera
                    u = int(50 * p_cam[0] / p_cam[2] + 50)
                    v = int(50 * p_cam[1] / p_cam[2] + 50)

                    if 0 <= u < 100 and 0 <= v < 100:
                        depth_camera_data[v, u] = p_cam[2]

        return {
            'lidar': self.lidar_data,
            'depth_camera': depth_camera_data
        }
    
    def get_system_status(self) -> Dict:
        """
        Get current system status
        
        Returns:
            Dictionary with system status information
        """
        return {
            'is_running': self.is_running,
            'controller_connected': self.controller.is_connected(),
            'current_mission': self.current_mission,
            'telemetry': self.controller.get_telemetry()
        }
