"""
ILAS Main Integration Module
Integrates all components for complete obstacle avoidance and landing system
"""

import numpy as np
from typing import Dict, List, Optional, Tuple
import time

from .detection.obstacle_detector import ObstacleDetector, Obstacle
from .avoidance.avoidance_system import AvoidanceSystem
from .landing.intelligent_landing import IntelligentLanding, LandingSite
from .controllers.controller_manager import ControllerManager
from .failsafe.modes import FailsafeManager, FailsafeMode
from .utils.watchdog import Watchdog


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
        self.failsafe = FailsafeManager(self.controller, config.get('failsafe', {}))

        # Watchdogs
        timeouts = config.get('timeouts', {})
        self.controller_watchdog = Watchdog(timeouts.get('controller', 5.0))
        self.sensor_watchdog = Watchdog(timeouts.get('sensor', 5.0))
        
        self.is_running = False
        self.current_mission = None
        self.update_rate = config.get('update_rate', 10.0)  # Hz
        
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
        # Get current state
        telemetry = self.controller.get_telemetry()
        self.controller_watchdog.reset()
        current_position = telemetry['position']
        current_velocity = telemetry['velocity']
        
        # Detect obstacles
        obstacles = self.detector.detect_obstacles(sensor_data)
        self.sensor_watchdog.reset()
        
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
        # Get current position
        telemetry = self.controller.get_telemetry()
        self.controller_watchdog.reset()
        current_position = telemetry['position']
        
        # Detect obstacles
        obstacles = self.detector.detect_obstacles(sensor_data)
        self.sensor_watchdog.reset()
        
        # Analyze landing area
        landing_sites = self.landing.analyze_landing_area(
            landing_area_center,
            search_radius,
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
        telemetry = self.controller.get_telemetry()
        current_position = telemetry['position']
        
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
        
        # Navigate through waypoints
        for waypoint in waypoints:
            target = np.array(waypoint)
            
            # Navigate with obstacle avoidance
            while True:
                # Get sensor data (placeholder)
                sensor_data = {}
                
                nav_command, is_safe = self.navigate_to_target(target, sensor_data)
                
                if not is_safe:
                    print("Warning: Unsafe conditions detected")
                
                # Check if reached waypoint
                telemetry = self.controller.get_telemetry()
                distance = np.linalg.norm(telemetry['position'] - target)
                
                if distance < 1.0:
                    break
                
                # Send navigation command
                self.controller.send_command('velocity', nav_command)
                time.sleep(1.0 / self.update_rate)
        
        # Land at final waypoint
        landing_center = waypoints[-1] if waypoints else [0, 0, 0]
        self.execute_landing(np.array(landing_center), 10.0, {})
        
        # Disarm
        self.controller.send_command('disarm', None)

    def monitor_subsystems(self):
        """Monitors the health of critical subsystems."""
        if self.controller_watchdog.check():
            print("Controller timeout detected!")
            self.trigger_failsafe(FailsafeMode.HOLD_POSITION)

        if self.sensor_watchdog.check():
            print("Sensor timeout detected!")
            self.trigger_failsafe(FailsafeMode.HOLD_POSITION)

    def trigger_failsafe(self, mode: FailsafeMode, sensor_data: Dict = None):
        """
        Triggers a failsafe mode.

        Args:
            mode: The failsafe mode to trigger.
            sensor_data: The current sensor data.
        """
        self.failsafe.activate_mode(mode, sensor_data)
    
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
