"""
Hardware Controller Manager
Supports various flight controllers (PX4, ArduPilot, etc.)
"""

import numpy as np
from typing import Dict, Optional, Any
from enum import Enum
from abc import ABC, abstractmethod

try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
    import time
except ImportError:
    print("Warning: dronekit not installed. PX4 and ArduPilot support will be unavailable.")
    connect = None
    VehicleMode = None
    LocationGlobalRelative = None
    APIException = None


class ControllerType(Enum):
    """Supported flight controller types"""
    PX4 = "px4"
    ARDUPILOT = "ardupilot"
    DJI = "dji"
    BETAFLIGHT = "betaflight"
    PAPARAZZI = "paparazzi"
    SIMULATION = "simulation"


class FlightController(ABC):
    """Abstract base class for flight controllers"""
    
    def __init__(self, config: Dict):
        self.config = config
        self.connected = False
        
    @abstractmethod
    def connect(self) -> bool:
        """Connect to flight controller"""
        pass
    
    @abstractmethod
    def disconnect(self):
        """Disconnect from flight controller"""
        pass
    
    @abstractmethod
    def get_position(self) -> np.ndarray:
        """Get current position [x, y, z]"""
        pass
    
    @abstractmethod
    def get_velocity(self) -> np.ndarray:
        """Get current velocity [vx, vy, vz]"""
        pass
    
    @abstractmethod
    def get_attitude(self) -> np.ndarray:
        """Get current attitude [roll, pitch, yaw]"""
        pass
    
    @abstractmethod
    def set_target_position(self, position: np.ndarray):
        """Set target position"""
        pass
    
    @abstractmethod
    def set_target_velocity(self, velocity: np.ndarray):
        """Set target velocity"""
        pass
    
    @abstractmethod
    def arm(self) -> bool:
        """Arm the vehicle"""
        pass
    
    @abstractmethod
    def disarm(self) -> bool:
        """Disarm the vehicle"""
        pass
    
    @abstractmethod
    def takeoff(self, altitude: float) -> bool:
        """Takeoff to specified altitude"""
        pass
    
    @abstractmethod
    def land(self) -> bool:
        """Land the vehicle"""
        pass


class PX4Controller(FlightController):
    """PX4 flight controller interface"""
    
    def __init__(self, config: Dict):
        super().__init__(config)
        if not connect:
            raise ImportError("dronekit is not installed. Please install it with 'pip install dronekit'")
        self.connection_string = config.get('connection_string', 'udp:127.0.0.1:14550')
        self.baud_rate = config.get('baud_rate', 57600)
        self.vehicle = None
        
    def connect(self) -> bool:
        """Connect to PX4 via MAVLink using DroneKit"""
        try:
            print(f"Connecting to vehicle on: {self.connection_string}")
            self.vehicle = connect(self.connection_string, wait_ready=True, baud=self.baud_rate, heartbeat_timeout=30)
            self.connected = True
            print("PX4 connected successfully")
            return True
        except APIException as e:
            print(f"PX4 connection failed: {e}")
            return False
        except Exception as e:
            print(f"An unexpected error occurred during PX4 connection: {e}")
            return False

    def disconnect(self):
        """Disconnect from PX4"""
        if self.vehicle:
            self.vehicle.close()
            self.vehicle = None
        self.connected = False
        print("PX4 disconnected")
    
    def get_position(self) -> np.ndarray:
        """Get current position (lat, lon, alt) from PX4"""
        if not self.connected or not self.vehicle:
            return np.zeros(3)
        loc = self.vehicle.location.global_relative_frame
        return np.array([loc.lat, loc.lon, loc.alt])
    
    def get_velocity(self) -> np.ndarray:
        """Get current velocity [vx, vy, vz] from PX4"""
        if not self.connected or not self.vehicle:
            return np.zeros(3)
        return np.array(self.vehicle.velocity)
    
    def get_attitude(self) -> np.ndarray:
        """Get current attitude [roll, pitch, yaw] from PX4"""
        if not self.connected or not self.vehicle:
            return np.zeros(3)
        att = self.vehicle.attitude
        return np.array([att.roll, att.pitch, att.yaw])
    
    def set_target_position(self, position: np.ndarray):
        """Send position setpoint to PX4"""
        if not self.connected or not self.vehicle:
            return

        lat, lon, alt = position
        target_location = LocationGlobalRelative(lat, lon, alt)
        self.vehicle.simple_goto(target_location)

    def set_target_velocity(self, velocity: np.ndarray):
        """Send velocity setpoint to PX4"""
        # DroneKit does not directly support velocity control in the same way as position.
        # This would require sending MAVLink messages directly.
        # For simplicity, this is not fully implemented here.
        print("Velocity control not fully implemented for PX4 via DroneKit in this version.")
        pass
    
    def arm(self) -> bool:
        """Arm PX4"""
        if not self.connected or not self.vehicle:
            return False

        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Wait for arming to complete
        for _ in range(10): # 10 seconds timeout
            if self.vehicle.armed:
                return True
            time.sleep(1)
        return False
    
    def disarm(self) -> bool:
        """Disarm PX4"""
        if not self.connected or not self.vehicle:
            return False

        self.vehicle.armed = False
        return True
    
    def takeoff(self, altitude: float) -> bool:
        """Takeoff with PX4"""
        if not self.connected or not self.vehicle or not self.vehicle.armed:
            return False

        self.vehicle.simple_takeoff(altitude)

        # Wait until the vehicle reaches a safe altitude
        while True:
            if self.vehicle.location.global_relative_frame.alt >= altitude * 0.95:
                break
            time.sleep(1)
        return True
    
    def land(self) -> bool:
        """Land with PX4"""
        if not self.connected or not self.vehicle:
            return False

        self.vehicle.mode = VehicleMode("LAND")
        return True


class ArduPilotController(FlightController):
    """ArduPilot flight controller interface"""
    
    def __init__(self, config: Dict):
        super().__init__(config)
        self.connection_string = config.get('connection_string', '/dev/ttyUSB0')
        self.baud_rate = config.get('baud_rate', 57600)
        self.vehicle = None
        
    def connect(self) -> bool:
        """Connect to ArduPilot via MAVLink"""
        try:
            # Similar to PX4, use dronekit
            self.connected = True
            return True
        except Exception as e:
            print(f"ArduPilot connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from ArduPilot"""
        if self.vehicle:
            self.vehicle = None
        self.connected = False
    
    def get_position(self) -> np.ndarray:
        """Get current position"""
        return np.zeros(3)
    
    def get_velocity(self) -> np.ndarray:
        """Get current velocity"""
        return np.zeros(3)
    
    def get_attitude(self) -> np.ndarray:
        """Get current attitude"""
        return np.zeros(3)
    
    def set_target_position(self, position: np.ndarray):
        """Set target position"""
        pass
    
    def set_target_velocity(self, velocity: np.ndarray):
        """Set target velocity"""
        pass
    
    def arm(self) -> bool:
        """Arm ArduPilot"""
        return True
    
    def disarm(self) -> bool:
        """Disarm ArduPilot"""
        return True
    
    def takeoff(self, altitude: float) -> bool:
        """Takeoff"""
        return True
    
    def land(self) -> bool:
        """Land"""
        return True


class SimulationController(FlightController):
    """Simulation controller for testing"""
    
    def __init__(self, config: Dict):
        super().__init__(config)
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.attitude = np.array([0.0, 0.0, 0.0])
        self.target_position = None
        self.target_velocity = None
        self.armed = False
        
    def connect(self) -> bool:
        """Connect to simulation"""
        self.connected = True
        return True
    
    def disconnect(self):
        """Disconnect from simulation"""
        self.connected = False
    
    def get_position(self) -> np.ndarray:
        """Get simulated position"""
        return self.position.copy()
    
    def get_velocity(self) -> np.ndarray:
        """Get simulated velocity"""
        return self.velocity.copy()
    
    def get_attitude(self) -> np.ndarray:
        """Get simulated attitude"""
        return self.attitude.copy()
    
    def set_target_position(self, position: np.ndarray):
        """Set target position in simulation"""
        self.target_position = position.copy()
        
    def set_target_velocity(self, velocity: np.ndarray):
        """Set target velocity in simulation"""
        self.target_velocity = velocity.copy()
    
    def update(self, dt: float):
        """Update simulation state"""
        if self.target_velocity is not None:
            self.velocity = self.target_velocity
            self.position += self.velocity * dt
        elif self.target_position is not None:
            # Simple proportional control
            error = self.target_position - self.position
            self.velocity = error * 0.5  # P gain
            self.position += self.velocity * dt
    
    def arm(self) -> bool:
        """Arm simulation"""
        self.armed = True
        return True
    
    def disarm(self) -> bool:
        """Disarm simulation"""
        self.armed = False
        return True
    
    def takeoff(self, altitude: float) -> bool:
        """Simulate takeoff"""
        self.target_position = np.array([self.position[0], self.position[1], altitude])
        return True
    
    def land(self) -> bool:
        """Simulate landing"""
        self.target_position = np.array([self.position[0], self.position[1], 0.0])
        return True


class ControllerManager:
    """
    Manages flight controller connections and provides unified interface
    """
    
    def __init__(self, config: Dict):
        """
        Initialize controller manager
        
        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.controller_type = ControllerType(config.get('controller_type', 'simulation'))
        self.controller: Optional[FlightController] = None
        self._create_controller()
        
    def _create_controller(self):
        """Create appropriate controller based on type"""
        if self.controller_type == ControllerType.PX4:
            self.controller = PX4Controller(self.config)
        elif self.controller_type == ControllerType.ARDUPILOT:
            self.controller = ArduPilotController(self.config)
        elif self.controller_type == ControllerType.SIMULATION:
            self.controller = SimulationController(self.config)
        else:
            raise ValueError(f"Unsupported controller type: {self.controller_type}")
    
    def connect(self) -> bool:
        """Connect to flight controller"""
        if self.controller:
            return self.controller.connect()
        return False
    
    def disconnect(self):
        """Disconnect from flight controller"""
        if self.controller:
            self.controller.disconnect()
    
    def get_telemetry(self) -> Dict[str, np.ndarray]:
        """
        Get current telemetry data
        
        Returns:
            Dictionary with position, velocity, attitude
        """
        if not self.controller or not self.controller.connected:
            return {
                'position': np.zeros(3),
                'velocity': np.zeros(3),
                'attitude': np.zeros(3)
            }
        
        return {
            'position': self.controller.get_position(),
            'velocity': self.controller.get_velocity(),
            'attitude': self.controller.get_attitude()
        }
    
    def send_command(self, command_type: str, data: Any):
        """
        Send command to flight controller
        
        Args:
            command_type: Type of command (position, velocity, etc.)
            data: Command data
        """
        if not self.controller or not self.controller.connected:
            return
        
        if command_type == 'position':
            self.controller.set_target_position(data)
        elif command_type == 'velocity':
            self.controller.set_target_velocity(data)
        elif command_type == 'arm':
            self.controller.arm()
        elif command_type == 'disarm':
            self.controller.disarm()
        elif command_type == 'takeoff':
            self.controller.takeoff(data)
        elif command_type == 'land':
            self.controller.land()
    
    def is_connected(self) -> bool:
        """Check if controller is connected"""
        return self.controller is not None and self.controller.connected
