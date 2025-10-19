from enum import Enum, auto

class FailsafeMode(Enum):
    """Enumeration of available failsafe modes."""
    NONE = auto()
    CANCEL_LANDING = auto()
    RETURN_TO_BASE = auto()
    HOLD_POSITION = auto()
    EMERGENCY_LANDING = auto()

class FailsafeManager:
    """Handles the activation and management of failsafe modes."""

    def __init__(self, controller, config):
        self.controller = controller
        self.config = config
        self.active_mode = FailsafeMode.NONE

    def activate_mode(self, mode, sensor_data=None):
        """Activates a specific failsafe mode."""
        self.active_mode = mode
        if mode == FailsafeMode.CANCEL_LANDING:
            self.cancel_landing()
        elif mode == FailsafeMode.RETURN_TO_BASE:
            self.return_to_base()
        elif mode == FailsafeMode.HOLD_POSITION:
            self.hold_position()
        elif mode == FailsafeMode.EMERGENCY_LANDING:
            self.emergency_landing(sensor_data)

    def cancel_landing(self):
        """Cancels the current landing procedure."""
        current_position = self.controller.get_telemetry()['position']
        safe_altitude = current_position[2] + self.config.get('hold_position_duration', 5.0)
        self.controller.send_command('position', [current_position[0], current_position[1], safe_altitude])
        self.hold_position()

    def return_to_base(self):
        """Initiates a return-to-base maneuver."""
        home_position = [0, 0, self.config.get('return_to_base_altitude', 30.0)]
        self.controller.send_command('position', home_position)

    def hold_position(self):
        """Commands the drone to hold its current position."""
        self.controller.send_command('velocity', [0, 0, 0])

    def emergency_landing(self, sensor_data):
        """Initiates an emergency landing."""
        self.controller.send_command('land', None)
