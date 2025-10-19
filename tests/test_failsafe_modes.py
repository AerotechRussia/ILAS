import unittest
import numpy as np
from ilas.failsafe.modes import FailsafeManager, FailsafeMode
from ilas.controllers.controller_manager import ControllerManager
from ilas.utils.config import get_default_config

class TestFailsafeModes(unittest.TestCase):
    """Unit tests for the FailsafeManager class."""

    def setUp(self):
        """Set up a default failsafe manager for testing."""
        config = get_default_config()
        self.controller = ControllerManager(config.get('controller', {}))
        self.controller.connect()
        self.failsafe = FailsafeManager(self.controller, config.get('failsafe', {}))

    def tearDown(self):
        """Stop the controller."""
        self.controller.disconnect()

    def test_hold_position(self):
        """Test the hold position failsafe mode."""
        self.failsafe.activate_mode(FailsafeMode.HOLD_POSITION)
        self.assertTrue(np.all(self.controller.controller.target_velocity == np.zeros(3)))

    def test_emergency_landing(self):
        """Test the emergency landing failsafe mode."""
        self.failsafe.activate_mode(FailsafeMode.EMERGENCY_LANDING)
        self.assertTrue(np.all(self.controller.controller.target_position == np.array([0.0, 0.0, 0.0])))

    def test_return_to_base(self):
        """Test the return to base failsafe mode."""
        self.failsafe.activate_mode(FailsafeMode.RETURN_TO_BASE)
        self.assertTrue(np.all(self.controller.controller.target_position == np.array([0, 0, 30.0])))

    def test_cancel_landing(self):
        """Test the cancel landing failsafe mode."""
        self.controller.set_telemetry({'position': np.array([10.0, 10.0, 10.0]), 'velocity': np.array([0.0, 0.0, 0.0])})
        self.failsafe.activate_mode(FailsafeMode.CANCEL_LANDING)
        self.assertTrue(np.all(self.controller.controller.target_position == np.array([10.0, 10.0, 15.0])))

if __name__ == '__main__':
    unittest.main()
