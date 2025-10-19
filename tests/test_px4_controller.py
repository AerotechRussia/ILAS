import unittest
from unittest.mock import MagicMock
import numpy as np
import sys

# Mock dronekit BEFORE importing the controller
mock_dronekit = MagicMock()
# The controller's code does `from dronekit import APIException`. So the mock needs this attribute.
# We create a mock exception that inherits from Exception.
mock_dronekit.APIException = type('APIException', (Exception,), {})
sys.modules['dronekit'] = mock_dronekit

# Now import the controller
from ilas.controllers.controller_manager import PX4Controller

class TestPX4Controller(unittest.TestCase):

    def setUp(self):
        self.config = {
            'connection_string': 'tcp:127.0.0.1:5760',
            'baud_rate': 57600
        }
        # Reset mocks for each test to ensure they are isolated
        mock_dronekit.connect.reset_mock()
        mock_dronekit.connect.side_effect = None

    def test_connect_success(self):
        """Test successful connection to PX4"""
        mock_vehicle = MagicMock()
        mock_dronekit.connect.return_value = mock_vehicle

        controller = PX4Controller(self.config)
        result = controller.connect()

        self.assertTrue(result)
        self.assertTrue(controller.connected)
        self.assertIsNotNone(controller.vehicle)
        mock_dronekit.connect.assert_called_once()

    def test_connect_fail(self):
        """Test failed connection to PX4"""
        mock_dronekit.connect.side_effect = mock_dronekit.APIException("Connection timed out")

        controller = PX4Controller(self.config)
        result = controller.connect()

        self.assertFalse(result)
        self.assertFalse(controller.connected)
        self.assertIsNone(controller.vehicle)

    def test_get_position(self):
        """Test getting position from PX4"""
        controller = PX4Controller(self.config)
        controller.connected = True
        controller.vehicle = MagicMock()

        # Mocking the location attribute
        mock_location = MagicMock()
        mock_location.global_relative_frame.lat = 1.0
        mock_location.global_relative_frame.lon = 2.0
        mock_location.global_relative_frame.alt = 3.0
        controller.vehicle.location = mock_location

        position = controller.get_position()
        self.assertTrue(np.array_equal(position, np.array([1.0, 2.0, 3.0])))

    def test_get_velocity(self):
        """Test getting velocity from PX4"""
        controller = PX4Controller(self.config)
        controller.connected = True
        controller.vehicle = MagicMock()
        controller.vehicle.velocity = [4.0, 5.0, 6.0]

        velocity = controller.get_velocity()
        self.assertTrue(np.array_equal(velocity, np.array([4.0, 5.0, 6.0])))

    def test_get_attitude(self):
        """Test getting attitude from PX4"""
        controller = PX4Controller(self.config)
        controller.connected = True
        controller.vehicle = MagicMock()

        # Mocking the attitude attribute
        mock_attitude = MagicMock()
        mock_attitude.roll = 0.1
        mock_attitude.pitch = 0.2
        mock_attitude.yaw = 0.3
        controller.vehicle.attitude = mock_attitude

        attitude = controller.get_attitude()
        self.assertTrue(np.allclose(attitude, np.array([0.1, 0.2, 0.3])))

if __name__ == '__main__':
    unittest.main()
