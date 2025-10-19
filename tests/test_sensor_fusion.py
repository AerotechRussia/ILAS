# tests/test_sensor_fusion.py

import unittest
import numpy as np

from ilas.fusion.sensor_fusion import SensorFusionEKF

class TestSensorFusionEKF(unittest.TestCase):

    def test_ekf_init(self):
        config = {}
        ekf = SensorFusionEKF(config)
        self.assertIsNotNone(ekf)

    def test_ekf_predict(self):
        config = {}
        ekf = SensorFusionEKF(config)

        try:
            ekf.predict(0.1)
        except Exception as e:
            self.fail(f"ekf.predict() raised an exception: {e}")

    def test_ekf_update(self):
        config = {}
        ekf = SensorFusionEKF(config)

        # Mock GPS data
        gps_data = np.array([10, 20, 30])
        try:
            ekf.update(gps_data, 'gps')
        except Exception as e:
            self.fail(f"ekf.update() with gps data raised an exception: {e}")

        # Mock IMU data
        imu_data = np.array([0.1, 0.2, 9.8, 0, 0, 0])
        try:
            ekf.update(imu_data, 'imu')
        except Exception as e:
            self.fail(f"ekf.update() with imu data raised an exception: {e}")

        # Mock SLAM data
        slam_data = np.array([1, 2, 0.5])
        try:
            ekf.update(slam_data, 'slam')
        except Exception as e:
            self.fail(f"ekf.update() with slam data raised an exception: {e}")

if __name__ == '__main__':
    unittest.main()
