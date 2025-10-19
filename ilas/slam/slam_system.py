# ilas/slam/slam_system.py

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser

class SLAMSystem:
    def __init__(self, config):
        self.config = config

        # For now, let's assume a laser sensor. This should be made configurable.
        self.laser = Laser(scan_size=360, scan_rate_hz=10, detection_angle_degrees=360, distance_no_detection_mm=12000)

        # Create a SLAM object
        self.slam = RMHC_SLAM(self.laser, map_size_pixels=500, map_size_meters=10)

    def update(self, sensor_data):
        """
        Update the SLAM map and pose with new sensor data.

        Args:
            sensor_data (dict): A dictionary containing sensor data.
                                For now, we expect 'lidar' data as a list of distances.
        """
        if 'lidar' in sensor_data:
            distances = sensor_data['lidar']
            self.slam.update(distances)

    def get_map(self):
        """
        Return the current map as a byte array.
        """
        mapbytes = bytearray(self.slam.get_map())
        return mapbytes

    def get_pose(self):
        """
        Return the current pose of the drone (x, y, theta).
        """
        x, y, theta = self.slam.get_pos()
        return x, y, theta
