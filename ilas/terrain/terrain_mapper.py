# ilas/terrain/terrain_mapper.py

import numpy as np
from scipy.spatial.transform import Rotation

class TerrainMapper:
    def __init__(self, config):
        self.config = config
        self.map_resolution = config.get('map_resolution', 0.1)  # meters per pixel
        self.map_size = config.get('map_size', 20)  # meters
        self.map_shape = (int(self.map_size / self.map_resolution), int(self.map_size / self.map_resolution))
        self.terrain_map = np.zeros(self.map_shape, dtype=np.float32)

    def update(self, sensor_data, pose):
        """
        Update the terrain map with new sensor data and the drone's pose.
        """
        if 'depth_camera' in sensor_data:
            depth_image = sensor_data['depth_camera']
            x, y, z, roll, pitch, yaw = pose

            # Create a rotation matrix from Euler angles
            rotation = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

            # Get camera intrinsics (assuming a simple pinhole camera model)
            f_x = self.config.get('camera_fx', 320)
            f_y = self.config.get('camera_fy', 320)
            c_x = self.config.get('camera_cx', 320)
            c_y = self.config.get('camera_cy', 240)

            # Create a grid of pixel coordinates
            u, v = np.meshgrid(np.arange(depth_image.shape[1]), np.arange(depth_image.shape[0]))

            # Convert pixel coordinates to camera coordinates
            x_c = (u - c_x) * depth_image / f_x
            y_c = (v - c_y) * depth_image / f_y
            z_c = depth_image

            # Reshape into a 3xN array of points
            points_camera = np.vstack((x_c.flatten(), y_c.flatten(), z_c.flatten()))

            # Transform points from camera to world frame
            points_world = rotation @ points_camera + np.array([[x], [y], [z]])

            # Update the terrain map
            for i in range(points_world.shape[1]):
                px, py, pz = points_world[:, i]

                # Convert world coordinates to map coordinates
                mx = int((px + self.map_size / 2) / self.map_resolution)
                my = int((py + self.map_size / 2) / self.map_resolution)

                if 0 <= mx < self.map_shape[1] and 0 <= my < self.map_shape[0]:
                    # Update the map with the new height value (moving average)
                    alpha = 0.1
                    self.terrain_map[my, mx] = (1 - alpha) * self.terrain_map[my, mx] + alpha * pz

    def get_terrain_map(self):
        return self.terrain_map

    def get_landing_suitability(self, position, radius):
        # ... (same as before)
        x_map = int((position[0] + self.map_size / 2) / self.map_resolution)
        y_map = int((position[1] + self.map_size / 2) / self.map_resolution)
        radius_pixels = int(radius / self.map_resolution)

        x_start = max(0, x_map - radius_pixels)
        x_end = min(self.map_shape[1], x_map + radius_pixels)
        y_start = max(0, y_map - radius_pixels)
        y_end = min(self.map_shape[0], y_map + radius_pixels)

        region = self.terrain_map[y_start:y_end, x_start:x_end]

        if region.size == 0:
            return 0.0

        roughness = np.std(region)

        max_roughness = 0.5
        suitability = 1.0 - min(roughness / max_roughness, 1.0)

        return suitability
