"""
Wind Estimator for ILAS
Estimates wind speed and direction based on drone telemetry
"""

import numpy as np
from typing import Dict, Tuple

class WindEstimator:
    """
    Estimates wind speed and direction
    """

    def __init__(self, config: Dict):
        """
        Initialize WindEstimator

        Args:
            config: Wind estimation configuration
        """
        self.config = config
        self.wind_estimate = np.zeros(3)

    def update(self, telemetry: Dict) -> np.ndarray:
        """
        Update wind estimate

        Args:
            telemetry: Current drone telemetry

        Returns:
            Updated wind estimate [x, y, z]
        """
        if 'airspeed' in telemetry and 'ground_velocity' in telemetry and 'attitude' in telemetry:
            # Wind triangle calculation
            # Vwind = Vair - Vground

            heading = telemetry['attitude'][2] # yaw

            # Rotate ground velocity to body frame
            v_ground_body = np.array([
                telemetry['ground_velocity'][0] * np.cos(-heading) - telemetry['ground_velocity'][1] * np.sin(-heading),
                telemetry['ground_velocity'][0] * np.sin(-heading) + telemetry['ground_velocity'][1] * np.cos(-heading),
                telemetry['ground_velocity'][2]
            ])

            # Airspeed is typically measured along the x-axis of the body frame
            v_air_body = np.array([telemetry['airspeed'], 0, 0])

            # Wind vector in body frame
            v_wind_body = v_air_body - v_ground_body

            # Rotate wind vector to world frame
            v_wind_world = np.array([
                v_wind_body[0] * np.cos(heading) - v_wind_body[1] * np.sin(heading),
                v_wind_body[0] * np.sin(heading) + v_wind_body[1] * np.cos(heading),
                v_wind_body[2]
            ])

            # Apply low-pass filter to wind estimate
            alpha = self.config.get('low_pass_alpha', 0.1)
            self.wind_estimate = alpha * v_wind_world + (1 - alpha) * self.wind_estimate

        return self.wind_estimate
