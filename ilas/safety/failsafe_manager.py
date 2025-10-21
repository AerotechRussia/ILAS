"""
Failsafe Manager for ILAS
Monitors critical system parameters and triggers failsafe procedures
"""

from typing import Dict, List, Any
from ..geofence.geofence_manager import GeofenceManager

class FailsafeManager:
    """
    Manages failsafe checks and triggers
    """

    def __init__(self, config: Dict):
        """
        Initialize FailsafeManager

        Args:
            config: Failsafe configuration
        """
        self.config = config
        self.failsafe_checks = [
            self._check_battery,
            self._check_gps,
            self._check_controller_link,
            self._check_geofence,
            self._check_sensor_health,
        ]
        geofence_config = {
            'enabled': config.get('check_geofence', False),
            'polygon_points': config.get('geofence', []),
        }
        self.geofence_manager = GeofenceManager(geofence_config)

    def check_failsafes(self, telemetry: Dict) -> List[str]:
        """
        Run all failsafe checks

        Args:
            telemetry: Current drone telemetry

        Returns:
            List of triggered failsafe events
        """
        triggered_events = []

        for check_func in self.failsafe_checks:
            event = check_func(telemetry)
            if event:
                triggered_events.append(event)

        return triggered_events

    def _check_battery(self, telemetry: Dict) -> str:
        """
        Check for low battery

        Args:
            telemetry: Current drone telemetry

        Returns:
            'low_battery' if failsafe triggered, else None
        """
        min_battery = self.config.get('min_battery_level', 20.0)
        if 'battery' in telemetry and telemetry['battery'] < min_battery:
            return 'low_battery'
        return None

    def _check_gps(self, telemetry: Dict) -> str:
        """
        Check for GPS signal loss

        Args:
            telemetry: Current drone telemetry

        Returns:
            'gps_loss' if failsafe triggered, else None
        """
        min_satellites = self.config.get('min_gps_satellites', 6)
        if 'gps_satellites' in telemetry and telemetry['gps_satellites'] < min_satellites:
            return 'gps_loss'
        return None

    def _check_controller_link(self, telemetry: Dict) -> str:
        """
        Check for controller link loss

        Args:
            telemetry: Current drone telemetry

        Returns:
            'controller_link_loss' if failsafe triggered, else None
        """
        # This is a placeholder - actual implementation depends on controller
        if 'last_heartbeat' in telemetry and telemetry['last_heartbeat'] > 5.0:
            return 'controller_link_loss'
        return None

    def _check_geofence(self, telemetry: Dict) -> str:
        """
        Check for geofence breach

        Args:
            telemetry: Current drone telemetry

        Returns:
            'geofence_breached' if failsafe triggered, else None
        """
        if self.config.get('check_geofence', False):
            if 'position' in telemetry and not self.geofence_manager.is_within_geofence(telemetry['position']):
                return 'geofence_breached'
        return None

    def _check_sensor_health(self, telemetry: Dict) -> str:
        """
        Check for sensor data loss

        Args:
            telemetry: Current drone telemetry

        Returns:
            'sensor_failure' if failsafe triggered, else None
        """
        max_sensor_delay = self.config.get('max_sensor_delay', 2.0)
        if 'sensor_status' in telemetry:
            for sensor, status in telemetry['sensor_status'].items():
                if 'last_update' in status and time.time() - status['last_update'] > max_sensor_delay:
                    return f'sensor_failure_{sensor}'
        return None
