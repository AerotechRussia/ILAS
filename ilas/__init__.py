"""
ILAS - Intelligent Landing and Avoidance System
Copyright (c) 2025 AerotechRussia. All rights reserved.
Licensed under Proprietary License. See LICENSE file for details.
"""

__version__ = "1.0.0"
__author__ = "AerotechRussia"

from .detection.obstacle_detector import ObstacleDetector
from .avoidance.avoidance_system import AvoidanceSystem
from .landing.intelligent_landing import IntelligentLanding
from .controllers.controller_manager import ControllerManager

__all__ = [
    'ObstacleDetector',
    'AvoidanceSystem',
    'IntelligentLanding',
    'ControllerManager'
]
