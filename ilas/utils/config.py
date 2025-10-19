"""
Configuration loader utility
"""

import yaml
import json
from typing import Dict, Optional


def load_config(config_path: str) -> Dict:
    """
    Load configuration from YAML or JSON file
    
    Args:
        config_path: Path to configuration file
        
    Returns:
        Configuration dictionary
    """
    if config_path.endswith('.yaml') or config_path.endswith('.yml'):
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    elif config_path.endswith('.json'):
        with open(config_path, 'r') as f:
            return json.load(f)
    else:
        raise ValueError(f"Unsupported config file format: {config_path}")


def save_config(config: Dict, config_path: str):
    """
    Save configuration to file
    
    Args:
        config: Configuration dictionary
        config_path: Path to save configuration
    """
    if config_path.endswith('.yaml') or config_path.endswith('.yml'):
        with open(config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
    elif config_path.endswith('.json'):
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=2)


def get_default_config() -> Dict:
    """
    Get default ILAS configuration
    
    Returns:
        Default configuration dictionary
    """
    return {
        'detection': {
            'detection_range': 20.0,
            'min_confidence': 0.6,
            'sensors': [
                {
                    'type': 'lidar',
                    'enabled': True
                },
                {
                    'type': 'camera',
                    'enabled': True
                }
            ]
        },
        'avoidance': {
            'strategy': 'potential_field',
            'safety_margin': 2.0,
            'max_avoidance_distance': 10.0,
            'avoidance_gain': 1.0,
            'max_velocity': 5.0,
            'max_acceleration': 2.0,
            'prediction_time': 1.0
        },
        'landing': {
            'landing_mode': 'terrain',
            'min_landing_size': [2.0, 2.0],
            'max_slope': 15.0,
            'max_roughness': 0.3,
            'descent_rate': 0.5,
            'hover_height': 2.0
        },
        'controller': {
            'controller_type': 'simulation',
            'connection_string': '/dev/ttyUSB0',
            'baud_rate': 57600
        },
        'update_rate': 10.0
    }
