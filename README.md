# ILAS - Intelligent Landing and Avoidance System

**ILAS** (Intelligent Landing and Avoidance System) is a comprehensive obstacle detection and avoidance system for VTOL (Vertical Take-Off and Landing) drones. The system provides intelligent landing capabilities on designated surfaces and supports multiple flight controller platforms.

## Features

### 🚁 Obstacle Detection
- Multi-sensor support (LiDAR, Camera, Ultrasonic, Radar, Depth Camera)
- Real-time obstacle detection and tracking
- Configurable detection range and confidence thresholds
- Sensor fusion for improved accuracy

### 🛡️ Obstacle Avoidance
- Multiple avoidance strategies:
  - Artificial Potential Field
  - Vector Field Histogram (VFH)
  - Dynamic Window Approach (DWA)
  - Rapidly-exploring Random Tree (RRT)
- Configurable safety margins
- Real-time path planning
- Dynamic obstacle handling

### 🎯 Intelligent Landing
- Automatic landing site analysis
- Terrain evaluation (slope, roughness)
- Multiple landing modes:
  - Precision landing on marked pads
  - Terrain analysis landing
  - Emergency landing
  - Vertical descent
  - Slope landing
- Trajectory planning for safe approach

### 🎮 Controller Support
- **PX4** flight controller
- **ArduPilot** flight controller
- **DJI** flight controller
- **Simulation mode** for testing
- Extensible architecture for custom controllers

## Installation

```bash
# Clone repository
git clone https://github.com/AerotechRussia/ILAS.git
cd ILAS

# Install dependencies
pip install -r requirements.txt
```

## Quick Start

### Basic Usage

```python
from ilas.core import ILASCore
from ilas.utils.config import get_default_config

# Initialize ILAS with default configuration
config = get_default_config()
ilas = ILASCore(config)

# Start system
ilas.start()

# Navigate to target with obstacle avoidance
import numpy as np
target = np.array([10.0, 0.0, 5.0])
sensor_data = {}  # Add your sensor data
nav_command, is_safe = ilas.navigate_to_target(target, sensor_data)

# Execute intelligent landing
landing_center = np.array([10.0, 0.0, 0.0])
success = ilas.execute_landing(landing_center, search_radius=10.0, sensor_data={})

# Stop system
ilas.stop()
```

### Command Line Interface

```bash
# Run with default configuration
python main.py

# Run with custom configuration
python main.py --config config/config_px4.yaml

# Run a mission
python main.py --mode mission --mission examples/mission_example.yaml

# Run test scenario
python main.py --mode test
```

### Interactive Mode

```bash
python main.py --mode interactive
```

Available commands:
- `status` - Show system status
- `navigate` - Navigate to target position
- `land` - Execute intelligent landing
- `emergency` - Emergency landing
- `help` - Show help
- `quit` - Exit

## Configuration

ILAS uses YAML configuration files. See `config/` directory for examples:

- `config_px4.yaml` - Configuration for PX4 controller
- `config_ardupilot.yaml` - Configuration for ArduPilot
- `config_simulation.yaml` - Configuration for simulation

### Configuration Structure

```yaml
detection:
  detection_range: 20.0        # Detection range in meters
  min_confidence: 0.6           # Minimum confidence threshold
  sensors:
    - type: lidar
      enabled: true
    - type: camera
      enabled: true

avoidance:
  strategy: potential_field     # Avoidance strategy
  safety_margin: 2.0            # Safety margin in meters
  max_avoidance_distance: 10.0  # Maximum avoidance distance

landing:
  landing_mode: terrain         # Landing mode
  min_landing_size: [2.0, 2.0]  # Minimum landing area size
  max_slope: 15.0               # Maximum slope in degrees
  hover_height: 2.0             # Hover height before landing

controller:
  controller_type: px4          # Controller type
  connection_string: /dev/ttyUSB0
  baud_rate: 57600
```

## Examples

See the `examples/` directory:

- `basic_usage.py` - Basic usage examples
- `mission_example.yaml` - Example mission configuration

Run examples:
```bash
python examples/basic_usage.py
```

## Architecture

```
ILAS/
├── ilas/
│   ├── detection/           # Obstacle detection modules
│   │   └── obstacle_detector.py
│   ├── avoidance/           # Avoidance algorithms
│   │   └── avoidance_system.py
│   ├── landing/             # Landing system
│   │   └── intelligent_landing.py
│   ├── controllers/         # Flight controller interfaces
│   │   └── controller_manager.py
│   ├── utils/               # Utilities
│   │   ├── config.py
│   │   └── visualization.py
│   └── core.py              # Main integration module
├── config/                  # Configuration files
├── examples/                # Usage examples
├── tests/                   # Test suite
├── main.py                  # Main entry point
├── requirements.txt         # Dependencies
└── LICENSE                  # Proprietary license
```

## System Requirements

- Python 3.7+
- NumPy
- SciPy
- OpenCV (for camera support)
- PyMAVLink (for PX4/ArduPilot support)
- DroneKit (optional, for enhanced MAVLink support)

## Supported Hardware

### Flight Controllers
- PX4 (via MAVLink)
- ArduPilot (via MAVLink)
- DJI (via SDK)
- Custom controllers (extensible)

### Sensors
- LiDAR (point cloud)
- Cameras (RGB, depth)
- Ultrasonic sensors
- Radar
- Multi-sensor fusion

### Platforms
- Multirotor drones
- VTOL aircraft
- Fixed-wing with VTOL capability

## License

Copyright (c) 2025 AerotechRussia. All rights reserved.

This software is proprietary and confidential. It is provided for **VIEWING PURPOSES ONLY**. 

See [LICENSE](LICENSE) file for full license terms.

**RESTRICTIONS:**
- No permission to use, copy, modify, or distribute
- Reverse engineering is strictly prohibited
- Viewing only - no execution or deployment allowed

For licensing inquiries, contact: AerotechRussia

## Safety Notice

⚠️ **IMPORTANT SAFETY NOTICE** ⚠️

This is an autonomous flight system. Always:
- Test in simulation first
- Follow local aviation regulations
- Maintain manual override capability
- Monitor the system during operation
- Ensure adequate safety margins
- Have emergency procedures in place
- Never fly over people or restricted areas

The developers assume no liability for misuse or accidents.

## Support

For questions and support, please contact AerotechRussia.

**Note:** This is proprietary software. Support is provided only to licensed users.

## Version

Current Version: 1.0.0

## Development Status

- ✅ Core obstacle detection system
- ✅ Multiple avoidance algorithms
- ✅ Intelligent landing system
- ✅ Multi-controller support
- ✅ Configuration system
- ✅ Command-line interface
- ✅ Documentation

---

**Copyright (c) 2025 AerotechRussia. All Rights Reserved.**