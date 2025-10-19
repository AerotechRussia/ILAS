<svg width="113" height="45" viewBox="0 0 113 45" fill="none" xmlns="http://www.w3.org/2000/svg">
<rect width="113" height="45" fill="#1E1E1E"/>
<path d="M0.25375 44V38.576H4.18975V15.92H0.25375V10.496H14.9898V15.92H11.0538V38.576H14.9898V44H0.25375ZM20.4359 44V10.496H27.2999V38H38.6759V44H20.4359ZM62.4985 44L60.4345 35.792H50.1625L48.0985 44H41.5225L51.0265 10.496H59.7625L69.2665 44H62.4985ZM56.6905 22.544L55.6345 16.736H54.8665L53.8105 22.544L51.8425 30.272H58.7545L56.6905 22.544ZM83.2904 44.576C78.2664 44.576 74.3304 42.816 71.4824 39.296L75.8984 34.88C77.8184 37.344 80.3304 38.576 83.4344 38.576C86.6344 38.576 88.2344 37.168 88.2344 34.352C88.2344 33.2 87.9464 32.304 87.3704 31.664C86.7944 31.024 85.8984 30.576 84.6824 30.32L81.6104 29.744C78.5064 29.168 76.2184 28.096 74.7464 26.528C73.3064 24.928 72.5864 22.72 72.5864 19.904C72.5864 16.64 73.5464 14.16 75.4664 12.464C77.3864 10.768 80.2024 9.92 83.9144 9.92C88.6504 9.92 92.2504 11.456 94.7144 14.528L90.2984 18.944C89.4664 17.92 88.5224 17.168 87.4664 16.688C86.4424 16.176 85.2264 15.92 83.8184 15.92C82.2824 15.92 81.1624 16.208 80.4584 16.784C79.7864 17.36 79.4504 18.288 79.4504 19.568C79.4504 20.688 79.7064 21.536 80.2184 22.112C80.7624 22.688 81.6264 23.104 82.8104 23.36L85.8824 23.984C87.5144 24.336 88.9064 24.768 90.0584 25.28C91.2424 25.76 92.2024 26.384 92.9384 27.152C93.7064 27.92 94.2504 28.832 94.5704 29.888C94.9224 30.944 95.0984 32.176 95.0984 33.584C95.0984 37.168 94.0904 39.904 92.0744 41.792C90.0584 43.648 87.1304 44.576 83.2904 44.576Z" fill="url(#paint0_linear_0_1)"/>
<path d="M101.628 2.551V10H99.7694V2.551H97.5984V0.926H103.799V2.551H101.628ZM111.084 5.268L111.188 3.747H110.98L110.421 5.19L108.913 8.479L107.418 5.19L106.859 3.747H106.651L106.755 5.268V10H105.013V0.926H106.963L108.198 3.63L108.874 5.567H109.004L109.68 3.63L110.889 0.926H112.826V10H111.084V5.268Z" fill="white"/>
<defs>
<linearGradient id="paint0_linear_0_1" x1="25.5" y1="3" x2="112.5" y2="38" gradientUnits="userSpaceOnUse">
<stop stop-color="#CA3AFF"/>
<stop offset="1" stop-color="#00529F"/>
</linearGradient>
</defs>
</svg>





# ILAS - Intelligent Landing and Avoidance System![ILAS](https://github.com/user-attachments/assets/d3788a57-8304-41c6-907a-89303ffc24cb)


**ILAS** (Intelligent Landing and Avoidance System) is a comprehensive obstacle detection and avoidance system for VTOL (Vertical Take-Off and Landing) drones. The system provides intelligent landing capabilities on designated surfaces and supports multiple flight controller platforms.

## Features

###  Obstacle Detection
- Multi-sensor support (LiDAR, Camera, Ultrasonic, Radar, Depth Camera)
- Real-time obstacle detection and tracking
- Configurable detection range and confidence thresholds
- Sensor fusion for improved accuracy

### Obstacle Avoidance
- Multiple avoidance strategies:
  - Artificial Potential Field
  - Vector Field Histogram (VFH)
  - Dynamic Window Approach (DWA)
  - Rapidly-exploring Random Tree (RRT)
- Configurable safety margins
- Real-time path planning
- Dynamic obstacle handling

###  Intelligent Landing
- Automatic landing site analysis
- Terrain evaluation (slope, roughness)
- Multiple landing modes:
  - Precision landing on marked pads
  - Terrain analysis landing
  - Emergency landing
  - Vertical descent
  - Slope landing
- Trajectory planning for safe approach

###  Controller Support
- **PX4** flight controller
- **ArduPilot** flight controller
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

 **IMPORTANT SAFETY NOTICE** 

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
