# ILAS Development Roadmap (2-Year Strategy)

This document outlines the strategic development plan for the Intelligent Landing and Avoidance System (ILAS) over the next two years. The roadmap is structured to prioritize stability, enhance core functionalities, and progressively introduce advanced autonomous capabilities, moving the system towards a commercial-grade navigation stack.

---

## Year 1: Foundation, Reliability, and Enhanced Perception

The first year is focused on hardening the existing system, expanding test coverage, and improving the drone's understanding of its environment.

### Q1-Q2: Core System Hardening & Integration
*   **Theme:** Stability and Reliability.
*   **Safety & Failsafes:**
    *   **Process Monitoring:** Implement comprehensive watchdog timers for all critical processes (e.g., state estimation, obstacle detection).
    *   **State Validation:** Develop an independent safety monitoring module (`HealthMonitor`) to continuously validate system state, sensor health, and process responsiveness.
    *   **Advanced Failsafes:**
        *   Enhance Return-to-Land (RTL) to trigger on critical sensor degradation, not just complete failure.
        *   Develop and test emergency landing procedures for GPS-denied environments (e.g., controlled descent, spiral landing).
        *   Implement geofence breach protocols (hover, land, or RTL).
*   **Testing & CI/CD:**
    *   **Unit & Integration Testing:**
        *   Increase unit test coverage to >80% for all core modules (`avoidance`, `landing`, `stabilization`, `core`).
        *   Write integration tests for interactions between key modules (e.g., `Detection` -> `Avoidance` -> `Controller`).
    *   **CI Pipeline:**
        *   Establish a Continuous Integration (CI) pipeline using GitHub Actions or Jenkins.
        *   Automate the execution of the full suite of SITL (Software-in-the-Loop) simulations on every commit to the `main` branch.
        *   Integrate static code analysis (linting, complexity checks) into the pipeline.
*   **Flight Controller Integration (PX4/ArduPilot):**
    *   **Deeper Integration:**
        *   Expand `PX4Controller` to support a wider range of MAVLink messages (e.g., precision landing targets, terrain data).
        *   Implement robust handling for a broader set of flight modes and vehicle states.
    *   **Reliability:** Improve connection stability, implement automatic reconnection logic, and refine error handling between ILAS and the flight controller.
*   **Code Quality & Refactoring:**
    *   **Modularization:** Conduct targeted refactoring of `ILASCore` to delegate more logic to specialized sub-modules, reducing its complexity.
    *   **API Definition:** Solidify internal APIs between modules to improve modularity and reduce code coupling.
    *   **Configuration:** Refactor `config.py` to support dynamic reloading and validation of configuration parameters.

### Q3-Q4: Advanced Sensing & Environmental Perception
*   **Theme:** Seeing and Understanding the World.
*   **Sensor Fusion:**
    *   **State Estimation:** Implement a robust Extended Kalman Filter (EKF) within the `sensor_fusion` module. This EKF will fuse IMU, GPS, barometer, and visual odometry data for highly accurate and smooth state estimation (position, velocity, attitude).
    *   **Multi-sensor Input:** Design the fusion system to be extensible for additional sensor types (e.g., wheel encoders for ground robots, magnetometer).
*   **SLAM Enhancement:**
    *   **Algorithm Evaluation:** Benchmark and integrate a more capable VSLAM (Visual Simultaneous Localization and Mapping) algorithm (e.g., ORB-SLAM3, Kimera) to replace or augment BreezySLAM. The goal is robust, real-time, loop-closing navigation in large-scale GPS-denied environments.
    *   **Map Representation:** The new SLAM system should produce a persistent or semi-persistent map (e.g., point cloud, octomap) that can be used by other modules.
*   **3D Terrain & Obstacle Mapping:**
    *   **Real-Time Mapping:** Develop the `TerrainMapping` module to consume sensor data (e.g., stereo depth, LiDAR) and build a real-time 3D map (OctoMap representation) of the operational environment.
    *   **Pathfinding Integration:** This map will be used by a new global pathfinder (e.g., A*, RRT*) to enable more sophisticated navigation than the current reactive avoidance.
*   **Machine Learning Model Improvement:**
    *   **Data Engine:** Build a data pipeline to curate, label, and augment a custom dataset for training the semantic segmentation (`DeepLabV3`) and object detection (`YOLOv5`) models.
    *   **Targeted Training:** Retrain models to improve detection accuracy and reduce false positives for critical classes (e.g., safe landing surfaces, wires, people, specific Search-and-Rescue targets).
    *   **Model Optimization:** Investigate model quantization and pruning for more efficient inference on embedded hardware.

---

## Year 2: Advanced Autonomy and Commercial Readiness

The second year focuses on leveraging the stable foundation to build higher-level autonomous capabilities and prepare the system for specific real-world applications.

### Q1-Q2: High-Level Autonomy & Intelligent Decision Making
*   **Theme:** Thinking and Planning.
*   **Advanced Mission Planning:**
    *   **Behavior Trees:** Evolve the `MissionPlanner` from a sequential script executor into a stateful, high-level decision-making engine. Implement a Behavior Tree framework (e.g., `py_trees`) to allow for the definition of complex, reactive, and multi-stage missions (e.g., "Survey Area A -> If Target Found, Loiter & Report -> Proceed to Landing Zone B").
    *   **Dynamic Tasking:** The system should be able to accept new tasks or modifications to the mission plan mid-flight.
*   **Dynamic Re-planning & Situational Awareness:**
    *   **Global & Local Planning:** Empower the `ILASCore` to use the 3D map for global pathfinding (A* on the map) and the existing VFH/DWA for local, reactive adjustments.
    *   **Adaptive Planning:** The system will dynamically adapt the mission plan in response to real-time events: pop-up obstacles, new geofence definitions, changes in wind conditions (from `WindEstimator`), or low battery.
*   **Multi-Drone Coordination (Stretch Goal):**
    *   **Communication Protocol:** Begin R&D on a lightweight, decentralized communication protocol (e.g., using ROS2, DDS, or a custom MAVLink-based protocol) for basic swarm intelligence.
    *   **Shared World State:** Enable multiple drones to share map data, target locations, and their own positions/intentions to deconflict flight paths and collaborate on tasks.

### Q3-Q4: Performance Optimization & Application Specialization
*   **Theme:** Real-World Deployment and Commercialization.
*   **Performance Optimization:**
    *   **Hardware Profiling:** Profile the complete software stack on target embedded hardware (e.g., NVIDIA Jetson, Raspberry Pi, Qualcomm RB5).
    *   **Targeted Optimization:** Use tools like CUDA (for GPU-bound tasks), TensorRT (for model inference), and/or Cython to optimize performance-critical code sections identified during profiling.
    *   **Power Management:** Implement strategies to optimize computational load based on mission phase to conserve battery.
*   **Hardware-in-the-Loop (HIL) Validation:**
    *   **Test Rig:** Establish a comprehensive HIL testing rig that interfaces the flight controller and companion computer with a high-fidelity simulator (e.g., AirSim, Gazebo).
    *   **Scenario Testing:** Validate the full software and hardware stack under realistic and edge-case scenarios that are difficult to test in the real world.
*   **Application-Specific Modules:**
    *   Develop and productize specialized modules for key commercial use-cases:
        *   **Infrastructure Inspection:** Autonomous flight paths for following power lines, orbiting cell towers, or performing facade inspections.
        *   **Precision Agriculture:** Integration with multispectral cameras and flight patterns for crop health analysis and targeted spraying.
        *   **Search and Rescue (SAR):** Autonomous search patterns, person detection, and payload delivery capabilities.
*   **Documentation & API:**
    *   **Public API:** Finalize a stable, well-documented public API for third-party integration and system extension.
    *   **Developer & User Guides:** Create comprehensive documentation covering architecture, setup, configuration, and usage for both developers and end-users.