# ILAS Feature Implementation Plan

This document lists the concrete features planned for the ILAS project, derived from the detailed development roadmap. It can be used as a checklist to track implementation progress.

---

## Year 1: Foundation, Reliability, and Enhanced Perception

### Q1-Q2: Core System Hardening & Integration

#### Safety & Failsafes
- [ ] Implement watchdog timers for critical processes (state estimation, detection).
- [ ] Develop independent `HealthMonitor` module for state and sensor validation.
- [ ] Enhance RTL to trigger on sensor degradation.
- [ ] Implement emergency landing procedures for GPS-denied environments.
- [ ] Implement geofence breach protocols (hover, land, RTL).

#### Testing & CI/CD
- [ ] Achieve >80% unit test coverage for core modules.
- [ ] Create integration tests for key module interactions (e.g., Detection -> Avoidance).
- [ ] Set up CI pipeline (GitHub Actions or Jenkins).
- [ ] Automate SITL simulation suite in CI pipeline.
- [ ] Integrate static code analysis (linting, complexity) in CI.

#### Flight Controller Integration
- [ ] Expand `PX4Controller` to support more MAVLink messages (precision landing, terrain).
- [ ] Add robust handling for more PX4 flight modes and vehicle states.
- [ ] Implement automatic reconnection logic to the flight controller.
- [ ] Refine error handling for the flight controller connection.

#### Code Quality & Refactoring
- [ ] Refactor `ILASCore` to delegate logic to specialized sub-modules.
- [ ] Solidify and document internal APIs between modules.
- [ ] Refactor `config.py` to support dynamic reloading and validation.

### Q3-Q4: Advanced Sensing & Environmental Perception

#### Sensor Fusion
- [ ] Implement an Extended Kalman Filter (EKF) in `sensor_fusion` for state estimation.
- [ ] Fuse IMU, GPS, barometer, and visual odometry in the EKF.
- [ ] Design the fusion system to be extensible for new sensor types.

#### SLAM Enhancement
- [ ] Benchmark and select a new VSLAM algorithm (e.g., ORB-SLAM3).
- [ ] Integrate the selected VSLAM algorithm to replace or augment BreezySLAM.
- [ ] Ensure the new SLAM system provides a persistent map (point cloud, octomap).

#### 3D Terrain & Obstacle Mapping
- [ ] Develop `TerrainMapping` module for real-time 3D map generation (OctoMap).
- [ ] Integrate a global pathfinder (A*, RRT*) that uses the 3D map.
- [ ] Consume stereo depth or LiDAR data for mapping.

#### Machine Learning Model Improvement
- [ ] Build a data pipeline for curating, labeling, and augmenting custom datasets.
- [ ] Retrain detection and segmentation models on the custom dataset.
- [ ] Improve model accuracy for critical classes (wires, people, safe landing zones).
- [ ] Investigate model quantization and pruning for embedded performance.

---

## Year 2: Advanced Autonomy and Commercial Readiness

### Q1-Q2: High-Level Autonomy & Intelligent Decision Making

#### Advanced Mission Planning
- [ ] Implement a Behavior Tree framework (e.g., `py_trees`) in `MissionPlanner`.
- [ ] Redesign `MissionPlanner` to be a stateful, high-level decision engine.
- [ ] Enable the system to accept new tasks or mission changes mid-flight.

#### Dynamic Re-planning & Situational Awareness
- [ ] Integrate global pathfinding (A*) with local reactive avoidance (VFH/DWA).
- [ ] Enable dynamic mission adaptation based on real-time events (obstacles, wind, low battery).
- [ ] Use the `WindEstimator` output for adaptive planning.

#### Multi-Drone Coordination (Stretch Goal)
- [ ] Research a decentralized communication protocol for drone-to-drone communication.
- [ ] Design a system for sharing world state (maps, targets, intentions) between drones.
- [ ] Implement basic collaborative behaviors (e.g., flight path deconfliction).

### Q3-Q4: Performance Optimization & Application Specialization

#### Performance Optimization
- [ ] Profile the full software stack on target embedded hardware (Jetson, RPi, etc.).
- [ ] Optimize GPU-bound tasks with CUDA.
- [ ] Optimize ML model inference with TensorRT.
- [ ] Optimize CPU-bound Python code with Cython where necessary.
- [ ] Implement power management strategies to reduce computational load.

#### Hardware-in-the-Loop (HIL) Validation
- [ ] Build a HIL test rig with a high-fidelity simulator (AirSim, Gazebo).
- [ ] Develop a suite of HIL tests for realistic and edge-case scenarios.
- [ ] Validate the full software and hardware stack using the HIL rig.

#### Application-Specific Modules
- [ ] Develop a specialized module for infrastructure inspection (power lines, towers).
- [ ] Develop a specialized module for precision agriculture (crop analysis).
- [ ] Develop a specialized module for Search and Rescue (SAR) with autonomous search patterns.

#### Documentation & API
- [ ] Define and document a stable public API for third-party integration.
- [ ] Write a comprehensive developer guide covering architecture and setup.
- [ ] Write a comprehensive user guide covering configuration and operation.
