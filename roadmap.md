# ILAS Development Roadmap (2-Year Strategy)

This document outlines the strategic development plan for the Intelligent Landing and Avoidance System (ILAS) over the next two years. The roadmap is structured to prioritize stability, enhance core functionalities, and progressively introduce advanced autonomous capabilities, moving the system towards a commercial-grade navigation stack.

---

## Year 1: Foundation, Reliability, and Enhanced Perception

The first year is focused on hardening the existing system, expanding test coverage, and improving the drone's understanding of its environment.

### Q1-Q2: Core System Hardening & Integration
*   **Theme:** Stability and Reliability.
*   **Safety & Failsafes:**
    *   Implement comprehensive watchdog timers for all critical processes.
    *   Develop an independent safety monitoring module to validate system state.
    *   Enhance fail-safe protocols: advanced Return-to-Land (RTL) on critical sensor failure, emergency landing procedures in GPS-denied environments.
*   **Testing:**
    *   Increase unit test coverage to >80% for all core modules (`avoidance`, `landing`, `stabilization`, `core`).
    *   Establish a Continuous Integration (CI) pipeline executing a full suite of SITL (Software-in-the-Loop) simulations on every commit.
*   **PX4 Integration:**
    *   Deepen `PX4Controller` integration by supporting a wider range of MAVLink messages and advanced offboard control modes.
    *   Improve connection stability and error handling logic.
*   **Code Quality:**
    *   Conduct targeted refactoring of core modules to improve modularity and reduce code coupling based on insights from increased testing.

### Q3-Q4: Advanced Sensing & Environmental Perception
*   **Theme:** Seeing and Understanding the World.
*   **Sensor Fusion:**
    *   Implement a robust Extended Kalman Filter (EKF) within the `sensor_fusion` module to fuse IMU, GPS, and visual odometry data for highly accurate state estimation.
*   **SLAM Enhancement:**
    *   Evaluate and integrate a more capable VSLAM (Visual Simultaneous Localization and Mapping) algorithm to replace or augment BreezySLAM, ensuring robust navigation in GPS-denied conditions.
*   **3D Terrain Mapping:**
    *   Begin development of the `TerrainMapping` module to build and maintain a real-time 3D map of the operational environment. This will enable more sophisticated pathfinding than the reactive VFH algorithm.
*   **Machine Learning Model Improvement:**
    *   Curate and label a custom dataset for training the semantic segmentation (`DeepLabV3`) and object detection (`YOLOv5`) models.
    *   Retrain models to improve detection accuracy for critical classes (e.g., safe landing surfaces, specific Search-and-Rescue targets).

---

## Year 2: Advanced Autonomy and Commercial Readiness

The second year focuses on leveraging the stable foundation to build higher-level autonomous capabilities and prepare the system for specific real-world applications.

### Q1-Q2: High-Level Autonomy & Intelligent Decision Making
*   **Theme:** Thinking and Planning.
*   **Advanced Mission Planning:**
    *   Evolve the `MissionPlanner` into a stateful, high-level decision-making engine.
    *   Implement a Behavior Tree framework to allow for the definition of complex, multi-stage missions (e.g., "Survey Area A -> If Target Found, Loiter & Report -> Proceed to Landing Zone B").
*   **Dynamic Re-planning:**
    *   Empower the `ILASCore` to dynamically adapt the mission plan in response to real-time events, such as pop-up obstacles, geofence triggers, or changes in weather conditions (via wind estimation).
*   **Multi-Drone Coordination (Stretch Goal):**
    *   Begin R&D on a communication protocol for basic swarm intelligence, enabling multiple drones to share map data, target locations, and deconflict flight paths.

### Q3-Q4: Performance Optimization & Application Specialization
*   **Theme:** Real-World Deployment and Commercialization.
*   **Performance Optimization:**
    *   Profile the complete software stack on target embedded hardware (e.g., NVIDIA Jetson, Raspberry Pi).
    *   Optimize performance-critical code sections using tools like CUDA (for GPU-bound tasks), TensorRT (for model inference), and/or Cython.
*   **Hardware-in-the-Loop (HIL) Validation:**
    *   Establish a comprehensive HIL testing rig to validate the full software and hardware stack under realistic conditions.
*   **Application-Specific Modules:**
    *   Develop specialized modules for key commercial use-cases, such as:
        *   **Infrastructure Inspection:** Autonomous flight paths for following power lines or orbiting structures.
        *   **Precision Agriculture:** Integration with multispectral cameras and flight patterns for crop health analysis.
*   **Documentation & API:**
    *   Finalize a stable, well-documented API for third-party integration and system extension.
    *   Create comprehensive user and developer documentation.
