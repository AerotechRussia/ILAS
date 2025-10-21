
import time
import threading
from enum import Enum
from collections import defaultdict

class ComponentStatus(Enum):
    """Enumeration for component health status."""
    OK = "OK"
    WARNING = "Warning"
    CRITICAL = "Critical"
    UNKNOWN = "Unknown"

class MonitoredComponent:
    """
    Represents a component being monitored by the HealthMonitor.
    
    Attributes:
        name (str): The name of the component.
        timeout (float): The time in seconds after which the component is considered unhealthy if no update is received.
        last_heartbeat (float): The timestamp of the last heartbeat.
        status (ComponentStatus): The current health status of the component.
        message (str): A message associated with the current status.
    """
    def __init__(self, name: str, timeout: float = 1.0):
        """
        Initializes a new MonitoredComponent.

        Args:
            name (str): The name of the component (e.g., "SensorFusion", "ObstacleDetector").
            timeout (float, optional): The heartbeat timeout in seconds. Defaults to 1.0.
        """
        self.name = name
        self.timeout = timeout
        self.last_heartbeat = 0.0
        self.status = ComponentStatus.UNKNOWN
        self.message = "No heartbeat received yet."

    def update_status(self):
        """
        Updates the component's status based on the last heartbeat time.
        """
        if self.last_heartbeat == 0.0:
            self.status = ComponentStatus.UNKNOWN
            self.message = "No heartbeat received yet."
            return

        elapsed_time = time.time() - self.last_heartbeat
        if elapsed_time > self.timeout:
            self.status = ComponentStatus.CRITICAL
            self.message = f"Heartbeat timeout. Last seen {elapsed_time:.2f}s ago."
        elif elapsed_time > self.timeout / 2:
            self.status = ComponentStatus.WARNING
            self.message = f"Heartbeat delayed. Last seen {elapsed_time:.2f}s ago."
        else:
            self.status = ComponentStatus.OK
            self.message = "Component is responsive."

    def record_heartbeat(self):
        """
        Records a heartbeat for this component, marking it as active.
        """
        self.last_heartbeat = time.time()

class HealthMonitor:
    """
    Monitors the health of various system components in real-time.

    This class acts as a central watchdog. Other modules are expected to
    register with the HealthMonitor and then periodically send heartbeats.
    The monitor runs a background thread to check the status of all
    registered components.
    """
    def __init__(self, check_interval: float = 0.5):
        """
        Initializes the HealthMonitor.

        Args:
            check_interval (float, optional): The interval in seconds at which to check component statuses. Defaults to 0.5.
        """
        self._components = {}
        self._lock = threading.Lock()
        self._check_interval = check_interval
        self._running = False
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)

    def register_component(self, name: str, timeout: float = 1.0):
        """
        Registers a new component for health monitoring.

        Args:
            name (str): The unique name of the component.
            timeout (float, optional): The heartbeat timeout in seconds for this component. Defaults to 1.0.
        
        Raises:
            ValueError: If a component with the same name is already registered.
        """
        with self._lock:
            if name in self._components:
                raise ValueError(f"Component '{name}' is already registered.")
            self._components[name] = MonitoredComponent(name, timeout)

    def heartbeat(self, name: str):
        """
        Records a heartbeat from a component.

        Args:
            name (str): The name of the component sending the heartbeat.
        """
        with self._lock:
            if name in self._components:
                self._components[name].record_heartbeat()
            # Silently ignore heartbeats from unregistered components for performance.

    def start(self):
        """
        Starts the background monitoring thread.
        """
        if not self._running:
            self._running = True
            self._monitor_thread.start()

    def stop(self):
        """
        Stops the background monitoring thread.
        """
        self._running = False
        if self._monitor_thread.is_alive():
            self._monitor_thread.join()

    def _monitor_loop(self):
        """
        The main loop for the monitoring thread. Periodically updates component statuses.
        """
        while self._running:
            with self._lock:
                for component in self._components.values():
                    component.update_status()
            time.sleep(self._check_interval)

    def get_system_status(self) -> dict[str, dict]:
        """
        Gets the current health status of all registered components.

        Returns:
            dict[str, dict]: A dictionary where keys are component names and
                             values are dictionaries containing the component's
                             'status' (ComponentStatus) and 'message' (str).
        """
        report = {}
        with self._lock:
            for name, component in self._components.items():
                report[name] = {
                    "status": component.status,
                    "message": component.message
                }
        return report

    def get_overall_health(self) -> ComponentStatus:
        """
        Determines the overall system health based on the worst-case component status.

        The overall health is determined by the most severe status among all components:
        - CRITICAL if any component is CRITICAL.
        - WARNING if any component is WARNING (and none are CRITICAL).
        - UNKNOWN if any component is UNKNOWN (and none are CRITICAL or WARNING).
        - OK if all components are OK.

        Returns:
            ComponentStatus: The overall system health status.
        """
        overall_status = ComponentStatus.OK
        with self._lock:
            statuses = [comp.status for comp in self._components.values()]
            if ComponentStatus.CRITICAL in statuses:
                overall_status = ComponentStatus.CRITICAL
            elif ComponentStatus.WARNING in statuses:
                overall_status = ComponentStatus.WARNING
            elif ComponentStatus.UNKNOWN in statuses:
                overall_status = ComponentStatus.UNKNOWN
        return overall_status

if __name__ == '__main__':
    # Example usage and demonstration
    print("--- HealthMonitor Demonstration ---")
    
    # 1. Initialize and start the monitor
    health_monitor = HealthMonitor(check_interval=0.5)
    health_monitor.start()
    
    # 2. Register components with different timeouts
    print("Registering components: 'Fusion', 'Detection', 'Controller'")
    health_monitor.register_component("Fusion", timeout=1.0)
    health_monitor.register_component("Detection", timeout=1.5)
    health_monitor.register_component("Controller", timeout=0.8)

    try:
        # 3. Simulate heartbeats
        print("\n--- Simulating normal operation (all components sending heartbeats) ---")
        for i in range(5):
            print(f"Simulation time: {i*0.5}s")
            health_monitor.heartbeat("Fusion")
            health_monitor.heartbeat("Detection")
            health_monitor.heartbeat("Controller")
            time.sleep(0.5)
            report = health_monitor.get_system_status()
            for name, data in report.items():
                print(f"  - {name}: {data['status'].value} ({data['message']})")
            print(f"Overall System Health: {health_monitor.get_overall_health().value}\n")

        # 4. Simulate a failure (Detection stops sending heartbeats)
        print("\n--- Simulating failure (Detection module stops) ---")
        for i in range(6):
            print(f"Simulation time: {2.5 + i*0.5}s")
            health_monitor.heartbeat("Fusion")
            # health_monitor.heartbeat("Detection") # <-- Detection is now silent
            health_monitor.heartbeat("Controller")
            time.sleep(0.5)
            report = health_monitor.get_system_status()
            for name, data in report.items():
                print(f"  - {name}: {data['status'].value} ({data['message']})")
            print(f"Overall System Health: {health_monitor.get_overall_health().value}\n")

    except KeyboardInterrupt:
        print("\n--- Simulation interrupted ---")
    finally:
        # 5. Stop the monitor
        print("--- Stopping HealthMonitor ---")
        health_monitor.stop()
        print("Monitor stopped.")
