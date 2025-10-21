# /ilas/safety/watchdog.py

import time

class WatchdogTimer:
    """
    A simple watchdog timer to monitor the health of a process.

    This timer is started with a specific timeout. The process being monitored
    must periodically call the `reset()` method (i.e., "pet" the watchdog) to
    prevent the timer from expiring. If `reset()` is not called within the
    timeout period, the `is_expired()` method will return True.
    """

    def __init__(self, timeout_s: float):
        """
        Initializes the WatchdogTimer.

        Args:
            timeout_s (float): The timeout period in seconds.
        """
        if timeout_s <= 0:
            raise ValueError("Timeout must be a positive number of seconds.")

        self.timeout_s = timeout_s
        self._end_time = None
        self.start()

    def start(self):
        """
        Starts or restarts the timer.
        """
        self._end_time = time.monotonic() + self.timeout_s

    def reset(self):
        """
        Resets the timer, preventing it from expiring. This should be called
        periodically by the monitored process.
        """
        self.start()

    def is_expired(self) -> bool:
        """
        Checks if the watchdog timer has expired.

        Returns:
            bool: True if the timer has expired, False otherwise.
        """
        return time.monotonic() > self._end_time

    def __repr__(self) -> str:
        """
        Provides a string representation of the watchdog timer's state.
        """
        remaining = max(0, self._end_time - time.monotonic())
        return (
            f"<WatchdogTimer(timeout={self.timeout_s}s, "
            f"expired={self.is_expired()}, remaining={remaining:.2f}s)>"
        )
