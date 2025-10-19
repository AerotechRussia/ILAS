import time

class Watchdog:
    """A simple watchdog timer to monitor subsystem health."""

    def __init__(self, timeout):
        self.timeout = timeout
        self.last_reset_time = time.time()

    def reset(self):
        """Resets the watchdog timer."""
        self.last_reset_time = time.time()

    def check(self):
        """Checks if the watchdog has timed out."""
        return (time.time() - self.last_reset_time) > self.timeout
