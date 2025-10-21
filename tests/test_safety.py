# tests/test_safety.py

import unittest
import time
from ilas.safety.watchdog import WatchdogTimer

class TestWatchdogTimer(unittest.TestCase):
    """
    Unit tests for the WatchdogTimer class.
    """

    def test_init_positive_timeout(self):
        """Test that the timer initializes correctly with a positive timeout."""
        timer = WatchdogTimer(timeout_s=1)
        self.assertEqual(timer.timeout_s, 1)
        self.assertFalse(timer.is_expired())

    def test_init_invalid_timeout(self):
        """Test that the timer raises an error for zero or negative timeouts."""
        with self.assertRaises(ValueError):
            WatchdogTimer(timeout_s=0)
        with self.assertRaises(ValueError):
            WatchdogTimer(timeout_s=-1)

    def test_not_expired_before_timeout(self):
        """Test that the timer does not expire before the timeout period."""
        timeout = 0.1
        timer = WatchdogTimer(timeout_s=timeout)
        time.sleep(timeout / 2)
        self.assertFalse(timer.is_expired())

    def test_expired_after_timeout(self):
        """Test that the timer expires after the timeout period has passed."""
        timeout = 0.1
        timer = WatchdogTimer(timeout_s=timeout)
        time.sleep(timeout * 1.1)
        self.assertTrue(timer.is_expired())

    def test_reset_prevents_expiration(self):
        """Test that resetting the timer prevents it from expiring."""
        timeout = 0.2
        timer = WatchdogTimer(timeout_s=timeout)

        # Sleep for a bit, then reset
        time.sleep(timeout * 0.6)
        timer.reset()

        # Sleep again, total sleep time is > timeout, but it shouldn't have expired
        time.sleep(timeout * 0.6)
        self.assertFalse(timer.is_expired())

        # Now let it expire
        time.sleep(timeout * 0.6)
        self.assertTrue(timer.is_expired())

    def test_start_restarts_timer(self):
        """Test that the start method properly restarts the timer."""
        timeout = 0.1
        timer = WatchdogTimer(timeout_s=timeout)

        # Let it expire
        time.sleep(timeout * 1.1)
        self.assertTrue(timer.is_expired())

        # Restart it and check that it's no longer expired
        timer.start()
        self.assertFalse(timer.is_expired())

        # Let it expire again
        time.sleep(timeout * 1.1)
        self.assertTrue(timer.is_expired())

if __name__ == '__main__':
    unittest.main()
