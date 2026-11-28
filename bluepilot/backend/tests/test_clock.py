import unittest
from unittest import mock

from bluepilot.backend.core.server_state import ServerState
from bluepilot.backend.utils import power
from bluepilot.backend.video.export import wait_for_ffmpeg_capacity

HOUR = 3600.0


class FakeClock:
  """A wall clock and a monotonic clock that advance together until the wall clock is jumped."""
  def __init__(self):
    self.wall = 1_700_000_000.0
    self.mono = 1000.0
    self.slept = 0.0
    self.jump_on_sleep = 0.0

  def advance(self, seconds):
    self.wall += seconds
    self.mono += seconds

  def sleep(self, seconds):
    self.advance(seconds)
    self.slept += seconds
    self.wall += self.jump_on_sleep
    self.jump_on_sleep = 0.0


class ClockTestCase(unittest.TestCase):
  def setUp(self):
    self.clock = FakeClock()
    for name, fake in (("time", lambda: self.clock.wall), ("monotonic", lambda: self.clock.mono), ("sleep", self.clock.sleep)):
      patcher = mock.patch(f"time.{name}", fake)
      patcher.start()
      self.addCleanup(patcher.stop)


class TestFfmpegCapacityWait(ClockTestCase):
  def test_waits_its_full_timeout_when_wall_clock_jumps_forward(self):
    server_state = mock.Mock()
    server_state.get_ffmpeg_count.return_value = 99  # capacity never frees up
    self.clock.jump_on_sleep = HOUR

    self.assertFalse(wait_for_ffmpeg_capacity(timeout=10.0, server_state=server_state))
    self.assertGreaterEqual(self.clock.slept, 10.0)
    self.assertLess(self.clock.slept, 11.0)


class TestIdleTimer(ClockTestCase):
  def setUp(self):
    super().setUp()
    self.restored = []
    for patcher in (
      mock.patch.object(power, "restore_power_save", lambda: self.restored.append(True)),
      mock.patch.object(power, "last_activity_time", None),
      mock.patch.object(power.os.path, "exists", lambda _path: False),  # keep sysfs untouched
    ):
      patcher.start()
      self.addCleanup(patcher.stop)

  def test_does_not_fire_early_when_wall_clock_jumps_forward(self):
    power.enable_performance_mode()
    self.clock.advance(5)
    self.clock.wall += HOUR

    power.check_and_restore_power_save(lambda: False)
    self.assertEqual(self.restored, [])


class TestServerUptime(ClockTestCase):
  def test_uptime_ignores_wall_clock_jumping_forward(self):
    state = ServerState()
    self.clock.advance(5)
    self.clock.wall += HOUR
    self.assertAlmostEqual(state.get_server_uptime(), 5.0)

  def test_uptime_is_not_negative_when_wall_clock_jumps_backward(self):
    state = ServerState()
    self.clock.advance(5)
    self.clock.wall -= HOUR
    self.assertAlmostEqual(state.get_server_uptime(), 5.0)


if __name__ == "__main__":
  unittest.main()
