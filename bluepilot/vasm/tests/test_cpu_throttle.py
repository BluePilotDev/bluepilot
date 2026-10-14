from openpilot.bluepilot.vasm.cpu_throttle import CpuThrottle, throttle_target


def test_throttle_target_stays_idle_below_threshold():
  assert throttle_target([20.0, 30.0, 40.0, 50.0]) == 1.0


def test_throttle_target_rises_for_sustained_average_load():
  assert throttle_target([90.0] * 8) > 2.0


def test_throttle_filter_changes_gradually():
  throttle = CpuThrottle()
  assert throttle.update([10.0] * 8, now=1.0) == 1.0
  assert 1.0 < throttle.update([99.0] * 8, now=1.1) < throttle_target([99.0] * 8)
