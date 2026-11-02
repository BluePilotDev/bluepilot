from __future__ import annotations

import math
import time


MAX_CPU_USAGE_PERCENT = 89.0
AVG_CPU_USAGE_PERCENT = 74.0
HOT_CORE_COUNT = 4


def throttle_target(cpu_usage: list[float] | tuple[float, ...]) -> float:
  if not cpu_usage:
    return 1.0

  average = sum(cpu_usage) / len(cpu_usage)
  hot_cores = sum(usage >= MAX_CPU_USAGE_PERCENT for usage in cpu_usage)
  average_factor = 1.0 if average < AVG_CPU_USAGE_PERCENT else 1.0 + (average - AVG_CPU_USAGE_PERCENT) / 8.0
  hot_core_factor = 1.0 + max(0, hot_cores - HOT_CORE_COUNT + 1) * 0.5
  return min(max(average_factor, hot_core_factor), 4.0)


class CpuThrottle:
  """Low-pass-filtered inference interval multiplier."""

  def __init__(self):
    self.factor = 1.0
    self.last_time: float | None = None
    self.last_logged_factor = 1.0

  def update(self, cpu_usage: list[float] | tuple[float, ...], name: str = "VASM", now: float | None = None) -> float:
    if not cpu_usage:
      return 1.0

    now = time.monotonic() if now is None else now
    dt = 0.0 if self.last_time is None else max(0.0, now - self.last_time)
    self.last_time = now

    alpha = min(1.0, 1.0 - math.exp(-0.8 * dt))
    target = throttle_target(cpu_usage)
    self.factor = max(1.0, min(5.0, target * alpha + self.factor * (1.0 - alpha)))

    if self.factor >= self.last_logged_factor + 0.6:
      average = sum(cpu_usage) / len(cpu_usage)
      hot_cores = sum(usage >= MAX_CPU_USAGE_PERCENT for usage in cpu_usage)
      print(f"[{name}] CPU throttle factor={self.factor:.1f}x (avg={average:.0f}%, hot={hot_cores})")
      self.last_logged_factor = self.factor
    elif self.factor <= 1.05 and self.last_logged_factor > 1.05:
      print(f"[{name}] CPU recovered")
      self.last_logged_factor = 1.0

    return self.factor
