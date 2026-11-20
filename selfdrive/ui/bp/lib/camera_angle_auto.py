"""Speed-based camera selection with the device's stock hysteresis thresholds."""

import math


def speed_camera_stream(speed, current, wide, road, wide_available, wide_below, road_above):
  if not wide_available or not math.isfinite(speed):
    return road
  if speed < wide_below:
    return wide
  if speed > road_above:
    return road
  # Keep the selected view through the transition band instead of flickering.
  return current if current in (wide, road) else road
