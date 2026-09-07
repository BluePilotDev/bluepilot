"""Pure state helpers for BluePilot's blind-spot environment cues."""

from __future__ import annotations


TESLA_LEFT_INNER_LANE_INDEX = 1
TESLA_RIGHT_INNER_LANE_INDEX = 2


def tesla_blindspot_lane_active(sm, lane_index: int) -> bool:
  """Map vehicle BSM state to the matching model inner-lane polygon."""
  if lane_index not in (TESLA_LEFT_INNER_LANE_INDEX, TESLA_RIGHT_INNER_LANE_INDEX):
    return False
  if not sm.valid.get("carState", False):
    return False

  car_state = sm["carState"]
  if lane_index == TESLA_LEFT_INNER_LANE_INDEX:
    return bool(car_state.leftBlindspot)
  return bool(car_state.rightBlindspot)
