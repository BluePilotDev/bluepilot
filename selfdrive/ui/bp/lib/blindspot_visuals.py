"""Pure state helpers for BluePilot's blind-spot environment cues."""

from __future__ import annotations

from openpilot.bluepilot.vasm.state import combined_blindspots


TESLA_LEFT_INNER_LANE_INDEX = 1
TESLA_RIGHT_INNER_LANE_INDEX = 2


def tesla_blindspot_lane_active(sm, lane_index: int, params_memory=None) -> bool:
  """Map vehicle BSM state to the matching model inner-lane polygon."""
  if lane_index not in (TESLA_LEFT_INNER_LANE_INDEX, TESLA_RIGHT_INNER_LANE_INDEX):
    return False
  if not sm.valid.get("carState", False):
    return False

  car_state = sm["carState"]
  left_blindspot, right_blindspot = combined_blindspots(car_state, params_memory) if params_memory is not None \
    else (bool(car_state.leftBlindspot), bool(car_state.rightBlindspot))
  if lane_index == TESLA_LEFT_INNER_LANE_INDEX:
    return left_blindspot
  return right_blindspot
