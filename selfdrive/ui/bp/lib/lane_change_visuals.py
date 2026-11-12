"""Pure state helpers for Tesla-inspired lane-change visualization."""

from __future__ import annotations

from cereal import log

from openpilot.selfdrive.ui.bp.lib.blindspot_visuals import (
  TESLA_LEFT_INNER_LANE_INDEX,
  TESLA_RIGHT_INNER_LANE_INDEX,
)


_LANE_CHANGE_UNDERWAY = (
  log.LaneChangeState.laneChangeStarting,
  log.LaneChangeState.laneChangeFinishing,
)


def tesla_lane_change_lane_active(sm, lane_index: int) -> bool:
  """Highlight the inner boundary on the destination side during a lane change."""
  if lane_index not in (TESLA_LEFT_INNER_LANE_INDEX, TESLA_RIGHT_INNER_LANE_INDEX):
    return False
  if not (sm.alive.get("modelV2", False) and sm.valid.get("modelV2", False)):
    return False

  meta = sm["modelV2"].meta
  if meta.laneChangeState not in _LANE_CHANGE_UNDERWAY:
    return False
  if lane_index == TESLA_LEFT_INNER_LANE_INDEX:
    return meta.laneChangeDirection == log.LaneChangeDirection.left
  return meta.laneChangeDirection == log.LaneChangeDirection.right
