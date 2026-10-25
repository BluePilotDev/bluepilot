import pytest

from openpilot.selfdrive.ui.bp.onroad.lane_center_indicator_bp import (
  LANE_CENTER_ICON_GAP,
  lane_center_indicator_rect,
  lane_center_indicator_visible,
)


@pytest.mark.parametrize(
  "toggle,alive,valid,active,expected",
  [
    (True, True, True, True, True),
    (False, True, True, True, False),
    (True, False, True, True, False),
    (True, True, False, True, False),
    (True, True, True, False, False),
  ],
)
def test_lane_center_indicator_visibility(toggle, alive, valid, active, expected):
  assert lane_center_indicator_visible(toggle, alive, valid, active) is expected


def test_lane_center_indicator_aligns_left_of_wheel():
  button_x, button_y, button_size = 1800.0, 60.0, 192.0
  rect = lane_center_indicator_rect(button_x, button_y, button_size)

  assert rect.x + rect.width == pytest.approx(button_x - LANE_CENTER_ICON_GAP)
  assert rect.y + rect.height / 2.0 == pytest.approx(button_y + button_size / 2.0)
  assert rect.width == rect.height
