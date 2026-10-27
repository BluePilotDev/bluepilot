from types import SimpleNamespace

import pyray as rl
import pytest

from openpilot.selfdrive.ui.bp.onroad.lane_center_indicator_bp import (
  LANE_CENTER_ICON_GAP,
  draw_lane_center_indicator,
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
  assert rect.width == button_size
  assert rect.width == rect.height


def test_lane_center_indicator_is_one_cached_texture_draw(monkeypatch):
  calls = []
  monkeypatch.setattr(rl, "draw_texture_pro", lambda *args: calls.append(args))

  draw_lane_center_indicator(
    rl.Rectangle(100.0, 50.0, 152.0, 152.0),
    SimpleNamespace(width=304, height=304),
  )

  assert len(calls) == 1
