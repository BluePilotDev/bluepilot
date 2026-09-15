from types import SimpleNamespace

import pyray as rl

from openpilot.bluepilot.ui.lib.bp_shaders import circle_shader_screen_space
from openpilot.selfdrive.ui.bp.mici.onroad.confidence_ball_bp import confidence_ball_colors
from openpilot.selfdrive.ui.bp.mici.onroad.confidence_ball_bp import (
  MICI_TESLA_STATUS_LAMP_BEZEL,
  MICI_TESLA_STATUS_LAMP_RADIUS,
  mici_tesla_status_layout,
)
from openpilot.selfdrive.ui.bp.onroad.augmented_road_view_bp import confidence_ball_presentation
from openpilot.selfdrive.ui.bp.onroad.hud_renderer_bp import (
  TESLA_CONF_BALL_RADIUS,
  TESLA_LEAD_FASTER_COLOR,
  TESLA_LEAD_FULL_RED_DELTA_MPS,
  TESLA_LEAD_LABEL_SIZE,
  TESLA_LEAD_SLOW_RED,
  TESLA_LEAD_SLOW_YELLOW,
  TESLA_LEAD_SPEED_OUTLINE,
  TESLA_LEAD_SPEED_OUTLINE_WIDTH,
  TESLA_LEAD_SPEED_SIZE,
  TESLA_MADS_LAMP_RADIUS,
  TESLA_MAX_LABEL_SIZE,
  TESLA_SET_SPEED_SIZE,
  TESLA_SET_SPEED_OUTLINE,
  TESLA_SET_SPEED_OUTLINE_WIDTH,
  TESLA_STATUS_LAMP_BEZEL,
  TESLA_STATUS_LAMP_RADIUS,
  TESLA_STATUS_LABEL_SIZE,
  HudRendererBP,
  tesla_column_text_x,
  tesla_lead_speed_color,
  tesla_lead_speed_state,
  tesla_status_row_layout,
  tesla_text_outline_offsets,
)
from openpilot.selfdrive.ui.bp.onroad.speed_limit_renderer_bp import (
  SpeedLimitRendererBP,
  tesla_speed_limit_sign_rect,
)
from openpilot.selfdrive.ui.bp.onroad.tesla_style_renderer_bp import LeadFadeState, color_with_opacity
from openpilot.selfdrive.ui.bp.lib.tesla_status import tesla_mads_active
from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_limit import SpeedLimitRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.hud_renderer import HudRendererSP
from openpilot.selfdrive.ui.ui_state import UIStatus


def test_non_tesla_theme_delegates_to_unchanged_set_speed_renderer(monkeypatch):
  renderer = object.__new__(HudRendererBP)
  renderer._tesla_style = False
  rect = rl.Rectangle(0, 0, 2160, 1080)
  calls = []

  def stock_draw(instance, draw_rect):
    calls.append((instance, draw_rect))

  monkeypatch.setattr(HudRendererSP, "_draw_set_speed", stock_draw)
  renderer._draw_set_speed(rect)

  assert calls == [(renderer, rect)]


def test_non_tesla_speed_limit_delegates_to_stock_renderer(monkeypatch):
  renderer = object.__new__(SpeedLimitRendererBP)
  renderer._tesla_style = False
  rect = rl.Rectangle(0, 0, 2160, 1080)
  calls = []

  monkeypatch.setattr(SpeedLimitRenderer, "_render", lambda instance, draw_rect: calls.append((instance, draw_rect)))
  renderer._render(rect)

  assert calls == [(renderer, rect)]


def test_tesla_speed_limit_keeps_c3x_stock_footprint() -> None:
  rect = rl.Rectangle(0, 0, 2160, 1080)
  sign = tesla_speed_limit_sign_rect(rect, is_metric=False)

  assert (sign.x, sign.y) == (60 + UI_CONFIG.set_speed_width_imperial + 24, 39)
  assert (sign.width, sign.height) == (UI_CONFIG.set_speed_width_imperial, UI_CONFIG.set_speed_height + 12)


class FakeSubMaster:
  def __init__(self, *, lead_status=True, radar_alive=True, radar_valid=True,
               car_alive=True, car_valid=True, d_rel=30.0, v_lead=20.0,
               v_lead_k=None, v_ego=18.0, mads_alive=True, mads_valid=True,
               mads_enabled=True):
    v_lead_k = v_lead if v_lead_k is None else v_lead_k
    self.alive = {
      "radarState": radar_alive,
      "carState": car_alive,
      "selfdriveStateSP": mads_alive,
    }
    self.valid = {
      "radarState": radar_valid,
      "carState": car_valid,
      "selfdriveStateSP": mads_valid,
    }
    self.messages = {
      "radarState": SimpleNamespace(leadOne=SimpleNamespace(
        status=lead_status, dRel=d_rel, vLead=v_lead, vLeadK=v_lead_k,
      )),
      "carState": SimpleNamespace(vEgo=v_ego),
      "selfdriveStateSP": SimpleNamespace(mads=SimpleNamespace(enabled=mads_enabled)),
    }

  def __getitem__(self, service):
    return self.messages[service]


def test_tesla_max_and_lead_typography_is_enlarged() -> None:
  assert TESLA_SET_SPEED_SIZE == 112
  assert TESLA_STATUS_LABEL_SIZE == 48
  assert TESLA_MAX_LABEL_SIZE == TESLA_STATUS_LABEL_SIZE
  assert TESLA_LEAD_LABEL_SIZE == TESLA_STATUS_LABEL_SIZE
  assert TESLA_LEAD_SPEED_SIZE == 54
  assert TESLA_CONF_BALL_RADIUS == TESLA_STATUS_LAMP_RADIUS
  assert TESLA_MADS_LAMP_RADIUS == TESLA_STATUS_LAMP_RADIUS
  assert TESLA_STATUS_LAMP_RADIUS == 24
  assert TESLA_STATUS_LAMP_BEZEL == 5
  assert TESLA_STATUS_LAMP_RADIUS < TESLA_STATUS_LABEL_SIZE


def test_tesla_set_and_lead_speeds_share_thin_black_eight_point_outline() -> None:
  offsets = tesla_text_outline_offsets(TESLA_SET_SPEED_OUTLINE_WIDTH)

  assert TESLA_SET_SPEED_OUTLINE_WIDTH == 2
  assert _rgba(TESLA_SET_SPEED_OUTLINE) == (0, 0, 0, 160)
  assert TESLA_LEAD_SPEED_OUTLINE_WIDTH == TESLA_SET_SPEED_OUTLINE_WIDTH
  assert _rgba(TESLA_LEAD_SPEED_OUTLINE) == _rgba(TESLA_SET_SPEED_OUTLINE)
  assert len(offsets) == 8
  assert set(offsets) == {
    (-2, -2), (0, -2), (2, -2),
    (-2, 0), (2, 0),
    (-2, 2), (0, 2), (2, 2),
  }


def test_status_lamp_shader_geometry_matches_scaled_framebuffer() -> None:
  assert circle_shader_screen_space(168, 435, 24, 0.5, 540) == (84, 322.5, 12)
  assert circle_shader_screen_space(30, 156, 15, 1.0, 240) == (30, 84, 15)


def test_tesla_left_column_rows_share_one_centerline() -> None:
  column_center = 146.0
  for text_width in (28.0, 67.5, 104.0, 172.0):
    left = tesla_column_text_x(column_center, text_width)
    assert left + text_width / 2 == column_center


def test_c4_tesla_status_stack_fits_confidence_strip() -> None:
  rect = rl.Rectangle(476, 0, 60, 240)
  center_x, conf_label_y, conf_lamp_y, mads_label_y, mads_lamp_y = mici_tesla_status_layout(rect)

  assert center_x == rect.x + rect.width / 2
  assert conf_label_y < conf_lamp_y < mads_label_y < mads_lamp_y
  outer_radius = MICI_TESLA_STATUS_LAMP_RADIUS + MICI_TESLA_STATUS_LAMP_BEZEL
  assert conf_lamp_y - outer_radius > conf_label_y
  assert mads_lamp_y + outer_radius < rect.y + rect.height
  assert MICI_TESLA_STATUS_LAMP_RADIUS == 15
  assert MICI_TESLA_STATUS_LAMP_BEZEL == 3


def test_c4_mads_uses_first_status_row_when_confidence_is_hidden() -> None:
  rect = rl.Rectangle(476, 0, 60, 240)
  center_x, conf_label_y, conf_lamp_y, mads_label_y, mads_lamp_y = mici_tesla_status_layout(
    rect, confidence_enabled=False,
  )

  assert center_x == rect.x + rect.width / 2
  assert conf_label_y is None
  assert conf_lamp_y is None
  assert (mads_label_y, mads_lamp_y) == (rect.y + 31, rect.y + 75)


def test_c3x_mads_uses_first_status_row_when_confidence_is_hidden() -> None:
  confidence_row, mads_row = tesla_status_row_layout(45, confidence_enabled=False)

  assert confidence_row is None
  assert mads_row == (351, 435)


def test_c3x_confidence_and_mads_lamps_have_equal_label_spacing() -> None:
  confidence_row, mads_row = tesla_status_row_layout(45, confidence_enabled=True)

  assert confidence_row is not None
  assert confidence_row[1] - confidence_row[0] == mads_row[1] - mads_row[0] == 84


def test_confidence_ball_presentation_is_mutually_exclusive() -> None:
  assert confidence_ball_presentation(False, False) == (False, False)
  assert confidence_ball_presentation(False, True) == (False, False)
  assert confidence_ball_presentation(True, False) == (True, False)
  assert confidence_ball_presentation(True, True) == (False, True)


def test_tesla_mads_state_uses_published_signal() -> None:
  assert tesla_mads_active(FakeSubMaster(mads_enabled=True))
  assert not tesla_mads_active(FakeSubMaster(mads_enabled=False))
  assert not tesla_mads_active(FakeSubMaster(mads_alive=False))
  assert not tesla_mads_active(FakeSubMaster(mads_valid=False))


def _rgba(color: rl.Color) -> tuple[int, int, int, int]:
  return color.r, color.g, color.b, color.a


def test_static_confidence_ball_reuses_bluepilot_color_scheme() -> None:
  high = confidence_ball_colors(0.8, UIStatus.ENGAGED)
  medium = confidence_ball_colors(0.4, UIStatus.LAT_ONLY)
  low = confidence_ball_colors(0.1, UIStatus.LONG_ONLY)
  override = confidence_ball_colors(0.8, UIStatus.OVERRIDE)
  disengaged = confidence_ball_colors(0.8, UIStatus.DISENGAGED)

  assert tuple(map(_rgba, high)) == ((0, 255, 204, 255), (0, 255, 38, 255))
  assert tuple(map(_rgba, medium)) == ((255, 200, 0, 255), (255, 115, 0, 255))
  assert tuple(map(_rgba, low)) == ((255, 0, 21, 255), (255, 0, 89, 255))
  assert tuple(map(_rgba, override)) == ((255, 255, 255, 255), (82, 82, 82, 255))
  assert tuple(map(_rgba, disengaged)) == ((50, 50, 50, 255), (13, 13, 13, 255))


def test_tesla_lead_speed_uses_fused_primary_lead() -> None:
  assert tesla_lead_speed_state(FakeSubMaster(v_lead=20.0, v_ego=18.0)) == (30.0, 20.0, 18.0)
  assert tesla_lead_speed_state(FakeSubMaster(v_lead=21.0, v_lead_k=20.0)) == (30.0, 20.0, 18.0)
  assert tesla_lead_speed_state(FakeSubMaster(lead_status=False)) is None
  assert tesla_lead_speed_state(FakeSubMaster(radar_alive=False)) is None
  assert tesla_lead_speed_state(FakeSubMaster(car_valid=False)) is None
  assert tesla_lead_speed_state(FakeSubMaster(d_rel=float("nan"))) is None


def test_tesla_lead_speed_clamps_negative_speed_to_zero() -> None:
  assert tesla_lead_speed_state(FakeSubMaster(v_lead=-1.0, v_ego=2.0)) == (30.0, 0.0, 2.0)


def test_tesla_lead_hud_fades_old_identity_out_before_new_value() -> None:
  state = LeadFadeState()
  old_lead = (30.0, 20.0, 18.0)
  new_lead = (48.0, 25.0, 18.0)

  assert state.update(old_lead, 1, 0.5) == (old_lead, 0.5)
  assert state.update(old_lead, 1, 0.5) == (old_lead, 1.0)
  assert state.update(new_lead, 2, 0.5) == (old_lead, 0.5)
  assert state.update(new_lead, 2, 0.5) == (old_lead, 0.0)
  assert state.update(new_lead, 2, 0.5) == (new_lead, 0.5)


def test_tesla_lead_hud_fade_scales_text_and_outline_alpha() -> None:
  color = color_with_opacity(rl.Color(10, 20, 30, 200), 0.25)

  assert (color.r, color.g, color.b, color.a) == (10, 20, 30, 50)


def test_tesla_lead_speed_color_rules() -> None:
  assert tesla_lead_speed_color(20.0, 20.0) == TESLA_LEAD_FASTER_COLOR
  assert tesla_lead_speed_color(21.0, 20.0) == TESLA_LEAD_FASTER_COLOR
  assert tesla_lead_speed_color(20.0, 20.001).r == TESLA_LEAD_SLOW_YELLOW.r
  assert tesla_lead_speed_color(10.0, 10.0 + TESLA_LEAD_FULL_RED_DELTA_MPS) == TESLA_LEAD_SLOW_RED
  assert tesla_lead_speed_color(0.0, 30.0) == TESLA_LEAD_SLOW_RED

  midpoint = tesla_lead_speed_color(10.0, 10.0 + TESLA_LEAD_FULL_RED_DELTA_MPS / 2.0)
  assert midpoint.r == round((TESLA_LEAD_SLOW_YELLOW.r + TESLA_LEAD_SLOW_RED.r) / 2.0)
  assert midpoint.g == round((TESLA_LEAD_SLOW_YELLOW.g + TESLA_LEAD_SLOW_RED.g) / 2.0)
  assert midpoint.b == round((TESLA_LEAD_SLOW_YELLOW.b + TESLA_LEAD_SLOW_RED.b) / 2.0)
