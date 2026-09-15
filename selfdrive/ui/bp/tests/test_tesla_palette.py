from openpilot.selfdrive.ui.bp.lib.tesla_palette import (
  DARK_PALETTE,
  LIGHT_PALETTE,
  TESLA_PATH_BLUE_DEEP,
  TESLA_PATH_BLUE_LIGHT,
  TeslaAutoPaletteState,
  blend_color,
  palette_for_dark_fraction,
  tesla_blue_cycle_color,
  tesla_path_gradient_colors,
  tesla_wheel_color,
)
from openpilot.selfdrive.ui.bp.onroad.tesla_style_renderer_bp import TeslaStyleRendererBP


def _brightness(color) -> int:
  return color.r + color.g + color.b


def _rgba(color) -> tuple[int, int, int, int]:
  return color.r, color.g, color.b, color.a


def test_dark_environment_is_materially_darker_than_light():
  assert _brightness(DARK_PALETTE.sky_top) < _brightness(LIGHT_PALETTE.sky_top) / 2
  assert _brightness(DARK_PALETTE.road_surface) < _brightness(LIGHT_PALETTE.road_surface) / 2


def test_dark_lane_colors_are_neutral_not_yellow():
  for color in (DARK_PALETTE.lane_inner, DARK_PALETTE.lane_outer):
    assert max(color.r, color.g, color.b) - min(color.r, color.g, color.b) < 16


def test_palette_blend_uses_exact_endpoints_and_midpoint():
  assert palette_for_dark_fraction(0.0) is LIGHT_PALETTE
  assert palette_for_dark_fraction(1.0) is DARK_PALETTE
  midpoint = palette_for_dark_fraction(0.5)
  expected_sky = blend_color(LIGHT_PALETTE.sky_top, DARK_PALETTE.sky_top, 0.5)
  expected_road = blend_color(LIGHT_PALETTE.road_surface, DARK_PALETTE.road_surface, 0.5)
  assert _rgba(midpoint.sky_top) == _rgba(expected_sky)
  assert _rgba(midpoint.road_surface) == _rgba(expected_road)


def _finish_transition(state: TeslaAutoPaletteState, light_sensor: float, start: float) -> float:
  result = state.dark_fraction
  for step in range(1, 5):
    result = state.update(light_sensor, start + step * 0.25)
  return result


def test_auto_palette_requires_three_continuous_dark_seconds():
  state = TeslaAutoPaletteState()

  assert state.update(20.0, 0.0) == 0.0
  assert state.update(20.0, 2.99) == 0.0
  assert not state.dark_mode
  assert state.update(20.0, 3.0) > 0.0
  assert state.dark_mode
  assert _finish_transition(state, 20.0, 3.0) == 1.0


def test_auto_palette_rejects_short_shadow_and_uses_hysteresis():
  state = TeslaAutoPaletteState()
  state.update(20.0, 0.0)
  state.update(20.0, 2.9)
  state.update(80.0, 3.0)
  state.update(20.0, 5.0)
  assert state.update(20.0, 7.9) == 0.0
  assert not state.dark_mode

  state.update(20.0, 8.0)
  assert state.dark_mode
  _finish_transition(state, 20.0, 8.0)
  state.update(42.0, 20.0)  # Between thresholds: remain Dark indefinitely.
  state.update(42.0, 30.0)
  assert state.dark_mode


def test_auto_palette_requires_sustained_brightness_to_return_light():
  state = TeslaAutoPaletteState()
  state.update(20.0, 0.0)
  state.update(20.0, 3.0)
  _finish_transition(state, 20.0, 3.0)

  state.update(80.0, 5.0)
  state.update(80.0, 7.99)
  assert state.dark_mode
  assert state.update(80.0, 8.0) < 1.0
  assert not state.dark_mode
  assert _finish_transition(state, 80.0, 8.0) == 0.0


def test_auto_palette_holds_last_state_when_sensor_is_unavailable():
  state = TeslaAutoPaletteState()
  state.update(20.0, 0.0)
  state.update(20.0, 3.0)
  before = state.dark_fraction

  assert state.update(-1.0, 3.25) >= before
  assert state.dark_mode


def test_only_max_label_uses_longitudinal_state_color():
  for palette in (LIGHT_PALETTE, DARK_PALETTE):
    assert palette.max_active.b > palette.max_active.r
    assert abs(palette.max_inactive.r - palette.max_inactive.b) < 16


def test_tesla_wheel_is_blue_only_while_longitudinal_is_active():
  disabled = tesla_wheel_color(True, False, 0.0, 180)
  non_tesla = tesla_wheel_color(False, True, 1.0, 180)
  light_active = tesla_wheel_color(True, True, 0.0, 180)
  dark_active = tesla_wheel_color(True, True, 1.0, 180)

  assert _rgba(disabled) == (255, 255, 255, 180)
  assert _rgba(non_tesla) == (255, 255, 255, 180)
  for active in (light_active, dark_active):
    assert active.b > active.g > active.r
    assert active.a == 180
  assert _brightness(light_active) < _brightness(LIGHT_PALETTE.max_active)


def test_centered_ego_actor_is_removed():
  assert not hasattr(TeslaStyleRendererBP, "_draw_ego_vehicle")


def test_tesla_blue_cycle_uses_sampled_reference_endpoints():
  deep = tesla_blue_cycle_color(0.0, 200)
  light = tesla_blue_cycle_color(0.5, 120)

  assert (deep.r, deep.g, deep.b, deep.a) == (*TESLA_PATH_BLUE_DEEP, 200)
  assert (light.r, light.g, light.b, light.a) == (*TESLA_PATH_BLUE_LIGHT, 120)


def test_static_and_cycling_tesla_paths_remain_blue_only():
  static = tesla_path_gradient_colors(LIGHT_PALETTE)
  cycling = tesla_path_gradient_colors(DARK_PALETTE, 0.25)

  assert len(static) == 3
  assert len(cycling) == 4
  for color in (*static, *cycling):
    assert color.b >= color.g >= color.r


def test_tesla_path_opacity_scales_without_changing_hue():
  full = tesla_path_gradient_colors(LIGHT_PALETTE)
  faded = tesla_path_gradient_colors(LIGHT_PALETTE, opacity=0.25)

  assert [(c.r, c.g, c.b) for c in faded] == [(c.r, c.g, c.b) for c in full]
  assert faded[0].a == round(full[0].a * 0.25)
