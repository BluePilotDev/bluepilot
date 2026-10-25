"""BluePilot comma 3X Lane Positioning confidence indicator."""

import pyray as rl


LANE_CENTER_ICON_SCALE = 0.79
LANE_CENTER_ICON_GAP = 24.0


def lane_center_indicator_rect(button_x: float, button_y: float, button_size: float) -> rl.Rectangle:
  """Place the indicator immediately left of, and centered with, the steering-wheel button."""
  size = button_size * LANE_CENTER_ICON_SCALE
  return rl.Rectangle(button_x - LANE_CENTER_ICON_GAP - size,
                      button_y + (button_size - size) / 2.0,
                      size, size)


def lane_center_indicator_visible(toggle_enabled: bool, message_alive: bool,
                                  message_valid: bool, lane_positioning_active: bool) -> bool:
  return toggle_enabled and message_alive and message_valid and lane_positioning_active


def draw_lane_center_indicator(rect: rl.Rectangle, texture: rl.Texture) -> None:
  """Draw the pre-rasterized glyph with one GPU call instead of rebuilding it each frame."""
  source = rl.Rectangle(0.0, 0.0, texture.width, texture.height)
  rl.draw_texture_pro(texture, source, rect, rl.Vector2(0.0, 0.0), 0.0, rl.WHITE)
