"""BluePilot comma 3X Lane Positioning confidence indicator."""

import pyray as rl


LANE_CENTER_ICON_GAP = 24.0


def lane_center_indicator_rect(button_x: float, button_y: float, button_size: float) -> rl.Rectangle:
  """Match the steering-wheel control's full footprint immediately to its left."""
  return rl.Rectangle(button_x - LANE_CENTER_ICON_GAP - button_size,
                      button_y, button_size, button_size)


def lane_center_indicator_visible(toggle_enabled: bool, message_alive: bool,
                                  message_valid: bool, lane_positioning_active: bool) -> bool:
  return toggle_enabled and message_alive and message_valid and lane_positioning_active


def draw_lane_center_indicator(rect: rl.Rectangle, texture: rl.Texture) -> None:
  """Draw the pre-rasterized glyph with one GPU call instead of rebuilding it each frame."""
  source = rl.Rectangle(0.0, 0.0, texture.width, texture.height)
  rl.draw_texture_pro(texture, source, rect, rl.Vector2(0.0, 0.0), 0.0, rl.WHITE)


def lane_center_dot_position(wheel_x: float, wheel_y: float, wheel_size: float) -> rl.Vector2:
  """Reserve the C4 wheel's turn-intent and power-flow perimeter before placing the dot."""
  return rl.Vector2(wheel_x + wheel_size / 2 + 25 + 16, wheel_y)


def draw_lane_center_dot(position: rl.Vector2) -> None:
  """Compact C4 lane-positioning status, visible against any camera or theme."""
  rl.draw_circle_v(position, 11, rl.Color(0, 0, 0, 230))
  rl.draw_circle_v(position, 8, rl.Color(93, 198, 255, 255))
