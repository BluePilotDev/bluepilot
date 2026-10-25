"""BluePilot comma 3X Lane Positioning confidence indicator."""

import pyray as rl


LANE_CENTER_ICON_COLOR = rl.Color(93, 198, 255, 255)
LANE_CENTER_ICON_BACKGROUND = rl.Color(0, 0, 0, 225)
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


def _draw_rounded_line(start: rl.Vector2, end: rl.Vector2, thickness: float, color: rl.Color) -> None:
  rl.draw_line_ex(start, end, thickness, color)
  radius = thickness / 2.0
  rl.draw_circle_v(start, radius, color)
  rl.draw_circle_v(end, radius, color)


def draw_lane_center_indicator(rect: rl.Rectangle) -> None:
  """Draw a Ford-inspired lane-centering-assist glyph on an always-legible black disc."""
  cx = rect.x + rect.width / 2.0
  cy = rect.y + rect.height / 2.0
  scale = rect.width / 152.0
  color = LANE_CENTER_ICON_COLOR

  rl.draw_circle_v(rl.Vector2(cx, cy), rect.width / 2.0, LANE_CENTER_ICON_BACKGROUND)

  # Dashed lane boundaries converge toward the top, echoing Ford's Lane Centering Assist mark.
  thickness = 10.0 * scale
  for side in (-1.0, 1.0):
    _draw_rounded_line(
      rl.Vector2(cx + side * 46.0 * scale, cy + 48.0 * scale),
      rl.Vector2(cx + side * 37.0 * scale, cy + 13.0 * scale),
      thickness, color,
    )
    _draw_rounded_line(
      rl.Vector2(cx + side * 31.0 * scale, cy - 13.0 * scale),
      rl.Vector2(cx + side * 26.0 * scale, cy - 43.0 * scale),
      thickness, color,
    )

  # Inward arrows make the lane-centering action distinct from lane-keeping assistance.
  arrow_y = cy - 1.0 * scale
  arrow_width = 13.0 * scale
  arrow_height = 17.0 * scale
  rl.draw_triangle(
    rl.Vector2(cx - 43.0 * scale, arrow_y),
    rl.Vector2(cx - 43.0 * scale - arrow_width, arrow_y - arrow_height / 2.0),
    rl.Vector2(cx - 43.0 * scale - arrow_width, arrow_y + arrow_height / 2.0),
    color,
  )
  rl.draw_triangle(
    rl.Vector2(cx + 43.0 * scale, arrow_y),
    rl.Vector2(cx + 43.0 * scale + arrow_width, arrow_y + arrow_height / 2.0),
    rl.Vector2(cx + 43.0 * scale + arrow_width, arrow_y - arrow_height / 2.0),
    color,
  )

  # Steering wheel and spokes.
  wheel_center = rl.Vector2(cx, cy + 17.0 * scale)
  rl.draw_ring(wheel_center, 24.0 * scale, 31.0 * scale, 0.0, 360.0, 48, color)
  _draw_rounded_line(rl.Vector2(cx - 25.0 * scale, wheel_center.y - 4.0 * scale),
                     rl.Vector2(cx + 25.0 * scale, wheel_center.y - 4.0 * scale),
                     6.0 * scale, color)
  _draw_rounded_line(rl.Vector2(cx, wheel_center.y - 4.0 * scale),
                     rl.Vector2(cx, wheel_center.y + 24.0 * scale),
                     6.0 * scale, color)
  rl.draw_circle_v(rl.Vector2(cx, wheel_center.y - 4.0 * scale), 7.0 * scale, color)
