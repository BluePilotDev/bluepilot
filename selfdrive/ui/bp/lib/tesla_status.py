import pyray as rl

from openpilot.bluepilot.ui.lib.bp_shaders import draw_shader_circle_gradient


def tesla_mads_active(sm) -> bool:
  """Return the published MADS engagement state, false when unavailable."""
  if not (sm.alive.get("selfdriveStateSP", False) and sm.valid.get("selfdriveStateSP", False)):
    return False
  return bool(sm["selfdriveStateSP"].mads.enabled)


def tesla_mads_lamp_colors(active: bool) -> tuple[rl.Color, rl.Color]:
  if active:
    return rl.Color(91, 235, 139, 255), rl.Color(18, 158, 77, 255)
  return rl.Color(255, 106, 88, 255), rl.Color(190, 35, 39, 255)


def draw_tesla_status_lamp(center_x: float, center_y: float, radius: float,
                           bezel: float, top: rl.Color, bottom: rl.Color) -> None:
  """Draw the shared Tesla-style recessed status lamp at any display scale."""
  draw_shader_circle_gradient(
    center_x, center_y, radius + bezel,
    rl.Color(78, 84, 89, 255), rl.Color(18, 21, 23, 255),
  )
  draw_shader_circle_gradient(center_x, center_y, radius, top, bottom)
  rl.draw_circle(
    int(center_x - radius * 0.3),
    int(center_y - radius * 0.3),
    max(2, round(radius * 0.16)),
    rl.Color(255, 255, 255, 120),
  )
