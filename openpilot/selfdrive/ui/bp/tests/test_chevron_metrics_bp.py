from types import SimpleNamespace
from unittest import mock

import pyray as rl

from openpilot.selfdrive.ui.bp.onroad.chevron_metrics_bp import ChevronMetricsBP


def _texture(texture_id: int, width: int = 64, height: int = 64) -> rl.Texture:
  texture = rl.Texture()
  texture.id = texture_id
  texture.width = width
  texture.height = height
  return texture


def test_inverted_radar_overlay_has_solid_fill_and_source_icon():
  renderer = ChevronMetricsBP.__new__(ChevronMetricsBP)
  renderer._font = rl.Font()
  renderer._lead_status_alpha = 1.0
  renderer.ford_overlay_enabled = True
  renderer.overlay_scale = 1.0
  renderer._radar_source_icon = _texture(11)
  renderer._vision_source_icon = _texture(12)

  triangle_calls = []
  texture_calls = []

  def capture_triangle(points, count, color):
    triangle_calls.append(list(points[:count]))

  def capture_texture(texture, source, dest, origin, rotation, tint):
    texture_calls.append((texture, dest, rotation))

  lead_vehicle = SimpleNamespace(
    chevron=[(850.0, 600.0), (960.0, 550.0), (1070.0, 600.0)],
    glow=[],
  )

  with (mock.patch("openpilot.selfdrive.ui.bp.onroad.chevron_metrics_bp.measure_text_cached",
                   return_value=rl.Vector2(100, 60)),
        mock.patch.object(rl, "draw_rectangle_rounded"),
        mock.patch.object(rl, "draw_rectangle_rounded_lines_ex"),
        mock.patch.object(rl, "draw_text_ex"),
        mock.patch.object(rl, "draw_triangle_fan", side_effect=capture_triangle),
        mock.patch.object(rl, "draw_line_ex"),
        mock.patch.object(rl, "draw_circle_v"),
        mock.patch.object(rl, "draw_texture_pro", side_effect=capture_texture)):
    renderer._render_text_lines_bp(
      ["30 m", "50 km/h", "1.2 s"], lead_vehicle, 0.0,
      rl.Rectangle(0, 0, 1920, 1080), is_radar=True, inverted=True,
    )

  assert len(triangle_calls) == 1
  apex, base_right, base_left = triangle_calls[0]
  winding = ((base_right.x - apex.x) * (base_left.y - apex.y) -
             (base_right.y - apex.y) * (base_left.x - apex.x))
  assert winding < 0  # Same winding as the upward triangle; required for a solid GPU fill.
  assert base_right.x > base_left.x

  assert len(texture_calls) == 1
  texture, dest, rotation = texture_calls[0]
  assert texture.id == renderer._radar_source_icon.id
  assert 0 < dest.width < 80 and 0 < dest.height < 80
  assert rotation == 90
