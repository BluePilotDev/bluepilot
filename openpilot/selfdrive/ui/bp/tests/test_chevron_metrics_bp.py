from types import SimpleNamespace
import unittest
from unittest import mock

import pyray as rl

from openpilot.selfdrive.ui.bp.onroad.chevron_metrics_bp import ChevronMetricsBP


def _texture(texture_id: int, width: int = 64, height: int = 64) -> rl.Texture:
  texture = rl.Texture()
  texture.id = texture_id
  texture.width = width
  texture.height = height
  return texture


class TestChevronMetricsBP(unittest.TestCase):
  def _render_overlay(self, scale: float, is_radar: bool, inverted: bool):
    renderer = ChevronMetricsBP.__new__(ChevronMetricsBP)
    renderer._font = rl.Font()
    renderer._lead_status_alpha = 1.0
    renderer.ford_overlay_enabled = True
    renderer.overlay_scale = scale
    renderer._radar_source_icon = _texture(11, 128, 128)
    renderer._vision_source_icon = _texture(12, 128, 128)

    triangle_calls = []
    texture_calls = []
    box_calls = []

    def capture_triangle(points, count, color):
      triangle_calls.append(list(points[:count]))

    def capture_texture(texture, source, dest, origin, rotation, tint):
      texture_calls.append((texture, source, dest, rotation))

    lead_vehicle = SimpleNamespace(
      chevron=[(850.0, 600.0), (960.0, 550.0), (1070.0, 600.0)],
      glow=[],
    )

    with (mock.patch("openpilot.selfdrive.ui.bp.onroad.chevron_metrics_bp.measure_text_cached",
                     return_value=rl.Vector2(100 * scale, 60 * scale)),
          mock.patch.object(rl, "draw_rectangle_rounded", side_effect=lambda rect, *_: box_calls.append(rect)),
          mock.patch.object(rl, "draw_rectangle_rounded_lines_ex"),
          mock.patch.object(rl, "draw_text_ex"),
          mock.patch.object(rl, "draw_triangle_fan", side_effect=capture_triangle),
          mock.patch.object(rl, "draw_line_ex"),
          mock.patch.object(rl, "draw_circle_v"),
          mock.patch.object(rl, "draw_texture_pro", side_effect=capture_texture)):
      renderer._render_text_lines_bp(
        ["30 m", "50 km/h", "1.2 s"], lead_vehicle, 0.0,
        rl.Rectangle(0, 0, 1920, 1080), is_radar=is_radar, inverted=inverted,
      )

    return renderer, triangle_calls, texture_calls, box_calls

  def test_source_icons_load_at_native_large_overlay_resolution(self):
    with (mock.patch("openpilot.selfdrive.ui.bp.onroad.chevron_metrics_bp.ChevronMetrics.__init__", return_value=None),
          mock.patch("openpilot.selfdrive.ui.bp.onroad.chevron_metrics_bp.Params"),
          mock.patch("openpilot.selfdrive.ui.bp.onroad.chevron_metrics_bp.gui_app.texture", side_effect=[_texture(11), _texture(12)]) as texture):
      ChevronMetricsBP()

    self.assertEqual(texture.call_args_list, [
      mock.call("../../../bluepilot/assets/images/img_radar.png", 128, 128),
      mock.call("../../../bluepilot/assets/images/img_vision.png", 128, 128),
    ])

  def test_radar_and_vision_glyph_geometry_is_pinned_for_every_overlay_layout(self):
    expected_icon_size = {0.6: 45.6, 1.0: 76.8, 1.5: 115.2}
    for scale in (0.6, 1.0, 1.5):
      for is_radar in (False, True):
        for inverted in (False, True):
          with self.subTest(scale=scale, is_radar=is_radar, inverted=inverted):
            renderer, triangle_calls, texture_calls, box_calls = self._render_overlay(scale, is_radar, inverted)

            self.assertEqual(len(triangle_calls), 1)
            apex, base_a, base_b = triangle_calls[0]
            winding = ((base_a.x - apex.x) * (base_b.y - apex.y) -
                       (base_a.y - apex.y) * (base_b.x - apex.x))
            self.assertLess(winding, 0)
            self.assertNotEqual(base_a.x, base_b.x)

            self.assertEqual(len(texture_calls), 1)
            texture, source, dest, rotation = texture_calls[0]
            self.assertEqual(texture.id, renderer._radar_source_icon.id if is_radar else renderer._vision_source_icon.id)
            self.assertEqual((source.width, source.height), (128, 128))
            self.assertAlmostEqual(dest.width, expected_icon_size[scale], places=4)
            self.assertAlmostEqual(dest.height, expected_icon_size[scale], places=4)
            self.assertEqual(rotation, 90 if is_radar else 0)

            chevron_h = max(72, int(128 * scale))
            expected_box_y = 350 if inverted else 550 + chevron_h
            self.assertEqual(len(box_calls), 3)
            self.assertTrue(all(box.y == expected_box_y for box in box_calls))
            self.assertTrue(all(0 <= box.y and box.y + box.height <= 1080 for box in box_calls))


if __name__ == "__main__":
  unittest.main()
