"""BluePilot's onroad overrides, called the way upstream's renderers call them.

No window here: fonts and textures are stubbed, and each case stops before anything is drawn.
"""
import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pyray as rl

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA
from openpilot.system.ui.lib.application import gui_app

RECT = rl.Rectangle(0, 0, 2160, 1080)
MICI_RECT = rl.Rectangle(10, 20, 536, 240)  # off the origin: upstream's bar is relative to the rect
# comma four test camera: center (268, 120), focal 500 px; car space x forward, y left, z down
MICI_TRANSFORM = np.array([[268, -500, 0], [120, 0, 500], [1, 0, 0]], dtype=np.float32)
ROAD_Z = 1.2  # camera height: the road is 1.2 m below it


class _SM(dict):
  valid = {"radarState": True}


def _lead_frame(present=True, available=True, d_rel=40.0):
  lead = SimpleNamespace(present=present, dRel=d_rel, yRel=0.0, vRel=0.0)
  no_lead = SimpleNamespace(present=False, dRel=0.0, yRel=0.0, vRel=0.0)
  return _SM(longitudinalPlan=SimpleNamespace(longitudinalPlanSource=None),
             radarState=SimpleNamespace(leadOne=lead, leadTwo=no_lead),
             selfdriveState=SimpleNamespace(enabled=available, engageable=available),
             carState=SimpleNamespace(brakePressed=False))


class TestOnroadOverrides(OpenpilotTestCase):
  def setUp(self):
    from openpilot.selfdrive.ui.ui_state import ui_state  # inside the test's params prefix: building it writes params
    self.ui_state = ui_state
    for patch in (mock.patch.object(gui_app, "font", return_value=rl.Font()),
                  mock.patch.object(gui_app, "texture", return_value=rl.Texture()),
                  mock.patch.object(ui_state, "rainbow_path", False)):
      patch.start()
      self.addCleanup(patch.stop)

  def _plain_model_renderer(self, cls):
    renderer = cls()
    renderer._rad_racer = False
    renderer._theme_pack = None
    return renderer  # empty path: the stock branch returns before drawing

  def test_mici_draw_path_takes_no_argument(self):
    from openpilot.selfdrive.ui.bp.mici.onroad.model_renderer_bp import ModelRendererBP
    self._plain_model_renderer(ModelRendererBP)._draw_path()  # upstream mici _render calls self._draw_path()

  def test_tici_draw_path_takes_no_argument(self):
    from openpilot.selfdrive.ui.bp.onroad.model_renderer_bp import ModelRendererBP
    self._plain_model_renderer(ModelRendererBP)._draw_path()  # upstream tici _render calls self._draw_path()

  def test_tici_render_reads_the_calibration_service_ui_subscribes_to(self):
    from openpilot.selfdrive.ui.bp.onroad.model_renderer_bp import ModelRendererBP
    renderer = self._plain_model_renderer(ModelRendererBP)
    with mock.patch.object(self.ui_state, "started_frame", 1):
      renderer._render(RECT)  # no calibration received since start: returns before drawing

  def test_tici_render_reaches_draw_path(self):
    from openpilot.selfdrive.ui.bp.onroad.model_renderer_bp import ModelRendererBP
    renderer = self._plain_model_renderer(ModelRendererBP)
    renderer._transform_dirty = False  # no model data: skip the update and go straight to drawing
    with mock.patch.object(renderer, "_draw_path", wraps=renderer._draw_path) as draw_path:
      renderer._render(RECT)
    draw_path.assert_called_once_with()

  def _tici_path_edge_renderer(self):
    from openpilot.selfdrive.ui.bp.onroad.model_renderer_bp import ModelRendererBP
    renderer = ModelRendererBP.__new__(ModelRendererBP)
    renderer._rect = RECT
    renderer._path = SimpleNamespace(projected_points=np.array([  # ty: ignore[invalid-assignment]  # focused draw seam
      [900.0, 300.0], [850.0, 700.0], [800.0, 1000.0],
      [1360.0, 1000.0], [1310.0, 700.0], [1260.0, 300.0],
    ], dtype=np.float32))
    renderer._theme_pack = None
    renderer._model_path_color = "#FF334B"
    renderer._disable_lane_line_status_color = False
    return renderer

  def test_tici_rainbow_path_suppresses_custom_edge_color(self):
    from openpilot.selfdrive.ui.bp.onroad import model_renderer_bp
    from openpilot.selfdrive.ui.ui_state import UIStatus
    renderer = self._tici_path_edge_renderer()

    with (mock.patch.object(self.ui_state, "rainbow_path", True),
          mock.patch.object(self.ui_state, "status", UIStatus.ENGAGED),
          mock.patch.object(model_renderer_bp, "model_path_color_rgb", return_value=(255, 51, 75)) as custom_color,
          mock.patch.object(model_renderer_bp, "draw_polygon"),
          mock.patch.object(rl, "draw_line_ex")):
      renderer._draw_path_edges()

    custom_color.assert_not_called()

  def test_tici_custom_color_replaces_disengaged_path_edges(self):
    from openpilot.selfdrive.ui.bp.onroad import model_renderer_bp
    from openpilot.selfdrive.ui.ui_state import UIStatus
    renderer = self._tici_path_edge_renderer()

    with (mock.patch.object(self.ui_state, "rainbow_path", False),
          mock.patch.object(self.ui_state, "status", UIStatus.DISENGAGED),
          mock.patch.object(model_renderer_bp, "model_path_color_rgb", return_value=(255, 51, 75)),
          mock.patch.object(model_renderer_bp, "draw_polygon") as draw_polygon,
          mock.patch.object(rl, "draw_line_ex")):
      renderer._draw_path_edges()

    edge_colors = [call.args[2] for call in draw_polygon.call_args_list]
    self.assertGreater(len(edge_colors), 0)
    self.assertTrue(all((color.r, color.g, color.b) == (255, 51, 75) for color in edge_colors))

  def test_tici_hud_places_the_wheel_like_bp7(self):
    from openpilot.selfdrive.ui.bp.onroad.hud_renderer_bp import HudRendererBP
    with mock.patch("openpilot.selfdrive.ui.bp.onroad.hud_renderer_bp.SteeringWheelBP") as steering_wheel:
      hud = HudRendererBP()
    steering_wheel.assert_called_once_with(144)  # BP7's wheel icon size
    hud.is_cruise_available = False
    hud._draw_current_speed = mock.Mock()
    for name in ("developer_ui", "road_name_renderer", "speed_limit_renderer", "smart_cruise_control_renderer",
                 "turn_signal_controller", "circular_alerts_renderer", "rocket_fuel"):
      setattr(hud, name, mock.Mock())
    with mock.patch("pyray.draw_rectangle_gradient_v"):
      hud._render(RECT)
    # BP7's sizes: border 30, button 192
    (button_rect,), _ = hud._steering_wheel.render.call_args
    self.assertEqual((button_rect.x, button_rect.y, button_rect.width, button_rect.height), (2160 - 30 - 192, 30, 192, 192))

  def _mici_rad_racer(self):
    """comma four renderer on a straight flat road, and the Rad Racer theme with stub textures."""
    from openpilot.selfdrive.ui.bp.mici.onroad.model_renderer_bp import ModelRendererBP
    from openpilot.selfdrive.ui.bp.mici.onroad.rad_racer_mici import RadRacerThemeMici
    renderer = self._plain_model_renderer(ModelRendererBP)
    renderer._path.raw_points = np.array([[x, 0.0, 0.0] for x in range(0, 101, 5)], dtype=np.float32)
    renderer._camera_offset = 0.0
    renderer._path_offset_z = ROAD_Z
    renderer.set_transform(MICI_TRANSFORM)
    theme = RadRacerThemeMici()
    theme._loaded = True
    theme._ego_tex = SimpleNamespace(width=24, height=12)
    theme._lead_tex = [SimpleNamespace(width=16, height=8), SimpleNamespace(width=16, height=8)]
    return renderer, theme

  def _lead_sprites(self, renderer, theme, sm):
    """Run upstream's lead update for one frame, then the theme's foreground; return the lead sprites' rects."""
    with mock.patch.object(self.ui_state, "sm", sm), mock.patch.object(rl, "draw_texture_pro") as draw:
      renderer._update_leads(sm)
      theme.render_foreground(MICI_RECT, renderer, MICI_RECT.y + MICI_RECT.height - 4)
    return [args[2] for args, _ in draw.call_args_list if any(args[0] is t for t in theme._lead_tex)]

  def test_mici_rad_racer_lead_sits_on_upstream_bar(self):
    renderer, theme = self._mici_rad_racer()
    (dst,) = self._lead_sprites(renderer, theme, _lead_frame(d_rel=40.0))
    # the lead's rear at road level, straight ahead: x = 40 m + radar-to-camera, 1.2 m below the camera
    x = 40.0 + RADAR_TO_CAMERA
    expected = (MICI_RECT.x + 268.0, MICI_RECT.y + 120.0 + 500.0 * ROAD_Z / x)
    self.assertAlmostEqual(dst.x + dst.width / 2, expected[0], places=2)  # bottom center of the sprite
    self.assertAlmostEqual(dst.y + dst.height, expected[1], places=2)

  def test_mici_rad_racer_no_lead_sprite_without_a_lead(self):
    renderer, theme = self._mici_rad_racer()
    self.assertEqual(self._lead_sprites(renderer, theme, _lead_frame(present=False)), [])

  def test_mici_rad_racer_no_lead_sprite_on_a_bar_upstream_stopped_showing(self):
    renderer, theme = self._mici_rad_racer()
    self.assertEqual(len(self._lead_sprites(renderer, theme, _lead_frame())), 1)
    # openpilot unavailable: upstream keeps the last bar and fades it out, though radar still has the lead
    self.assertEqual(self._lead_sprites(renderer, theme, _lead_frame(available=False)), [])

  def _mici_bars_drawn(self, rad_racer):
    renderer, theme = self._mici_rad_racer()
    self._lead_sprites(renderer, theme, _lead_frame())  # gives lead one a bar
    renderer._rad_racer = rad_racer
    with mock.patch("openpilot.selfdrive.ui.mici.onroad.model_renderer.draw_polygon") as draw_polygon:
      renderer._draw_lead_indicator()  # upstream mici _render calls self._draw_lead_indicator()
    return draw_polygon.call_count

  def test_mici_rad_racer_hides_upstream_lead_bar(self):
    self.assertEqual(self._mici_bars_drawn(rad_racer=True), 0)

  def test_mici_other_themes_keep_upstream_lead_bar(self):
    self.assertGreater(self._mici_bars_drawn(rad_racer=False), 0)

  def test_torque_bar_estimate_on_angle_control_cars(self):
    from openpilot.selfdrive.ui.bp.onroad.torque_bar_state_bp import TorqueBarStateBP
    state = TorqueBarStateBP()
    state._torque_filter = mock.Mock()
    sm = {
      "controlsState": SimpleNamespace(lateralControlState=SimpleNamespace(which=lambda: "angleState"),
                                       curvature=0.01, desiredCurvature=0.012),
      "carState": SimpleNamespace(vEgo=10.0),
      "vehicleParameters": SimpleNamespace(roll=0.04),
      "carControl": SimpleNamespace(latActive=True),
    }
    with mock.patch.object(self.ui_state, "sm", sm), mock.patch.object(self.ui_state, "CP", None):
      state._update_torque_filter_bp()
    # actual 1.0 m/s^2, desired 1.2, roll 0.04 * 9.81 * 0.25 = 0.0981; (1.0 - 0.0981 + 0.2) / 3.0
    (value,), _ = state._torque_filter.update.call_args
    self.assertAlmostEqual(value, 0.36730, places=4)


if __name__ == "__main__":
  unittest.main()
