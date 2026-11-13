"""BluePilot's onroad overrides, called the way upstream's renderers call them.

No window here: fonts and textures are stubbed, and each case stops before anything is drawn.
"""
import unittest
from types import SimpleNamespace
from unittest import mock

import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app

RECT = rl.Rectangle(0, 0, 2160, 1080)


class TestOnroadOverrides(unittest.TestCase):
  def setUp(self):
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
    with mock.patch.object(ui_state, "started_frame", 1):
      renderer._render(RECT)  # no calibration received since start: returns before drawing

  def test_tici_render_reaches_draw_path(self):
    from openpilot.selfdrive.ui.bp.onroad.model_renderer_bp import ModelRendererBP
    renderer = self._plain_model_renderer(ModelRendererBP)
    renderer._transform_dirty = False  # no model data: skip the update and go straight to drawing
    with mock.patch.object(renderer, "_draw_path", wraps=renderer._draw_path) as draw_path:
      renderer._render(RECT)
    draw_path.assert_called_once_with()

  def test_tici_hud_places_the_wheel_button_like_bp7(self):
    from openpilot.selfdrive.ui.bp.onroad.hud_renderer_bp import HudRendererBP
    with mock.patch("openpilot.selfdrive.ui.bp.onroad.hud_renderer_bp.ExpButtonBP") as exp_button:
      hud = HudRendererBP()
    exp_button.assert_called_once_with(192, 144)  # BP7's button and wheel icon sizes
    hud.is_cruise_available = False
    hud._draw_current_speed = mock.Mock()
    for name in ("developer_ui", "road_name_renderer", "speed_limit_renderer", "smart_cruise_control_renderer",
                 "turn_signal_controller", "circular_alerts_renderer", "rocket_fuel"):
      setattr(hud, name, mock.Mock())
    with mock.patch("pyray.draw_rectangle_gradient_v"):
      hud._render(RECT)
    # BP7's sizes: border 30, button 192
    (button_rect,), _ = hud._exp_button.render.call_args
    self.assertEqual((button_rect.x, button_rect.y, button_rect.width, button_rect.height), (2160 - 30 - 192, 30, 192, 192))

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
    with mock.patch.object(ui_state, "sm", sm), mock.patch.object(ui_state, "CP", None):
      state._update_torque_filter_bp()
    # actual 1.0 m/s^2, desired 1.2, roll 0.04 * 9.81 * 0.25 = 0.0981; (1.0 - 0.0981 + 0.2) / 3.0
    (value,), _ = state._torque_filter.update.call_args
    self.assertAlmostEqual(value, 0.36730, places=4)


if __name__ == "__main__":
  unittest.main()
