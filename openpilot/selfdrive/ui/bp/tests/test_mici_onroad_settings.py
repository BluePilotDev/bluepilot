"""comma four onroad settings: Hide Onroad Fade, and settings read once a second instead of every frame.

No window here: fonts and textures are stubbed, and drawing calls are patched out.
"""
import unittest
from types import SimpleNamespace
from unittest import mock

import pyray as rl

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.system.ui.lib.application import gui_app

MICI_RECT = rl.Rectangle(0, 0, 536, 240)


class _SM(dict):
  """The few messages these widgets read, all alive and received after start."""
  @property
  def alive(self):
    return dict.fromkeys(self, True)

  @property
  def recv_frame(self):
    return dict.fromkeys(self, 1)


class TestMiciOnroadSettings(OpenpilotTestCase):
  def setUp(self):
    from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus  # inside the test's params prefix
    self.ui_state = ui_state
    for patch in (mock.patch.object(gui_app, "font", return_value=rl.Font()),
                  mock.patch.object(gui_app, "texture", return_value=rl.Texture()),
                  mock.patch.object(ui_state, "status", UIStatus.ENGAGED)):
      patch.start()
      self.addCleanup(patch.stop)

  def _road_view(self):
    from openpilot.selfdrive.ui.bp.mici.onroad.augmented_road_view_bp import MiciAugmentedRoadViewBP
    view = MiciAugmentedRoadViewBP()
    view._content_rect = MICI_RECT
    return view

  def _fade_draws(self, view) -> int:
    with mock.patch.object(rl, "draw_texture_ex") as draw:
      view._draw_bottom_fade()
    return sum(1 for args, _ in draw.call_args_list if args[0] is view._fade_texture)

  def test_fade_drawn_when_engaged(self):
    Params().put_bool("mici_hide_onroad_fade", False)
    self.assertEqual(self._fade_draws(self._road_view()), 1)

  def test_hide_onroad_fade_skips_the_fade(self):
    Params().put_bool("mici_hide_onroad_fade", True)
    self.assertEqual(self._fade_draws(self._road_view()), 0)

  @staticmethod
  def _count_reads(widget, attr: str, frames: int) -> dict[str, int]:
    """Step the widget's per-frame update `frames` times; count get_bool reads per key on its params object."""
    params = mock.Mock(wraps=getattr(widget, attr))
    setattr(widget, attr, params)
    for _ in range(frames):
      widget._update_state()
    counts: dict[str, int] = {}
    for args, _ in params.get_bool.call_args_list:
      counts[args[0]] = counts.get(args[0], 0) + 1
    return counts

  def test_road_view_reads_its_settings_once_a_second(self):
    view = self._road_view()
    counts = self._count_reads(view, "_bp_params", 120)
    self.assertEqual(counts.get("BPHideOnroadBorder"), 2)
    self.assertEqual(counts.get("mici_hide_onroad_fade"), 2)

  def test_road_view_fade_setting_takes_effect_at_the_next_refresh(self):
    Params().put_bool("mici_hide_onroad_fade", False)
    view = self._road_view()
    Params().put_bool("mici_hide_onroad_fade", True)
    for _ in range(59):
      view._update_state()
    self.assertEqual(self._fade_draws(view), 1)  # not yet refreshed
    view._update_state()
    self.assertEqual(self._fade_draws(view), 0)

  def _hud(self):
    """comma four HUD with upstream's per-frame update stubbed; the frame has brake lights on and a lateral mode."""
    from openpilot.selfdrive.ui.bp.mici.onroad.hud_renderer_bp import LateralMode, MiciHudRendererBP
    from openpilot.selfdrive.ui.sunnypilot.mici.onroad.hud_renderer import HudRendererSP
    sm = _SM(carStateBP=SimpleNamespace(brakeLightStatus=SimpleNamespace(dataAvailable=True, brakeLightsOn=True)),
             controllerStateBP=SimpleNamespace(activeLateralMode=LateralMode.angle))
    for patch in (mock.patch.object(HudRendererSP, "_update_state"), mock.patch.object(self.ui_state, "sm", sm)):
      patch.start()
      self.addCleanup(patch.stop)
    return MiciHudRendererBP()

  def test_hud_reads_its_settings_once_a_second(self):
    counts = self._count_reads(self._hud(), "_bp_params", 120)
    self.assertEqual(counts.get("ShowBrakeStatus"), 2)
    self.assertEqual(counts.get("BpShowLateralControl"), 2)

  def test_hud_settings_take_effect_at_the_next_refresh(self):
    Params().put_bool("ShowBrakeStatus", False)
    Params().put_bool("BpShowLateralControl", False)
    hud = self._hud()
    Params().put_bool("ShowBrakeStatus", True)
    Params().put_bool("BpShowLateralControl", True)
    for _ in range(59):
      hud._update_state()
    self.assertEqual((hud._brakes_on, hud.show_lateral_control), (False, False))  # not yet refreshed
    hud._update_state()
    self.assertEqual((hud._brakes_on, hud.show_lateral_control), (True, True))

  def _gauge(self):
    """comma four power-flow gauge on a frame with hybrid data."""
    from openpilot.selfdrive.ui.bp.mici.onroad.powerflow_gauge import MiciPowerflowGauge
    hybrid = SimpleNamespace(dataAvailable=True, throttleDemandPercent=50.0, powerFlowModeValue=1)
    for patch in (mock.patch.object(self.ui_state, "sm", _SM(carStateBP=SimpleNamespace(hybridDrive=hybrid))),
                  mock.patch.object(self.ui_state, "started_frame", 0)):
      patch.start()
      self.addCleanup(patch.stop)
    return MiciPowerflowGauge()

  def test_power_flow_gauge_reads_its_settings_once_a_second(self):
    counts = self._count_reads(self._gauge(), "_bp_params", 120)
    self.assertEqual(counts.get("FordPrefHybridPowerFlow"), 2)
    self.assertEqual(counts.get("FordPrefHybridPowerFlowAlternate"), 2)

  def test_power_flow_setting_takes_effect_at_the_next_refresh(self):
    Params().put_bool("FordPrefHybridPowerFlow", False)
    gauge = self._gauge()
    Params().put_bool("FordPrefHybridPowerFlow", True)
    for _ in range(59):
      gauge._update_state()
    self.assertFalse(gauge._should_render())  # not yet refreshed
    gauge._update_state()
    self.assertTrue(gauge._should_render())


if __name__ == "__main__":
  unittest.main()
