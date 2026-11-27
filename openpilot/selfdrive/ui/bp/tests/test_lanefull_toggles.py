"""Lanefull toggles on the comma 3X and comma four lateral settings (#285): the angle-mode toggle, and both Lanefull
toggles shown as off and locked while Blendful runs, without writing the saved settings.

No window here: fonts and textures are stubbed, and a tap is fed through the widget's own event handling with
drawing patched out.
"""
from unittest import mock

import pyray as rl

from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.system.ui.lib.application import gui_app, MouseEvent

LANEFULL_CURV, LANEFULL_ANGLE = "enable_lane_full_mode_curv", "enable_lane_full_mode_ang"
TINYGRAD, STOCK = int(custom.ModelManagerSP.Runner.tinygrad), int(custom.ModelManagerSP.Runner.stock)
REASON = "Off while Blendful is on"


def _save(**values):
  params = Params()
  for key, value in values.items():
    if isinstance(value, bool):
      params.put_bool(key, value, block=True)
    else:
      params.put(key, value, block=True)


def _tap(widget) -> list[tuple[str, bool]]:
  """Press and release inside the widget in one frame, through Widget.render's event handling (drawing patched out).
  Returns the bool params the tap wrote, as (key, value)."""
  pos = rl.Vector2(20, 20)
  events = [MouseEvent(pos, 0, True, False, True, 0.0), MouseEvent(pos, 0, False, True, False, 0.0)]
  writes: list[tuple[str, bool]] = []
  real_put_bool = Params.put_bool

  def put_bool(self, key, val, block=False):
    writes.append((key, val))
    real_put_bool(self, key, val, block=True)

  with mock.patch.object(type(gui_app), "mouse_events", new_callable=mock.PropertyMock, return_value=events), \
       mock.patch.object(Params, "put_bool", put_bool):
    widget.render(rl.Rectangle(0, 0, 160, 80))  # the size of the 3X toggle, so the press lands on it
  return writes


class _LanefullCase(OpenpilotTestCase):
  def setUp(self):
    from openpilot.selfdrive.ui.ui_state import ui_state
    # ui_state is a singleton: its Params can belong to an earlier test's params directory
    for patch in (mock.patch.object(gui_app, "font", return_value=rl.Font()),
                  mock.patch.object(gui_app, "texture", return_value=rl.Texture()),
                  mock.patch.object(ui_state, "params", Params()),
                  mock.patch("pyray.get_time", return_value=0.0)):
      patch.start()
      self.addCleanup(patch.stop)
    # angle mode, lane positioning on in both modes, both Lanefull settings saved on, Blendful off
    _save(FordPrefLateralControl=1, enable_lane_positioning_ang=True, enable_lane_positioning_curv=True,
          enable_lane_full_mode_curv=True, enable_lane_full_mode_ang=True, Blendful=False, ModelRunnerTypeCache=TINYGRAD)


class TestTiciLanefull(_LanefullCase):
  def _layout(self):
    from openpilot.selfdrive.ui.bp.layouts.settings.bluepilot import BluePilotLayout
    layout = BluePilotLayout()
    layout.show_event()
    return layout

  def _tap(self, item):
    with mock.patch.object(item.action_item.toggle, "_render"):
      return _tap(item.action_item)

  def test_angle_toggle_sits_under_angle_lane_positioning(self):
    layout = self._layout()
    items = layout._angle_header._managed_items
    self.assertIs(items[items.index(layout._enable_lane_positioning_ang) + 1], layout._enable_lane_full_mode_ang)
    self.assertEqual(layout._enable_lane_full_mode_ang.title, "Enable Lanefull Mode")

  def test_angle_toggle_reflects_and_writes_the_saved_setting(self):
    for saved in (True, False):
      _save(enable_lane_full_mode_ang=saved)
      item = self._layout()._enable_lane_full_mode_ang
      with self.subTest(saved=saved):
        self.assertEqual(item.action_item.get_state(), saved)
        self.assertTrue(item.action_item.enabled)
        self.assertEqual(self._tap(item), [(LANEFULL_ANGLE, not saved)])
        self.assertEqual(Params().get_bool(LANEFULL_ANGLE), not saved)

  def test_angle_toggle_needs_angle_mode_and_angle_lane_positioning(self):
    for mode, lane_pos, enabled in ((1, True, True), (1, False, False), (0, True, False)):
      _save(FordPrefLateralControl=mode, enable_lane_positioning_ang=lane_pos)
      item = self._layout()._enable_lane_full_mode_ang
      with self.subTest(mode=mode, lane_pos=lane_pos):
        self.assertEqual(item.action_item.enabled, enabled)
        self.assertEqual(self._tap(item), [(LANEFULL_ANGLE, False)] if enabled else [])

  # each Lanefull toggle in the mode where it is otherwise enabled: (key, FordPrefLateralControl, layout attribute)
  LANEFULL = ((LANEFULL_CURV, 0, "_enable_lane_full_mode"), (LANEFULL_ANGLE, 1, "_enable_lane_full_mode_ang"))

  def test_both_lanefull_toggles_locked_off_while_blendful_runs(self):
    _save(Blendful=True, ModelRunnerTypeCache=TINYGRAD)
    for key, mode, attr in self.LANEFULL:
      _save(FordPrefLateralControl=mode)
      item = getattr(self._layout(), attr)
      with self.subTest(key=key):
        self.assertFalse(item.action_item.get_state())
        self.assertFalse(item.action_item.enabled)
        self.assertIn(REASON, item.description)
        self.assertEqual(self._tap(item), [])
        self.assertTrue(Params().get_bool(key))  # the saved setting is untouched

  def test_no_lock_unless_blendful_is_on_and_running(self):
    for blendful, runner in ((True, STOCK), (False, TINYGRAD)):
      _save(Blendful=blendful, ModelRunnerTypeCache=runner)
      for key, mode, attr in self.LANEFULL:
        _save(FordPrefLateralControl=mode, **{key: True})
        item = getattr(self._layout(), attr)
        with self.subTest(blendful=blendful, runner=runner, key=key):
          self.assertTrue(item.action_item.get_state())
          self.assertTrue(item.action_item.enabled)
          self.assertNotIn(REASON, item.description)
          self.assertEqual(self._tap(item), [(key, False)])

  def test_saved_value_returns_when_blendful_stops(self):
    for key, mode, attr in self.LANEFULL:
      _save(FordPrefLateralControl=mode, Blendful=True, ModelRunnerTypeCache=TINYGRAD)
      layout = self._layout()
      item = getattr(layout, attr)
      self.assertFalse(item.action_item.get_state())
      _save(Blendful=False)
      layout.show_event()
      with self.subTest(key=key):
        self.assertTrue(item.action_item.get_state())
        self.assertTrue(item.action_item.enabled)
        self.assertNotIn(REASON, item.description)


class TestMiciLanefull(_LanefullCase):
  LANEFULL = ((LANEFULL_CURV, 0, "enable_lane_full_mode"), (LANEFULL_ANGLE, 1, "enable_lane_full_mode_ang"))

  def _layout(self):
    from openpilot.selfdrive.ui.bp.mici.layouts.settings.lateral_mici import LateralLayoutMici
    layout = LateralLayoutMici()
    layout.show_event()
    return layout

  def _tap(self, item):
    with mock.patch.object(item, "_render"):
      return _tap(item)

  def test_angle_toggle_sits_after_angle_lane_positioning(self):
    layout = self._layout()
    items = layout._scroller.items
    self.assertIs(items[items.index(layout.enable_lane_positioning_ang) + 1], layout.enable_lane_full_mode_ang)
    self.assertEqual(layout.enable_lane_full_mode_ang.text, "Enable Lanefull Mode")

  def test_angle_toggle_reflects_and_writes_the_saved_setting(self):
    for saved in (True, False):
      _save(enable_lane_full_mode_ang=saved)
      item = self._layout().enable_lane_full_mode_ang
      with self.subTest(saved=saved):
        self.assertEqual(item._checked, saved)
        self.assertTrue(item.is_visible and item.enabled)
        self.assertEqual(self._tap(item), [(LANEFULL_ANGLE, not saved)])
        self.assertEqual(Params().get_bool(LANEFULL_ANGLE), not saved)

  def test_angle_toggle_needs_angle_mode_and_angle_lane_positioning(self):
    _save(enable_lane_positioning_ang=False)
    item = self._layout().enable_lane_full_mode_ang
    self.assertTrue(item.is_visible)
    self.assertFalse(item.enabled)
    self.assertEqual(self._tap(item), [])
    _save(FordPrefLateralControl=0, enable_lane_positioning_ang=True)
    self.assertFalse(self._layout().enable_lane_full_mode_ang.is_visible)  # angle items are hidden in curvature mode

  def test_both_lanefull_toggles_locked_off_while_blendful_runs(self):
    _save(Blendful=True, ModelRunnerTypeCache=TINYGRAD)
    for key, mode, attr in self.LANEFULL:
      _save(FordPrefLateralControl=mode)
      item = getattr(self._layout(), attr)
      with self.subTest(key=key):
        self.assertTrue(item.is_visible)
        self.assertFalse(item._checked)
        self.assertFalse(item.enabled)
        self.assertEqual(item.value, REASON)
        self.assertEqual(self._tap(item), [])
        self.assertTrue(Params().get_bool(key))

  def test_no_lock_unless_blendful_is_on_and_running(self):
    for blendful, runner in ((True, STOCK), (False, TINYGRAD)):
      _save(Blendful=blendful, ModelRunnerTypeCache=runner)
      for key, mode, attr in self.LANEFULL:
        _save(FordPrefLateralControl=mode, **{key: True})
        item = getattr(self._layout(), attr)
        with self.subTest(blendful=blendful, runner=runner, key=key):
          self.assertTrue(item._checked)
          self.assertTrue(item.enabled)
          self.assertEqual(item.value, "")
          self.assertEqual(self._tap(item), [(key, False)])

  def test_saved_value_returns_when_blendful_stops(self):
    for key, mode, attr in self.LANEFULL:
      _save(FordPrefLateralControl=mode, Blendful=True, ModelRunnerTypeCache=TINYGRAD)
      layout = self._layout()
      item = getattr(layout, attr)
      self.assertFalse(item._checked)
      _save(Blendful=False)
      layout.show_event()
      with self.subTest(key=key):
        self.assertTrue(item._checked)
        self.assertTrue(item.enabled)
        self.assertEqual(item.value, "")
