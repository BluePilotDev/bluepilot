"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from openpilot.sunnypilot.selfdrive.controls.tests.test_lateral_params import _Written
from opendbc.bluepilot_lateral.core.values_ext import BP_LATERAL_PARAMS

LANEFULL = ("enable_lane_full_mode_curv", "enable_lane_full_mode_ang")
TINYGRAD = custom.ModelManagerSP.Runner.tinygrad
STOCK = custom.ModelManagerSP.Runner.stock


def _ext(params):
  ext = ControlsExt.__new__(ControlsExt)
  ext.params = params
  ext._bp_unknown_keys_logged = set()
  return ext


def _published(ext) -> dict[str, bytes]:
  return {e["key"]: e["value"] for e in ext.published_params()}


class TestLanefullOffWhileBlendfulRuns(OpenpilotTestCase):
  """BluePilot (ford-op/bp-sunnypilot#45): while Blendful runs (its toggle on and a tinygrad model active), both
  Lanefull settings reach the car code as off, whatever is saved. Nothing saved is written."""

  def setup_method(self):
    self.params = Params()
    for key in (*BP_LATERAL_PARAMS, "Blendful", "ModelRunnerTypeCache"):
      self.params.remove(key)

  def _publish(self, blendful, runner, lanefull):
    return _published(_ext(_Written(self.params, Blendful=blendful, ModelRunnerTypeCache=runner,
                                    **dict.fromkeys(LANEFULL, lanefull))))

  def test_saved_on_is_off_only_while_blendful_runs(self):
    for blendful in (True, False):
      for runner in (TINYGRAD, STOCK):
        expected = b"0" if (blendful and runner == TINYGRAD) else b"1"
        published = self._publish(blendful, runner, lanefull=True)
        for key in LANEFULL:
          with self.subTest(blendful=blendful, runner=runner, key=key):
            self.assertEqual(published[key], expected)

  def test_saved_off_stays_off(self):
    for blendful in (True, False):
      for runner in (TINYGRAD, STOCK):
        published = self._publish(blendful, runner, lanefull=False)
        for key in LANEFULL:
          with self.subTest(blendful=blendful, runner=runner, key=key):
            self.assertEqual(published[key], b"0")

  def test_runner_never_cached_is_not_blendful(self):
    published = _published(_ext(_Written(self.params, Blendful=True, **dict.fromkeys(LANEFULL, True))))
    for key in LANEFULL:
      self.assertEqual(published[key], b"1")

  def test_other_settings_untouched(self):
    others = {"LC_PID_gain_UI_curv": 2.5, "enable_lane_positioning_ang": True, "custom_path_offset_ang": 0.3}
    on = _published(_ext(_Written(self.params, Blendful=True, ModelRunnerTypeCache=TINYGRAD, **others)))
    off = _published(_ext(_Written(self.params, Blendful=False, ModelRunnerTypeCache=TINYGRAD, **others)))
    self.assertEqual(set(on), set(off))
    self.assertEqual({k: v for k, v in on.items() if k not in LANEFULL}, {k: v for k, v in off.items() if k not in LANEFULL})

  def test_saved_settings_not_written(self):
    self.params.put_bool("Blendful", True, block=True)
    self.params.put("ModelRunnerTypeCache", int(TINYGRAD), block=True)
    for key in LANEFULL:
      self.params.put_bool(key, True, block=True)
    published = _published(_ext(self.params))
    for key in LANEFULL:
      self.assertEqual(published[key], b"0")
      self.assertTrue(Params().get_bool(key))
    self.assertTrue(Params().get_bool("Blendful"))
    self.assertEqual(Params().get("ModelRunnerTypeCache"), TINYGRAD)

  def test_blendful_is_read_on_the_refresh_not_every_frame(self):
    reads = []

    class Counting(_Written):
      def get(self, key, return_default=False):
        reads.append(key)
        return super().get(key, return_default=return_default)

    ext = _ext(Counting(self.params, Blendful=False, ModelRunnerTypeCache=TINYGRAD, **dict.fromkeys(LANEFULL, True)))
    self.assertEqual(_published(ext)["enable_lane_full_mode_ang"], b"1")
    n = len(reads)
    ext.params._written["Blendful"] = True  # toggled on between refreshes
    for _ in range(20):
      self.assertEqual(_published(ext)["enable_lane_full_mode_ang"], b"1")
    self.assertEqual(len(reads), n)
    ext._bp_params_cache_time -= 1.0
    self.assertEqual(_published(ext)["enable_lane_full_mode_ang"], b"0")
