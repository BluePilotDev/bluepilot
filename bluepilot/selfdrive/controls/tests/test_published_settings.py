from collections import defaultdict
from unittest import mock

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt

SETTINGS = ("disable_BP_long_UI", "disable_downhill_comp_UI", "send_hands_free_cluster_msg")


class _SM(dict):
  seen = defaultdict(bool)


class _Written:
  """Params with some keys written, layered over Params ."""
  def __init__(self, params: Params, **written):
    self._params, self._written = params, written

  def get(self, key, return_default=False):
    return self._written[key] if key in self._written else self._params.get(key, return_default=return_default)


def _published(params, bluepilot: bool) -> dict[str, bytes]:
  """CarControlSP.params as controlsd_ext builds it for a Ford (lateralInputsRequired)."""
  ext = ControlsExt.__new__(ControlsExt)  # state_control_ext needs only these
  ext.params = params
  ext._bp_unknown_keys_logged = set()
  ext.CP_SP = custom.CarParamsSP.new_message(lateralInputsRequired=True)
  sm = _SM({s: getattr(messaging.new_message(s), s) for s in ('radarState', 'selfdriveStateSP', 'modelV2', 'vehicleParameters')})
  with mock.patch("openpilot.common.bluepilot.is_bluepilot", return_value=bluepilot):
    CC_SP = ext.state_control_ext(sm)
  return {p.key: p.value for p in CC_SP.params}


class TestBluePilotSettingsPublished(OpenpilotTestCase):
  """The longitudinal and cluster settings the Ford car code on bp-full reads from CarControlSP.params."""

  def setup_method(self):
    self.params = Params()
    for key in SETTINGS:
      self.params.remove(key)

  def test_never_set_publishes_the_default(self):
    published = _published(self.params, bluepilot=True)
    for key in SETTINGS:
      self.assertEqual(published[key], b"0", key)

  def test_menu_value_is_published(self):
    for key in SETTINGS:
      self.assertEqual(_published(_Written(self.params, **{key: True}), bluepilot=True)[key], b"1", key)

  def test_not_published_when_not_bluepilot(self):
    published = _published(self.params, bluepilot=False)
    for key in SETTINGS:
      self.assertNotIn(key, published)
    self.assertEqual(published, {})  # the lateral library reads no setting while driving


class TestWithoutBpxCarCode(OpenpilotTestCase):
  def test_missing_car_code_publishes_nothing(self):
    import sys
    from unittest import mock
    from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
    ext = ControlsExt.__new__(ControlsExt)
    with mock.patch("openpilot.common.bluepilot.is_bluepilot", return_value=True), \
         mock.patch.dict(sys.modules, {"opendbc.sunnypilot.car.ford.bluepilot.hud_ext": None}):
      assert ext.read_bpx_params() == []


class TestPublishedSettingsAreCached(OpenpilotTestCase):
  def test_bpx_settings_are_cached(self):
    calls = []

    class Counting(_Written):
      def get(self, key, return_default=False):
        calls.append(key)
        return super().get(key, return_default=return_default)

    ext = ControlsExt.__new__(ControlsExt)
    ext.params = Counting(Params())
    ext._bp_unknown_keys_logged = set()
    with mock.patch("openpilot.common.bluepilot.is_bluepilot", return_value=True):
      first = ext.published_params()
      keys = {e["key"] for e in first}
      for key in SETTINGS:
        self.assertTrue(key in keys, key)
      reads = len(calls)
      for _ in range(20):  # 20 more frames: nothing is read again
        self.assertIs(ext.published_params(), first)
      self.assertEqual(len(calls), reads)
      ext._bp_params_cache_time -= 1.0  # past the refresh period: everything is read once more
      ext.published_params()
      self.assertEqual(len(calls), 2 * reads)

