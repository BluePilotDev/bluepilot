"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Regression test for BluePilotDev/bluepilot#193: holding a Ford combo cruise button
# (Set+Inc/Set+Dec/Cancel+Res) produced a self-sustaining stream of spurious press/release
# events every frame, because the "did my signal change" check compared against derived,
# shared button_states slots instead of each raw signal's own previous value. See the
# comments in CarStateExt.update() for the full mechanism.

import unittest

from opendbc.car import Bus, structs
from opendbc.sunnypilot.car.ford.carstate_ext import CarStateExt

ButtonType = structs.CarState.ButtonEvent.Type


class _FakeCANParser:
  def __init__(self, vl):
    self.vl = vl


def _cs(cruise_enabled: bool) -> structs.CarState:
  ret = structs.CarState.new_message()
  ret.cruiseState.enabled = cruise_enabled
  return ret


def _parsers(dec_press: int, inc_press: int = 0, cncl_res_press: int = 0, on_off_press: int = 0):
  vl = {
    "Steering_Data_FD1": {
      "CcAslButtnSetDecPress": dec_press,
      "CcAslButtnSetIncPress": inc_press,
      "CcAslButtnCnclResPress": cncl_res_press,
      "CcButtnOnOffPress": on_off_press,
    },
  }
  return {Bus.pt: _FakeCANParser(vl)}


class TestComboButtonEvents(unittest.TestCase):
  def setUp(self):
    self.ext = CarStateExt(structs.CarParams(), structs.CarParamsSP())

  def _events(self):
    return [(e.type, e.pressed) for e in self.ext.button_events]

  def test_sustained_hold_emits_exactly_one_press_and_one_release(self):
    """The core #193 regression: holding Set+Dec should emit exactly one press event
    (on the rising edge) and exactly one release event (on the falling edge) -- not a
    fresh press/release pair every single frame it stays held."""
    ret = _cs(cruise_enabled=False)

    # Rising edge.
    self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=1))
    self.assertEqual(self._events(), [(ButtonType.setCruise, True)])

    # Held steady for many frames: must be silent every time.
    for _ in range(50):
      self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=1))
      self.assertEqual(self._events(), [], "spurious event while button held steady (#193)")

    # Cruise engages partway through the hold (as it would in the real failure) -- still silent.
    ret = _cs(cruise_enabled=True)
    for _ in range(20):
      self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=1))
      self.assertEqual(self._events(), [], "spurious event after cruise_enabled changed mid-hold (#193)")

    # Falling edge: exactly one release, for the event type that was actually emitted on press.
    self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=0))
    self.assertEqual(self._events(), [(ButtonType.setCruise, False)])

  def test_inc_and_dec_do_not_cross_contaminate_setcruise(self):
    """Holding Set+Dec (emitting setCruise) must not be disturbed by the Inc signal's own
    steady 0 state being reprocessed each frame -- the original bug's exact mechanism."""
    ret = _cs(cruise_enabled=False)
    self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=1, inc_press=0))
    self.assertEqual(self._events(), [(ButtonType.setCruise, True)])

    for _ in range(10):
      self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=1, inc_press=0))
      self.assertEqual(self._events(), [])

  def test_tap_while_engaged_emits_decel_not_setcruise(self):
    """A quick tap while cruise is already enabled should emit decelCruise, matching stock
    adjust-speed behavior -- unaffected by the fix."""
    ret = _cs(cruise_enabled=True)
    self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=1))
    self.assertEqual(self._events(), [(ButtonType.decelCruise, True)])

    self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=0))
    self.assertEqual(self._events(), [(ButtonType.decelCruise, False)])

  def test_cancel_resume_combo_unaffected(self):
    ret = _cs(cruise_enabled=True)
    self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=0, cncl_res_press=1))
    self.assertEqual(self._events(), [(ButtonType.cancel, True)])

    for _ in range(10):
      self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=0, cncl_res_press=1))
      self.assertEqual(self._events(), [])

    self.ext.update(ret, structs.CarStateSP(), _parsers(dec_press=0, cncl_res_press=0))
    self.assertEqual(self._events(), [(ButtonType.cancel, False)])


if __name__ == "__main__":
  unittest.main()
