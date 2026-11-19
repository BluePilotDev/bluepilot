"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from pathlib import Path
from types import SimpleNamespace

from openpilot.cereal import custom
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_LATERAL_ACCEL_NO_ROLL, clip_curvature
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from opendbc.bluepilot_lateral.params.manifest import params_keys_lines

PARAMS_KEYS_H = Path(__file__).resolve().parents[4] / "common" / "params_keys.h"


def _registered(text: str) -> set[str]:
  """Every entry of params_keys.h, whitespace normalised."""
  return {" ".join(line.split()) for line in text.splitlines() if line.strip().startswith("{")}


class TestParamsKeysHoldTheManifest(OpenpilotTestCase):
  """Every setting the lateral library reads is registered in params_keys.h with its type and default."""

  def test_manifest_lines_are_registered(self):
    registered = _registered(PARAMS_KEYS_H.read_text())
    for line in params_keys_lines():
      self.assertIn(" ".join(line.split()), registered, "paste the lines `python -m opendbc.bluepilot_lateral.params.manifest` prints")


class _SM(dict):
  """The parts of a SubMaster fill_lateral_inputs reads."""
  def __init__(self, panda_states, seen=True):
    model = SimpleNamespace(laneLines=[], laneLineProbs=[], laneLineStds=[])
    super().__init__(pandaStates=panda_states, modelV2=model, vehicleParameters=None, lateralDelay=None)
    self.seen = {"pandaStates": seen, "modelV2": False, "vehicleParameters": False}


def _panda(controls_allowed=False, lateral=False, blocked=0):
  return SimpleNamespace(controlsAllowed=controls_allowed, controlsAllowedLateral=lateral, safetyTxBlocked=blocked)


class TestPandaInputs(OpenpilotTestCase):
  """What the panda allows and has blocked reaches the Ford lateral controller every frame."""

  def _fill(self, sm):
    li = custom.LateralInputs.new_message()
    ControlsExt.fill_lateral_inputs(li, sm)
    return li

  def test_allowed_until_a_panda_state_was_seen(self):
    self.assertTrue(self._fill(_SM([], seen=False)).pandaLateralAllowed)
    self.assertTrue(self._fill(_SM([])).pandaLateralAllowed)

  def test_every_panda_must_allow(self):
    self.assertTrue(self._fill(_SM([_panda(controls_allowed=True)])).pandaLateralAllowed)
    self.assertTrue(self._fill(_SM([_panda(lateral=True)])).pandaLateralAllowed)
    self.assertFalse(self._fill(_SM([_panda()])).pandaLateralAllowed)
    self.assertFalse(self._fill(_SM([_panda(controls_allowed=True), _panda()])).pandaLateralAllowed)

  def test_blocked_messages_are_summed(self):
    self.assertEqual(self._fill(_SM([_panda(blocked=2), _panda(blocked=3)])).safetyTxBlocked, 5)


class TestLateralAccelLimit(OpenpilotTestCase):
  """The Ford steering module's own lateral acceleration limit reaches the plan's curvature clip."""

  def _limit(self, reported, seen=True):
    state = custom.ControllerStateBP.new_message()
    state.lateral.lateralAccelLimit = reported
    return ControlsExt.lateral_accel_limit(_Limits(state, seen))

  def test_reported_limit_is_used(self):
    self.assertAlmostEqual(self._limit(2.0), 2.0, places=5)
    self.assertAlmostEqual(self._limit(3.6), 3.6, places=5)

  def test_nothing_reported_is_openpilots_own(self):
    self.assertIsNone(self._limit(0.))   # the stock path, or before the first steering frame
    self.assertIsNone(self._limit(2.0, seen=False))  # not a Ford

  def test_clip_holds_the_plan_to_the_limit(self):
    v = 20.
    wanted = 2.8 / v ** 2
    curvature = 0.
    for _ in range(300):
      curvature, limited = clip_curvature(v, curvature, wanted, 0., 2.0)
    self.assertAlmostEqual(curvature, 2.0 / v ** 2)
    self.assertTrue(limited)
    for _ in range(300):
      curvature, limited = clip_curvature(v, curvature, wanted, 0.)
    self.assertAlmostEqual(curvature, wanted)
    self.assertFalse(limited)

  def test_road_roll_does_not_widen_the_steering_limit(self):
    """openpilot widens its own limit by the road's roll on the side the bank helps; the steering
    module's and the panda's limits are on the command itself, so the plan stays inside them on a
    banked curve, and the car controller has nothing left to clip."""
    v, roll = 20., 0.05  # 0.49 m/s^2 of the bank toward the turn
    for wanted in (3.4 / v ** 2, -3.4 / v ** 2):
      curvature = 0.
      for _ in range(300):
        curvature, limited = clip_curvature(v, curvature, wanted, roll, 2.0)
      self.assertAlmostEqual(abs(curvature), 2.0 / v ** 2)
      self.assertTrue(limited)
    # without a steering limit, openpilot's own roll-compensated clip, as upstream
    curvature = 0.
    for _ in range(300):
      curvature, _ = clip_curvature(v, curvature, 3.8 / v ** 2, roll)
    self.assertAlmostEqual(curvature, (MAX_LATERAL_ACCEL_NO_ROLL + roll * 9.81) / v ** 2, places=6)
    # where the bank works against the turn, openpilot's own limit is the lower one and still holds
    curvature = 0.
    for _ in range(300):
      curvature, _ = clip_curvature(v, curvature, -3.8 / v ** 2, roll, 2.9)
    self.assertAlmostEqual(curvature, (-MAX_LATERAL_ACCEL_NO_ROLL + roll * 9.81) / v ** 2, places=6)


class _Limits(dict):
  def __init__(self, state, seen):
    super().__init__(controllerStateBP=state)
    self.seen = {"controllerStateBP": seen}
