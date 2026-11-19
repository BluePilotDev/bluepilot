"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from pathlib import Path
from types import SimpleNamespace

from openpilot.cereal import custom
from openpilot.common.test import OpenpilotTestCase
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
