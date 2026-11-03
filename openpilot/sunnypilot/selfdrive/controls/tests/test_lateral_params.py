"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from pathlib import Path

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from opendbc.bluepilot_lateral.core.values_ext import BP_LATERAL_PARAMS
from opendbc.bluepilot_lateral.params.manifest import params_keys_lines

PARAMS_KEYS_H = Path(__file__).resolve().parents[4] / "common" / "params_keys.h"


def _published(params) -> dict[str, bytes]:
  ext = ControlsExt.__new__(ControlsExt)  # read_lateral_params only needs params and the log set
  ext.params = params
  ext._bp_unknown_keys_logged = set()
  return {e["key"]: e["value"] for e in ext.read_lateral_params()}


class _Written:
  """Params with some keys written. Writes inside the test's isolated params directory did not
  persist on a device, so written values are layered over Params instead of put()."""
  def __init__(self, params: Params, **written):
    self._params, self._written = params, written

  def get(self, key, return_default=False):
    return self._written[key] if key in self._written else self._params.get(key, return_default=return_default)


class TestReadLateralParams(OpenpilotTestCase):
  """BluePilot (#6): the Ford lateral gets the value the settings UI shows, including for a param
  that was never written on the device."""

  def setup_method(self):
    self.params = Params()
    for key in BP_LATERAL_PARAMS:
      self.params.remove(key)

  def test_unset_keys_publish_the_registry_default(self):
    published = _published(self.params)
    self.assertEqual(set(published), set(BP_LATERAL_PARAMS))
    self.assertEqual(published["enable_human_turn_detection_curv"], b"1")
    self.assertEqual(published["LC_PID_gain_UI_curv"], b"1.0")

  def test_decided_defaults(self):
    self.assertTrue(self.params.get("enable_human_turn_detection_curv", return_default=True))
    self.assertEqual(self.params.get("LC_PID_gain_UI_curv", return_default=True), 1.0)

  def test_written_value_wins(self):
    published = _published(_Written(self.params, LC_PID_gain_UI_curv=2.5, enable_human_turn_detection_curv=False))
    self.assertEqual(published["LC_PID_gain_UI_curv"], b"2.5")
    self.assertEqual(published["enable_human_turn_detection_curv"], b"0")


def _bluepilot_block(text: str) -> set[str]:
  """The entries under params_keys.h's BluePilot comment, up to the next blank line or the map's end."""
  lines = text.splitlines()
  start = next(i for i, line in enumerate(lines) if line.strip().startswith("// BluePilot Ford lateral control"))
  block = set()
  for line in lines[start + 1:]:
    entry = line.strip()
    if not entry or entry == "};":
      break
    if entry.startswith("{"):
      block.add(" ".join(entry.split()))
  return block


class TestParamsKeysMatchManifest(OpenpilotTestCase):
  """BluePilot's params_keys.h entries are the library manifest's, both ways: every setting registered
  with its type and default, and nothing the manifest dropped."""

  def test_block_matches_manifest(self):
    block = _bluepilot_block(PARAMS_KEYS_H.read_text())
    manifest = {" ".join(line.split()) for line in params_keys_lines()}
    self.assertEqual(block, manifest, "paste the lines `python -m opendbc.bluepilot_lateral.params.manifest` prints")
