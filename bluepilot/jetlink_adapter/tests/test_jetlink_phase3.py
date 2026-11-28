"""
JetLink phase 3 on BluePilot: modelDataV2BP (D2 b), JetLink's big model as both modelds saw it, for logs.

Runs with jetlink_repo present or absent; the test that reads JetLink's own STATES skips without it.
"""
import re
import sys
import types
import unittest
from pathlib import Path

from openpilot.common.basedir import BASEDIR

_JETLINK_REPO = Path(BASEDIR) / 'jetlink_repo'
if (_JETLINK_REPO / 'jetlink' / '__init__.py').is_file() and str(_JETLINK_REPO) not in sys.path:
  sys.path.append(str(_JETLINK_REPO))

import openpilot.cereal.messaging as messaging
from openpilot.cereal.services import SERVICE_LIST
from openpilot.bluepilot import jetlink_adapter
from openpilot.bluepilot.jetlink_adapter import telemetry

try:
  from jetlink.openpilot.interface import STATES
except ImportError:
  STATES = None

MODELDS = ('openpilot/selfdrive/modeld/modeld.py', 'openpilot/sunnypilot/modeld_v2/modeld.py')


class Hooks:
  def __init__(self, answer):
    self.answer = answer

  def state(self, model):
    if isinstance(self.answer, Exception):
      raise self.answer
    return self.answer


def sent(hooks, model, valid=True):
  """The message as a reader of the log gets it."""
  return messaging.log_from_bytes(telemetry.message(hooks, model, valid).to_bytes())


class TestModelDataV2BP(unittest.TestCase):
  def test_service(self):
    self.assertIn(telemetry.SERVICE, SERVICE_LIST)
    self.assertTrue(SERVICE_LIST[telemetry.SERVICE].should_log)
    self.assertEqual(SERVICE_LIST[telemetry.SERVICE].frequency, 20.)

  @unittest.skipIf(STATES is None, "jetlink_repo is not checked out")
  def test_states_are_jetlinks(self):
    # JetLink's state() answers one of its STATES, and the enum must hold each
    self.assertEqual(telemetry.STATES, frozenset(STATES))

  def test_link_off(self):
    msg = sent(jetlink_adapter._Off('jetlink is missing'), types.SimpleNamespace())
    self.assertEqual(msg.which(), 'modelDataV2BP')
    self.assertEqual(str(msg.modelDataV2BP.acceleratorState), 'none')
    self.assertEqual(msg.modelDataV2BP.handovers, 0)

  def test_joined_model(self):
    msg = sent(Hooks('running'), types.SimpleNamespace(handovers=3), valid=False)
    self.assertEqual(str(msg.modelDataV2BP.acceleratorState), 'running')
    self.assertEqual(msg.modelDataV2BP.handovers, 3)
    self.assertFalse(msg.valid)

  def test_bad_answers_log_none(self):
    # a name outside the enum would raise on modeld's frame thread
    for answer in ('warp drive', None, 7, RuntimeError('hook failed')):
      with self.subTest(answer=answer):
        self.assertEqual(telemetry.accelerator_state(Hooks(answer), object()), 'none')
    for count, want in ((-1, 0), (True, 0), ('3', 0), (2 ** 40, telemetry.UINT32_MAX), (5, 5)):
      with self.subTest(count=count):
        self.assertEqual(telemetry.handovers(types.SimpleNamespace(handovers=count)), want)

  def test_both_modelds_publish(self):
    for modeld in MODELDS:
      with self.subTest(modeld=modeld):
        source = (Path(BASEDIR) / modeld).read_text()
        self.assertIn('pub_socks += [jetlink_telemetry.SERVICE]', source)
        self.assertEqual(len(re.findall(r'pm\.send\(jetlink_telemetry\.SERVICE, jetlink_telemetry\.message\(HOOKS, model,', source)), 1)


if __name__ == '__main__':
  unittest.main()
