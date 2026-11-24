"""Onroad state in the portal and the route preprocessor, read from the real Params (unknown keys raise)."""
import logging
from unittest import mock

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase

from bluepilot.backend.network import utils
from bluepilot.backend.routes import preprocessor


class BrokenParams:
  def get_bool(self, key):
    raise RuntimeError("params unavailable")


class TestOnroadState(OpenpilotTestCase):
  def setUp(self):
    self.params = Params()
    self.params.put_bool("IsDriverViewEnabled", False, block=True)
    self.params.put_bool("IsEngaged", False, block=True)

  def test_portal_onroad_when_device_is_onroad(self):
    self.params.put_bool("IsOffroad", False, block=True)
    with mock.patch.object(utils, "params", self.params):
      self.assertTrue(utils.is_onroad())

  def test_portal_offroad_when_device_is_offroad(self):
    self.params.put_bool("IsOffroad", True, block=True)
    with mock.patch.object(utils, "params", self.params):
      self.assertFalse(utils.is_onroad())

  def test_portal_assumes_onroad_when_state_cannot_be_read(self):
    # Fail closed: a broken read must lock the portal, not unlock it.
    with mock.patch.object(utils, "params", BrokenParams()), self.assertLogs(utils.logger, logging.WARNING):
      self.assertTrue(utils.is_onroad())

  def test_preprocessor_idle_when_offroad(self):
    self.params.put_bool("IsOffroad", True, block=True)
    with mock.patch.object(preprocessor, "params", self.params):
      self.assertTrue(preprocessor.is_device_idle())

  def test_preprocessor_busy_when_onroad(self):
    self.params.put_bool("IsOffroad", False, block=True)
    with mock.patch.object(preprocessor, "params", self.params):
      self.assertFalse(preprocessor.is_device_idle())

  def test_preprocessor_busy_when_state_cannot_be_read(self):
    with mock.patch.object(preprocessor, "params", BrokenParams()):
      self.assertFalse(preprocessor.is_device_idle())
