from types import SimpleNamespace

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from opendbc.bluepilot_lateral.core import strategy
from opendbc.bluepilot_lateral.core.pscm_read import pscm_fw_entries
from opendbc.bluepilot_lateral.host_types import LateralStatus, SupportLevel
from opendbc.bluepilot_lateral.tests import harness
from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.ford.values import CAR
from opendbc.sunnypilot.car.interfaces import setup_interfaces

from openpilot.bluepilot.selfdrive.car import ford_lateral_support as support

F150 = ("ML3V-14D003-BD", "ML34-14D007-EDL")
UNKNOWN = ("ZZ9V-14D003-AA", "ZZ94-14D007-AAA")


def _ci(parts=None, vin="1FTFW1E80MFA00001", active=True):
  """A Ford as card holds it after get_car: the start-up read in carFw, the library's verdict in status().
  parts: strategy, data file and, where given, calibration."""
  CP = structs.CarParams(carFingerprint="FORD_F_150_MK14", carVin=vin)
  CP.fingerprintSource = structs.CarParams.FingerprintSource.fw
  dids = (strategy.DID_STRATEGY, strategy.DID_DATA_FILE, strategy.DID_CALIBRATION)
  answers = {} if parts is None else {did: part.encode() for did, part in zip(dids, parts, strict=False)}
  CP.carFw = pscm_fw_entries(answers)
  match = strategy.match_car(CP)
  status = LateralStatus(active=active, supportLevel=int(match.level), supportReason=match.reason)
  return SimpleNamespace(CP=CP, CC=SimpleNamespace(bp=SimpleNamespace(status=lambda: status)))


def _real_ci(module, params_list=None):
  """The real Ford car interface as card holds it when it calls record_start: built by get_car's own
  steps, no steering frame yet."""
  CP = harness.car_params(CAR.FORD_F_150_MK14, module)
  CP.carVin = "1FTFW1E80MFA00001"
  CI = interfaces[CAR.FORD_F_150_MK14]
  CP_SP = CI.get_non_essential_params_sp(CP, CAR.FORD_F_150_MK14)
  setup_interfaces(CI, CP, CP_SP, params_list)
  return CI(CP, CP_SP)


class TestStartRecordWithTheCarInterface(OpenpilotTestCase):
  """record_start as card calls it: once, before any steering frame, with the library's own status."""

  def setup_method(self):
    self.params = Params()
    self.params.remove(support.SUPPORT_PARAM)
    self.params.remove(support.NOTICE_ALERT)

  def test_a_row_reads_its_level_before_the_first_frame(self):
    support.record_start(_real_ci(harness.F150_2021), self.params)
    stored = self.params.get(support.SUPPORT_PARAM)
    self.assertEqual((stored["active"], stored["level"], stored["reason"]), (True, SupportLevel.full, strategy.MATCHED))
    self.assertEqual(support.support_row(stored)[0], "Full")
    self.assertIsNone(self.params.get(support.NOTICE_ALERT))

  def test_the_fallback_is_named_and_told_before_the_first_frame(self):
    support.record_start(_real_ci(harness.UNKNOWN), self.params)
    stored = self.params.get(support.SUPPORT_PARAM)
    value, text = support.support_row(stored)
    self.assertEqual(value, "Fallback")
    self.assertIn(strategy.UNKNOWN_STRATEGY, text)
    self.assertIn(strategy.UNKNOWN_STRATEGY, self.params.get(support.NOTICE_ALERT)["extra"])

  def test_the_setting_reads_off(self):
    support.record_start(_real_ci(harness.F150_2021, [{"disable_BP_lat_UI": "1"}]), self.params)
    stored = self.params.get(support.SUPPORT_PARAM)
    self.assertEqual((stored["active"], support.support_row(stored)[0]), (False, "Off"))
    self.assertIsNone(self.params.get(support.NOTICE_ALERT))


class TestStartRecord(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.params.remove(support.SUPPORT_PARAM)
    self.params.remove(support.NOTICE_ALERT)

  def _start(self, **kwargs) -> str:
    """One car start; the notice it showed ("" for none). The onroad transition clears the last one."""
    self.params.remove(support.NOTICE_ALERT)
    support.record_start(_ci(**kwargs), self.params)
    return (self.params.get(support.NOTICE_ALERT) or {}).get("extra", "")

  def test_a_supported_car_is_told_nothing(self):
    self.assertEqual(self._start(parts=F150), "")
    stored = self.params.get(support.SUPPORT_PARAM)
    self.assertEqual((stored["level"], stored["strategy"], stored["dataFile"]), (SupportLevel.full, *F150))

  def test_the_fallback_is_told_on_its_first_starts(self):
    """The alert is cleared by the next start whether or not anyone saw it, so one notice is shown on
    a few starts, then no more."""
    notice = self._start(parts=UNKNOWN)
    self.assertIn(strategy.UNKNOWN_STRATEGY, notice)
    self.assertIn(UNKNOWN[0], notice)
    self.assertIn("upload a route", notice)
    for _ in range(support.FALLBACK_NOTICE_STARTS - 1):
      self.assertEqual(self._start(parts=UNKNOWN), notice)
    self.assertEqual(self._start(parts=UNKNOWN), "")

  def test_no_answer_asks_for_no_route(self):
    notice = self._start()
    self.assertIn(strategy.NO_ANSWER, notice)
    self.assertNotIn("upload", notice)

  def test_a_replaced_level_points_to_the_dealer(self):
    """The 2024 F-150's first level on a truck Ford's service message covers: the notice and the
    settings row say so, and nobody is asked for a route."""
    notice = self._start(parts=("RL34-14D003-AD", "RL34-14D007-ECC", "RL34-14D004-EE"))
    self.assertIn("SSM 53105", notice)
    self.assertIn("Ask a Ford dealer to update the power steering module", notice)
    self.assertNotIn("upload", notice)
    value, text = support.support_row(self.params.get(support.SUPPORT_PARAM))
    self.assertEqual(value, "Fallback")
    self.assertIn("Ask a Ford dealer to update the power steering module.", text)
    notice = self._start(parts=("RL34-14D003-AD", "RL34-14D007-BCD", "RL34-14D004-BE"), vin="1FTFW1E50RKF00002")
    self.assertIn(strategy.OLDER_SOFTWARE, notice)
    self.assertNotIn("SSM", notice)

  def test_changed_software_is_told_with_old_and_new(self):
    self._start(parts=F150)
    notice = self._start(parts=UNKNOWN)
    self.assertIn("changed since the last drive", notice)
    for part in (*F150, *UNKNOWN):
      self.assertIn(part, notice)
    self.assertEqual(self.params.get(support.SUPPORT_PARAM)["changedFrom"], {"strategy": F150[0], "dataFile": F150[1], "calibration": ""})
    self.assertNotIn("changed", self._start(parts=UNKNOWN))  # the start after: the change isn't told again

  def test_a_start_without_an_answer_is_not_a_change(self):
    self._start(parts=F150)
    self._start()  # the module didn't answer: on the fallback for this drive, part numbers kept
    self.assertEqual(self.params.get(support.SUPPORT_PARAM)["strategy"], F150[0])
    self.assertEqual(self._start(parts=F150), "")

  def test_another_car_is_not_a_change(self):
    self._start(parts=F150)
    self.assertEqual(self._start(parts=("RL38-14D003-AA", "RL38-14D007-DD"), vin="1FT6W1EV0PWG00002"), "")

  def test_stock_steering_is_not_the_fallback(self):
    self.assertEqual(self._start(parts=UNKNOWN, active=False), "")

  def test_a_car_without_the_library_is_left_alone(self):
    support.record_start(SimpleNamespace(CP=structs.CarParams(), CC=SimpleNamespace()), self.params)
    self.assertIsNone(self.params.get(support.SUPPORT_PARAM))


class TestSupportRow(OpenpilotTestCase):
  def test_before_the_first_start(self):
    self.assertEqual(support.support_row(None)[0], "Not read yet")

  def test_row_names_level_parts_and_match(self):
    record = support.start_record(_ci(parts=F150).CP, _ci(parts=F150).CC.bp.status())
    value, text = support.support_row(record)
    self.assertEqual(value, "Full")
    for part in (strategy.MATCHED, *F150, "FORD_F_150_MK14", "module software"):
      self.assertIn(part, text)

  def test_degraded_shows_in_value_and_text(self):
    params = Params()
    support.record_start(_ci(parts=F150), params)
    value, text = support.support_row(support.record_degraded(params, "the steering gain is low"))
    self.assertEqual(value, "Full, degraded")
    self.assertIn("the steering gain is low", text)

  def test_disabled_reads_off(self):
    ci = _ci(parts=F150, active=False)
    self.assertEqual(support.support_row(support.start_record(ci.CP, ci.CC.bp.status()))[0], "Off")
