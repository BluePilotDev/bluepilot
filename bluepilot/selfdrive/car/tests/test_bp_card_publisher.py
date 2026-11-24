from types import SimpleNamespace

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.test import OpenpilotTestCase
from opendbc.bluepilot_lateral.host_types import LateralStatus
from opendbc.car import structs

from bluepilot.selfdrive.car import bp_card_publisher
from bluepilot.selfdrive.car.bp_card_publisher import publish_car_state_bp, publish_controller_state_bp

LateralMode = custom.ControllerStateBP.LateralMode


class _PM:
  def __init__(self):
    self.sent = {}

  def send(self, name, msg):
    self.sent[name] = messaging.log_from_bytes(msg.to_bytes())


def _ford(status=None, car_state_bp=None, fingerprint_source="fw"):
  CP = structs.CarParams(carFingerprint="FORD_F_150_MK14")
  CP.fingerprintSource = getattr(structs.CarParams.FingerprintSource, fingerprint_source)
  return SimpleNamespace(CP=CP,
                         CC=SimpleNamespace(bp=SimpleNamespace(status=lambda: status or LateralStatus())),
                         CS=SimpleNamespace(bp=SimpleNamespace(car_state_bp=car_state_bp or {})))


def _controller_state_bp(CI):
  pm = _PM()
  publish_controller_state_bp(CI, pm)
  return pm.sent["controllerStateBP"]


class TestControllerStateBP(OpenpilotTestCase):
  def test_carries_the_lateral_status(self):
    msg = _controller_state_bp(_ford(LateralStatus(lateralUncertainty=0.25, angleRateLimited=True, curvatureRateLimited=True,
                                                   curvatureDeviationLimited=True, humanTurnLateralPaused=True,
                                                   stallBlipActive=True, primary_lateral_control=1)))
    self.assertTrue(msg.valid)
    cs = msg.controllerStateBP
    self.assertAlmostEqual(cs.lateralUncertainty, 0.25)
    self.assertTrue(cs.angleRateLimited)
    self.assertTrue(cs.curvatureRateLimited)
    self.assertTrue(cs.curvatureDeviationLimited)
    self.assertTrue(cs.humanTurnLateralPaused)
    self.assertTrue(cs.stallBlipActive)

  def test_flags_default_off(self):
    cs = _controller_state_bp(_ford()).controllerStateBP
    self.assertFalse(cs.angleRateLimited or cs.curvatureRateLimited or cs.curvatureDeviationLimited
                     or cs.humanTurnLateralPaused or cs.stallBlipActive)

  def test_lateral_mode(self):
    cases = [
      (LateralStatus(disable_BP_lat_UI=True, primary_lateral_control=1), LateralMode.openpilot),
      (LateralStatus(primary_lateral_control=1), LateralMode.angle),
      (LateralStatus(primary_lateral_control=0), LateralMode.curvature),
    ]
    for status, mode in cases:
      with self.subTest(status=status):
        self.assertEqual(_controller_state_bp(_ford(status)).controllerStateBP.activeLateralMode, mode)

  def test_fingerprint_fields(self):
    cs = _controller_state_bp(_ford(fingerprint_source="fixed")).controllerStateBP
    self.assertTrue(cs.bmsFingerprintForced)
    self.assertEqual(cs.bmsFingerprint, "FORD_F_150_MK14")
    self.assertFalse(_controller_state_bp(_ford(fingerprint_source="fw")).controllerStateBP.bmsFingerprintForced)

  def test_settings_snapshot(self):
    # the 48 bms* fields of ControllerStateBP: 46 settings plus the two fingerprint fields
    snapshot = bp_card_publisher._refresh_settings_cache()
    bms_fields = {f for f in custom.ControllerStateBP.schema.fields if f.startswith("bms")}
    self.assertEqual(len(bms_fields), 48)
    self.assertEqual(set(snapshot) | {"bmsFingerprintForced", "bmsFingerprint"}, bms_fields)
    cs = _controller_state_bp(_ford()).controllerStateBP
    self.assertAlmostEqual(cs.bmsTwelveVBatteryLimit, snapshot["bmsTwelveVBatteryLimit"], places=5)
    self.assertEqual(cs.bmsShowBlindspotOverlay, snapshot["bmsShowBlindspotOverlay"])


class TestCarStateBP(OpenpilotTestCase):
  CAR_STATE_BP = {
    "hybridDrive": {"dataAvailable": True, "throttleDemandPercent": 42.0, "throttleThresholdPercent": 60.0, "powerFlowMode": "EV",
                    "powerFlowModeValue": 3, "engineOnReason": "", "engineOnReasonValue": 0},
    "hybridBattery": {"dataAvailable": True, "voltHighLimit": 400.0, "voltLowLimit": 300.0, "voltActual": 350.0, "ampsActual": -12.5,
                      "socMinPerc": 20.0, "socMaxPerc": 80.0, "socActual": 55.0},
    "brakeLightStatus": {"dataAvailable": True, "brakeLightsOn": True},
  }

  def _send(self, CI, can_valid):
    pm = _PM()
    publish_car_state_bp(CI, pm, can_valid)
    return pm.sent

  def test_carries_the_car_state(self):
    msg = self._send(_ford(car_state_bp=self.CAR_STATE_BP), True)["carStateBP"]
    self.assertTrue(msg.valid)
    cs = msg.carStateBP
    self.assertTrue(cs.brakeLightStatus.dataAvailable)
    self.assertTrue(cs.brakeLightStatus.brakeLightsOn)
    self.assertAlmostEqual(cs.hybridDrive.throttleDemandPercent, 42.0)
    self.assertEqual(cs.hybridDrive.powerFlowMode, "EV")
    self.assertEqual(cs.hybridDrive.powerFlowModeValue, 3)
    self.assertAlmostEqual(cs.hybridBattery.ampsActual, -12.5)
    self.assertAlmostEqual(cs.hybridBattery.socActual, 55.0)

  def test_follows_can_validity(self):
    self.assertFalse(self._send(_ford(car_state_bp=self.CAR_STATE_BP), False)["carStateBP"].valid)

  def test_nothing_before_the_first_update(self):
    self.assertEqual(self._send(_ford(), True), {})


class TestNotBluePilotCarCode(OpenpilotTestCase):
  """A car without the BluePilot car code publishes neither message."""

  def test_no_bp(self):
    CI = SimpleNamespace(CP=structs.CarParams(), CC=SimpleNamespace(), CS=SimpleNamespace())
    pm = _PM()
    publish_controller_state_bp(CI, pm)
    publish_car_state_bp(CI, pm, True)
    self.assertEqual(pm.sent, {})

  def test_bp_without_status(self):
    CI = SimpleNamespace(CP=structs.CarParams(), CC=SimpleNamespace(bp=SimpleNamespace()), CS=SimpleNamespace(bp=SimpleNamespace()))
    pm = _PM()
    publish_controller_state_bp(CI, pm)
    publish_car_state_bp(CI, pm, True)
    self.assertEqual(pm.sent, {})
