"""BluePilot (bp-full): Steer_Assist_Data as a one-track radar (sunnypilot/car/ford/bluepilot/radar_ext.py),
and the Delphi MRR hold after a scan-index freeze (car/ford/radar_interface.py)."""
import unittest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.radar_interface import DELPHI_MRR_RADAR_RANGE_COVERAGE, RadarInterface
from opendbc.car.ford.values import RADAR
from opendbc.sunnypilot.car.ford.bluepilot.radar_ext import SteerAssistRadar


def msg(d_long, v_long=-1.0, d_lat=0.0, confidence=3):
  return {'CmbbObjDistLong_L_Actl': d_long, 'CmbbObjRelLong_V_Actl': v_long, 'CmbbObjDistLat_L_Actl': d_lat,
          'CmbbObjConfdnc_D_Stat': confidence}


class TestSteerAssistRadar(unittest.TestCase):
  def setUp(self):
    self.radar = SteerAssistRadar()
    self.pts = {}

  def test_track_lifecycle(self):
    self.radar.update(msg(30.0, d_lat=-0.5), self.pts)
    self.assertEqual((self.pts[0].trackId, self.pts[0].dRel, self.pts[0].yRel, self.pts[0].vRel), (0, 30.0, -0.5, -1.0))

    self.radar.update(msg(29.9), self.pts)
    self.assertEqual(self.pts[0].trackId, 0)  # same object

    self.radar.update(msg(29.9, confidence=0), self.pts)
    self.assertEqual(self.pts, {})  # object lost

    self.radar.update(msg(40.0), self.pts)
    self.assertEqual(self.pts[0].trackId, 1)  # a new object is a new track

  def test_jump_is_new_track(self):
    self.radar.update(msg(30.0), self.pts)
    self.radar.update(msg(40.0), self.pts)  # > 5 m jump
    self.assertEqual(self.pts[0].trackId, 1)
    self.radar.update(msg(40.0, v_long=-5.0), self.pts)  # > 2 m/s jump
    self.assertEqual(self.pts[0].trackId, 2)

  def test_speed_from_distance_when_unreported(self):
    self.radar.update(msg(30.0, v_long=0.0), self.pts)
    self.radar.update(msg(30.5, v_long=0.0), self.pts)
    self.radar.update(msg(31.0, v_long=0.0), self.pts)
    self.assertAlmostEqual(self.pts[0].vRel, 1.0, places=4)  # sum of the distance changes
    self.assertEqual(self.pts[0].trackId, 0)


class TestDelphiMrrRadarHold(unittest.TestCase):
  """Delphi MRR on a CAN Ford: radarUnavailableTemporary stays set for 33 radar frames after a scan-index freeze clears."""
  HOLD = 33  # BP7's value, written out here on purpose: the test must fail if the interface's constant changes

  def setUp(self):
    CP = CarInterface.get_params("FORD_EXPLORER_MK6", gen_empty_fingerprint(), [], False, False, False)
    self.assertFalse(CP.radarUnavailable)
    self.bus = CanBus(CP).radar
    self.packer = CANPacker(RADAR.DELPHI_MRR)
    self.ri = RadarInterface(CP, structs.CarParamsSP())
    self.t = 0

  def step(self, scan):
    """One radar cycle at this scan index: headers, one valid detection 40 m ahead, 63 empty detection slots."""
    msgs = [self.packer.make_can_msg("MRR_Header_InformationDetections", self.bus, {"CAN_SCAN_INDEX": scan})]
    msgs += [self.packer.make_can_msg(f"MRR_Detection_{i:03d}", self.bus, {f"CAN_SCAN_INDEX_2LSB_{i:02d}": scan}) for i in range(2, 65)]
    msgs.append(self.packer.make_can_msg("MRR_Detection_001", self.bus, {"CAN_DET_VALID_LEVEL_01": 1, "CAN_DET_RANGE_01": 40.0,
                                                                         "CAN_DET_AZIMUTH_01": 0.0, "CAN_DET_RANGE_RATE_01": -1.0,
                                                                         "CAN_SCAN_INDEX_2LSB_01": scan}))
    msgs.append(self.packer.make_can_msg("MRR_Header_SensorCoverage", self.bus, {"CAN_RANGE_COVERAGE": DELPHI_MRR_RADAR_RANGE_COVERAGE[scan]}))
    self.t += 30_000_000
    return self.ri.update([(self.t, [(a, bytes(d), b) for a, d, b in msgs])])

  def scan_normally(self, cycles):
    for _ in range(cycles):
      for scan in range(4):
        rd = self.step(scan)
    return rd

  def freeze(self, scan, frames):
    return [self.step(scan) for _ in range(frames)]

  def assert_held(self, rd):
    self.assertIsNotNone(rd)
    self.assertTrue(rd.errors.radarUnavailableTemporary)
    self.assertEqual(len(rd.points), 0)

  def assert_tracking(self, rd):
    self.assertIsNotNone(rd)
    self.assertFalse(rd.errors.radarUnavailableTemporary)
    self.assertEqual([round(p.dRel, 2) for p in rd.points], [40.0])

  def freeze_then_recover(self, during_hold=False):
    """A freeze stuck at scan 1, then recovery: returns the HOLD + 1 frames after the freeze.
    Stuck at 1, the hold covers scans 2..34 and the frame after it is a scan-3 (publishing) frame."""
    frozen = self.freeze(1, 5)
    for rd in (frozen if during_hold else frozen[-1:]):  # the 5th repeat trips; during a hold every repeat is held
      self.assert_held(rd)
    return [self.step((2 + i) % 4) for i in range(self.HOLD + 1)]

  def start(self):
    self.assert_tracking(self.scan_normally(3))
    self.step(0)
    self.step(1)

  def test_hold_lasts_33_frames_after_recovery(self):
    self.start()
    after = self.freeze_then_recover()
    for i, rd in enumerate(after[:self.HOLD]):
      with self.subTest(hold_frame=i + 1):
        self.assert_held(rd)
    self.assertFalse(after[self.HOLD].errors.radarUnavailableTemporary)

  def test_points_resume_on_the_frame_after_the_hold(self):
    self.start()
    self.assert_tracking(self.freeze_then_recover()[self.HOLD])

  def test_freeze_during_the_hold_restarts_it(self):
    self.start()
    self.freeze(1, 5)
    for scan in (2, 3, 0, 1, 2, 3, 0, 1):  # 8 frames into the first hold
      self.assert_held(self.step(scan))
    after = self.freeze_then_recover(during_hold=True)  # a second freeze: a full hold again from its recovery
    for i, rd in enumerate(after[:self.HOLD]):
      with self.subTest(hold_frame=i + 1):
        self.assert_held(rd)
    self.assert_tracking(after[self.HOLD])

  def test_short_repeat_during_the_hold_keeps_counting(self):
    self.start()
    self.freeze(1, 5)
    scans = [2, 3, 3, 3, 3, 3] + [k % 4 for k in range(27)]  # scan 3 repeated 4 times: under the 5 that trip a freeze
    self.assertEqual(len(scans), self.HOLD)
    for i, scan in enumerate(scans):
      with self.subTest(hold_frame=i + 1):
        self.assert_held(self.step(scan))
    self.assert_tracking(self.step(3))  # the repeats counted toward the hold


if __name__ == '__main__':
  unittest.main()
