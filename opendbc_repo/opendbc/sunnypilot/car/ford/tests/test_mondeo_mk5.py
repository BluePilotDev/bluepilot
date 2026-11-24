"""Ford Mondeo MK5 (BluePilot 7's platform): firmware, CarSpecs and docs years."""
import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, FordFlags
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu

# BluePilot 7's Mondeo firmware (its engine and debug entries aren't ported)
MONDEO_FW = {
  (0x706, Ecu.fwdCamera): b'KT4T-14F397-AE\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
  (0x760, Ecu.abs): b'KG9C-2D053-DF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
  (0x730, Ecu.eps): b'K2GC-14D003-AJ\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
  (0x764, Ecu.fwdRadar): b'JX7T-14D049-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
}

# BluePilot 7's Edge firmware: the same radar part number as the Mondeo
EDGE_FW = {
  (0x730, Ecu.eps): b'M2GC-14D003-AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
  (0x760, Ecu.abs): b'M2GC-2D053-EA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
  (0x764, Ecu.fwdRadar): b'JX7T-14D049-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
  (0x706, Ecu.fwdCamera): b'KT4T-14F397-AF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
}


def _match(fw, vin):
  car_fw = [CarParams.CarFw(ecu=ecu, fwVersion=v, address=addr, subAddress=0, brand='ford') for (addr, ecu), v in fw.items()]
  return match_fw_to_car(car_fw, vin, allow_fuzzy=False)


class TestMondeoFingerprint(unittest.TestCase):
  def test_mondeo_firmware_matches_only_the_mondeo(self):
    exact, matches = _match(MONDEO_FW, 'WF0EXXWPCEXXXXXXX')
    self.assertTrue(exact)
    self.assertEqual(matches, {'FORD_MONDEO_MK5'})

  def test_edge_firmware_still_matches_only_the_edge(self):
    exact, matches = _match(EDGE_FW, '2FMPK4J97NBXXXXXX')
    self.assertTrue(exact)
    self.assertEqual(matches, {'FORD_EDGE_MK2'})


class TestMondeoPlatform(unittest.TestCase):
  def test_platform_as_bp7_declares_it(self):
    self.assertEqual(CAR.FORD_MONDEO_MK5.config.flags, FordFlags.CANFD)
    CP = CarInterface.get_params(CAR.FORD_MONDEO_MK5, gen_empty_fingerprint(), [], False, False, False)
    self.assertAlmostEqual(CP.steerRatio, 14.8, places=5)
    self.assertAlmostEqual(CP.wheelbase, 2.85, places=5)

  def test_docs(self):
    docs = CAR.FORD_MONDEO_MK5.config.car_docs
    self.assertEqual([d.name for d in docs], ["Ford Mondeo 2014-22"])
    self.assertEqual(docs[0].package, "Adaptive Cruise Control with Lane Centering")
    self.assertEqual(docs[0].year_list, [str(y) for y in range(2014, 2023)])


if __name__ == '__main__':
  unittest.main()
