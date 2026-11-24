"""Ford Edge MK2 (sunnypilot/car/ford/carstate_ext.py): no SteeringPinion_Data or PowertrainData_10."""
import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.ford.carstate import CarState
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, DBC
from opendbc.car.fingerprints import MIGRATION
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.ford.fingerprints_ext import match_vin_to_car


class TestEdgeFingerprint(unittest.TestCase):
  def test_vin(self):
    self.assertEqual(match_vin_to_car('2FMPK4J97NBXXXXXX'), {'FORD_EDGE_MK2'})  # Edge 2022
    self.assertEqual(match_vin_to_car('2FMPK4J97MBXXXXXX'), set())  # Edge 2021: not supported

  def test_vin_years_stay_2022_only(self):
    # the docs name is BP7's "2019-24", but the VIN fallback keeps matching only 2022
    for year, letter in ((2019, 'K'), (2020, 'L'), (2021, 'M'), (2022, 'N'), (2023, 'P'), (2024, 'R')):
      with self.subTest(year=year):
        expected = {'FORD_EDGE_MK2'} if year == 2022 else set()
        self.assertEqual(match_vin_to_car(f'2FMPK4J97{letter}BXXXXXX'), expected)

  def test_docs(self):
    # BluePilot 7's car list name, so a saved manual selection keeps working
    docs = CAR.FORD_EDGE_MK2.config.car_docs
    self.assertEqual([d.name for d in docs], ["Ford Edge 2019-24"])
    self.assertEqual(docs[0].year_list, [str(y) for y in range(2019, 2025)])

  def test_migration(self):
    self.assertEqual(MIGRATION["FORD EDGE 2ND GEN"], CAR.FORD_EDGE_MK2)

  def test_firmware(self):
    # BluePilot 7's Edge firmware
    fw = {
      (0x730, CarParams.Ecu.eps): b'M2GC-14D003-AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      (0x760, CarParams.Ecu.abs): b'M2GC-2D053-EA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      (0x764, CarParams.Ecu.fwdRadar): b'JX7T-14D049-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      (0x706, CarParams.Ecu.fwdCamera): b'KT4T-14F397-AF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    }
    car_fw = [CarParams.CarFw(ecu=ecu, fwVersion=v, address=addr, subAddress=0, brand='ford') for (addr, ecu), v in fw.items()]
    exact, matches = match_fw_to_car(car_fw, '2FMPK4J97NBXXXXXX', allow_fuzzy=False)
    self.assertTrue(exact)
    self.assertEqual(matches, {'FORD_EDGE_MK2'})


class EdgeCarState:
  """The real Ford CarState for the Edge, fed packed powertrain frames."""
  def __init__(self):
    fingerprint = gen_empty_fingerprint()
    fingerprint[0][0x5A] = 8  # Gear_Shift_by_Wire_FD1: automatic
    self.CP = CarInterface.get_params(CAR.FORD_EDGE_MK2, fingerprint, [], False, False, False)
    self.CS = CarState(self.CP, structs.CarParamsSP())
    self.parsers = self.CS.get_can_parsers(self.CP, structs.CarParamsSP())
    self.packer = CANPacker(DBC[CAR.FORD_EDGE_MK2][Bus.pt])
    self.t = 0

  def update(self, rel_angle, abs_angle=None, ext_stat=0, apa_stat=0, gear=3) -> structs.CarState:
    """rel_angle: SteeringPinion_Data_Alt; abs_angle: ParkAid_Data's angle, None = not available (raw 32767)."""
    bus = self.parsers[Bus.pt].bus
    park_aid = {"EPASExtAngleStatReq": ext_stat, "ApaSys_D_Stat": apa_stat}
    park_aid["ExtSteeringAngleReq2"] = 32767 * 0.1 - 1000 if abs_angle is None else abs_angle
    msgs = [
      self.packer.make_can_msg("SteeringPinion_Data_Alt", bus, {"StePinRelInit_An_Sns": rel_angle}),
      self.packer.make_can_msg("ParkAid_Data", bus, park_aid),
      self.packer.make_can_msg("TransGearData", bus, {"GearLvrPos_D_Actl": gear}),
    ]
    self.t += 10_000_000
    for p in self.parsers.values():
      p.update([(self.t, msgs)])
    ret, _ = self.CS.update(self.parsers)
    return ret


class TestEdgeSteeringAngle(unittest.TestCase):
  def test_offset_learned_while_park_aid_valid(self):
    edge = EdgeCarState()
    self.assertAlmostEqual(edge.update(10.0, abs_angle=25.0).steeringAngleDeg, 25.0, places=3)
    # park aid drops out: the learned 15 degree offset carries the relative angle
    self.assertAlmostEqual(edge.update(-20.0).steeringAngleDeg, -5.0, places=3)

  def test_offset_held_while_park_aid_invalid(self):
    invalid = {
      "angle unknown": dict(abs_angle=32766 * 0.1 - 1000),
      "angle invalid": dict(abs_angle=None),
      "external angle requested": dict(abs_angle=90.0, ext_stat=1),
      # ApaSys_D_Stat is 3 bits: only 0 (null) and 1 (off) are valid
      **{f"park assist state {stat}": dict(abs_angle=90.0, apa_stat=stat) for stat in range(2, 8)},
    }
    for name, park_aid in invalid.items():
      with self.subTest(name):
        edge = EdgeCarState()
        edge.update(10.0, abs_angle=25.0)
        self.assertAlmostEqual(edge.update(30.0, **park_aid).steeringAngleDeg, 45.0, places=3)

  def test_park_assist_off_is_valid(self):
    edge = EdgeCarState()
    self.assertAlmostEqual(edge.update(10.0, abs_angle=-30.0, apa_stat=1).steeringAngleDeg, -30.0, places=3)

  def test_relative_angle_until_first_valid_reading(self):
    edge = EdgeCarState()
    self.assertAlmostEqual(edge.update(12.5).steeringAngleDeg, 12.5, places=3)
    self.assertAlmostEqual(edge.update(-40.0, abs_angle=90.0, ext_stat=1).steeringAngleDeg, -40.0, places=3)

  def test_never_sets_vehicle_sensors_invalid(self):
    # BluePilot 7: a bad park aid angle only freezes the offset, it never blocks engagement
    edge = EdgeCarState()
    self.assertFalse(edge.update(10.0).vehicleSensorsInvalid)
    self.assertFalse(edge.update(10.0, abs_angle=25.0).vehicleSensorsInvalid)


class TestEdgeGear(unittest.TestCase):
  def test_gear_from_lever_position(self):
    edge = EdgeCarState()
    GearShifter = structs.CarState.GearShifter
    for raw, gear in ((0, GearShifter.park), (1, GearShifter.reverse), (2, GearShifter.neutral), (3, GearShifter.drive),
                      (5, GearShifter.low)):
      with self.subTest(gear=gear):
        self.assertEqual(edge.update(0.0, gear=raw).gearShifter, gear)

  def test_unmapped_lever_positions_are_unknown(self):
    # as in BluePilot 7: Sport, the numbered gears and the fault values have no GearShifter
    edge = EdgeCarState()
    for raw in (4, *range(6, 16)):
      with self.subTest(raw=raw):
        self.assertEqual(edge.update(0.0, gear=raw).gearShifter, structs.CarState.GearShifter.unknown)


def _pt_messages(car, fingerprint_addrs=()):
  """Powertrain messages the parser knows after one update: registered up front or read by the car state."""
  fingerprint = gen_empty_fingerprint()
  fingerprint[0].update({a: 8 for a in (0x5A, *fingerprint_addrs)})
  CP = CarInterface.get_params(car, fingerprint, [], False, False, False)
  CS = CarState(CP, structs.CarParamsSP())
  parsers = CS.get_can_parsers(CP, structs.CarParamsSP())
  CS.update(parsers)
  return {s.name: s for s in parsers[Bus.pt].message_states.values()}


class TestEdgeMessages(unittest.TestCase):
  EDGE = {"SteeringPinion_Data_Alt": 100, "ParkAid_Data": 50, "TransGearData": 10}
  STOCK = ("SteeringPinion_Data", "PowertrainData_10")

  def test_edge(self):
    msgs = _pt_messages(CAR.FORD_EDGE_MK2)
    for name, freq in self.EDGE.items():
      self.assertEqual(msgs[name].frequency, freq)
      self.assertFalse(msgs[name].ignore_alive)
    for name in self.STOCK:
      self.assertNotIn(name, msgs)

  def test_other_fords_unchanged(self):
    for car in (CAR.FORD_ESCAPE_MK4, CAR.FORD_F_150_MK14):
      with self.subTest(car=car):
        msgs = _pt_messages(car)
        for name in self.STOCK:
          self.assertTrue(name in msgs, name)
        for name in self.EDGE:
          self.assertNotIn(name, msgs)


if __name__ == '__main__':
  unittest.main()
