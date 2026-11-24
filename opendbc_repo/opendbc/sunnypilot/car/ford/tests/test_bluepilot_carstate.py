"""BluePilot (bp-full): Ford car state extras (sunnypilot/car/ford/bluepilot/carstate_ext.py)."""
import unittest
from types import SimpleNamespace as NS

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.carstate import CarState
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, DBC
from opendbc.sunnypilot.car.ford.bluepilot.carstate_ext import (CarStateExt, FORD_FLAG_BP_HEV_BATTERY_DATA,
                                                                FORD_FLAG_BP_HEV_CLUSTER_DATA)

ButtonType = structs.CarState.ButtonEvent.Type


def _cs(cruise_enabled: bool) -> structs.CarState:
  ret = structs.CarState()
  ret.cruiseState.enabled = cruise_enabled
  return ret


def _cp(dec_press=0, inc_press=0, cncl_res_press=0, on_off_press=0):
  return NS(vl={"Steering_Data_FD1": {"CcAslButtnSetDecPress": dec_press, "CcAslButtnSetIncPress": inc_press,
                                       "CcAslButtnCnclResPress": cncl_res_press, "CcButtnOnOffPress": on_off_press}})


class TestComboButtonEvents(unittest.TestCase):
  """BluePilotDev/bluepilot#193: holding a combo cruise button produced a press/release storm."""
  def setUp(self):
    self.ext = CarStateExt(structs.CarParams())

  def _update(self, ret, **presses):
    self.ext.update_buttons(ret, _cp(**presses))
    return [(e.type, e.pressed) for e in self.ext.button_events]

  def test_sustained_hold_emits_exactly_one_press_and_one_release(self):
    ret = _cs(cruise_enabled=False)
    self.assertEqual(self._update(ret, dec_press=1), [(ButtonType.setCruise, True)])
    for _ in range(50):
      self.assertEqual(self._update(ret, dec_press=1), [], "spurious event while held (#193)")
    ret = _cs(cruise_enabled=True)  # cruise engages partway through the hold
    for _ in range(20):
      self.assertEqual(self._update(ret, dec_press=1), [], "spurious event after cruise engaged mid-hold (#193)")
    self.assertEqual(self._update(ret, dec_press=0), [(ButtonType.setCruise, False)])

  def test_inc_and_dec_do_not_cross_contaminate_setcruise(self):
    ret = _cs(cruise_enabled=False)
    self.assertEqual(self._update(ret, dec_press=1), [(ButtonType.setCruise, True)])
    for _ in range(10):
      self.assertEqual(self._update(ret, dec_press=1), [])

  def test_tap_while_engaged_emits_decel_not_setcruise(self):
    ret = _cs(cruise_enabled=True)
    self.assertEqual(self._update(ret, dec_press=1), [(ButtonType.decelCruise, True)])
    self.assertEqual(self._update(ret, dec_press=0), [(ButtonType.decelCruise, False)])

  def test_cancel_resume_combo_unaffected(self):
    ret = _cs(cruise_enabled=True)
    self.assertEqual(self._update(ret, cncl_res_press=1), [(ButtonType.cancel, True)])
    for _ in range(10):
      self.assertEqual(self._update(ret, cncl_res_press=1), [])
    self.assertEqual(self._update(ret, cncl_res_press=0), [(ButtonType.cancel, False)])

  def test_main_cruise_turning_cruise_on_sets_speed(self):
    self.assertEqual(self._update(_cs(False), on_off_press=1), [(ButtonType.mainCruise, True)])
    self.assertEqual(self._update(_cs(True), on_off_press=0),  # cruise comes on the next frame
                     [(ButtonType.mainCruise, False), (ButtonType.setCruise, True), (ButtonType.setCruise, False)])


def _car_params(car, extra_addrs=()) -> structs.CarParams:
  """CarParams as the interface decides them; extra_addrs: main-bus fingerprint addresses
  (0x5A Gear_Shift_by_Wire_FD1 makes an automatic, its absence a manual)."""
  fingerprint = gen_empty_fingerprint()
  fingerprint[0].update({a: 8 for a in extra_addrs})
  return CarInterface.get_params(car, fingerprint, [], False, False, False)


def _car_state(car, extra_addrs=()):
  CP = _car_params(car, extra_addrs)
  return CP, CarState(CP, structs.CarParamsSP())


class TestCarStateExt(unittest.TestCase):
  def test_hev_flags_from_fingerprint(self):
    CP, _ = _car_state(CAR.FORD_ESCAPE_MK4, (0x365, 0x07A, 0x24B, 0x24C))
    self.assertTrue(CP.flags & FORD_FLAG_BP_HEV_CLUSTER_DATA and CP.flags & FORD_FLAG_BP_HEV_BATTERY_DATA)
    CP, _ = _car_state(CAR.FORD_ESCAPE_MK4, (0x07A, 0x24B))  # one battery message missing
    self.assertFalse(CP.flags & (FORD_FLAG_BP_HEV_CLUSTER_DATA | FORD_FLAG_BP_HEV_BATTERY_DATA))

  def test_values_from_can(self):
    for car in (CAR.FORD_F_150_MK14, CAR.FORD_ESCAPE_MK4, CAR.FORD_MUSTANG_MACH_E_MK1):
      with self.subTest(car=car):
        CP, CS = _car_state(car, (0x365, 0x07A, 0x24B, 0x24C))
        parsers = CS.get_can_parsers(CP, structs.CarParamsSP())
        packer = CANPacker(DBC[car][Bus.pt])
        pt, cam = parsers[Bus.pt].bus, parsers[Bus.cam].bus
        msgs = [
          packer.make_can_msg("Traffic_RecognitnData", cam, {"TsrVLim1MsgTxt_D_Rq": 55, "TsrVlUnitMsgTxt_D_Rq": 2}),
          packer.make_can_msg("Cluster_HEV_Data2", pt, {"PwrFlowTxt_D_Dsply": 5, "EngOnMsg1_D_Dsply": 1, "EffWhlLvl2_Pc_Dsply": 40}),
          packer.make_can_msg("Battery_Traction_1_FD1", pt, {"BattTrac_U_Actl": 350, "BattTrac_I_Actl": -20}),
          packer.make_can_msg("Battery_Traction_4_FD1", pt, {"BattTracSoc2_Pc_Actl": 61}),
          packer.make_can_msg("MtrTracData_1_FD1", pt, {"MtrTrac2_I_Actl": 30}),
          packer.make_can_msg("BCM_Lamp_Stat_FD1", pt, {"StopLghtOn_B_Stat": 1}),
        ]
        for p in parsers.values():
          p.update([(0, msgs)])
        _, ret_sp = CS.update(parsers)
        self.assertAlmostEqual(ret_sp.speedLimit, 55 * CV.MPH_TO_MS, places=3)
        bp = CS.bp.car_state_bp
        self.assertEqual((bp["hybridDrive"]["powerFlowMode"], bp["hybridDrive"]["engineOnReason"]), ("Electric Drive", "Acceleration"))
        self.assertEqual((bp["hybridBattery"]["voltActual"], bp["hybridBattery"]["socActual"]), (350, 61))
        self.assertEqual(bp["hybridBattery"]["ampsActual"], -30 if car == CAR.FORD_MUSTANG_MACH_E_MK1 else -20)
        self.assertTrue(bp["brakeLightStatus"]["brakeLightsOn"])

  def test_optional_messages_never_required(self):
    # a car without the hybrid, lamp or sign recognition messages keeps CAN validity, CAN and CAN FD alike
    for car in (CAR.FORD_ESCAPE_MK4, CAR.FORD_F_150_MK14):
      with self.subTest(car=car):
        CP, CS = _car_state(car, (0x365, 0x07A, 0x24B, 0x24C))
        pt, cam = CS.get_can_parsers(CP, structs.CarParamsSP()).values()
        for p in (pt, cam):
          required = {s.name for s in p.message_states.values() if not s.ignore_alive}
          self.assertFalse(required & {"Cluster_HEV_Data2", "Battery_Traction_1_FD1", "BCM_Lamp_Stat_FD1", "Traffic_RecognitnData"})

  def test_canfd_reads_required_at_bp7_frequencies(self):
    # registered up front as BP7 does: required, with BP7's timeouts from the first frame
    CP, CS = _car_state(CAR.FORD_F_150_MK14, (0x5A,))  # an automatic
    pt, cam = CS.get_can_parsers(CP, structs.CarParamsSP()).values()
    # PowertrainData_10: no longer read for the gear, still required as in BP7
    for p, expected in ((pt, {"Gear_Shift_by_Wire_FD1": 10, "PowertrainData_10": 10}),
                        (cam, {"IPMA_Data2": 1})):
      states = {s.name: s for s in p.message_states.values()}
      for name, freq in expected.items():
        with self.subTest(name=name):
          self.assertEqual((states[name].frequency, states[name].ignore_alive), (freq, False))

  def test_canfd_manual_gets_no_gear_messages(self):
    # the gear registration is for automatics only: a manual reads neither message
    CP, CS = _car_state(CAR.FORD_F_150_MK14)  # no shift-by-wire ECU, no 0x5A: a manual
    self.assertEqual(CP.transmissionType, structs.CarParams.TransmissionType.manual)
    pt, _ = CS.get_can_parsers(CP, structs.CarParamsSP()).values()
    registered = {s.name for s in pt.message_states.values()}
    self.assertFalse(registered & {"Gear_Shift_by_Wire_FD1", "PowertrainData_10"})


class PackedFrameHarness:
  """Test helper: the real Ford CarInterface (car state plus the base interface's cluster speed handling), fed
  packed frames."""
  def __init__(self, car, fingerprint_addrs=(0x5A,)):  # 0x5A Gear_Shift_by_Wire_FD1: an automatic
    self.CP = _car_params(car, fingerprint_addrs)
    self.CI = CarInterface(self.CP, structs.CarParamsSP())
    self.packer = CANPacker(DBC[car][Bus.pt])
    self.pt, self.cam = self.CI.can_parsers[Bus.pt].bus, self.CI.can_parsers[Bus.cam].bus
    self.t = 0

  def update(self, pt=None, cam=None) -> structs.CarState:
    """pt/cam: {message: {signal: value}}"""
    msgs = [self.packer.make_can_msg(m, self.pt, v) for m, v in (pt or {}).items()]
    msgs += [self.packer.make_can_msg(m, self.cam, v) for m, v in (cam or {}).items()]
    self.t += 10_000_000
    ret, _ = self.CI.update([(self.t, msgs)])
    return ret


class TestCanFdSetSpeedUnit(unittest.TestCase):
  """BP7: a CAN FD Ford's set speed unit is the camera's speed limit unit (IPMA_Data2)."""
  def _set_speed(self, unit, displayed):
    ford = PackedFrameHarness(CAR.FORD_F_150_MK14)
    for _ in range(2):  # the parser learns a message read for the first time on the next frame
      ret = ford.update(pt={"EngBrakeData": {"Veh_V_DsplyCcSet": displayed}}, cam={"IPMA_Data2": {"IsaVLimUnit_D_Rq": unit}})
    return ret.cruiseState.speed

  def test_metric(self):
    self.assertAlmostEqual(self._set_speed(unit=1, displayed=100), 27.7778, places=3)  # 100 km/h

  def test_imperial(self):
    self.assertAlmostEqual(self._set_speed(unit=2, displayed=62), 27.7165, places=3)  # 62 mph

  def test_no_unit_reads_mph(self):
    self.assertAlmostEqual(self._set_speed(unit=0, displayed=62), 27.7165, places=3)


class TestCanSetSpeedUnit(unittest.TestCase):
  """A CAN Ford keeps upstream's unit, INSTRUMENT_PANEL.METRIC_UNITS, whatever IPMA_Data2 says."""
  def _set_speed(self, metric_units, ipma_unit):
    ford = PackedFrameHarness(CAR.FORD_ESCAPE_MK4)
    for _ in range(2):
      ret = ford.update(pt={"EngBrakeData": {"Veh_V_DsplyCcSet": 100}, "INSTRUMENT_PANEL": {"METRIC_UNITS": metric_units}},
                        cam={"IPMA_Data2": {"IsaVLimUnit_D_Rq": ipma_unit}})
    return ret.cruiseState.speed

  def test_metric_panel(self):
    self.assertAlmostEqual(self._set_speed(metric_units=1, ipma_unit=2), 27.7778, places=3)  # 100 km/h

  def test_imperial_panel(self):
    self.assertAlmostEqual(self._set_speed(metric_units=0, ipma_unit=1), 44.704, places=3)  # 100 mph


class TestCanFdGear(unittest.TestCase):
  """BP7: a CAN FD Ford's gear is the shifter's request (Gear_Shift_by_Wire_FD1), not PowertrainData_10."""
  def test_gear_from_shift_by_wire(self):
    GearShifter = structs.CarState.GearShifter
    ford = PackedFrameHarness(CAR.FORD_F_150_MK14)
    for raw, gear in ((0, GearShifter.park), (1, GearShifter.reverse), (2, GearShifter.neutral), (3, GearShifter.drive),
                      (5, GearShifter.low), (4, GearShifter.unknown), (15, GearShifter.unknown)):
      with self.subTest(raw=raw):
        for _ in range(2):
          # PowertrainData_10 says park throughout: it must not be the source
          ret = ford.update(pt={"Gear_Shift_by_Wire_FD1": {"TrnRng_D_RqGsm": raw}, "PowertrainData_10": {"TrnRng_D_Rq": 0}})
        self.assertEqual(ret.gearShifter, gear)


def _kph(v):
  return v * CV.KPH_TO_MS


class TestClusterSpeed(unittest.TestCase):
  """BP7: every Ford's dash speed (here the measured speed) reads 0 below 1.5 km/h and moves only by more than
  0.5 km/h. Speed steps over 2 m/s reset the speed filter to the wheel speed exactly; smaller ones settle with
  some overshoot."""
  def _steady(self, ford, kph, frames=100, **extra):
    for _ in range(frames):
      ret = ford.update(pt={"BrakeSysFeatures": {"Veh_V_ActlBrk": kph}, **extra})
    return ret

  CARS = (CAR.FORD_ESCAPE_MK4, CAR.FORD_F_150_MK14)  # CAN and CAN FD

  def test_zero_below_1_5_kph(self):
    for car in self.CARS:
      with self.subTest(car=car):
        ford = PackedFrameHarness(car)
        self._steady(ford, 20)
        self.assertEqual(self._steady(ford, 1.49).vEgoCluster, 0.0)
        self._steady(ford, 20)
        self.assertAlmostEqual(self._steady(ford, 1.51).vEgoCluster, _kph(2.01), places=3)  # falling: 0.5 km/h above

  def test_hysteresis(self):
    for car in self.CARS:
      with self.subTest(car=car):
        ford = PackedFrameHarness(car)
        self.assertAlmostEqual(self._steady(ford, 20).vEgoCluster, _kph(19.5), places=3)  # rising: 0.5 km/h below
        self.assertAlmostEqual(self._steady(ford, 19.7, frames=1000).vEgoCluster, _kph(19.5), places=3)  # inside the gap: held
        self.assertAlmostEqual(self._steady(ford, 10).vEgoCluster, _kph(10.5), places=3)  # falling: 0.5 km/h above


if __name__ == '__main__':
  unittest.main()
