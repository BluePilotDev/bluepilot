"""BluePilot (bp-full): Ford ICBM (sunnypilot/car/ford/bluepilot/icbm.py)."""
import inspect
import unittest
from types import SimpleNamespace as NS

from opendbc.can import CANPacker, CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR
from opendbc.sunnypilot.car.ford.bluepilot.icbm import IcbmExt

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState
DBC = "ford_lincoln_base_pt"
CAN = NS(main=0, camera=2)


def stock_buttons(**values):
  """Steering_Data_FD1 as the car state hands it over: every signal, all 0 unless given."""
  stock = dict(CANParser(DBC, [("Steering_Data_FD1", 10)], 0).vl["Steering_Data_FD1"])
  stock.update(values)
  return stock


def cc_sp(button):
  return NS(intelligentCruiseButtonManagement=NS(sendButton=button))


def decode(msg):
  address, dat, bus = msg
  parser = CANParser(DBC, [("Steering_Data_FD1", 10)], bus)
  parser.update([(0, [(address, dat, bus)])])
  return parser.vl["Steering_Data_FD1"]


class TestIcbmExt(unittest.TestCase):
  def setUp(self):
    self.icbm = IcbmExt()
    self.packer = CANPacker(DBC)

  def sends(self, button, frame, stock=None):
    return self.icbm.update(cc_sp(button), NS(buttons_stock_values=stock or stock_buttons()), self.packer, CAN, frame)

  def test_increase_presses_set_plus_on_both_buses(self):
    msgs = self.sends(SendButtonState.increase, 6)
    self.assertEqual(sorted(bus for _, _, bus in msgs), [CAN.main, CAN.camera])
    for m in msgs:
      self.assertEqual((decode(m)["CcAslButtnSetIncPress"], decode(m)["CcAslButtnSetDecPress"]), (1, 0))

  def test_decrease_presses_set_minus(self):
    for m in self.sends(SendButtonState.decrease, 6):
      self.assertEqual((decode(m)["CcAslButtnSetIncPress"], decode(m)["CcAslButtnSetDecPress"]), (0, 1))

  def test_no_request_sends_nothing(self):
    self.assertEqual(self.sends(SendButtonState.none, 100), [])

  def test_rate_is_bp7s(self):
    # BP7: sent once (frame - last press) * DT_CTRL > 0.05, so every 6th frame (about 17 Hz)
    sent = [f for f in range(31) if self.sends(SendButtonState.increase, f)]
    self.assertEqual(sent, [6, 12, 18, 24, 30])

  def test_stock_values_pass_through_unchanged(self):
    stock = stock_buttons(TurnLghtSwtch_D_Stat=1, CcAslButtnSetDecPress=0)
    before = dict(stock)
    msgs = self.sends(SendButtonState.increase, 6, stock)
    self.assertEqual(decode(msgs[0])["TurnLghtSwtch_D_Stat"], 1)
    self.assertEqual(stock, before)  # the car state's dict is not written to


class TestFordIcbmAvailable(unittest.TestCase):
  def test_every_ford_is_icbm_available(self):
    for car in CAR:
      with self.subTest(car=car):
        CP = CarInterface.get_params(car, gen_empty_fingerprint(), [], False, False, False)
        CP_SP = CarInterface.get_params_sp(CP, car, gen_empty_fingerprint(), [], False, False, False)
        self.assertTrue(CP_SP.intelligentCruiseButtonManagementAvailable)

  def test_one_get_params_sp(self):
    # a clean upstream merge that adds its own Ford _get_params_sp would silently replace one of the two
    import opendbc.car.ford.interface as ford_interface
    self.assertEqual(inspect.getsource(ford_interface).count("def _get_params_sp("), 1)


if __name__ == '__main__':
  unittest.main()
