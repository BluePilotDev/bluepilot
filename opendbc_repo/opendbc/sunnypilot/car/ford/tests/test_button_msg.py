"""BluePilot: generated Steering_Data_FD1 frames never replay a Set+/Set- the driver is holding."""
import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car.ford import fordcan
from opendbc.sunnypilot.car.ford import fordcan_ext

DBC = "ford_lincoln_base_pt"
SET_BITS = ("CcAslButtnSetIncPress", "CcAslButtnSetDecPress")


def _stock_values(held):
  packer = CANPacker(DBC)
  parser = CANParser(DBC, [("Steering_Data_FD1", 10)], 0)
  parser.update([(0, [packer.make_can_msg("Steering_Data_FD1", 0, dict.fromkeys(held, 1))])])
  return parser.vl["Steering_Data_FD1"]


def _decode(msg):
  parser = CANParser(DBC, [("Steering_Data_FD1", 10)], 0)
  parser.update([(0, [msg])])
  return parser.vl["Steering_Data_FD1"]


@pytest.mark.parametrize("held", [(), ("CcAslButtnSetIncPress",), ("CcAslButtnSetDecPress",), SET_BITS])
@pytest.mark.parametrize("kwargs, pressed", [({"cancel": True}, "CcAslButtnCnclPress"), ({"resume": True}, "CcAsllButtnResPress"),
                                             ({"tja_toggle": True}, "TjaButtnOnOffPress")])
@pytest.mark.parametrize("builder", [fordcan.create_button_msg, fordcan_ext.create_button_msg])
def test_generated_frames_drop_held_set_buttons(builder, kwargs, pressed, held):
  out = _decode(builder(CANPacker(DBC), 0, _stock_values(held), **kwargs))
  assert out[pressed] == 1
  assert all(out[s] == 0 for s in SET_BITS)


@pytest.mark.parametrize("held", [(), SET_BITS])
@pytest.mark.parametrize("button, other", [SET_BITS, SET_BITS[::-1]])
def test_icbm_press_sets_only_its_own_bit(button, other, held):
  out = _decode(fordcan_ext.create_button_msg(CANPacker(DBC), 0, _stock_values(held), icbm_button=button))
  assert out[button] == 1 and out[other] == 0
