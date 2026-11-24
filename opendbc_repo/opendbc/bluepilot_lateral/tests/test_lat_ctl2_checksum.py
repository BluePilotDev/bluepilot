"""LateralMotionControl2's checksum (LatCtlPath_No_Cs) is computed in can/fordcan_ext.py, since a host's
packer may not fill Ford checksums. This checks it against the fields' raw values as the DBC lays them
out, and that the frame carries it."""
import unittest

from ...can.dbc import DBC
from ...can.packer import CANPacker
from ...car.ford.fordcan import CanBus
from ..can import fordcan_ext

DBC_NAME = "ford_lincoln_base_pt"
CHECKSUM_FIELDS = ("LatCtl_D2_Rq", "LatCtlPath_No_Cnt", "LatCtlCurv_No_Actl", "LatCtlCrv_NoRate2_Actl",
                   "LatCtlPath_An_Actl", "LatCtlPathOffst_L_Actl")


def _raw(sig, dat):
  """A big-endian DBC signal's raw value."""
  assert not sig.is_little_endian, sig.name
  value, b = 0, sig.msb
  for _ in range(sig.size):
    value = (value << 1) | ((dat[b // 8] >> (b % 8)) & 1)
    b = b + 15 if b % 8 == 0 else b - 1
  return value


class TestLatCtl2Checksum(unittest.TestCase):
  def test_checksum_matches_the_dbc_fields(self):
    msg = next(m for m in DBC(DBC_NAME).msgs.values() if m.name == "LateralMotionControl2")
    packer = CANPacker(DBC_NAME)
    CAN = CanBus(fingerprint={})
    cases = [
      (0, 0, 0., 0., 0., 0., 0),
      (2, 3, 0.4, 0.02, 0.0123, 0.0005, 7),
      (2, 1, -1.2, -0.3, -0.0199, -0.001, 15),
      (1, 2, 5.11, 0.5235, 0.02094, 0.001023, 9),
    ]
    for mode, ramp, offset, angle, curvature, rate, counter in cases:
      with self.subTest(mode=mode, curvature=curvature, counter=counter):
        _, dat, _ = fordcan_ext.create_lat_ctl2_msg(packer, CAN, mode, ramp, 1, offset, angle, curvature, rate, counter)
        checksum = 0
        for name in CHECKSUM_FIELDS:
          value = _raw(msg.sigs[name], dat)
          checksum += value + (value >> 8)
        self.assertEqual(_raw(msg.sigs["LatCtlPath_No_Cs"], dat), ~checksum & 0xFF)


if __name__ == "__main__":
  unittest.main()
