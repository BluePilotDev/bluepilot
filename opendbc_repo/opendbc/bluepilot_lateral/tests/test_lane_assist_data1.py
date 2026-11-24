"""The Lane_Assist_Data1 side channel (can/LANE_ASSIST_DATA1.md) is written on both sides: Python packs
it (can/fordcan_ext.py), the panda reads it (safety/ford.h). This pins the two to each other and to
bits the DBC leaves free."""
import re
import unittest
from pathlib import Path

from ...can.dbc import DBC
from ..can import fordcan_ext

DECLARATIONS = Path(__file__).resolve().parents[1] / "safety" / "ford_declarations.h"
CURVATURE_TO_CAN = 50000  # curvature CAN units per 1/m (FORD_LIMITS .curvature_to_can)


def _c_define(text, name):
  m = re.search(rf"#define {name}\s+(0x[0-9A-Fa-f]+|[0-9.]+)[Uf]?\b", text)
  assert m, name
  return float(m.group(1)) if "." in m.group(1) else int(m.group(1), 0)


def _dbc_bits(sig):
  """Bit indices (byte * 8 + bit) a big-endian DBC signal occupies."""
  assert not sig.is_little_endian, sig.name  # Lane_Assist_Data1's signals are all big-endian
  bits, b = set(), sig.msb
  for _ in range(sig.size):
    bits.add(b)
    b = b + 15 if b % 8 == 0 else b - 1
  return bits


class TestLaneAssistData1(unittest.TestCase):
  def test_python_and_c_agree(self):
    c = DECLARATIONS.read_text()
    self.assertEqual(_c_define(c, "FORD_BP_LKA_ANGLE_MODE_BYTE"), fordcan_ext.BP_LKA_ANGLE_MODE_BYTE)
    self.assertEqual(_c_define(c, "FORD_BP_LKA_ANGLE_MODE_MASK"), fordcan_ext.BP_LKA_ANGLE_MODE_MASK)
    self.assertEqual(_c_define(c, "FORD_BP_LKA_SHADOW_MSB_BYTE"), fordcan_ext.BP_LKA_SHADOW_MSB_BYTE)
    self.assertEqual(_c_define(c, "FORD_BP_LKA_SHADOW_LSB_BYTE"), fordcan_ext.BP_LKA_SHADOW_LSB_BYTE)
    # the panda turns the raw value (scale BP_LKA_SHADOW_CURVATURE_SCALE) into curvature CAN units
    self.assertAlmostEqual(_c_define(c, "FORD_BP_LKA_SHADOW_RAW_TO_CAN"), fordcan_ext.BP_LKA_SHADOW_CURVATURE_SCALE * CURVATURE_TO_CAN)

  def test_side_channel_bits_are_free_in_the_dbc(self):
    used = set().union(*(_dbc_bits(s) for s in DBC("ford_lincoln_base_pt").msgs[970].sigs.values()))
    ours = {fordcan_ext.BP_LKA_ANGLE_MODE_BYTE * 8 + i for i in range(8) if fordcan_ext.BP_LKA_ANGLE_MODE_MASK >> i & 1}
    ours |= {byte * 8 + i for byte in (fordcan_ext.BP_LKA_SHADOW_MSB_BYTE, fordcan_ext.BP_LKA_SHADOW_LSB_BYTE) for i in range(8)}
    self.assertFalse(ours & used, sorted(ours & used))


if __name__ == "__main__":
  unittest.main()
