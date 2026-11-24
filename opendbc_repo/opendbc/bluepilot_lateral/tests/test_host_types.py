import ast
import dataclasses
import sys
import unittest
from pathlib import Path

from . import host
from .. import host_types

HOST_TYPES = Path(host_types.__file__)

# LateralInputs as it was when it lived in the structs.py hook (#63 moved it, fields unchanged): the
# host fills these, so a rename or a new default changes what every host sends
LATERAL_INPUTS = {
  "valid": False,
  "pathYawRate": [], "pathX": [], "pathY": [],
  "laneLineLeftX": [], "laneLineLeftY": [], "laneLineRightX": [], "laneLineRightY": [],
  "laneLineProbs": [], "laneLineStds": [],
  "laneChangeState": 0, "laneChangeDirection": 0,
  "steerRatio": 0.0, "stiffnessFactor": 0.0, "roll": 0.0, "angleOffsetDeg": 0.0, "lateralDelay": 0.0,
}


class TestHostTypes(unittest.TestCase):
  def test_lateral_inputs_fields_and_defaults(self):
    li = host_types.LateralInputs()
    self.assertEqual([f.name for f in dataclasses.fields(li)], list(LATERAL_INPUTS))
    for name, default in LATERAL_INPUTS.items():
      self.assertEqual(getattr(li, name), default, name)
      self.assertIs(type(getattr(li, name)), type(default), name)

  def test_list_defaults_are_not_shared(self):
    a, b = host_types.LateralInputs(), host_types.LateralInputs()
    a.pathX.append(1.0)
    self.assertEqual(b.pathX, [])

  def test_host_car_control_carries_the_library_type(self):
    self.assertIsInstance(host.CarControlX().lateralInputs, host_types.LateralInputs)

  def test_imports_only_the_standard_library(self):
    # structs.py imports this module, so anything beyond the standard library risks an import cycle
    tree = ast.parse(HOST_TYPES.read_text())
    modules = {alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
    modules |= {node.module or "" for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
    for module in modules:
      self.assertTrue(module.split(".")[0] in sys.stdlib_module_names, module)


if __name__ == "__main__":
  unittest.main()
