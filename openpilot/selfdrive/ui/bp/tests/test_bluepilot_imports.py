"""BluePilot starts: the BluePilot check is on and every BluePilot module imports against the car code."""
import importlib
import unittest
from pathlib import Path

from openpilot.common.basedir import BASEDIR
from openpilot.common.bluepilot import is_bluepilot

ROOT = Path(BASEDIR)
BLUEPILOT_DIRS = ("bluepilot", "openpilot/selfdrive/ui/bp")

# upstream modules whose BluePilot imports sit behind is_bluepilot() at module level
GATED = (
  "openpilot.selfdrive.car.card",
  "openpilot.system.manager.process_config",
  "openpilot.selfdrive.ui.layouts.main",
)


def bluepilot_modules() -> list[str]:
  modules = []
  for folder in BLUEPILOT_DIRS:
    for path in sorted((ROOT / folder).rglob("*.py")):
      rel = path.relative_to(ROOT).with_suffix("")
      if "tests" in rel.parts or rel.name.startswith("test_"):
        continue
      if rel.name == "__init__":
        rel = rel.parent
      modules.append(".".join(rel.parts))
  return modules


class TestBluePilotImports(unittest.TestCase):
  def test_bluepilot_check_is_on(self):
    self.assertTrue(is_bluepilot())

  def test_every_bluepilot_module_imports(self):
    modules = bluepilot_modules()
    self.assertGreater(len(modules), 100)
    for name in modules + list(GATED):
      with self.subTest(module=name):
        importlib.import_module(name)


if __name__ == "__main__":
  unittest.main()
