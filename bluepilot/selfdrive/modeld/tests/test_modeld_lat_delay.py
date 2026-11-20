"""The stock-runner modeld (selfdrive/modeld/modeld.py, which runs whenever no model bundle is selected)
plans with the delay that respects sunnypilot's live-delay toggle: with the toggle off, the fixed
steerActuatorDelay plus the toggle's delay, as modeld_v2 does. Its main loop needs the cameras and the
model, so this reads the assignment from the source."""
import ast
import unittest
from pathlib import Path

MODELD = Path(__file__).resolve().parents[4] / "openpilot" / "selfdrive" / "modeld" / "modeld.py"


class TestModeldLatDelay(unittest.TestCase):
  def test_the_plan_uses_the_toggle_aware_delay(self):
    main = next(n for n in ast.parse(MODELD.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == "main")
    assigned = [ast.unparse(n.value) for n in ast.walk(main)
                if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "lat_delay" for t in n.targets)]
    self.assertEqual(assigned, ["model.lat_delay + LAT_SMOOTH_SECONDS"])


if __name__ == "__main__":
  unittest.main()
