"""BluePilot starts: the BluePilot check is on and every BluePilot module imports against the car code."""
import importlib
import unittest
from pathlib import Path

from openpilot.common.basedir import BASEDIR
from openpilot.common.bluepilot import is_bluepilot

ROOT = Path(BASEDIR)
BLUEPILOT_DIRS = ("bluepilot", "openpilot/selfdrive/ui/bp")

# upstream modules with is_bluepilot() checks; most hold BluePilot imports behind it at module level
GATED = (
  "openpilot.selfdrive.car.card",
  "openpilot.selfdrive.car.cruise",
  "openpilot.selfdrive.controls.lib.desire_helper",
  "openpilot.system.manager.process_config",
  "openpilot.selfdrive.ui.ui_state",
  "openpilot.selfdrive.ui.layouts.main",
  "openpilot.selfdrive.ui.mici.layouts.main",
  "openpilot.selfdrive.ui.sunnypilot.layouts.settings.settings",
  "openpilot.selfdrive.ui.sunnypilot.mici.layouts.settings",
  "openpilot.system.ui.lib.wifi_manager",
)
# BluePilot's build wrapper: launch_chffrplus.sh falls back to the stock build when it can't import
OTHER = ("openpilot.system.manager.bp_build",)
# BluePilot imports that upstream files make inside functions: (module, name)
FUNCTION_IMPORTS = (
  ("openpilot.bluepilot.system.ui.lib.bp_wifi", "SUPPORTS_WPA_EXTENDED"),  # wifi_manager
  ("openpilot.selfdrive.ui.bp.lib.wifi_favorite", "WifiFavoriteManager"),  # wifi_manager
  ("openpilot.selfdrive.ui.bp.widgets.platform_selector_bp", "PlatformSelectorBP"),  # vehicle settings
  ("openpilot.selfdrive.ui.bp.widgets.platform_selector_bp", "LegendWidgetBP"),  # vehicle settings
  ("bluepilot.backend_switch", "get_connect_backend"),  # pairing dialogs
  ("bluepilot.backend_switch", "pairing_host"),  # pairing dialogs
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
    for name in modules + list(GATED) + list(OTHER):
      with self.subTest(module=name):
        importlib.import_module(name)
    for module, name in FUNCTION_IMPORTS:
      with self.subTest(module=module, name=name):
        self.assertTrue(hasattr(importlib.import_module(module), name))


if __name__ == "__main__":
  unittest.main()
