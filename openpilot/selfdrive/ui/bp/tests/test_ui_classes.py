"""BluePilot UI classes against the upstream classes they extend (no window: fonts and textures stubbed)."""
import ast
import importlib
import inspect
import textwrap
import unittest
from unittest import mock

import pyray as rl

from openpilot.selfdrive.ui.bp.tests.test_bluepilot_imports import bluepilot_modules
from openpilot.system.ui.lib.application import gui_app


def is_bluepilot_class(cls) -> bool:
  return cls.__module__.startswith(("bluepilot.", "openpilot.bluepilot.")) or ".bp." in cls.__module__


def self_attributes(func_or_cls) -> tuple[set[str], set[str]]:
  """(names read as self.<name>, names assigned as self.<name>) in the source."""
  try:
    tree = ast.parse(textwrap.dedent(inspect.getsource(func_or_cls)))
  except (OSError, TypeError):
    return set(), set()
  read, assigned = set(), set()
  for node in ast.walk(tree):
    if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "self":
      (assigned if isinstance(node.ctx, ast.Store) else read).add(node.attr)
  return read, assigned


def unresolved_upstream_references(cls) -> dict[str, str]:
  """self.<name> used by an upstream method that runs on cls (not overridden), but that nothing in cls's MRO
  defines or assigns. Such a name usually lives on a sunnypilot subclass the BluePilot class doesn't extend."""
  assigned = set().union(*(self_attributes(c)[1] for c in cls.__mro__ if c is not object))
  missing = {}
  for base in cls.__mro__[1:]:
    if is_bluepilot_class(base) or base.__module__ in ("builtins", "abc", "enum", "typing"):
      continue
    for name, func in vars(base).items():
      if not inspect.isfunction(func) or getattr(cls, name, None) is not func:
        continue  # overridden further down the MRO: never runs on cls
      for attr in self_attributes(func)[0]:
        if not hasattr(cls, attr) and attr not in assigned:
          missing[attr] = f"{base.__qualname__}.{name}"
  return missing


class TestUIClasses(unittest.TestCase):
  def setUp(self):
    for patch in (mock.patch.object(gui_app, "font", return_value=rl.Font()),
                  mock.patch.object(gui_app, "texture", return_value=rl.Texture())):
      patch.start()
      self.addCleanup(patch.stop)

  def test_comma_four_home_layout(self):
    from openpilot.selfdrive.ui.bp.mici.layouts.home import BpMiciHomeLayout
    home = BpMiciHomeLayout()
    self.assertEqual(home._openpilot_label.text, "bluepilot")
    with mock.patch("pyray.get_time", return_value=0.0):
      home._set_chestnut_visibility()  # upstream's mici home _render calls this under the sunnypilot UI

  def test_upstream_methods_find_what_they_use(self):
    checked = 0
    for module_name in bluepilot_modules():
      module = importlib.import_module(module_name)
      for _, cls in inspect.getmembers(module, inspect.isclass):
        if cls.__module__ != module.__name__ or not is_bluepilot_class(cls):
          continue
        checked += 1
        with self.subTest(cls=f"{cls.__module__}.{cls.__qualname__}"):
          self.assertEqual(unresolved_upstream_references(cls), {})
    self.assertGreater(checked, 50)


if __name__ == "__main__":
  unittest.main()
