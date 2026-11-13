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


def _tree(func_or_cls):
  try:
    return ast.parse(textwrap.dedent(inspect.getsource(func_or_cls)))
  except (OSError, TypeError):
    return ast.Module(body=[], type_ignores=[])


def self_attributes(func_or_cls) -> tuple[set[str], set[str]]:
  """(names read as self.<name>, names assigned as self.<name>) in the source; reads guarded by
  hasattr(self, "<name>") don't count."""
  read, assigned, guarded = set(), set(), set()
  for node in ast.walk(_tree(func_or_cls)):
    if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "self":
      (assigned if isinstance(node.ctx, ast.Store) else read).add(node.attr)
    elif (isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "hasattr" and len(node.args) == 2
          and isinstance(node.args[0], ast.Name) and node.args[0].id == "self" and isinstance(node.args[1], ast.Constant)):
      guarded.add(node.args[1].value)
  return read - guarded, assigned


def super_attributes(func) -> set[str]:
  """Names used as super().<name> in the source."""
  return {node.attr for node in ast.walk(_tree(func))
          if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Call)
          and isinstance(node.value.func, ast.Name) and node.value.func.id == "super"}


def upstream_bases(cls) -> list[type]:
  return [b for b in cls.__mro__[1:] if not is_bluepilot_class(b) and b.__module__ not in ("builtins", "abc", "enum", "typing")]


def unresolved_references(cls) -> dict[str, str]:
  """Names that don't resolve on a BluePilot class with an upstream base, usually because upstream renamed or
  moved them (often onto a sunnypilot subclass the BluePilot class doesn't extend):
  - self.<name> used by an upstream method that runs on cls (not overridden) or by cls's own methods, that nothing
    in cls's MRO defines or assigns;
  - super().<name> in cls's own methods that no class after cls in its MRO defines."""
  assigned = set().union(*(self_attributes(c)[1] for c in cls.__mro__ if c is not object))
  after_cls = cls.__mro__[1:]
  missing = {}
  for owner in [cls, *upstream_bases(cls)]:
    for name, func in vars(owner).items():
      if not inspect.isfunction(func) or (owner is not cls and getattr(cls, name, None) is not func):
        continue  # overridden further down the MRO: never runs on cls
      for attr in self_attributes(func)[0]:
        if not hasattr(cls, attr) and attr not in assigned:
          missing[f"self.{attr}"] = f"{owner.__qualname__}.{name}"
      if owner is cls:
        for attr in super_attributes(func):
          if not any(attr in vars(c) for c in after_cls):
            missing[f"super().{attr}"] = f"{owner.__qualname__}.{name}"
  return missing


def sized_texture(_path, width=None, height=None, *args, **kwargs) -> rl.Texture:
  texture = rl.Texture()
  texture.width, texture.height = width or 0, height or 0
  return texture


class TestUIClasses(unittest.TestCase):
  def setUp(self):
    for patch in (mock.patch.object(gui_app, "font", return_value=rl.Font()),
                  mock.patch.object(gui_app, "texture", side_effect=sized_texture)):
      patch.start()
      self.addCleanup(patch.stop)

  def test_comma_four_home_layout(self):
    from openpilot.selfdrive.ui.bp.mici.layouts.home import BpMiciHomeLayout
    home = BpMiciHomeLayout()
    self.assertEqual(home._openpilot_label.text, "bluepilot")
    with mock.patch("pyray.get_time", return_value=0.0):
      home._set_chestnut_visibility()  # upstream's mici home _render calls this under the sunnypilot UI

  def test_comma_four_big_button_text_widths_match_bp7(self):
    from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigButtonBP, BigMultiToggleBP
    # upstream's _draw_content asks for these two widths (it had one, _width_hint, in BP7)
    plain = BigButtonBP("Lateral", "value")
    self.assertEqual((plain._title_width_hint(), plain._subtitle_width_hint()), (402 - 2 * 40,) * 2)
    icon = rl.Texture()
    icon.width = 54
    with_icon = BigButtonBP("Lateral", "value", icon=icon)
    self.assertEqual((with_icon._title_width_hint(), with_icon._subtitle_width_hint()), (402 - 2 * 40 - 54 - 20,) * 2)
    multi = BigMultiToggleBP("Mode", ["A", "B"])
    self.assertEqual((multi._title_width_hint(), multi._subtitle_width_hint()), (402 - 2 * 40 - 84,) * 2)  # clear of the pills

  def test_upstream_methods_find_what_they_use(self):
    checked = 0
    for module_name in bluepilot_modules():
      module = importlib.import_module(module_name)
      for _, cls in inspect.getmembers(module, inspect.isclass):
        if cls.__module__ != module.__name__ or not is_bluepilot_class(cls) or not upstream_bases(cls):
          continue
        checked += 1
        with self.subTest(cls=f"{cls.__module__}.{cls.__qualname__}"):
          self.assertEqual(unresolved_references(cls), {})
    self.assertGreater(checked, 50)


if __name__ == "__main__":
  unittest.main()
