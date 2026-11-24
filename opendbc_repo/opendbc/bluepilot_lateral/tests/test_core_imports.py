import ast
import unittest
from pathlib import Path

LIBRARY = Path(__file__).resolve().parents[1]
# the host-independent parts: only hosts/ (an adapter per host) may know a host's types
CORE_DIRS = ("core", "can", "params", "safety")
# The library reaches its host's opendbc package only through relative imports (HOSTS.md, "Mounting the
# library"), so it runs under whatever name the host gives that package (opendbc, iqdbc). An absolute
# import of any of these anywhere in the library ties it to one host.
HOST_PACKAGES = ("opendbc", "iqdbc", "openpilot", "iqpilot", "cereal")
# host trees the core must not use, resolved against the host package root
FORBIDDEN_HOST_TREES = ("sunnypilot", "lvbs")


def _package(path):
  """The module's package, as parts below the host package root: ['bluepilot_lateral', 'core']."""
  return ["bluepilot_lateral", *path.relative_to(LIBRARY).parts[:-1]]


def _imports(path):
  """Every module an import names, absolute, under <host> for the host package root. Relative imports
  are resolved; importing a name from a package also yields package.name (from .. import hosts)."""
  pkg = _package(path)
  tree = ast.parse(path.read_text())
  for node in ast.walk(tree):
    if isinstance(node, ast.Import):
      yield from (alias.name for alias in node.names)
    elif isinstance(node, ast.ImportFrom):
      if node.level:
        base = pkg[:len(pkg) - (node.level - 1)]
        module = ".".join(["<host>", *base, *([node.module] if node.module else [])])
      else:
        module = node.module or ""
      yield module
      yield from (f"{module}.{alias.name}" for alias in node.names)


class TestCoreImports(unittest.TestCase):
  def test_core_imports_no_host(self):
    files = [p for d in CORE_DIRS for p in sorted((LIBRARY / d).rglob("*.py"))]
    self.assertTrue(files)
    forbidden = ("<host>.bluepilot_lateral.hosts", *(f"<host>.{t}" for t in FORBIDDEN_HOST_TREES))
    for path in files:
      rel = path.relative_to(LIBRARY)
      for module in _imports(path):
        self.assertFalse(module.startswith(forbidden), f"{rel} imports {module}")

  def test_no_absolute_host_imports(self):
    files = [p for p in sorted(LIBRARY.rglob("*.py")) if ".git" not in p.parts]
    self.assertTrue(files)
    for path in files:
      rel = path.relative_to(LIBRARY)
      for module in _imports(path):
        self.assertFalse(module.split(".")[0] in HOST_PACKAGES, f"{rel} imports {module}: use a relative import")


if __name__ == "__main__":
  unittest.main()
