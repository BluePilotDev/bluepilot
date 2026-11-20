"""Every test file in BluePilot's folders gives the unit tests job at least one test (#238).

The test runner loads tests with unittest, so a file written for pytest, or a script named test_*.py,
contributes nothing and the job still passes. Collected here the way tools/test_runner.py collects them."""
import importlib.util
import unittest
from pathlib import Path

from openpilot.common.basedir import BASEDIR

ROOT = Path(BASEDIR)
FOLDERS = ("bluepilot", "openpilot/selfdrive/ui/bp")


def runner_collect():
  spec = importlib.util.spec_from_file_location("bp_test_runner", ROOT / "tools/test_runner.py")
  runner = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(runner)
  return runner.collect


class TestEveryTestFileRuns(unittest.TestCase):
  def test_every_test_file_contributes_a_test(self):
    collect = runner_collect()
    files = [path for folder in FOLDERS for path in sorted((ROOT / folder).rglob("test_*.py")) if "vendor" not in path.parts]
    self.assertGreater(len(files), 15)  # the folders were found
    for path in files:
      with self.subTest(file=str(path.relative_to(ROOT))):
        tests, errors = collect([str(path)], None)
        self.assertEqual(errors, [])
        self.assertTrue(tests, "no unittest tests in this file: write them as unittest.TestCase methods")
