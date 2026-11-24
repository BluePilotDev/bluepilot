"""BluePilot: the BluePilot safety tests live in the library (opendbc/bluepilot_lateral/safety/tests/).
This loader hands them to upstream's discovery, so test.sh and mutation.py run them and the required
safety job keeps modes/ford.h's hook lines covered (#63). mutation.py catalogs each test module two
levels deep (suite -> class -> test), so they come back in that shape."""
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
LIBRARY_SAFETY_TESTS = ROOT / "opendbc" / "bluepilot_lateral" / "safety" / "tests"


def _class_suites(suite):
  for test in suite:
    if isinstance(test, unittest.TestCase):
      yield unittest.TestSuite([test])
    elif any(isinstance(child, unittest.TestSuite) for child in test):
      yield from _class_suites(test)
    else:
      yield test


def load_tests(loader, standard_tests, pattern):
  # a fresh loader: discover() inside another discover() would share the outer one's top-level dir.
  # It keeps the outer one's -k patterns.
  inner = unittest.TestLoader()
  inner.testNamePatterns = loader.testNamePatterns
  found = inner.discover(str(LIBRARY_SAFETY_TESTS), top_level_dir=str(ROOT))
  return unittest.TestSuite(_class_suites(found))
