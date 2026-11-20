"""The portal reads openpilot's preserve marker; it no longer sets or clears it (#236)."""
import os
import tempfile
import unittest
from unittest import mock

from openpilot.system.loggerd.deleter import PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE
from openpilot.system.loggerd import xattr_cache
from openpilot.system.loggerd.xattr_cache import setxattr

from bluepilot.backend.routes import segments  # before storage: the two packages import each other
from bluepilot.backend import storage
from bluepilot.backend.storage import preservation

MARKED = "00000001--abcdef0123"
UNMARKED = "00000002--abcdef0123"


class TestPreserveMarkerIsRead(unittest.TestCase):
  def setUp(self):
    # A fresh folder per test: openpilot's getxattr caches each path's answer.
    tmp = tempfile.TemporaryDirectory()
    self.addCleanup(tmp.cleanup)
    for route in (MARKED, UNMARKED):
      for n in range(2):
        os.mkdir(os.path.join(tmp.name, f"{route}--{n}"))
    # openpilot marks a bookmarked segment by itself; mark segment 1 of one route the same way
    setxattr(os.path.join(tmp.name, f"{MARKED}--1"), PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE)

    for patch in (mock.patch.object(segments, "ROUTES_DIR", tmp.name),
                  mock.patch.object(preservation, "ROUTES_DIR", tmp.name)):
      patch.start()
      self.addCleanup(patch.stop)
    segments.get_route_segments.cache_clear()
    self.addCleanup(segments.get_route_segments.cache_clear)
    self.all_segments = sorted(os.listdir(tmp.name))
    self.tmp = tmp.name

  def test_marked_route_is_reported_preserved(self):
    self.assertTrue(preservation.check_route_preserve_status(MARKED))

  def test_unmarked_route_is_not_reported_preserved(self):
    self.assertFalse(preservation.check_route_preserve_status(UNMARKED))

  def test_marker_set_after_the_first_look_is_seen(self):
    # loggerd marks a segment from another process, possibly after the portal already listed the route
    self.assertFalse(preservation.check_route_preserve_status(UNMARKED))
    setxattr(os.path.join(self.tmp, f"{UNMARKED}--0"), PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE)
    xattr_cache._cached_attributes[(os.path.join(self.tmp, f"{UNMARKED}--0"), PRESERVE_ATTR_NAME)] = None  # what another process's write leaves behind
    self.assertTrue(preservation.check_route_preserve_status(UNMARKED))

  def test_marked_segment_and_the_ones_before_it_are_held_back_from_deletion(self):
    self.assertEqual(preservation.get_preserved_segments_set(self.all_segments),
                     {f"{MARKED}--0", f"{MARKED}--1"})


class TestPreserveWriteSideIsGone(unittest.TestCase):
  def test_storage_has_no_preserve_setter(self):
    self.assertFalse(hasattr(preservation, "set_route_preserve"))
    self.assertNotIn("set_route_preserve", storage.__all__)


if __name__ == "__main__":
  unittest.main()
