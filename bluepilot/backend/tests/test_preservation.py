"""Route preservation from the portal: mark, unmark and mark again on a temp routes folder."""
import os
import tempfile
import unittest
from unittest import mock

from openpilot.system.loggerd.deleter import PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE
from openpilot.system.loggerd.xattr_cache import _getxattr

from bluepilot.backend.routes import segments
from bluepilot.backend.storage import preservation

ROUTE = "00000001--abcdef0123"


class TestRoutePreservation(unittest.TestCase):
  def setUp(self):
    tmp = tempfile.TemporaryDirectory()
    self.addCleanup(tmp.cleanup)
    self.segment_paths = []
    for n in range(3):
      path = os.path.join(tmp.name, f"{ROUTE}--{n}")
      os.mkdir(path)
      self.segment_paths.append(path)

    for patch in (mock.patch.object(segments, "ROUTES_DIR", tmp.name),
                  mock.patch.object(preservation, "get_disk_space_info",
                                    return_value={'can_record': True, 'warning_level': 'ok'})):
      patch.start()
      self.addCleanup(patch.stop)
    segments.get_route_segments.cache_clear()
    self.addCleanup(segments.get_route_segments.cache_clear)

  def marked_segments(self):
    marked = []
    for path in self.segment_paths:
      try:
        if _getxattr(path, PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE:
          marked.append(path)
      except OSError:
        pass
    return marked

  def test_mark_unmark_and_mark_again(self):
    self.assertFalse(preservation.check_route_preserve_status(ROUTE))

    self.assertTrue(preservation.set_route_preserve(ROUTE, True)['success'])
    self.assertTrue(preservation.check_route_preserve_status(ROUTE))
    self.assertEqual(self.marked_segments(), self.segment_paths)

    self.assertTrue(preservation.set_route_preserve(ROUTE, False)['success'])
    self.assertFalse(preservation.check_route_preserve_status(ROUTE))
    self.assertEqual(self.marked_segments(), [])

    self.assertTrue(preservation.set_route_preserve(ROUTE, True)['success'])
    self.assertTrue(preservation.check_route_preserve_status(ROUTE))
    self.assertEqual(self.marked_segments(), self.segment_paths)

  def test_unmark_a_route_that_was_never_marked(self):
    self.assertTrue(preservation.set_route_preserve(ROUTE, False)['success'])
    self.assertFalse(preservation.check_route_preserve_status(ROUTE))
    self.assertEqual(self.marked_segments(), [])


if __name__ == "__main__":
  unittest.main()
