import logging
import os
import tempfile
import unittest

from bluepilot.backend.core.logging_handler import portal_file_handler
from bluepilot.backend.utils import file_ops


class TestUnexpectedErrorsKeepTraceback(unittest.TestCase):
  def test_failed_atomic_write_logs_the_traceback(self):
    with tempfile.TemporaryDirectory() as tmp:
      not_a_dir = os.path.join(tmp, "file")
      open(not_a_dir, "w").close()

      with self.assertLogs(file_ops.logger, level="ERROR") as logs:
        self.assertFalse(file_ops.atomic_write(os.path.join(not_a_dir, "target.json"), "{}"))

    self.assertEqual(len(logs.records), 1)
    self.assertIsNotNone(logs.records[0].exc_info)


class TestPortalLogFileIsCapped(unittest.TestCase):
  def test_log_rotates_at_2_mb_keeping_one_old_copy(self):
    with tempfile.TemporaryDirectory() as tmp:
      log_file = os.path.join(tmp, "bp_portal.log")
      handler = portal_file_handler(log_file)
      logger = logging.getLogger("test_tracebacks.portal_log")
      logger.propagate = False
      logger.setLevel(logging.INFO)
      logger.addHandler(handler)
      try:
        for _ in range(5000):  # about 5 MB
          logger.info("x" * 1000)
      finally:
        logger.removeHandler(handler)
        handler.close()

      self.assertLessEqual(os.path.getsize(log_file), 2 * 1024 * 1024)
      self.assertTrue(os.path.exists(log_file + ".1"))
      self.assertFalse(os.path.exists(log_file + ".2"))


if __name__ == "__main__":
  unittest.main()
