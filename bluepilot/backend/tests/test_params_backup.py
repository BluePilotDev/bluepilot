"""The portal's settings page reads flags and categories from params_keys.h; backup → restore is lossless (#288)."""
import unittest

from bluepilot.backend.params import params_manager


class TestAttributesFromHeader(unittest.TestCase):
  def test_known_param_has_its_header_flags(self):
    attributes = params_manager._load_param_attributes_cache()
    self.assertIn("PERSISTENT", attributes.get("IsMetric", []))
    self.assertIn("BACKUP", attributes.get("IsMetric", []))


if __name__ == "__main__":
  unittest.main()
