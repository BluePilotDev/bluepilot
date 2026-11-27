"""The portal's settings page reads flags and categories from params_keys.h; backup → restore is lossless (#288)."""
import datetime
import json
import os
import tempfile
import unittest
from unittest import mock

from openpilot.common.params import Params, ParamKeyType

from bluepilot.backend.params import params_manager


def backup_keys_in_header():
  return sorted(key for key, flags in params_manager._load_param_attributes_cache().items() if "BACKUP" in flags)


def non_default_value(params, key):
  """A value of the param's declared type that differs from its default."""
  t = params.get_type(key)
  default = params.get_default_value(key)
  if t == ParamKeyType.BOOL:
    return not default
  if t == ParamKeyType.INT:
    return (default or 0) + 7
  if t == ParamKeyType.FLOAT:
    return (default or 0.0) + 0.25
  if t == ParamKeyType.STRING:
    return f"restored {key}"
  if t == ParamKeyType.JSON:
    return {"key": key, "list": [1, 2.5, "three"], "nested": {"on": True}}
  if t == ParamKeyType.TIME:
    return datetime.datetime(2026, 11, 26, 8, 30, 15)
  if t == ParamKeyType.BYTES:
    return b"\x00\xff\x10 " + key.encode()
  raise AssertionError(f"{key}: unhandled type {t!r}")


class ParamsTestCase(unittest.TestCase):
  def setUp(self):
    tmp = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)  # Params may still be writing in the background
    self.addCleanup(tmp.cleanup)
    patcher = mock.patch.dict(os.environ, {"PARAMS_ROOT": tmp.name})
    patcher.start()
    self.addCleanup(patcher.stop)
    self.params = Params()
    self.assertTrue(self.params.get_param_path().startswith(tmp.name))


class TestAttributesFromHeader(unittest.TestCase):
  def test_known_param_has_its_header_flags(self):
    attributes = params_manager._load_param_attributes_cache()
    self.assertIn("PERSISTENT", attributes.get("IsMetric", []))
    self.assertIn("BACKUP", attributes.get("IsMetric", []))


class TestBackupRoundTrip(ParamsTestCase):
  def test_every_backup_param_survives_export_and_restore(self):
    keys = backup_keys_in_header()
    self.assertGreater(len(keys), 100)
    expected = {key: non_default_value(self.params, key) for key in keys}
    for key, value in expected.items():
      self.params.put(key, value, block=True)

    # the page saves the export as a JSON file and posts it back
    exported = json.loads(json.dumps(params_manager.export_backup_params(self.params)))
    self.assertEqual(sorted(exported), keys)

    for key in keys:
      self.params.remove(key)
      self.assertIsNone(self.params.get(key), key)

    result = params_manager.restore_backup_params(exported, self.params, onroad=False)
    self.assertEqual(result["failed"], [])
    self.assertEqual(result["skipped"], [])
    self.assertEqual(sorted(result["restored"]), keys)
    for key, value in expected.items():
      restored = self.params.get(key)
      self.assertEqual(restored, value, key)
      self.assertIs(type(restored), type(value), key)


class TestRestoreRules(ParamsTestCase):
  def test_restore_is_refused_onroad(self):
    result = params_manager.restore_backup_params({"IsMetric": True}, self.params, onroad=True)
    self.assertFalse(result["success"])
    self.assertIn("driving", result["error"])
    self.assertIsNone(self.params.get("IsMetric"))

  def test_param_without_backup_flag_is_skipped(self):
    self.assertNotIn("BACKUP", params_manager._load_param_attributes_cache()["BPUIDebugLog"])
    result = params_manager.restore_backup_params({"BPUIDebugLog": True}, self.params, onroad=False)
    self.assertEqual(result["skipped"], ["BPUIDebugLog"])
    self.assertIsNone(self.params.get("BPUIDebugLog"))

  def test_unknown_name_is_reported_and_the_rest_restored(self):
    backup = {"NoSuchParamFromBP7": 1, "IsMetric": True, "custom_path_offset_curv": 0.3}
    result = params_manager.restore_backup_params(backup, self.params, onroad=False)
    self.assertEqual(result["skipped"], ["NoSuchParamFromBP7"])
    self.assertEqual(sorted(result["restored"]), ["IsMetric", "custom_path_offset_curv"])
    self.assertIs(self.params.get("IsMetric"), True)
    self.assertEqual(self.params.get("custom_path_offset_curv"), 0.3)

  def test_value_of_the_wrong_type_fails_alone(self):
    result = params_manager.restore_backup_params({"IsMetric": "yes", "FordPrefLateralControl": 1}, self.params, onroad=False)
    self.assertEqual([f["param"] for f in result["failed"]], ["IsMetric"])
    self.assertEqual(result["restored"], ["FordPrefLateralControl"])
    self.assertIsNone(self.params.get("IsMetric"))

  def test_whole_number_float_from_the_browser_is_restored_as_float(self):
    # JavaScript writes 2.0 as 2
    params_manager.restore_backup_params({"custom_path_offset_curv": 2}, self.params, onroad=False)
    restored = self.params.get("custom_path_offset_curv")
    self.assertEqual(restored, 2.0)
    self.assertIs(type(restored), float)


if __name__ == "__main__":
  unittest.main()
