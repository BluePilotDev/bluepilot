import tempfile
import unittest
from pathlib import Path

from openpilot.selfdrive.test.process_replay.bp_refs import bp_ref_repo


class TestBpRefRepo(unittest.TestCase):
  def setUp(self):
    tmp = tempfile.TemporaryDirectory()
    self.addCleanup(tmp.cleanup)
    self.root = Path(tmp.name)

  def test_bp_sunnypilot_tree_reads_ford_op(self):
    self.assertEqual(bp_ref_repo({}, str(self.root)), "ford-op/bp-sunnypilot")

  def test_full_bluepilot_tree_reads_its_own_repo(self):
    (self.root / "BPVERSION").write_text("7.1\n")
    self.assertEqual(bp_ref_repo({}, str(self.root)), "BluePilotDev/bluepilot")

  def test_ci_repo_does_not_decide(self):
    # a fork's CI (or the mirror's tree) has no refs of its own: the tree picks the source
    self.assertEqual(bp_ref_repo({"GITHUB_REPOSITORY": "someone/bluepilot"}, str(self.root)), "ford-op/bp-sunnypilot")

  def test_override_wins(self):
    (self.root / "BPVERSION").write_text("7.1\n")
    self.assertEqual(bp_ref_repo({"BP_REF_REPO": "someone/bluepilot"}, str(self.root)), "someone/bluepilot")

  def test_empty_override_is_ignored(self):
    self.assertEqual(bp_ref_repo({"BP_REF_REPO": ""}, str(self.root)), "ford-op/bp-sunnypilot")


if __name__ == "__main__":
  unittest.main()
