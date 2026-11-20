"""The bluepilot-hooks check (.github/bluepilot_hooks.py, #224), run against a throwaway git repo."""
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from openpilot.common.basedir import BASEDIR

SCRIPT = Path(BASEDIR) / ".github/bluepilot_hooks.py"
GIT_ENV = {"GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t", "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@t"}


class TestBluePilotHooks(unittest.TestCase):
  def setUp(self):
    tmp = tempfile.TemporaryDirectory()
    self.addCleanup(tmp.cleanup)
    self.repo = Path(tmp.name)
    self.git("init", "-q", "-b", "bp-sunnypilot")
    self.write("upstream.py", "x = 1\n")
    self.write("listed.py", "x = 1\n")
    self.commit("upstream")
    self.git("checkout", "-q", "-b", "work")
    self.write(".github/bluepilot-hooks.txt", "listed.py  # a hook\n")

  def git(self, *args):
    return subprocess.run(["git", "-c", "commit.gpgsign=false", *args], cwd=self.repo, check=True, capture_output=True, text=True,
                          env={**os.environ, **GIT_ENV}).stdout

  def write(self, path, text):
    (self.repo / path).parent.mkdir(parents=True, exist_ok=True)
    (self.repo / path).write_text(text)

  def commit(self, message):
    self.git("add", "-A")
    self.git("commit", "-q", "--allow-empty", "-m", message)

  def check(self):
    self.commit("work")
    return subprocess.run([sys.executable, str(SCRIPT), "bp-sunnypilot"], cwd=self.repo, capture_output=True, text=True)

  def test_change_to_a_listed_upstream_file_passes(self):
    self.write("listed.py", "x = 2\n")
    result = self.check()
    self.assertEqual(result.returncode, 0, result.stdout)

  def test_change_to_an_unlisted_upstream_file_fails(self):
    self.write("listed.py", "x = 2\n")
    self.write("upstream.py", "x = 2\n")
    result = self.check()
    self.assertEqual(result.returncode, 1)
    self.assertIn("::error file=upstream.py::", result.stdout)

  def test_new_bluepilot_file_passes(self):
    self.write("listed.py", "x = 2\n")
    self.write("bluepilot/new.py", "x = 1\n")
    self.assertEqual(self.check().returncode, 0)

  def test_deleting_an_unlisted_upstream_file_fails(self):
    self.write("listed.py", "x = 2\n")
    (self.repo / "upstream.py").unlink()
    result = self.check()
    self.assertEqual(result.returncode, 1)
    self.assertIn("::error file=upstream.py::", result.stdout)

  def test_moved_submodule_pointer_must_be_listed(self):
    self.git("checkout", "-q", "bp-sunnypilot")
    self.git("update-index", "--add", "--cacheinfo", f"160000,{'1' * 40},opendbc_repo")
    self.git("commit", "-q", "-m", "submodule")
    self.git("checkout", "-q", "work")
    self.git("merge", "-q", "bp-sunnypilot")
    self.write("listed.py", "x = 2\n")
    self.git("update-index", "--cacheinfo", f"160000,{'2' * 40},opendbc_repo")
    self.assertEqual(self.check().returncode, 1)
    self.write(".github/bluepilot-hooks.txt", "listed.py\nopendbc_repo  # bp-full\n")
    self.assertEqual(self.check().returncode, 0)

  def test_upstream_commits_not_yet_merged_dont_count(self):
    self.write("listed.py", "x = 2\n")
    self.commit("work")
    self.git("checkout", "-q", "bp-sunnypilot")
    self.write("upstream.py", "x = 3\n")
    self.commit("upstream moves ahead")
    self.git("checkout", "-q", "work")
    self.assertEqual(self.check().returncode, 0)

  def test_listed_file_that_matches_upstream_again_only_warns(self):
    result = self.check()
    self.assertEqual(result.returncode, 0)
    self.assertIn("::warning file=.github/bluepilot-hooks.txt::listed.py", result.stdout)
