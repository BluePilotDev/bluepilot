"""tools/vendor.sh and tools/check_vendored.sh (#24), on a scratch library repo: a fresh copy is the tagged commit
without .github/ plus VERSION, and the check passes on it and fails on an edited, added or removed file."""
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1] / "tools"


def git(repo, *args):
  return subprocess.run(["git", "-C", str(repo), "-c", "user.name=t", "-c", "user.email=t@t", *args],
                        check=True, capture_output=True, text=True).stdout.strip()


@unittest.skipUnless(shutil.which("git") and shutil.which("bash"), "needs git and bash")
class TestVendorTools(unittest.TestCase):
  def setUp(self):
    self.tmp = Path(tempfile.mkdtemp())
    self.lib = self.tmp / "lib"
    (self.lib / "tools").mkdir(parents=True)
    for name in ("vendor.sh", "check_vendored.sh"):
      shutil.copy(TOOLS / name, self.lib / "tools" / name)
    (self.lib / "core").mkdir()
    (self.lib / "core" / "a.py").write_text("A = 1\n")
    (self.lib / ".github").mkdir()
    (self.lib / ".github" / "ci.yml").write_text("ci\n")
    (self.lib / ".gitattributes").write_text(".github export-ignore\n")
    git(self.lib, "init", "-q")
    git(self.lib, "add", "-A")
    git(self.lib, "commit", "-q", "-m", "lib")
    git(self.lib, "tag", "v9.9.9")
    self.commit = git(self.lib, "rev-parse", "HEAD")
    self.copy = self.tmp / "host" / "pkg" / "bluepilot_lateral"

  def tearDown(self):
    shutil.rmtree(self.tmp)

  def run_tool(self, *args):
    return subprocess.run(["bash", *args], capture_output=True, text=True)

  def vendor(self, ref="v9.9.9"):
    r = self.run_tool(str(self.lib / "tools" / "vendor.sh"), ref, str(self.copy))
    self.assertEqual(r.returncode, 0, r.stderr)

  def check(self):
    return self.run_tool(str(self.lib / "tools" / "check_vendored.sh"), str(self.copy))

  def test_copy_is_the_tag_with_its_version(self):
    self.vendor()
    self.assertEqual((self.copy / "VERSION").read_text(), f"v9.9.9 {self.commit}\n")
    self.assertEqual((self.copy / "core" / "a.py").read_text(), "A = 1\n")
    self.assertFalse((self.copy / ".github").exists())
    r = self.check()
    self.assertEqual(r.returncode, 0, r.stderr)

  def test_untagged_ref(self):
    (self.lib / "core" / "b.py").write_text("B = 2\n")
    git(self.lib, "add", "-A")
    git(self.lib, "commit", "-q", "-m", "after the tag")
    self.vendor("HEAD")
    self.assertTrue((self.copy / "VERSION").read_text().startswith("untagged "))

  def test_vendor_replaces_the_folder(self):
    self.copy.mkdir(parents=True)
    (self.copy / "stale.py").write_text("old\n")
    self.vendor()
    self.assertFalse((self.copy / "stale.py").exists())

  def test_caches_are_ignored(self):
    self.vendor()
    (self.copy / "core" / "__pycache__").mkdir()
    (self.copy / "core" / "__pycache__" / "a.cpython-312.pyc").write_bytes(b"\0")
    self.assertEqual(self.check().returncode, 0)

  def test_drift_fails(self):
    for change in ("edit", "add", "remove"):
      with self.subTest(change=change):
        self.vendor()
        if change == "edit":
          (self.copy / "core" / "a.py").write_text("A = 2\n")
        elif change == "add":
          (self.copy / "core" / "extra.py").write_text("X = 0\n")
        else:
          (self.copy / "core" / "a.py").unlink()
        r = self.check()
        self.assertEqual(r.returncode, 1, r.stdout)
        self.assertRegex(r.stderr, "differs")


if __name__ == "__main__":
  unittest.main()
