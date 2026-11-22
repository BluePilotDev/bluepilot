"""The AGNOS update screen bundled in this tree (openpilot/common/hardware/comma/updater, a zipapp packed by
tools/release/pack.py) holds only files of the tree and no absolute home path.

pack.py copies every file of its kinds under openpilot/ from the working tree it runs in, ignored build outputs
included, so a bundle packed from a builder's tree can carry that tree's generated files and the paths written into
them. This test reads the committed bundle: every file in it must be one git tracks here (directly, through a tracked
symlink, or in a submodule at its recorded commit), and no file may hold a path under a home directory."""
import os
import re
import subprocess
import unittest
import zipfile
from pathlib import Path

from openpilot.common.basedir import BASEDIR

BUNDLE = Path(BASEDIR) / "openpilot/common/hardware/comma/updater"
HOME_PATH = re.compile(rb"/(?:home|Users)/[A-Za-z0-9._-]+/|/root/")


def tracked(root: Path) -> set[str]:
  """Every file git tracks in root and in its checked-out submodules, as paths relative to root."""
  out = subprocess.run(["git", "-C", str(root), "ls-files", "-z", "--recurse-submodules"], capture_output=True, check=True)
  return {p for p in out.stdout.decode().split("\0") if p}


def missing_submodules(root: Path) -> list[str]:
  """The submodules the tree records but has not checked out (git marks them '-'), as paths relative to root."""
  out = subprocess.run(["git", "-C", str(root), "submodule", "status", "--recursive"], capture_output=True, check=True)
  return [line[1:].split()[1] for line in out.stdout.decode().splitlines() if line.startswith("-")]


class TestUpdaterBundle(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    root = Path(BASEDIR)
    if not (root / ".git").exists():
      raise unittest.SkipTest("not a git checkout")
    cls.root = root.resolve()
    cls.files = tracked(cls.root)
    cls.missing = missing_submodules(cls.root)
    cls.zip = zipfile.ZipFile(BUNDLE)

  def test_every_file_is_one_the_tree_tracks(self):
    # a submodule not checked out has no tracked files to compare its bundled ones with: skip, as comma's tests do
    # without their submodules' data
    missing = [m for m in self.missing if any(name.startswith(m + "/") for name in self.zip.namelist())]
    if missing:
      self.skipTest(f"submodules not checked out: {', '.join(missing)}")
    extra = []
    for name in self.zip.namelist():
      if name.endswith("/") or name == "__main__.py":
        continue
      # pack.py follows symlinks: the bundled path is the file the tree's path resolves to
      real = Path(os.path.realpath(self.root / name))
      rel = real.relative_to(self.root).as_posix() if real.is_relative_to(self.root) else None
      if name not in self.files and rel not in self.files:
        extra.append(name)
    self.assertEqual(extra, [], f"{len(extra)} files in the bundle that the tree does not track: pack it from a clean checkout")

  def test_no_absolute_home_path(self):
    found = [name for name in self.zip.namelist() if HOME_PATH.search(self.zip.read(name))]
    self.assertEqual(found, [], "files in the bundle with a path under a home directory")


if __name__ == "__main__":
  unittest.main()
