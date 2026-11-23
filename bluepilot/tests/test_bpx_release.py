"""The BluePilot release job's rules (.github/bpx_release.py, #227)."""
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from openpilot.common.basedir import BASEDIR

GITHUB_DIR = Path(BASEDIR) / ".github"
GIT_ENV = {"GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t", "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@t"}


def load_script():
  sys.path.insert(0, str(GITHUB_DIR))  # the script imports bpx_sync from its own folder
  spec = importlib.util.spec_from_file_location("bpx_release", GITHUB_DIR / "bpx_release.py")
  assert spec and spec.loader
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


rel = load_script()


class TestParseTag(unittest.TestCase):
  def test_a_release_tag(self):
    self.assertEqual(rel.parse_tag("v7.1.0", "v"), (7, 1, 0))
    self.assertEqual(rel.parse_tag("v10.12.3", "v"), (10, 12, 3))

  def test_anything_else_is_refused(self):
    for tag in ("v7.1", "7.1.0", "v7.1.0-rc1", "v7.01.0", "v7.1.0\n", "v7.1.0;rm -rf /", "bpx-test/v7.1.0", ""):
      with self.subTest(tag=tag), self.assertRaisesRegex(rel.Refused, "isn't a release tag: use vMAJOR.MINOR.PATCH"):
        rel.parse_tag(tag, "v")

  def test_another_prefix(self):
    self.assertEqual(rel.parse_tag("bpx-test/v7.1.0", "bpx-test/v"), (7, 1, 0))
    with self.assertRaises(rel.Refused):
      rel.parse_tag("v7.1.0", "bpx-test/v")


A, B, OLD = "a" * 40, "b" * 40, "c" * 40
CFG = rel.Config()


def facts(**kw):
  """A green bp-dev commit A with BPVERSION 7.1.0 and no branches or other tags yet."""
  return rel.Facts(**{"commit": A, "bpversion": "7.1.0", "on_source": True, "version_branch": None, "stable": None,
                      "tags": ["v7.1.0"], "not_green": [], **kw})


class TestNewVersion(unittest.TestCase):
  def test_creates_the_version_branch_and_bp_stable(self):
    self.assertEqual(rel.plan((7, 1, 0), facts(), CFG), [rel.Update("bp-7.1", A, None), rel.Update("bp-stable", A, None)])

  def test_moves_bp_stable_from_the_last_release(self):
    self.assertEqual(rel.plan((7, 2, 0), facts(bpversion="7.2.0", stable=OLD, tags=["v7.1.0", "v7.1.1", "v7.2.0"]), CFG),
                     [rel.Update("bp-7.2", A, None), rel.Update("bp-stable", A, OLD)])

  def test_the_commit_must_be_on_bp_dev(self):
    with self.assertRaisesRegex(rel.Refused, "v7.1.0 is a new version, but aaaaaaaa isn't on bp-dev: tag a bp-dev commit"):
      rel.plan((7, 1, 0), facts(on_source=False), CFG)

  def test_a_version_is_released_once(self):
    with self.assertRaisesRegex(rel.Refused, "bp-7.1 already exists at bbbbbbbb: .*release a fix as a patch tag on bp-7.1"):
      rel.plan((7, 1, 0), facts(version_branch=B), CFG)

  def test_a_repeated_run_writes_nothing(self):
    self.assertEqual(rel.plan((7, 1, 0), facts(version_branch=A, stable=A), CFG), [])

  def test_an_older_new_version_leaves_bp_stable(self):
    # 7.1.0 released after 7.2.0: its branch is created, bp-stable stays on 7.2
    self.assertEqual(rel.plan((7, 1, 0), facts(stable=OLD, tags=["v7.1.0", "v7.2.0"]), CFG), [rel.Update("bp-7.1", A, None)])


class TestPatch(unittest.TestCase):
  def patch(self, **kw):
    return facts(**{"bpversion": "7.1.1", "on_source": False, "version_branch": A, "stable": OLD, "tags": ["v7.1.0", "v7.1.1"], **kw})

  def test_the_latest_version_moves_bp_stable_only(self):
    self.assertEqual(rel.plan((7, 1, 1), self.patch(), CFG), [rel.Update("bp-stable", A, OLD)])

  def test_an_older_version_leaves_bp_stable(self):
    self.assertEqual(rel.plan((7, 1, 1), self.patch(tags=["v7.1.0", "v7.1.1", "v7.2.0"]), CFG), [])

  def test_the_commit_must_be_the_version_branch_tip(self):
    with self.assertRaisesRegex(rel.Refused, "v7.1.1 is a patch, but aaaaaaaa isn't the tip of bp-7.1 .bbbbbbbb.: tag the tip of bp-7.1"):
      rel.plan((7, 1, 1), self.patch(version_branch=B), CFG)

  def test_the_version_branch_must_exist(self):
    with self.assertRaisesRegex(rel.Refused, "v7.1.1 is a patch, but there is no branch bp-7.1: release v7.1.0 first"):
      rel.plan((7, 1, 1), self.patch(version_branch=None), CFG)


class TestEveryRelease(unittest.TestCase):
  def test_the_tag_must_match_bpversion(self):
    for bpversion, shown in (("7.0.0", "7.0.0"), (None, "none")):
      with self.subTest(bpversion=bpversion), self.assertRaisesRegex(
          rel.Refused, f"v7.1.0 doesn't match BPVERSION in aaaaaaaa .{shown}.: set BPVERSION to 7.1.0 in a PR, or tag the commit that has it"):
        rel.plan((7, 1, 0), facts(bpversion=bpversion), CFG)

  def test_the_checks_must_be_green(self):
    msg = "aaaaaaaa's push checks aren't all green .tests / process replay, install smoke.: wait for them and run the job again, or fix them"
    with self.assertRaisesRegex(rel.Refused, msg):
      rel.plan((7, 1, 0), facts(not_green=["tests / process replay", "install smoke"]), CFG)


PUSH, PR = 1, 2  # check suite IDs: one push run, one PR run


def run(name, conclusion="success", suite=PUSH, status="completed"):
  return {"name": name, "status": status, "conclusion": conclusion if status == "completed" else None, "suite": suite}


GREEN = [run(n) for n in ("tests / build release", "tests / static analysis", "tests / unit tests", "tests / process replay", "install smoke")]


class TestNotGreen(unittest.TestCase):
  def test_all_five_passed_on_push(self):
    self.assertEqual(rel.not_green(GREEN + [run("tests / build macOS", "failure"), run("bluepilot-hooks", "failure", PR)], {PUSH}), [])

  def test_a_pr_run_does_not_count(self):
    runs = [r for r in GREEN if r["name"] != "install smoke"] + [run("install smoke", suite=PR)]
    self.assertEqual(rel.not_green(runs, {PUSH}), ["install smoke"])

  def test_missing_running_failed_and_skipped_are_not_green(self):
    runs = [run("tests / static analysis", status="in_progress"), run("tests / unit tests", "failure"),
            run("tests / process replay", "skipped"), run("install smoke")]
    self.assertEqual(rel.not_green(runs, {PUSH}),
                     ["tests / build release", "tests / static analysis", "tests / unit tests", "tests / process replay"])

  def test_every_push_copy_must_pass(self):
    # the same commit pushed to two branches runs twice
    self.assertEqual(rel.not_green(GREEN + [run("install smoke", "failure", 3)], {PUSH, 3}), ["install smoke"])


TEST_ENV = {"SOURCE_BRANCH": "bpx-test/dev", "STABLE_BRANCH": "bpx-test/stable", "BRANCH_PREFIX": "bpx-test/bp-", "TAG_PREFIX": "bpx-test/v"}


class TestConfig(unittest.TestCase):
  def test_defaults(self):
    self.assertEqual(rel.config({"SOURCE_BRANCH": "", "TAG": "v7.1.0"}), rel.Config("bp-dev", "bp-stable", "bp-", "v"))

  def test_throwaway_names(self):
    self.assertEqual(rel.config(TEST_ENV), rel.Config("bpx-test/dev", "bpx-test/stable", "bpx-test/bp-", "bpx-test/v"))
    # reading a real branch is fine: only the names written must be throwaway ones
    self.assertEqual(rel.config({**TEST_ENV, "SOURCE_BRANCH": "bp-7.1-dev"}).source, "bp-7.1-dev")

  def test_an_override_never_writes_a_real_branch(self):
    for env in ({"STABLE_BRANCH": "bp-dev"}, {"TAG_PREFIX": "x"}, {**TEST_ENV, "BRANCH_PREFIX": "bp-"},
                {**TEST_ENV, "STABLE_BRANCH": "bp-stable"}, {"SOURCE_BRANCH": "bp-7.1-dev"}):
      with self.subTest(env=env), self.assertRaisesRegex(rel.Refused, "Other names are for tests only: .* with bpx-test/"):
        rel.config(env)

  def test_names_are_plain(self):
    for bad in ("bpx-test/a:refs/heads/bp-dev", "bpx-test/a b", "bpx-test/$(x)", "bpx-test/a..b"):
      with self.subTest(bad=bad), self.assertRaisesRegex(rel.Refused, "isn't a plain branch or tag name"):
        rel.config({**TEST_ENV, "STABLE_BRANCH": bad})


class Origin:
  """A bare repo standing in for GitHub, a work clone to build history in, and a fresh clone the job runs in."""
  def __init__(self, test):
    tmp = tempfile.TemporaryDirectory()
    test.addCleanup(tmp.cleanup)
    root = Path(tmp.name)
    self.bare, self.work, self.job = root / "origin.git", root / "work", root / "job"
    self.git("init", "-q", "--bare", "-b", "bp-dev", str(self.bare), cwd=root)
    self.git("clone", "-q", str(self.bare), str(self.work), cwd=root)
    self.git("symbolic-ref", "HEAD", "refs/heads/bp-dev")

  def git(self, *args, cwd=None):
    return subprocess.run(["git", "-c", "commit.gpgsign=false", "-c", "tag.gpgsign=false", *args], cwd=cwd or self.work, check=True,
                          capture_output=True, text=True, env={**os.environ, **GIT_ENV}).stdout.strip()

  def commit(self, bpversion, branch="bp-dev"):
    """A commit on the branch's own tip; a new branch starts where the work clone is."""
    if self.git("symbolic-ref", "--short", "HEAD") != branch:
      exists = subprocess.run(["git", "rev-parse", "-q", "--verify", f"refs/heads/{branch}"], cwd=self.work, capture_output=True).returncode == 0
      unborn = subprocess.run(["git", "rev-parse", "-q", "--verify", "HEAD"], cwd=self.work, capture_output=True).returncode != 0
      self.git("checkout", "-q", *([] if exists else ["--orphan"] if unborn else ["-b"]), branch)
    (self.work / "BPVERSION").write_text(bpversion + "\n")
    self.git("add", "BPVERSION")
    self.git("commit", "-q", "--allow-empty", "-m", bpversion)
    self.git("push", "-q", "origin", branch)
    return self.git("rev-parse", "HEAD")

  def tag(self, name, sha):
    self.git("tag", "-a", "-m", name, name, sha)  # annotated, as a release with notes is
    self.git("push", "-q", "origin", name)

  def heads(self):
    out = self.git("ls-remote", "--heads", str(self.bare))
    return {ref.removeprefix("refs/heads/"): sha for sha, ref in (line.split("\t") for line in out.splitlines())}

  def clone_job(self):
    self.git("clone", "-q", str(self.bare), str(self.job), cwd=self.work.parent)
    return rel.Git(self.job)


class FakeGh:
  """The two API reads: the commit's push runs and its check runs, after gh's jq filter. Green unless told otherwise."""
  def __init__(self, failing=()):
    self.failing = failing

  def __call__(self, token, *args, check=True):
    url = next(a for a in args if a.startswith("repos/"))
    if "/actions/runs" in url:
      return "1"
    assert "/check-runs" in url, url
    return "\n".join(json.dumps({"name": n, "status": "completed", "conclusion": "failure" if n in self.failing else "success", "suite": 1})
                     for n in rel.REQUIRED)


class TestRelease(unittest.TestCase):
  def setUp(self):
    self.o = Origin(self)
    patches = [mock.patch.object(rel, "gh", FakeGh()), mock.patch.object(rel, "push_url", lambda token, repo: str(self.o.bare))]
    for p in patches:
      p.start()
      self.addCleanup(p.stop)

  def release(self, tag, **env):
    return rel.release({"TAG": tag, "GH_TOKEN": "read", "BP_TOKEN": "write", **env}, self.o.clone_job())

  def test_a_new_version_then_a_repeated_run(self):
    a = self.o.commit("7.1.0")
    self.o.tag("v7.1.0", a)
    self.release("v7.1.0")
    self.assertEqual(self.o.heads(), {"bp-dev": a, "bp-7.1": a, "bp-stable": a})
    self.o.job = self.o.job.with_name("job2")
    self.release("v7.1.0")
    self.assertEqual(self.o.heads(), {"bp-dev": a, "bp-7.1": a, "bp-stable": a})

  def test_the_next_version_after_a_patch_moves_bp_stable_back_onto_bp_dev(self):
    a = self.o.commit("7.1.0")
    self.o.tag("v7.1.0", a)
    self.release("v7.1.0")
    fix = self.o.commit("7.1.1", branch="bp-7.1")
    self.o.tag("v7.1.1", fix)
    self.o.job = self.o.job.with_name("job2")
    self.release("v7.1.1")
    self.assertEqual(self.o.heads()["bp-stable"], fix)
    b = self.o.commit("7.2.0")  # on bp-dev: not a descendant of the patch, so bp-stable's move is forced
    self.assertFalse(rel.Git(self.o.work).is_ancestor(fix, b))
    self.o.tag("v7.2.0", b)
    self.o.job = self.o.job.with_name("job3")
    self.release("v7.2.0")
    self.assertEqual(self.o.heads(), {"bp-dev": b, "bp-7.1": fix, "bp-7.2": b, "bp-stable": b})

  def test_a_dry_run_writes_nothing(self):
    a = self.o.commit("7.1.0")
    self.o.tag("v7.1.0", a)
    self.release("v7.1.0", DRY_RUN="true")
    self.assertEqual(self.o.heads(), {"bp-dev": a})

  def test_a_refused_release_writes_nothing(self):
    a = self.o.commit("7.1.0")
    self.o.tag("v7.1.0", a)
    with mock.patch.object(rel, "gh", FakeGh(failing=["install smoke"])), self.assertRaisesRegex(rel.Refused, "install smoke"):
      self.release("v7.1.0")
    self.assertEqual(self.o.heads(), {"bp-dev": a})

  def test_the_tag_must_exist(self):
    self.o.commit("7.1.0")
    with self.assertRaisesRegex(rel.Refused, "There is no tag v7.1.0: push the tag first"):
      self.release("v7.1.0")

  def test_throwaway_names(self):
    a = self.o.commit("7.1.0", branch="bpx-test/dev")
    self.o.tag("bpx-test/v7.1.0", a)
    self.release("bpx-test/v7.1.0", **TEST_ENV)
    self.assertEqual(self.o.heads(), {"bpx-test/dev": a, "bpx-test/bp-7.1": a, "bpx-test/stable": a})


if __name__ == "__main__":
  unittest.main()
