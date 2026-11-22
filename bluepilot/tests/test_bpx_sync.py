"""The BPx sync job's merge logic (.github/bpx_sync.py, #225), run against throwaway git repos."""
import importlib.util
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from openpilot.common.basedir import BASEDIR

GITHUB_DIR = Path(BASEDIR) / ".github"
GIT_ENV = {"GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t", "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@t"}


def load_script():
  sys.path.insert(0, str(GITHUB_DIR))  # the script imports bluepilot_hooks from its own folder
  spec = importlib.util.spec_from_file_location("bpx_sync", GITHUB_DIR / "bpx_sync.py")
  assert spec and spec.loader
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


bpx = load_script()
PIN_OLD, PIN_UP, PIN_OURS, Q = ("1" * 40), ("2" * 40), ("3" * 40), ("4" * 40)


class Repo:
  def __init__(self, test):
    tmp = tempfile.TemporaryDirectory()
    test.addCleanup(tmp.cleanup)
    self.path = Path(tmp.name)
    self.git = bpx.Git(self.path)
    os.environ.update(GIT_ENV)
    self.run("init", "-q", "-b", "main")

  def run(self, *args):
    return subprocess.run(["git", "-c", "commit.gpgsign=false", *args], cwd=self.path, check=True, capture_output=True, text=True,
                          env={**os.environ, **GIT_ENV}).stdout.strip()

  def commit(self, message, files=None, pin=None):
    """Commit on the current branch: files {path: text}, pin = the opendbc_repo gitlink. Returns the sha."""
    for path, text in (files or {}).items():
      (self.path / path).parent.mkdir(parents=True, exist_ok=True)
      (self.path / path).write_text(text)
      self.run("add", path)
    if pin:
      self.run("update-index", "--add", "--cacheinfo", f"160000,{pin},opendbc_repo")
    self.run("commit", "-q", "--allow-empty", "-m", message)
    return self.run("rev-parse", "HEAD")

  def branch(self, name, start="main"):
    self.run("checkout", "-q", "-b", name, start)

  def checkout(self, name):
    self.run("checkout", "-q", name)


class TestPickQ(unittest.TestCase):
  def setUp(self):
    # an opendbc history: bp-full at the old pin, P on bp-opendbc, then bp-full merges P
    self.r = Repo(self)
    self.r.commit("root", {"car.py": "x = 1\n"})
    self.r.branch("bp-opendbc")
    self.p = self.r.commit("P", {"car.py": "x = 2\n"})
    self.r.checkout("main")
    self.old_pin = self.r.commit("bp-full work", {"ford.py": "y = 1\n"})
    self.r.run("merge", "-q", "--no-ff", "-m", "merge P", self.p)
    self.merged = self.r.run("rev-parse", "HEAD")

  def test_bp_full_tip_when_the_pin_lacks_p(self):
    self.assertEqual(bpx.pick_q(self.r.git, self.p, self.old_pin, self.merged), self.merged)

  def test_current_pin_that_has_p_stays(self):
    # bp-full moved on after the pin, but the pin already has P: never bump past what a sync needs
    tip = self.r.commit("later bp-full work", {"ford.py": "y = 2\n"})
    self.assertEqual(bpx.pick_q(self.r.git, self.p, self.merged, tip), self.merged)


class SyncRepo(unittest.TestCase):
  """A target branch (main) and the bp-sunnypilot mirror, both moved opendbc_repo since their merge base."""
  def setUp(self):
    self.r = Repo(self)
    self.r.commit("base", {"shared.py": "a = 1\n", "hooked.py": "h = 1\n"}, pin=PIN_OLD)
    self.r.branch("bp-sunnypilot")
    self.t = self.r.commit("upstream", {"shared.py": "a = 1\nb = 2\n"}, pin=PIN_UP)
    self.r.checkout("main")
    self.target = self.r.commit("bluepilot", {"bluepilot/x.py": "x = 1\n"}, pin=PIN_OURS)

  def plan(self, branch=None, t=None, target=None):
    return bpx.plan(self.r.git, "main", target or self.target, t or self.t, Q, branch)

  def pin(self, commit):
    return self.r.run("rev-parse", f"{commit}:opendbc_repo")

  def parents(self, commit):
    return self.r.run("rev-list", "--parents", "-n1", commit).split()[1:]


class TestPlan(SyncRepo):
  def test_opendbc_repo_conflict_is_resolved_to_q(self):
    plan = self.plan()
    self.assertEqual(plan.action, "ready")
    self.assertEqual(plan.conflicts, [])
    self.assertTrue(plan.pin_conflicted)
    self.assertEqual(self.pin(plan.commit), Q)
    self.assertEqual(self.parents(plan.commit), [self.target, self.t])
    self.assertEqual(self.r.run("show", f"{plan.commit}:shared.py"), "a = 1\nb = 2")

  def test_a_file_conflict_stops_with_its_name_and_builds_nothing(self):
    target = self.r.commit("bluepilot edits shared.py", {"shared.py": "a = 1\nc = 3\n"})
    commits = self.r.run("rev-list", "--all", "--count")
    plan = self.plan(target=target)
    self.assertEqual(plan.action, "conflict")
    self.assertEqual(plan.conflicts, ["shared.py"])  # opendbc_repo conflicts too, but isn't reported
    self.assertEqual(plan.commit, "")
    self.assertEqual(self.r.run("rev-list", "--all", "--count"), commits)

  def test_a_repeated_run_with_the_branch_open_reuses_its_head(self):
    first = self.plan()
    again = self.plan(branch=first.commit)
    self.assertEqual((again.action, again.commit), ("ready", first.commit))

  def test_a_repeated_run_after_the_merge_does_nothing(self):
    merged = self.plan().commit
    self.assertEqual(self.plan(target=merged).action, "nothing")

  def test_a_merged_branch_left_behind_is_not_built_on(self):
    old = self.plan().commit
    self.r.checkout("main")
    self.r.run("merge", "-q", "--no-ff", "-m", "sync merged", old)
    target = self.r.commit("more bluepilot work", {"bluepilot/y.py": "y = 1\n"})
    self.r.checkout("bp-sunnypilot")
    t2 = self.r.commit("upstream again", {"other.py": "o = 1\n"})
    plan = self.plan(branch=old, t=t2, target=target)
    self.assertEqual(self.parents(plan.commit), [target, t2])

  def test_a_moved_mirror_is_merged_on_top_of_the_open_branch(self):
    first = self.plan().commit
    self.r.checkout("bp-sunnypilot")
    t2 = self.r.commit("upstream again", {"other.py": "o = 1\n"}, pin=PIN_UP[:-1] + "5")
    plan = self.plan(branch=first, t=t2)
    self.assertEqual(plan.action, "ready")
    self.assertEqual(self.parents(plan.commit), [first, t2])  # a fast-forward of the open branch: no force-push
    self.assertEqual(self.pin(plan.commit), Q)
    self.assertEqual(self.r.run("show", f"{plan.commit}:other.py"), "o = 1")

  def test_a_new_t_that_changes_no_files_is_still_merged(self):
    # else the target never contains T, and every run would find work to do
    first = self.plan().commit
    self.r.checkout("bp-sunnypilot")
    t2 = self.r.commit("empty upstream merge")
    self.assertEqual(self.parents(self.plan(branch=first, t=t2).commit), [first, t2])

  def test_only_q_moving_adds_a_pin_commit_to_the_open_branch(self):
    first = self.plan().commit
    plan = bpx.plan(self.r.git, "main", self.target, self.t, "6" * 40, first)
    self.assertEqual(self.parents(plan.commit), [first])
    self.assertEqual(self.pin(plan.commit), "6" * 40)


class TestPrBody(SyncRepo):
  HOOKS = "# comment\nshared.py   # a hook\nhooked.py  # another\nopendbc_repo  # bp-full\n"

  def summary(self, **kw):
    base = self.r.run("merge-base", self.target, self.t)
    fields = {"target": "main", "t": self.t, "p": "a" * 40, "q": Q, "commits": [f"{self.t[:9]} upstream"], "total": 1,
              "pin_conflicted": True, "overlap": bpx.overlap(self.r.git, base, self.t, self.HOOKS), "firmware": [],
              "lib_old": "`v0.2.1`", "lib_new": "`v0.2.1`", "bp_full_pr": None}
    return bpx.Summary(**{**fields, **kw})

  def row(self, body, name):
    return next(line for line in body.splitlines() if line.startswith("| ") and f"| {name} |" in line)

  def test_overlap_is_the_hook_list_files_upstream_changed(self):
    # opendbc_repo moved on both sides but is resolved by the job, so it isn't overlap
    self.assertEqual(self.summary().overlap, ["shared.py"])

  def test_merge_commit_line_comes_first(self):
    self.assertTrue(bpx.pr_body(self.summary()).startswith("❌ **Merge commit only"))

  def test_rows(self):
    body = bpx.pr_body(self.summary())
    self.assertTrue(self.row(body, "opendbc").startswith("| ✅ |"))
    self.assertTrue(self.row(body, "Conflicts").startswith("| ✅ |"))
    self.assertTrue(self.row(body, "Conflicts").count("opendbc_repo"))
    self.assertTrue(self.row(body, "Overlap").startswith("| ⚠️ | Overlap | 1 file "))
    self.assertEqual(self.row(body, "Panda firmware"), "| ✅ | Panda firmware | unchanged |")
    self.assertEqual(self.row(body, "Library"), "| ✅ | Library | unchanged at `v0.2.1` |")

  def test_sections_list_commits_and_overlap(self):
    body = bpx.pr_body(self.summary())
    commits = body.split("### Upstream commits\n")[1]
    self.assertTrue(commits.startswith(f"- {self.t[:9]} upstream\n"))
    self.assertTrue(body.split("### Files upstream changed that BluePilot also changes\n")[1].startswith("- `shared.py`\n"))
    self.assertTrue(body.split("### Conflicts\n")[1].startswith("- `opendbc_repo`"))

  def test_clean_sync_rows(self):
    body = bpx.pr_body(self.summary(overlap=[], pin_conflicted=False))
    self.assertEqual(self.row(body, "Overlap"), "| ✅ | Overlap | none |")
    self.assertEqual(self.row(body, "Conflicts"), "| ✅ | Conflicts | none |")
    self.assertTrue(body.split("### Conflicts\n")[1].startswith("None."))

  def test_firmware_and_library_moves_are_flagged(self):
    body = bpx.pr_body(self.summary(firmware=["opendbc/safety/modes/ford.h", "opendbc/safety/modes/honda.h"], lib_new="`v0.2.2`"))
    self.assertEqual(self.row(body, "Panda firmware"),
                     "| ⚠️ | Panda firmware | Ford safety changes (`opendbc/safety/modes/ford.h`, `opendbc/safety/modes/honda.h`) |")
    self.assertEqual(self.row(body, "Library"), "| ⚠️ | Library | moves `v0.2.1` → `v0.2.2` |")
    body = bpx.pr_body(self.summary(firmware=["opendbc/safety/modes/honda.h"]))
    self.assertTrue(self.row(body, "Panda firmware").startswith("| ✅ | Panda firmware | upstream safety changes only"))

  def test_step_one_pr_is_linked(self):
    row = self.row(bpx.pr_body(self.summary(bp_full_pr=57)), "opendbc")
    self.assertTrue(row.count("ford-op/opendbc#57"))

  def test_validation_lists_the_six_checks_unticked(self):
    validation = bpx.pr_body(self.summary()).split("## Validation\n")[1].strip().splitlines()
    self.assertEqual(validation, [f"- [ ] {c}" for c in
                                  ("build release", "static analysis", "unit tests", "process replay", "install smoke", "bluepilot-hooks")])


class TestIssueBody(unittest.TestCase):
  T, P = "b" * 40, "c" * 40

  def test_step_two_conflict(self):
    body = bpx.issue_body(step=2, target="bp-7.1-dev", t=self.T, p=self.P, q=Q, start="bpx-sync/bp-7.1-dev",
                          reason="merging bp-sunnypilot conflicts in 2 files.", files=["a.py", "b/c.py"])
    self.assertTrue(body.count("Step 2 (bp-7.1-dev) is waiting: merging bp-sunnypilot conflicts in 2 files."))
    self.assertTrue(body.count(f"[`bbbbbbbb`](https://github.com/BluePilotDev/bluepilot/commit/{self.T})"))
    self.assertTrue(body.count("- `a.py`\n- `b/c.py`\n"))
    commands = body.split("```sh\n")[1].split("```")[0].splitlines()
    self.assertEqual(commands, [
      "git clone --filter=blob:none https://github.com/BluePilotDev/bluepilot.git && cd bluepilot",
      "git checkout -B bpx-sync/bp-7.1-dev origin/bpx-sync/bp-7.1-dev",
      f"git merge --no-ff --no-commit {self.T}",
      f"git update-index --cacheinfo 160000,{Q},opendbc_repo",
      "# resolve the files listed above and git add them, then:",
      "git commit",
      "git push origin bpx-sync/bp-7.1-dev",
    ])

  def test_step_one_red_check_links_it_and_merges_p_into_bp_full(self):
    body = bpx.issue_body(step=1, target="bp-7.1-dev", t=self.T, p=self.P, q="", start="bp-full",
                          reason="ford-op/opendbc#60 has a failing required check.", link="[safety](https://example.com/run/1)")
    self.assertTrue(body.count("Step 1 (bp-full) is waiting: ford-op/opendbc#60 has a failing required check."))
    self.assertTrue(body.count("[safety](https://example.com/run/1)"))
    self.assertEqual(body.count("Conflicted files"), 0)
    commands = body.split("```sh\n")[1].split("```")[0]
    self.assertTrue(commands.count("git clone --filter=blob:none https://github.com/ford-op/opendbc.git && cd opendbc\n"))
    self.assertTrue(commands.count("git checkout -B sync/bp-full-cccccccc origin/bp-full\n"))
    self.assertTrue(commands.count(f"git merge --no-ff {self.P}\n"))
