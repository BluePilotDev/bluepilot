"""The BPx sync job's merge logic (.github/bpx_sync.py, #225), run against throwaway git repos."""
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

  def test_a_target_that_moved_and_conflicts_with_the_open_branch_stops(self):
    # GitHub would refuse the PR's merge and run no CI: the job stops and says so instead of waiting forever
    first = self.plan().commit
    self.r.checkout("main")
    target = self.r.commit("bluepilot edits shared.py", {"shared.py": "a = 1\nc = 3\n"})
    plan = self.plan(branch=first, target=target)
    self.assertEqual((plan.action, plan.conflicts, plan.against_target), ("conflict", ["shared.py"], True))

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
    # in a code block: a "Fixes #N" in an upstream subject must not close an issue here
    self.assertTrue(commits.startswith(f"```text\n{self.t[:9]} upstream\n```\n"))
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

  def test_a_subject_with_a_fence_cannot_close_the_block(self):
    commits = bpx.pr_body(self.summary(commits=["abc ``` Fixes #3"])).split("### Upstream commits\n")[1]
    self.assertTrue(commits.startswith("````text\nabc ``` Fixes #3\n````\n"))

  def test_p_and_q_are_links(self):
    row = self.row(bpx.pr_body(self.summary()), "opendbc")
    self.assertTrue(row.count(f"[`aaaaaaaa`](https://github.com/ford-op/opendbc/commit/{'a' * 40})"))
    self.assertTrue(row.count(f"[`44444444`](https://github.com/ford-op/opendbc/commit/{Q})"))

  def test_step_one_pr_is_linked(self):
    row = self.row(bpx.pr_body(self.summary(bp_full_pr=57)), "opendbc")
    self.assertTrue(row.count("ford-op/opendbc#57"))

  def test_validation_lists_the_six_checks_unticked(self):
    validation = bpx.pr_body(self.summary()).split("## Validation\n")[1].strip().splitlines()
    self.assertEqual(validation, [f"- [ ] {c}" for c in
                                  ("build release", "static analysis", "unit tests", "process replay", "install smoke", "bluepilot-hooks")])


class TestIssueBody(unittest.TestCase):
  T, P = "b" * 40, "c" * 40

  def body(self, **kw):
    return bpx.issue_body(**{"target": "bp-7.1-dev", "t": self.T, "p": self.P, "q": Q, "reason": "why.", **kw})

  def commands(self, body):
    return body.split("```sh\n")[1].split("```")[0].splitlines()

  def test_step_two_conflict_redoes_the_merge_with_q(self):
    body = self.body(step=2, fix="merge", start="bpx-sync/bp-7.1-dev", reason="merging bp-sunnypilot conflicts in 2 files.",
                     files=["a.py", "b/c.py"], link="Open sync PR: #70")
    self.assertTrue(body.count("Step 2 (bp-7.1-dev) is waiting: merging bp-sunnypilot conflicts in 2 files."))
    self.assertTrue(body.count(f"[`bbbbbbbb`](https://github.com/BluePilotDev/bluepilot/commit/{self.T})"))
    self.assertTrue(body.count(f"[`cccccccc`](https://github.com/ford-op/opendbc/commit/{self.P})"))
    self.assertTrue(body.count("- Open sync PR: #70\n"))
    self.assertTrue(body.count("- `a.py`\n- `b/c.py`\n"))
    self.assertEqual(self.commands(body), [
      "git clone --filter=blob:none https://github.com/BluePilotDev/bluepilot.git && cd bluepilot",
      "git checkout -B bpx-sync/bp-7.1-dev origin/bpx-sync/bp-7.1-dev",
      f"git merge --no-ff --no-commit {self.T}",
      f"git update-index --cacheinfo 160000,{Q},opendbc_repo",
      "# resolve the files listed above and git add them, then:",
      "git commit",
      "git push origin bpx-sync/bp-7.1-dev",
    ])

  def test_step_two_target_conflict_merges_the_target_into_the_branch(self):
    self.assertEqual(self.commands(self.body(step=2, fix="merge-target", files=["a.py"])), [
      "git clone --filter=blob:none https://github.com/BluePilotDev/bluepilot.git && cd bluepilot",
      "git checkout -B bpx-sync/bp-7.1-dev origin/bpx-sync/bp-7.1-dev",
      "git merge --no-ff origin/bp-7.1-dev",
      "# resolve the files listed above and git add them, then:",
      "git commit",
      "git push origin bpx-sync/bp-7.1-dev",
    ])

  def test_step_two_red_check_pushes_a_fix_onto_the_job_branch(self):
    self.assertEqual(self.commands(self.body(step=2, fix="push")), [
      "git clone --filter=blob:none https://github.com/BluePilotDev/bluepilot.git && cd bluepilot",
      "git checkout -B bpx-sync/bp-7.1-dev origin/bpx-sync/bp-7.1-dev",
      "# fix what the check reports and commit it, then:",
      "git push origin bpx-sync/bp-7.1-dev",
    ])

  def test_step_one_conflict_merges_p_on_the_job_branch(self):
    body = self.body(step=1, fix="merge", start="bp-full", q="", files=["x.h"])
    self.assertTrue(body.count("Step 1 (bp-full) is waiting: why."))
    self.assertEqual(self.commands(body), [
      "git clone --filter=blob:none https://github.com/ford-op/opendbc.git && cd opendbc",
      "git checkout -B bpx-sync/bp-full origin/bp-full",
      f"git merge --no-ff {self.P}",
      "# resolve the files listed above and git add them, then:",
      "git commit",
      "git push origin bpx-sync/bp-full",
    ])

  def test_step_one_red_check_links_it_and_pushes_a_fix(self):
    body = self.body(step=1, fix="push", q="", link="Failing: [safety](https://example.com/run/1)")
    self.assertTrue(body.count("- Failing: [safety](https://example.com/run/1)\n"))
    self.assertEqual(body.count("Conflicted files"), 0)
    self.assertEqual(self.commands(body)[1:], ["git checkout -B bpx-sync/bp-full origin/bpx-sync/bp-full",
                                               "# fix what the check reports and commit it, then:", "git push origin bpx-sync/bp-full"])


class TestBpFullHead(unittest.TestCase):
  """Step 1's PR head: P, or the job branch with P merged in when someone pushed to it."""
  def setUp(self):
    self.r = Repo(self)
    self.r.commit("root", {"car.py": "x = 1\n", "safety.h": "s = 1\n"})
    self.r.branch("bp-opendbc")
    self.p_old = self.r.commit("old P", {"car.py": "x = 2\n"})
    self.p = self.r.commit("P", {"car.py": "x = 3\n"})
    self.r.checkout("main")
    self.full = self.r.commit("bp-full work", {"ford.py": "y = 1\n"})

  def head(self, branch=None, full=None):
    return bpx.bp_full_head(self.r.git, full or self.full, self.p, branch)

  def test_no_branch_is_p(self):
    self.assertEqual(self.head(), (self.p, []))

  def test_a_branch_at_an_older_p_moves_to_p(self):
    self.assertEqual(self.head(self.p_old), (self.p, []))

  def test_a_branch_that_has_p_is_kept(self):
    self.r.branch("hand", self.p)
    fixed = self.r.commit("hand fix", {"fix.py": "f = 1\n"})
    self.assertEqual(self.head(fixed), (fixed, []))

  def test_a_fix_pushed_onto_an_older_p_gets_p_merged_on_top(self):
    self.r.branch("hand", self.p_old)
    fixed = self.r.commit("hand fix", {"fix.py": "f = 1\n"})
    head, conflicts = self.head(fixed)
    self.assertEqual(conflicts, [])
    self.assertEqual(self.r.run("rev-list", "--parents", "-n1", head).split()[1:], [fixed, self.p])

  def test_a_branch_already_in_bp_full_is_not_built_on(self):
    self.r.run("merge", "-q", "--no-ff", "-m", "merged", self.p_old)
    self.assertEqual(self.head(self.p_old, full=self.r.run("rev-parse", "HEAD")), (self.p, []))

  def test_p_conflicting_with_bp_full_is_reported(self):
    full = self.r.commit("bp-full edits car.py", {"car.py": "x = 9\n"})
    self.assertEqual(self.head(full=full), ("", ["car.py"]))


class FakeGh:
  """Stands in for the gh CLI: answers by the command's first words, records every call."""
  def __init__(self, answers):
    self.answers, self.calls = answers, []

  def __call__(self, token, *args, check=True):
    self.calls.append(args)
    for prefix, answer in self.answers.items():
      if args[:len(prefix)] == prefix:
        return answer(args) if callable(answer) else answer
    return ""


def check(name, bucket, event="pull_request"):
  return {"name": name, "bucket": bucket, "event": event, "link": f"https://example.com/{name}"}


class TestWaitChecks(unittest.TestCase):
  HEAD = "d" * 40
  NAMES = ("build", "test")

  def wait(self, polls, head=HEAD):
    """polls: one list of checks per poll; the last one repeats."""
    seen = []
    def checks(args):
      seen.append(1)
      return json.dumps(polls[min(len(seen), len(polls)) - 1])
    gh = FakeGh({("pr", "view"): head, ("pr", "checks"): checks})
    with mock.patch.object(bpx, "gh", gh), mock.patch.object(bpx.time, "sleep"):
      return bpx.wait_checks("tok", "o/r", "5", self.HEAD, self.NAMES, minutes=1)

  def test_all_pass(self):
    self.assertEqual(self.wait([[check("build", "pass"), check("test", "pass")]]), ("pass", ""))

  def test_a_stale_head_never_passes(self):
    self.assertEqual(self.wait([[check("build", "pass"), check("test", "pass")]], head="e" * 40)[0], "timeout")

  def test_a_missing_check_waits(self):
    self.assertEqual(self.wait([[check("build", "pass")]])[0], "timeout")

  def test_a_duplicate_still_pending_waits_until_both_pass(self):
    pending = [check("build", "pass"), check("test", "pass"), check("test", "pending")]
    self.assertEqual(self.wait([pending])[0], "timeout")
    self.assertEqual(self.wait([pending, [check("build", "pass"), check("test", "pass"), check("test", "pass")]])[0], "pass")

  def test_push_event_copies_are_ignored(self):
    # step 1's head is a bp-opendbc commit: its green push runs must not stand in for the PR's own
    self.assertEqual(self.wait([[check("build", "pass", "push"), check("test", "pass", "push")]])[0], "timeout")
    self.assertEqual(self.wait([[check("build", "fail", "push"), check("build", "pass"), check("test", "pass")]])[0], "pass")

  def test_skipped_counts_as_passed(self):
    self.assertEqual(self.wait([[check("build", "pass"), check("test", "skipping")]])[0], "pass")

  def test_cancelled_is_pending_not_red(self):
    self.assertEqual(self.wait([[check("build", "pass"), check("test", "cancel")]])[0], "timeout")

  def test_a_failure_is_red_with_its_link(self):
    self.assertEqual(self.wait([[check("build", "fail"), check("test", "pass")]]), ("fail", "[build](https://example.com/build)"))


class TestReport(unittest.TestCase):
  """The one issue: written when the stuck state is new or changed, left alone when it is the same."""
  def report(self, existing, body="new body"):
    gh = FakeGh({("issue", "list"): "12" if existing is not None else "", ("issue", "view"): existing or "",
                 ("issue", "create"): "https://github.com/o/r/issues/13"})
    with mock.patch.object(bpx, "gh", gh):
      return bpx.report("tok", "bp-7.1-dev", body), [c[:2] for c in gh.calls]

  def test_new_issue(self):
    result, calls = self.report(None)
    self.assertEqual(result, ("13", True))
    self.assertTrue(calls.count(("issue", "create")))

  def test_changed_state_rewrites_it(self):
    result, calls = self.report("old body")
    self.assertEqual(result, ("12", True))
    self.assertTrue(calls.count(("issue", "edit")))

  def test_same_state_is_left_alone(self):
    result, calls = self.report("new body\n")
    self.assertEqual(result, ("12", False))
    self.assertEqual(calls.count(("issue", "edit")), 0)


class TestMergeGuards(unittest.TestCase):
  def env(self, **kw):
    return {"BP_TOKEN": "tok", "PR": "7", "HEAD": "d" * 40, "T": "b" * 40, "P": "c" * 40, "Q": Q, "TARGET": "bp-7.1-dev", **kw}

  def merge(self, pr_view, **env):
    gh = FakeGh({("pr", "view"): json.dumps(pr_view)})
    with mock.patch.object(bpx, "gh", gh), mock.patch.dict(os.environ, self.env(**env)), \
         mock.patch.object(bpx, "wait_checks", return_value=("timeout", "")):
      return bpx.merge("bp-7.1-dev")

  GOOD = {"headRefName": "bpx-sync/bp-7.1-dev", "baseRefName": "bp-7.1-dev", "isCrossRepository": False, "state": "OPEN"}

  def test_the_job_pr_is_accepted(self):
    self.assertEqual(self.merge(self.GOOD), 0)

  def test_another_pr_is_refused(self):
    for change in ({"headRefName": "evil"}, {"baseRefName": "bp-dev"}, {"isCrossRepository": True}, {"state": "CLOSED"}):
      with self.subTest(change=change), self.assertRaises(SystemExit):
        self.merge({**self.GOOD, **change})

  def test_malformed_inputs_are_refused(self):
    for env in ({"PR": "7\nx"}, {"HEAD": "abc"}, {"T": "b" * 39}):
      with self.subTest(env=env), self.assertRaises(SystemExit):
        self.merge(self.GOOD, **env)


class TestOutput(unittest.TestCase):
  def written(self, **values):
    with tempfile.NamedTemporaryFile("r", suffix=".out") as f, mock.patch.dict(os.environ, {"GITHUB_OUTPUT": f.name}):
      bpx.output(**values)
      return f.read()

  def test_a_value_cannot_forge_another_output(self):
    text = self.written(files="a.py\nEOF\npr=666")
    delimiter = text.split("<<", 1)[1].split("\n", 1)[0]
    self.assertNotEqual(delimiter, "EOF")
    self.assertEqual(text, f"files<<{delimiter}\na.py\nEOF\npr=666\n{delimiter}\n")

  def test_a_value_holding_the_delimiter_is_refused(self):
    with mock.patch.object(bpx.secrets, "token_hex", return_value="DELIM"), self.assertRaises(ValueError):
      self.written(files="x DELIM y")


class TestReviewHandOff(unittest.TestCase):
  """The review job builds its prompt from the sync job's inputs; the finish job posts its text once per upstream commit."""
  UP = "b" * 40

  def review(self, **kw):
    return json.dumps({"on": "9", "repo": "BluePilotDev/bluepilot", "base": "a" * 40, "up": self.UP, "integration": "c" * 40, "q": Q,
                       "files": ["x.py", "y.py"], "conflicts": ["x.py"], **kw})

  def test_prompt_has_the_inputs(self):
    with tempfile.NamedTemporaryFile("w", suffix=".md") as f, mock.patch.dict(os.environ, {"REVIEW": self.review()}):
      f.write("PROMPT")
      f.flush()
      prompt = bpx.review_prompt(f.name)
    self.assertTrue(prompt.startswith("PROMPT\n\n## Inputs\n"))
    for line in (f"- UPSTREAM: {self.UP}", f"- `opendbc_repo`: resolved by the job to ford-op/opendbc `bp-full` {Q}",
                 "- Files that conflict: x.py", "  - x.py", "  - y.py"):
      self.assertTrue(prompt.count(line + "\n"), line)

  def post(self, comments, text="✅ fine", **kw):
    gh = FakeGh({("api",): comments})
    with tempfile.NamedTemporaryFile("w", suffix=".md") as f, mock.patch.object(bpx, "gh", gh), \
         mock.patch.dict(os.environ, {"REVIEW": self.review(**kw), "BP_TOKEN": "tok"}):
      f.write(text)
      f.flush()
      bpx.post(f.name)
    return [c for c in gh.calls if c[:2] == ("issue", "comment")]

  def test_posts_with_the_marker(self):
    self.assertEqual(self.post(""), [("issue", "comment", "9", "-R", bpx.REPO, "--body", f"<!-- bpx-sync-review {self.UP} -->\n✅ fine")])

  def test_once_per_upstream_commit(self):
    self.assertEqual(self.post(f"x\n<!-- bpx-sync-review {self.UP} -->\nold"), [])

  def test_malformed_inputs_are_refused(self):
    for kw in ({"on": "9 --repo x"}, {"up": "HEAD"}):
      with self.subTest(kw=kw), self.assertRaises(SystemExit):
        self.post("", **kw)

  def want(self, comments):
    with tempfile.NamedTemporaryFile("r", suffix=".out") as f, mock.patch.dict(os.environ, {"GITHUB_OUTPUT": f.name}), \
         mock.patch.object(bpx, "gh", FakeGh({("api",): comments})):
      bpx.want_review("tok", "9", up=self.UP, files=["x.py"])
      return f.read()

  def test_the_review_job_is_asked_once_per_upstream_commit(self):
    self.assertTrue(self.want("").startswith("review<<"))
    self.assertEqual(self.want(f"<!-- bpx-sync-review {self.UP} -->"), "")
