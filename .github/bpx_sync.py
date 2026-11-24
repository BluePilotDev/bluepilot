#!/usr/bin/env python3
"""BluePilot: the BPx sync job (#225). Keeps a full BluePilot branch current with bp-sunnypilot.

T = this repo's bp-sunnypilot branch (the mirror public-sync publishes), P = the opendbc commit T pins,
Q = the ford-op/opendbc bp-full commit with P that the target's opendbc_repo points at.
usage: bpx_sync.py bp-full | prepare | review-prompt <prompt file> | post <review file> | merge   (env: see bpx-sync.yml)"""
import json
import os
import re
import secrets
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from bluepilot_hooks import listed_hooks

REPO = os.environ.get("GITHUB_REPOSITORY", "BluePilotDev/bluepilot")
ODBC = "ford-op/opendbc"
# required checks, mirrored from branch protection (the App can't read protection; keep in sync if a list changes)
REQUIRED = ("build release", "static analysis", "unit tests", "process replay", "install smoke", "bluepilot-hooks")
ODBC_REQUIRED = ("bluepilot-hooks", "safety", "bluepilot-safety", "./test.sh (Linux)")
BP_FULL_BRANCH = "bpx-sync/bp-full"
REPLAY = " A process replay difference gets the `replay-accepted` label once it is traced and written in the PR (the job never adds it)."
SHA = re.compile(r"[0-9a-f]{40}")
BOT = ["-c", "user.name=bluepilot-sync[bot]", "-c", "user.email=348581487+bluepilot-sync[bot]@users.noreply.github.com"]


class Git:
  def __init__(self, cwd):
    self.cwd = Path(cwd)

  def __call__(self, *args: str, env: dict | None = None) -> str:
    return subprocess.run(["git", *args], cwd=self.cwd, check=True, stdout=subprocess.PIPE, text=True, env=env).stdout.strip()

  def ok(self, *args: str) -> bool:
    return subprocess.run(["git", *args], cwd=self.cwd, capture_output=True).returncode == 0

  def is_ancestor(self, a: str, b: str) -> bool:
    return self.ok("merge-base", "--is-ancestor", a, b)


def job_branch(target: str) -> str:
  return f"bpx-sync/{target}"


def pick_q(odbc: Git, p: str, pin: str, bp_full: str) -> str:
  """The current pin when it already has P (a sync never moves bp-full work in that it doesn't need), else bp-full's tip."""
  return pin if odbc.is_ancestor(p, pin) else bp_full


def merge_tree(git: Git, ours: str, theirs: str) -> tuple[str, list[str]]:
  """The merged tree and the conflicted paths, without touching a work tree or index."""
  out = subprocess.run(["git", "merge-tree", "--write-tree", "--name-only", "--no-messages", ours, theirs],
                       cwd=git.cwd, capture_output=True, text=True)
  lines = out.stdout.split("\n")
  # exit 1 means conflicts or an error; only a real result starts with the merged tree's ID
  if out.returncode not in (0, 1) or len(lines[0]) != 40:
    raise RuntimeError(f"merge-tree {ours} {theirs} failed: {out.stderr}")
  return lines[0], [line for line in lines[1:] if line]


def bp_full_head(git: Git, full: str, p: str, branch: str | None) -> tuple[str, list[str]]:
  """Step 1's PR head and the conflicts with bp-full. P itself, unless the job branch carries someone's commits: then
  P merged on top of them, so the branch only fast-forwards. ("", conflicts) when it can't be built."""
  head = p
  if branch and not git.is_ancestor(branch, full) and not git.is_ancestor(branch, p):
    if git.is_ancestor(p, branch):
      head = branch
    else:
      tree, conflicts = merge_tree(git, branch, p)
      if conflicts:
        return "", conflicts
      head = git(*BOT, "commit-tree", tree, "-p", branch, "-p", p, "-m", f"Merge {p[:8]} into {BP_FULL_BRANCH}")
  _, conflicts = merge_tree(git, full, head)
  return ("", conflicts) if conflicts else (head, [])


@dataclass
class Plan:
  action: str  # "nothing", "conflict" (nothing is pushed) or "ready" (commit is the job branch's head)
  base: str = ""  # the target or the job branch: what T is merged into
  commit: str = ""
  conflicts: list[str] = field(default_factory=list)  # merging T: other than opendbc_repo
  pin_conflicted: bool = False
  against_target: bool = False  # the conflicts are between the open job branch and a target that moved since


def plan(git: Git, target_name: str, target: str, t: str, q: str, branch: str | None) -> Plan:
  """Merge T into the target (or into the job branch when one is open), with opendbc_repo at Q."""
  if git.is_ancestor(t, target) and git("rev-parse", f"{target}:opendbc_repo") == q:
    return Plan("nothing")
  # an open sync PR's branch only ever moves forward (a later T is merged on top); one already in the target is done
  base = branch if branch and not git.is_ancestor(branch, target) else target
  tree, conflicts = merge_tree(git, base, t)
  others = [c for c in conflicts if c != "opendbc_repo"]
  if others:
    return Plan("conflict", base, conflicts=others)
  pin_conflicted = "opendbc_repo" in conflicts
  with tempfile.TemporaryDirectory() as tmp:
    env = {**os.environ, "GIT_INDEX_FILE": os.path.join(tmp, "index")}
    git("read-tree", tree, env=env)
    git("update-index", "--cacheinfo", f"160000,{q},opendbc_repo", env=env)
    tree = git("write-tree", env=env)
  if tree == git("rev-parse", f"{base}^{{tree}}") and git.is_ancestor(t, base):
    commit = base
  else:
    parents = ["-p", base] + ([] if git.is_ancestor(t, base) else ["-p", t])
    commit = git(*BOT, "commit-tree", tree, *parents, "-m", f"Merge bp-sunnypilot {t[:8]} into {target_name}",
                 "-m", f"opendbc_repo: ford-op/opendbc bp-full {q[:8]}. Built by the BPx sync job (#225).")
  if base != target:
    # GitHub refuses a PR that conflicts with its base and runs no CI on it: waiting would time out forever
    _, against = merge_tree(git, target, commit)
    if against:
      return Plan("conflict", base, conflicts=against, against_target=True)
  return Plan("ready", base, commit=commit, pin_conflicted=pin_conflicted)


def overlap(git: Git, base: str, t: str, hooks_text: str) -> list[str]:
  """Files on the hook list that bp-sunnypilot changed since the merge base (opendbc_repo is the job's to resolve)."""
  changed = git("diff", "--no-renames", "--name-only", base, t).splitlines()
  return sorted(set(changed) & (listed_hooks(hooks_text) - {"opendbc_repo"}))


@dataclass
class Summary:
  target: str
  t: str
  p: str
  q: str
  commits: list[str]  # "<short sha> <subject>", newest first, at most 200
  total: int
  pin_conflicted: bool
  overlap: list[str]
  firmware: list[str]  # panda safety sources (not tests) that differ between the old pin and Q
  lib_old: str  # the library at the old pin and at Q, as labels
  lib_new: str
  bp_full_pr: int | None  # the ford-op/opendbc PR step 1 merged in this run


def files(paths: list[str]) -> str:
  return ", ".join(f"`{p}`" for p in paths)


def count(n: int) -> str:
  return "1 file" if n == 1 else f"{n} files"


def commit_link(repo: str, sha: str) -> str:
  return f"[`{sha[:8]}`](https://github.com/{repo}/commit/{sha})"


def fenced(lines: list[str]) -> list[str]:
  """Upstream-written text in a code block, where GitHub doesn't act on "Fixes #N"; the fence outgrows any backticks in it."""
  longest = max([len(run) for line in lines for run in re.findall(r"`+", line)] or [0])
  fence = "`" * max(3, longest + 1)
  return [f"{fence}text", *lines, fence]


def pr_body(s: Summary) -> str:
  merged = f" (merged by ford-op/opendbc#{s.bp_full_pr})" if s.bp_full_pr else ""
  pins = f"bp-sunnypilot's pin {commit_link(ODBC, s.p)}{merged}; `opendbc_repo` → {commit_link(ODBC, s.q)}"
  rows = [f"| ✅ | opendbc | ford-op/opendbc `bp-full` contains {pins} |",
          f"| ✅ | Conflicts | `opendbc_repo`, resolved by the job (→ `{s.q[:8]}`) |" if s.pin_conflicted else "| ✅ | Conflicts | none |"]
  if s.overlap:
    rows.append(f"| ⚠️ | Overlap | {count(len(s.overlap))} on the hook list that bp-sunnypilot changed, listed below: the review comment looks at each |")
  else:
    rows.append("| ✅ | Overlap | none |")
  if "opendbc/safety/modes/ford.h" in s.firmware:
    rows.append(f"| ⚠️ | Panda firmware | Ford safety changes ({files(s.firmware)}) |")
  elif s.firmware:
    rows.append(f"| ✅ | Panda firmware | upstream safety changes only ({files(s.firmware)}) |")
  else:
    rows.append("| ✅ | Panda firmware | unchanged |")
  rows.append(f"| ✅ | Library | unchanged at {s.lib_old} |" if s.lib_old == s.lib_new else f"| ⚠️ | Library | moves {s.lib_old} → {s.lib_new} |")
  more = [f"…and {s.total - len(s.commits)} more"] if s.total > len(s.commits) else []
  pin_note = f"- `opendbc_repo`: both sides moved it; the job set it to ford-op/opendbc `bp-full` `{s.q[:8]}`"
  return "\n".join([
    f"❌ **Merge commit only: never squash or rebase.** A squash or rebase leaves bp-sunnypilot's commits out of `{s.target}`.",
    "",
    f"Automated sync of bp-sunnypilot {commit_link(REPO, s.t)} into `{s.target}` (#225), {s.total} commits, with `opendbc_repo` resolved by the job.",
    "The job merges this with a merge commit once the six required checks pass: the rows below inform, they don't block.",
    "",
    "| | Check | Result |",
    "|---|---|---|",
    *rows,
    "",
    "### Upstream commits",
    *fenced(s.commits), *more,
    "",
    "### Conflicts",
    pin_note if s.pin_conflicted else "None.",
    "",
    "### Files upstream changed that BluePilot also changes",
    *([f"- `{f}`" for f in s.overlap] or ["None."]),
    "",
    "## Validation",
    *[f"- [ ] {c}" for c in REQUIRED],
    "",
  ])


def issue_body(*, step: int, target: str, t: str, p: str, q: str, reason: str, fix: str, start: str = "", link: str = "",
               files: list[str] | tuple = ()) -> str:
  """The one "needs a maintainer" issue: what waits and why, and how to finish it by hand.
  fix: "merge" (redo the merge from start), "merge-target" (merge the moved target into the job branch), "push" (a fix)."""
  repo, branch = (ODBC, BP_FULL_BRANCH) if step == 1 else (REPO, job_branch(target))
  base = "bp-full" if step == 1 else target
  commands = [f"git clone --filter=blob:none https://github.com/{repo}.git && cd {repo.split('/')[1]}"]
  if fix == "push":
    commands += [f"git checkout -B {branch} origin/{branch}", "# fix what the check reports and commit it, then:"]
  else:
    commands.append(f"git checkout -B {branch} origin/{start or branch}")
    if fix == "merge-target":
      commands.append(f"git merge --no-ff origin/{base}")
    elif step == 1:
      commands.append(f"git merge --no-ff {p}")
    else:
      commands += [f"git merge --no-ff --no-commit {t}", f"git update-index --cacheinfo 160000,{q},opendbc_repo"]
    commands += ["# resolve the files listed above and git add them, then:", "git commit"]
  commands.append(f"git push origin {branch}")
  lines = [f"The BPx sync job (#225) can't finish on its own. Step {step} ({base}) is waiting: {reason}", "",
           f"- bp-sunnypilot: {commit_link(REPO, t)}, pinning ford-op/opendbc {commit_link(ODBC, p)}"]
  if link:
    lines.append(f"- {link}")
  if files:
    lines += ["", "Conflicted files:", *[f"- `{f}`" for f in files]]
  return "\n".join([
    *lines, "",
    "To finish it by hand (LFS isn't needed: `export GIT_LFS_SKIP_SMUDGE=1`):",
    "```sh", *commands, "```",
    f"The job then merges `{branch}` into `{base}` on green. Run it again (Actions → bpx sync → Run workflow, target `{target}`),",
    "or wait for the next bp-sunnypilot publish: it picks up where this stopped and closes this issue when the sync goes through.",
    "",
  ])



# --- the job: everything below talks to GitHub ---

def gh(token: str, *args: str, check: bool = True) -> str:
  return subprocess.run(["gh", *args], check=check, stdout=subprocess.PIPE, text=True, env={**os.environ, "GH_TOKEN": token}).stdout.strip()


def push_url(token: str, repo: str) -> str:
  return f"https://x-access-token:{token}@github.com/{repo}.git"


def output(**values: object) -> None:
  """Step outputs, each with its own random delimiter: a value (a file name, say) can't end it and forge another output."""
  with open(os.environ.get("GITHUB_OUTPUT", "/dev/stdout"), "a") as f:
    for key, value in values.items():
      delimiter = secrets.token_hex(16)
      if delimiter in str(value):
        raise ValueError(f"output {key} holds its delimiter")
      f.write(f"{key}<<{delimiter}\n{value}\n{delimiter}\n")


def require(ok: bool, what: str) -> None:
  if not ok:
    sys.exit(f"::error::{what}")


def env_sha(name: str) -> str:
  value = os.environ[name]
  require(bool(SHA.fullmatch(value)), f"{name} isn't a commit ID")
  return value


def env_number(name: str) -> str:
  value = os.environ[name]
  require(value.isdigit(), f"{name} isn't a number")
  return value


def ensure(git: Git, sha: str) -> None:
  if not git.ok("cat-file", "-e", f"{sha}^{{commit}}"):
    git("fetch", "-q", "--no-tags", "origin", sha)


def opendbc() -> Git:
  """ford-op/opendbc's history without file contents; merge-tree fetches the few blobs it needs."""
  path = Path(os.environ.get("RUNNER_TEMP", tempfile.gettempdir())) / "bpx-opendbc"
  if not path.exists():
    subprocess.run(["git", "clone", "-q", "--filter=blob:none", "--no-checkout", f"https://github.com/{ODBC}.git", str(path)], check=True)
  odbc = Git(path)
  odbc("fetch", "-q", "--no-tags", "origin", "+refs/heads/bp-full:refs/remotes/origin/bp-full")
  return odbc


def remote_branch(git: Git, branch: str) -> str | None:
  """Fetch a branch if it exists; its head."""
  if not git("ls-remote", "origin", f"refs/heads/{branch}"):
    return None
  git("fetch", "-q", "--no-tags", "origin", f"+refs/heads/{branch}:refs/remotes/origin/{branch}")
  return git("rev-parse", f"origin/{branch}")


def mirror(bp: Git) -> tuple[str, str]:
  """T and P."""
  t = remote_branch(bp, "bp-sunnypilot")
  assert t
  return t, bp("rev-parse", f"{t}:opendbc_repo")


def open_pr(token: str, repo: str, head: str, base: str) -> str:
  """The job's open PR from its own branch (not a fork's branch with the same name)."""
  return gh(token, "pr", "list", "-R", repo, "--head", head, "--base", base, "--state", "open", "--json", "number,isCrossRepository",
            "-q", "map(select(.isCrossRepository | not)) | .[0].number // empty")


def upsert_pr(token: str, repo: str, head: str, base: str, title: str, body: str) -> str:
  pr = open_pr(token, repo, head, base)
  if pr:
    gh(token, "pr", "edit", pr, "-R", repo, "--title", title, "--body", body)
    return pr
  return gh(token, "pr", "create", "-R", repo, "--head", head, "--base", base, "--title", title, "--body", body).rsplit("/", 1)[-1]


def wait_checks(token: str, repo: str, pr: str, head: str, names: tuple, minutes: int = 45) -> tuple[str, str]:
  """("pass" | "fail" | "timeout", links to the failed checks). Only the PR's own runs count: step 1's head is a bp-opendbc
  commit whose push runs are green already. Skipped counts as passed, as in branch protection; cancelled as still pending."""
  for _ in range(minutes * 3):
    # only trust checks for the new head: right after a push the PR can still show the old one's
    if gh(token, "pr", "view", pr, "-R", repo, "--json", "headRefOid", "-q", ".headRefOid", check=False) == head:
      out = gh(token, "pr", "checks", pr, "-R", repo, "--json", "name,bucket,event,link", check=False)  # exits non-zero while pending
      checks = [c for c in json.loads(out or "[]") if c["name"] in names and c["event"] == "pull_request"]
      failed = [c for c in checks if c["bucket"] == "fail"]
      if failed:
        return "fail", ", ".join(f"[{c['name']}]({c['link']})" for c in failed)
      # a name passes only when every run with it has: a check can run twice on one commit, and GitHub refuses the
      # merge while either copy runs
      if all(any(c["name"] == n for c in checks) and all(c["bucket"] in ("pass", "skipping") for c in checks if c["name"] == n)
             for n in names):
        return "pass", ""
    time.sleep(20)
  return "timeout", ""


def merge_pr(token: str, repo: str, pr: str, head: str, branch: str, subject: str, body: str) -> None:
  text = gh(token, "pr", "view", pr, "-R", repo, "--json", "body", "-q", ".body")
  gh(token, "pr", "edit", pr, "-R", repo, "--body", text.replace("\n- [ ] ", "\n- [x] "))
  # our own message: GitHub's default would add co-authors. Only the head this job checked.
  gh(token, "pr", "merge", pr, "-R", repo, "--merge", "--match-head-commit", head, "--subject", subject, "--body", body)
  gh(token, "api", "-X", "DELETE", f"repos/{repo}/git/refs/heads/{branch}")


def issue_title(target: str) -> str:
  return f"BPx sync into {target} needs a maintainer"


def find_issue(token: str, target: str) -> str:
  title = issue_title(target)
  return gh(token, "issue", "list", "-R", REPO, "--state", "open", "--search", f'in:title "{title}"', "--json", "number,title",
            "-q", f"map(select(.title == {json.dumps(title)})) | .[0].number // empty")


def report(token: str, target: str, body: str) -> tuple[str, bool]:
  """Write the one issue; (its number, whether this is news). The same stuck state is left as it is."""
  num = find_issue(token, target)
  if not num:
    return gh(token, "issue", "create", "-R", REPO, "--title", issue_title(target), "--body", body).rsplit("/", 1)[-1], True
  if gh(token, "issue", "view", num, "-R", REPO, "--json", "body", "-q", ".body").strip() == body.strip():
    return num, False
  gh(token, "issue", "edit", num, "-R", REPO, "--body", body)
  return num, True


def stuck(token: str, target: str, body: str, what: str) -> tuple[str, int]:
  """Record a stuck state. The run fails only when the state is new or changed: public-sync starts a run every few hours."""
  num, news = report(token, target, body)
  if news:
    print(f"::error::{what}; see #{num}")
    return num, 1
  print(f"::notice::still waiting, as #{num} already says: {what}")
  return num, 0


def close_issue(token: str, target: str, why: str) -> None:
  num = find_issue(token, target)
  if num:
    gh(token, "issue", "close", num, "-R", REPO, "--comment", why)


def marker(up: str) -> str:
  return f"<!-- bpx-sync-review {up} -->"


def reviewed(token: str, on: str, up: str) -> bool:
  return marker(up) in gh(token, "api", f"repos/{REPO}/issues/{on}/comments", "--paginate", "-q", ".[].body")


def want_review(token: str, on: str, **inputs: object) -> None:
  """Hand the review job its inputs, once per upstream commit. The review job holds no App token (bpx-sync.yml)."""
  if not reviewed(token, on, str(inputs["up"])):
    output(review=json.dumps({"on": on, **inputs}))


def library_label(odbc: Git, commit: str, tags: dict[str, str]) -> str:
  lib = odbc("rev-parse", f"{commit}:opendbc/bluepilot_lateral")
  return f"`{tags[lib]}`" if lib in tags else f"`{lib[:8]}`"


def library_tags() -> dict[str, str]:
  out = subprocess.run(["git", "ls-remote", "--tags", "https://github.com/BluePilotDev/bluepilot-lateral.git"],
                       check=True, stdout=subprocess.PIPE, text=True).stdout
  tags: dict[str, str] = {}
  for line in out.splitlines():
    sha, ref = line.split("\t")
    tags[sha] = ref.removeprefix("refs/tags/").removesuffix("^{}")  # an annotated tag's commit is on its ^{} line
  return tags


def step_bp_full(target: str) -> int:
  """Step 1: bp-full has P, or a PR merging P into it is merged on green. Output ready=yes lets step 2 run."""
  bp_token, odbc_token = os.environ["BP_TOKEN"], os.environ["ODBC_TOKEN"]
  t, p = mirror(Git("."))
  odbc = opendbc()
  ensure(odbc, p)
  full = odbc("rev-parse", "origin/bp-full")
  if odbc.is_ancestor(p, full):
    print(f"bp-full already contains {p[:8]}")
    output(ready="yes")
    return 0
  branch = remote_branch(odbc, BP_FULL_BRANCH)
  head, conflicts = bp_full_head(odbc, full, p, branch)
  def issue(**kw: Any) -> str:
    return issue_body(step=1, target=target, t=t, p=p, q="", **kw)
  pr = open_pr(odbc_token, ODBC, BP_FULL_BRANCH, "bp-full")
  if conflicts:
    built_on = branch and not odbc.is_ancestor(branch, full) and not odbc.is_ancestor(branch, p)
    num, rc = stuck(bp_token, target, issue(fix="merge", start=BP_FULL_BRANCH if built_on else "bp-full", files=conflicts,
                                                 link=f"Open PR: ford-op/opendbc#{pr}" if pr else "",
                                                 reason=f"merging bp-sunnypilot's opendbc pin `{p[:8]}` into `bp-full` conflicts in {count(len(conflicts))}."),
                    f"merging {p[:8]} into bp-full conflicts")
    want_review(bp_token, num, repo=ODBC, base=odbc("merge-base", full, p), up=p, integration=full, q="", files=conflicts, conflicts=conflicts)
    return rc
  if head != branch:
    odbc("push", "-q", "--no-verify", push_url(odbc_token, ODBC), f"{head}:refs/heads/{BP_FULL_BRANCH}")
  body = "\n".join([
    f"Merges {commit_link(ODBC, p)}, the ford-op/opendbc commit that bp-sunnypilot {commit_link(REPO, t)} pins, into `bp-full`.",
    "Opened by the BPx sync job (BluePilotDev/bluepilot#225), which merges it with a merge commit once the required checks pass.", "",
    "## Validation", *[f"- [ ] {c}" for c in ODBC_REQUIRED], ""])
  pr = upsert_pr(odbc_token, ODBC, BP_FULL_BRANCH, "bp-full", f"sync: bp-sunnypilot's opendbc pin {p[:8]} into bp-full", body)
  print(f"ford-op/opendbc#{pr}")
  result, links = wait_checks(odbc_token, ODBC, pr, head, ODBC_REQUIRED)
  if result == "pass":
    merge_pr(odbc_token, ODBC, pr, head, BP_FULL_BRANCH, f"Merge bp-sunnypilot's opendbc pin {p[:8]} into bp-full (#{pr})",
             f"Merged on green by the BPx sync job (BluePilotDev/bluepilot#225), for bp-sunnypilot {t[:8]}.")
    output(ready="yes", bp_full_pr=pr)
    return 0
  if result == "fail":
    return stuck(bp_token, target, issue(fix="push", link=f"ford-op/opendbc#{pr}, failing: {links}",
                                              reason=f"ford-op/opendbc#{pr} (bp-sunnypilot's pin into `bp-full`) has a failing required check."),
                 f"ford-op/opendbc#{pr}: a required check failed")[1]
  print(f"::notice::ford-op/opendbc#{pr}: checks still running after 45 minutes; the next run picks it up")
  output(ready="no")
  return 0


def prepare(target: str) -> int:
  """Step 2: build the merge of T into the target with opendbc_repo = Q, push the job branch and open or update the PR."""
  token = os.environ["BP_TOKEN"]
  bp = Git(".")
  t, p = mirror(bp)
  branch = job_branch(target)
  target_sha = remote_branch(bp, target)
  require(target_sha is not None, f"no branch {target}")
  assert target_sha
  branch_sha = remote_branch(bp, branch)
  odbc = opendbc()
  pin = bp("rev-parse", f"{target_sha}:opendbc_repo")
  for sha in (p, pin):
    ensure(odbc, sha)
  q = pick_q(odbc, p, pin, odbc("rev-parse", "origin/bp-full"))
  pl = plan(bp, target, target_sha, t, q, branch_sha)
  if pl.action == "nothing":
    print(f"{target} already contains bp-sunnypilot {t[:8]} and pins {q[:8]}")
    close_issue(token, target, f"Nothing left to do: `{target}` contains bp-sunnypilot `{t[:8]}` and pins `bp-full` `{q[:8]}`.")
    return 0
  if pl.action == "conflict":
    pr = open_pr(token, REPO, branch, target)
    def issue(**kw: Any) -> str:
      return issue_body(step=2, target=target, t=t, p=p, q=q, files=pl.conflicts, link=f"Open sync PR: #{pr}" if pr else "", **kw)
    if pl.against_target:
      body = issue(fix="merge-target",
                        reason=f"the open sync PR's branch `{branch}` conflicts with `{target}`, which moved since, in {count(len(pl.conflicts))}.")
      review = {"base": bp("merge-base", target_sha, pl.base), "up": t, "integration": target_sha}
    else:
      start = branch if pl.base == branch_sha else target
      body = issue(fix="merge", start=start,
                        reason=f"merging bp-sunnypilot into `{start}` conflicts in {count(len(pl.conflicts))}.")
      review = {"base": bp("merge-base", pl.base, t), "up": t, "integration": pl.base}
    num, rc = stuck(token, target, body, f"the sync into {target} conflicts")
    want_review(token, num, repo=REPO, q=q, files=pl.conflicts, conflicts=pl.conflicts, **review)
    return rc
  if pl.commit != branch_sha:
    bp("push", "-q", "--no-verify", push_url(token, REPO), f"{pl.commit}:refs/heads/{branch}")
  base = bp("merge-base", target_sha, t)
  tags = library_tags()
  summary = Summary(
    target=target, t=t, p=p, q=q,
    commits=bp("log", "-n", "200", "--format=%h %s", f"{target_sha}..{t}").splitlines(),
    total=int(bp("rev-list", "--count", f"{target_sha}..{t}")),
    pin_conflicted=pl.pin_conflicted,
    overlap=overlap(bp, base, t, bp("show", f"{pl.commit}:.github/bluepilot-hooks.txt")),
    firmware=[f for f in odbc("diff", "--name-only", pin, q, "--", "opendbc/safety").splitlines() if "/tests/" not in f],
    lib_old=library_label(odbc, pin, tags), lib_new=library_label(odbc, q, tags),
    bp_full_pr=int(os.environ["BP_FULL_PR"]) if os.environ.get("BP_FULL_PR") else None)
  pr = upsert_pr(token, REPO, branch, target, f"sync: bp-sunnypilot {t[:8]} into {target}", pr_body(summary))
  print(f"#{pr} at {pl.commit}")
  output(pr=pr, head=pl.commit, t=t, p=p, q=q)
  if summary.overlap:
    want_review(token, pr, repo=REPO, base=base, up=t, integration=target_sha, q=q, files=summary.overlap, conflicts=[])
  return 0


def review_prompt(path: str) -> str:
  """The review's prompt: the prompt file and the inputs the sync job handed over (REVIEW)."""
  r = json.loads(os.environ["REVIEW"])
  lines = [Path(path).read_text(), "", "## Inputs", f"- BASE: {r['base']}", f"- UPSTREAM: {r['up']}", f"- INTEGRATION: {r['integration']}"]
  if r["repo"] == ODBC:
    lines.append("- Step 1, in ford-op/opendbc: UPSTREAM is the bp-opendbc commit bp-sunnypilot pins, INTEGRATION is `bp-full`.")
  else:
    lines.append(f"- `opendbc_repo`: resolved by the job to ford-op/opendbc `bp-full` {r['q']}")
  lines += [f"- Files that conflict: {', '.join(r['conflicts']) or 'none'}", "- Files to review:", *[f"  - {f}" for f in r["files"]], ""]
  return "\n".join(lines)


def post(path: str) -> int:
  """Post the review job's text as a comment, once per upstream commit. Runs with the App token: it only reads the file."""
  token, r = os.environ["BP_TOKEN"], json.loads(os.environ["REVIEW"])
  on, up = str(r["on"]), str(r["up"])
  require(on.isdigit() and bool(SHA.fullmatch(up)), "malformed review inputs")
  text = Path(path).read_text().strip()
  require(bool(text), "the review came back empty")
  if reviewed(token, on, up):
    print(f"#{on} already has a review for {up}")
    return 0
  gh(token, "issue", "comment", on, "-R", REPO, "--body", f"{marker(up)}\n{text}")  # an issue or a PR: both take issue comments
  return 0


def merge(target: str) -> int:
  """Step 2, last part: merge the sync PR on green, delete the job branch, close the issue."""
  token, pr, head, t, p, q = os.environ["BP_TOKEN"], env_number("PR"), env_sha("HEAD"), env_sha("T"), env_sha("P"), env_sha("Q")
  branch = job_branch(target)
  view = json.loads(gh(token, "pr", "view", pr, "-R", REPO, "--json", "headRefName,baseRefName,isCrossRepository,state"))
  require(view == {"headRefName": branch, "baseRefName": target, "isCrossRepository": False, "state": "OPEN"},
          f"#{pr} isn't the job's open PR from {branch} into {target}: {view}")
  result, links = wait_checks(token, REPO, pr, head, REQUIRED)
  if result == "pass":
    merge_pr(token, REPO, pr, head, branch, f"Merge bp-sunnypilot {t[:8]} into {target} (#{pr})",
             f"opendbc_repo: ford-op/opendbc bp-full {q[:8]}. Merged on green by the BPx sync job (#225).")
    close_issue(token, target, f"Synced: #{pr} merged bp-sunnypilot `{t[:8]}` into `{target}`.")
    print(f"merged #{pr}")
    return 0
  if result == "fail":
    return stuck(token, target, issue_body(step=2, target=target, t=t, p=p, q=q, fix="push", link=f"Sync PR: #{pr}; failing: {links}",
                                           reason=f"#{pr} has a failing required check. Fix it on the PR's branch." + REPLAY),
                 f"#{pr}: a required check failed; it stays open")[1]
  print(f"::notice::#{pr}: checks still running after 45 minutes; the next run picks it up")
  return 0


def main(argv: list[str]) -> int:
  if argv[1] == "review-prompt":
    print(review_prompt(argv[2]))
    return 0
  if argv[1] == "post":
    return post(argv[2])
  target = os.environ["TARGET"]
  require(target != "bp-sunnypilot", "the target can't be the bp-sunnypilot mirror")
  return {"bp-full": step_bp_full, "prepare": prepare, "merge": merge}[argv[1]](target)


if __name__ == "__main__":
  sys.exit(main(sys.argv))
