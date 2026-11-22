#!/usr/bin/env python3
"""BluePilot: the BPx sync job (#225). Keeps a full BluePilot branch current with bp-sunnypilot.

T = this repo's bp-sunnypilot branch (the mirror public-sync publishes), P = the opendbc commit T pins,
Q = the ford-op/opendbc bp-full commit with P that the target's opendbc_repo points at.
usage: bpx_sync.py bp-full | prepare | merge   (env: TARGET, BP_TOKEN, ODBC_TOKEN; see bpx-sync.yml)"""
import json
import os
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path

from bluepilot_hooks import listed_hooks

REPO = os.environ.get("GITHUB_REPOSITORY", "BluePilotDev/bluepilot")
ODBC = "ford-op/opendbc"
# required checks, mirrored from branch protection (the App can't read protection; keep in sync if a list changes)
REQUIRED = ("build release", "static analysis", "unit tests", "process replay", "install smoke", "bluepilot-hooks")
ODBC_REQUIRED = ("bluepilot-hooks", "safety", "bluepilot-safety", "./test.sh (Linux)")
BP_FULL_BRANCH = "bpx-sync/bp-full"
REPLAY = " A process replay difference gets the `replay-accepted` label once it is traced and written in the PR (the job never adds it)."
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


@dataclass
class Plan:
  action: str  # "nothing", "conflict" (nothing is pushed) or "ready" (commit is the job branch's head)
  base: str = ""  # the target or the job branch: what T is merged into
  commit: str = ""
  conflicts: list[str] = field(default_factory=list)  # other than opendbc_repo
  pin_conflicted: bool = False


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
  with tempfile.TemporaryDirectory() as tmp:
    env = {**os.environ, "GIT_INDEX_FILE": os.path.join(tmp, "index")}
    git("read-tree", tree, env=env)
    git("update-index", "--cacheinfo", f"160000,{q},opendbc_repo", env=env)
    tree = git("write-tree", env=env)
  if tree == git("rev-parse", f"{base}^{{tree}}") and git.is_ancestor(t, base):
    return Plan("ready", base, commit=base, pin_conflicted="opendbc_repo" in conflicts)
  parents = ["-p", base] + ([] if git.is_ancestor(t, base) else ["-p", t])
  commit = git(*BOT, "commit-tree", tree, *parents, "-m", f"Merge bp-sunnypilot {t[:8]} into {target_name}",
               "-m", f"opendbc_repo: ford-op/opendbc bp-full {q[:8]}. Built by the BPx sync job (#225).")
  return Plan("ready", base, commit=commit, pin_conflicted="opendbc_repo" in conflicts)


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


def pr_body(s: Summary) -> str:
  merged = f" (merged by ford-op/opendbc#{s.bp_full_pr})" if s.bp_full_pr else ""
  rows = [f"| ✅ | opendbc | ford-op/opendbc `bp-full` contains bp-sunnypilot's pin `{s.p[:8]}`{merged}; `opendbc_repo` → `{s.q[:8]}` |",
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
  more = [f"- …and {s.total - len(s.commits)} more"] if s.total > len(s.commits) else []
  pin_note = f"- `opendbc_repo`: both sides moved it; the job set it to ford-op/opendbc `bp-full` `{s.q[:8]}`"
  return "\n".join([
    f"❌ **Merge commit only: never squash or rebase.** A squash or rebase leaves bp-sunnypilot's commits out of `{s.target}`.",
    "",
    f"Automated sync of bp-sunnypilot `{s.t[:8]}` into `{s.target}` (#225), {s.total} commits, with `opendbc_repo` resolved by the job.",
    "The job merges this with a merge commit once the six required checks pass: the rows below inform, they don't block.",
    "",
    "| | Check | Result |",
    "|---|---|---|",
    *rows,
    "",
    "### Upstream commits",
    *[f"- {c}" for c in s.commits], *more,
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


def issue_body(*, step: int, target: str, t: str, p: str, q: str, start: str, reason: str, link: str = "",
               files: list[str] | tuple = ()) -> str:
  """The one "needs a maintainer" issue: what waits and why, and how to finish it by hand."""
  if step == 1:
    branch = f"sync/bp-full-{p[:8]}"
    commands = ["git clone --filter=blob:none https://github.com/ford-op/opendbc.git && cd opendbc",
                f"git checkout -B {branch} origin/bp-full",
                f"git merge --no-ff {p}",
                "# if it stops on conflicts: resolve the files listed above, git add them and git commit",
                f"git push origin {branch}"]
    after = f"Then open a PR from `{branch}` into `bp-full` in ford-op/opendbc and merge it with a merge commit once its checks pass. "
  else:
    commands = ["git clone --filter=blob:none https://github.com/BluePilotDev/bluepilot.git && cd bluepilot",
                f"git checkout -B bpx-sync/{target} origin/{start}",
                f"git merge --no-ff --no-commit {t}",
                f"git update-index --cacheinfo 160000,{q},opendbc_repo",
                "# resolve the files listed above and git add them, then:",
                "git commit",
                f"git push origin bpx-sync/{target}"]
    after = "The job then opens the sync PR from that branch and merges it on green. "
  where = "bp-full" if step == 1 else target
  lines = [f"The BPx sync job (#225) can't finish on its own. Step {step} ({where}) is waiting: {reason}", "",
           f"- bp-sunnypilot: [`{t[:8]}`](https://github.com/BluePilotDev/bluepilot/commit/{t}), pinning ford-op/opendbc `{p[:8]}`"]
  if link:
    lines.append(f"- {link}")
  if files:
    lines += ["", "Conflicted files:", *[f"- `{f}`" for f in files]]
  return "\n".join([
    *lines, "",
    "To redo the merge by hand (LFS isn't needed: `export GIT_LFS_SKIP_SMUDGE=1`):",
    "```sh", *commands, "```",
    f"{after}Then run the job again (Actions → bpx sync → Run workflow, target `{target}`).",
    "It picks up where this stopped and closes this issue when the sync goes through.",
    "",
  ])


# --- the job: everything below talks to GitHub ---

def gh(token: str, *args: str, check: bool = True) -> str:
  return subprocess.run(["gh", *args], check=check, stdout=subprocess.PIPE, text=True, env={**os.environ, "GH_TOKEN": token}).stdout.strip()


def push_url(token: str, repo: str) -> str:
  return f"https://x-access-token:{token}@github.com/{repo}.git"


def output(**values: object) -> None:
  with open(os.environ.get("GITHUB_OUTPUT", "/dev/stdout"), "a") as f:
    for key, value in values.items():
      f.write(f"{key}<<EOF\n{value}\nEOF\n")


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


def mirror(bp: Git) -> tuple[str, str]:
  """T and P."""
  bp("fetch", "-q", "--no-tags", "origin", "+refs/heads/bp-sunnypilot:refs/remotes/origin/bp-sunnypilot")
  t = bp("rev-parse", "origin/bp-sunnypilot")
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
  """("pass" | "fail" | "timeout", links to the failed checks). Skipped counts as passed, as in branch protection."""
  for _ in range(minutes * 3):
    # only trust checks for the new head: right after a push the PR can still show the old one's
    if gh(token, "pr", "view", pr, "-R", repo, "--json", "headRefOid", "-q", ".headRefOid", check=False) == head:
      out = gh(token, "pr", "checks", pr, "-R", repo, "--json", "name,bucket,link", check=False)  # exits non-zero while pending
      checks = [c for c in json.loads(out or "[]") if c["name"] in names]
      failed = [c for c in checks if c["bucket"] in ("fail", "cancel")]
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


def report(token: str, target: str, body: str) -> str:
  """Write the one issue (updated in place); returns its number."""
  num = find_issue(token, target)
  if num:
    gh(token, "issue", "edit", num, "-R", REPO, "--body", body)
    return num
  return gh(token, "issue", "create", "-R", REPO, "--title", issue_title(target), "--body", body).rsplit("/", 1)[-1]


def close_issue(token: str, target: str, why: str) -> None:
  num = find_issue(token, target)
  if num:
    gh(token, "issue", "close", num, "-R", REPO, "--comment", why)


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
  _, conflicts = merge_tree(odbc, full, p)
  if conflicts:
    report(bp_token, target, issue_body(step=1, target=target, t=t, p=p, q="", start="bp-full", files=conflicts,
                                        reason=f"merging bp-sunnypilot's opendbc pin `{p[:8]}` into `bp-full` conflicts in {count(len(conflicts))}."))
    print(f"::error::merging {p[:8]} into bp-full conflicts; see the issue")
    return 1
  # the PR's head is P itself, pushed to the job's branch: P only moves forward on bp-opendbc, so this never needs a force
  odbc("push", "-q", "--no-verify", push_url(odbc_token, ODBC), f"{p}:refs/heads/{BP_FULL_BRANCH}")
  body = "\n".join([
    f"Merges `{p[:8]}`, the ford-op/opendbc commit that bp-sunnypilot `{t[:8]}` pins, into `bp-full`.",
    "Opened by the BPx sync job (BluePilotDev/bluepilot#225), which merges it with a merge commit once the required checks pass.", "",
    "## Validation", *[f"- [ ] {c}" for c in ODBC_REQUIRED], ""])
  pr = upsert_pr(odbc_token, ODBC, BP_FULL_BRANCH, "bp-full", f"sync: bp-sunnypilot's opendbc pin {p[:8]} into bp-full", body)
  print(f"ford-op/opendbc#{pr}")
  result, links = wait_checks(odbc_token, ODBC, pr, p, ODBC_REQUIRED)
  if result == "pass":
    merge_pr(odbc_token, ODBC, pr, p, BP_FULL_BRANCH, f"Merge bp-sunnypilot's opendbc pin {p[:8]} into bp-full (#{pr})",
             f"Merged on green by the BPx sync job (BluePilotDev/bluepilot#225), for bp-sunnypilot {t[:8]}.")
    output(ready="yes", bp_full_pr=pr)
    return 0
  if result == "fail":
    report(bp_token, target, issue_body(step=1, target=target, t=t, p=p, q="", start="bp-full", link=f"Failing: {links}",
                                        reason=f"ford-op/opendbc#{pr} (bp-sunnypilot's pin into `bp-full`) has a failing required check."))
    print(f"::error::ford-op/opendbc#{pr}: a required check failed; see the issue")
    return 1
  print(f"::notice::ford-op/opendbc#{pr}: checks still running after 45 minutes; the next run picks it up")
  output(ready="no")
  return 0


def prepare(target: str) -> int:
  """Step 2: build the merge of T into the target with opendbc_repo = Q, push the job branch and open or update the PR."""
  token = os.environ["BP_TOKEN"]
  bp = Git(".")
  t, p = mirror(bp)
  branch = f"bpx-sync/{target}"
  bp("fetch", "-q", "--no-tags", "origin", f"+refs/heads/{target}:refs/remotes/origin/{target}")
  have_branch = bool(bp("ls-remote", "origin", f"refs/heads/{branch}"))
  if have_branch:
    bp("fetch", "-q", "--no-tags", "origin", f"+refs/heads/{branch}:refs/remotes/origin/{branch}")
  target_sha = bp("rev-parse", f"origin/{target}")
  branch_sha = bp("rev-parse", f"origin/{branch}") if have_branch else None
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
    start = branch if pl.base == branch_sha else target
    num = report(token, target, issue_body(step=2, target=target, t=t, p=p, q=q, start=start, files=pl.conflicts,
                                           reason=f"merging bp-sunnypilot into `{start}` conflicts in {count(len(pl.conflicts))}."))
    output(review="yes", review_on=num, files="\n".join(pl.conflicts), conflicts="\n".join(pl.conflicts),
           base=bp("merge-base", pl.base, t), up=t, integration=pl.base, q=q)
    print(f"::error::merging bp-sunnypilot {t[:8]} into {start} conflicts; see the issue")
    return 1
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
  output(pr=pr, head=pl.commit, t=t, q=q)
  if summary.overlap:
    output(review="yes", review_on=pr, files="\n".join(summary.overlap), conflicts="", base=base, up=t, integration=target_sha, q=q)
  return 0


def merge(target: str) -> int:
  """Step 2, last part: merge the sync PR on green, delete the job branch, close the issue."""
  token, pr, head, t, q = (os.environ[k] for k in ("BP_TOKEN", "PR", "HEAD", "T", "Q"))
  result, links = wait_checks(token, REPO, pr, head, REQUIRED)
  if result == "pass":
    merge_pr(token, REPO, pr, head, f"bpx-sync/{target}", f"Merge bp-sunnypilot {t[:8]} into {target} (#{pr})",
             f"opendbc_repo: ford-op/opendbc bp-full {q[:8]}. Merged on green by the BPx sync job (#225).")
    close_issue(token, target, f"Synced: #{pr} merged bp-sunnypilot `{t[:8]}` into `{target}`.")
    print(f"merged #{pr}")
    return 0
  if result == "fail":
    p = Git(".")("rev-parse", f"{t}:opendbc_repo")
    report(token, target, issue_body(step=2, target=target, t=t, p=p, q=q, start=target, link=f"PR #{pr}; failing: {links}",
                                     reason=f"#{pr} has a failing required check. Fix it on the PR's branch, or redo the merge." + REPLAY))
    print(f"::error::#{pr}: a required check failed; it stays open, see the issue")
    return 1
  print(f"::notice::#{pr}: checks still running after 45 minutes; the next run picks it up")
  return 0


def main(argv: list[str]) -> int:
  target = os.environ["TARGET"]
  if target == "bp-sunnypilot":
    sys.exit("the target can't be the bp-sunnypilot mirror")
  return {"bp-full": step_bp_full, "prepare": prepare, "merge": merge}[argv[1]](target)


if __name__ == "__main__":
  sys.exit(main(sys.argv))
