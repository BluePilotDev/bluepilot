#!/usr/bin/env python3
"""BluePilot: the release job (#227). A pushed release tag creates the version's branch and moves bp-stable.
usage: bp_release.py   (env: see bp-release.yml)"""
import json
import os
import re
import sys
from dataclasses import dataclass

from bpx_sync import REPO, Git, gh, push_url, remote_branch

# a pushed commit's checks (#227): bp tests' calls to tests.yaml and install smoke. bluepilot-hooks runs on PRs only.
# Process replay isn't one: on bp-dev the push run publishes the references instead of comparing, and on a version branch
# it compares against bp-dev's. The PR check (required by branch protection, passing with the replay-accepted label once a
# difference is explained) is the replay gate.
REQUIRED = ("tests / build release", "tests / static analysis", "tests / unit tests", "install smoke")


VERSION = r"(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)"  # MAJOR.MINOR.PATCH, no leading zeros


class Refused(Exception):
  """A release rule failed: one sentence saying which and what to do. Nothing has been written."""


def parse_tag(tag: str, prefix: str) -> tuple[int, int, int]:
  """The tag is untrusted input: nothing uses it before this match."""
  m = re.fullmatch(re.escape(prefix) + VERSION, tag)
  if not m:
    raise Refused(f"{tag!r} isn't a release tag: use {prefix}MAJOR.MINOR.PATCH, for example {prefix}7.1.0.")
  return int(m[1]), int(m[2]), int(m[3])


def not_green(check_runs: list[dict], push_suites: set[int]) -> list[str]:
  """The required checks that didn't pass in the commit's push runs: missing, running, failed or skipped all count.
  check_runs: {name, status, conclusion, suite}; push_suites: the check suites of the commit's push-event workflow runs."""
  pushed = [c for c in check_runs if c["suite"] in push_suites]
  return [n for n in REQUIRED if not (any(c["name"] == n for c in pushed) and
                                      all(c["conclusion"] == "success" for c in pushed if c["name"] == n))]


@dataclass(frozen=True)
class Config:
  source: str = "bp-dev"  # where a new version's commit must be
  stable: str = "bp-stable"
  branch_prefix: str = "bp-"  # bp-7.1
  tag_prefix: str = "v"  # v7.1.0


TEST_SPACE = "bpx-test/"  # throwaway names for testing the job, as with the BPx sync job's targets


def config(environ) -> Config:
  """The names from the environment (empty: the default). They come from dispatch inputs: any writer can set them, and the
  job pushes with the App, so a run that changes any of them writes only throwaway branches."""
  default = Config()
  cfg = Config(*(environ.get(key) or getattr(default, name) for key, name in
                 (("SOURCE_BRANCH", "source"), ("STABLE_BRANCH", "stable"), ("BRANCH_PREFIX", "branch_prefix"), ("TAG_PREFIX", "tag_prefix"))))
  for name in vars(cfg).values():
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._/-]*", name) or ".." in name or "//" in name:
      raise Refused(f"{name!r} isn't a plain branch or tag name: use letters, digits and . _ - /.")
  if cfg != default and not (cfg.stable.startswith(TEST_SPACE) and cfg.branch_prefix.startswith(TEST_SPACE)):
    raise Refused(f"Other names are for tests only: start the stable branch and the version-branch prefix with {TEST_SPACE}, or leave all four as they are.")
  return cfg


@dataclass
class Facts:
  commit: str  # the tagged commit
  bpversion: str | None  # BPVERSION in it, stripped; None when it has none
  on_source: bool  # reachable from the source branch's tip
  version_branch: str | None  # the version branch's tip, None when it doesn't exist
  stable: str | None
  tags: list[str]  # release tags counted as released: higher versions whose version branch holds their commit (a stray or
                   # refused tag doesn't count); lower ones never matter
  not_green: list[str]  # required push checks that didn't pass
  stable_bpversion: str | None = None  # BPVERSION on bp-stable's commit, when bp-stable exists; None when it has none


def version_branch(cfg: Config, version: tuple[int, int, int]) -> str:
  return f"{cfg.branch_prefix}{version[0]}.{version[1]}"


@dataclass(frozen=True)
class Update:
  branch: str
  sha: str
  old: str | None  # what the branch must still be at (None: it must not exist)


def plan(version: tuple[int, int, int], f: Facts, cfg: Config) -> list[Update]:
  """The branch writes the release takes, or Refused. An empty list: everything is where it should be already."""
  major, minor, patch = version
  tag, branch, short = f"{cfg.tag_prefix}{major}.{minor}.{patch}", version_branch(cfg, version), f.commit[:8]
  if f.bpversion is None or not re.fullmatch(VERSION, f.bpversion):  # the file's content is shown only once it's a version
    raise Refused(f"BPVERSION in {short} is missing or isn't MAJOR.MINOR.PATCH: fix it in a PR, or tag a commit that has a good one.")
  if f.bpversion != f"{major}.{minor}.{patch}":
    want = f"{major}.{minor}.{patch}"
    raise Refused(f"{tag} doesn't match BPVERSION in {short} ({f.bpversion}): set BPVERSION to {want} in a PR, or tag the commit that has it.")
  updates = []
  if patch == 0:
    if not f.on_source:
      raise Refused(f"{tag} is a new version, but {short} isn't on {cfg.source}: tag a {cfg.source} commit.")
    if f.version_branch is None:
      updates.append(Update(branch, f.commit, None))
    elif f.version_branch != f.commit:
      raise Refused(f"{branch} already exists at {f.version_branch[:8]}: a version is released once, so release a fix as a patch tag on {branch}.")
  elif f.version_branch is None:
    raise Refused(f"{tag} is a patch, but there is no branch {branch}: release {cfg.tag_prefix}{major}.{minor}.0 first.")
  elif f.version_branch != f.commit:  # the tag is already the branch's tip: a patch writes no version branch
    raise Refused(f"{tag} is a patch, but {short} isn't the tip of {branch} ({f.version_branch[:8]}): tag the tip of {branch}.")
  if f.not_green:
    failing = ", ".join(f.not_green)
    raise Refused(f"{short}'s push checks aren't all green ({failing}): wait for them and run the job again, or fix them and tag the fixed commit.")
  if f.stable != f.commit and stable_stays(version, f, cfg) is None:
    updates.append(Update(cfg.stable, f.commit, f.stable))
  return updates


def stable_stays(version: tuple[int, int, int], f: Facts, cfg: Config) -> str | None:
  """Why bp-stable doesn't move to this release (None: it moves, or is there already). It never moves back: not past a higher
  release still on its version branch, and not past the version on bp-stable itself, whatever branches were deleted."""
  if f.stable == f.commit:
    return None
  if higher := higher_release(version, f.tags, cfg.tag_prefix):  # checked first: it holds with no bp-stable too
    return f"{cfg.stable} doesn't move: {higher} is a higher release."
  if f.stable is None:
    return None
  if f.stable_bpversion is None or not re.fullmatch(VERSION, f.stable_bpversion):
    where = f"{cfg.stable} ({f.stable[:8]})"
    raise Refused(f"BPVERSION on {where} is missing or isn't MAJOR.MINOR.PATCH: put {cfg.stable} right by hand, then run the job again.")
  at = parse_tag(f.stable_bpversion, "")
  if at == version:
    where = f"{f.stable_bpversion} at another commit ({f.stable[:8]})"
    raise Refused(f"{cfg.stable} already has {where}: a release is never re-pointed, so release the change as a patch.")
  if at > version:
    return f"{cfg.stable} doesn't move: it is at {f.stable_bpversion}, a higher version."
  return None


def higher_release(version: tuple[int, int, int], tags: list[str], prefix: str) -> str | None:
  """The highest of the tags above this version: while there is one, bp-stable stays where it is."""
  higher = sorted((v, t) for t in tags if (v := _version(t, prefix)) and v > version)
  return higher[-1][1] if higher else None


def _version(tag: str, prefix: str) -> tuple[int, int, int] | None:
  try:
    return parse_tag(tag, prefix)
  except Refused:
    return None


# --- the job: everything below talks to GitHub ---

def gather(git: Git, cfg: Config, tag: str, version: tuple[int, int, int], token: str) -> Facts:
  """What plan() decides on. token reads the Actions and Checks APIs (the App can't read Actions)."""
  commits: dict[str, str] = {}
  for line in git("ls-remote", "--tags", "origin").splitlines():
    sha, ref = line.split("\t")
    name = ref.removeprefix("refs/tags/")
    if name.endswith("^{}"):  # an annotated tag's commit is on its ^{} line
      commits[name.removesuffix("^{}")] = sha
    else:
      commits.setdefault(name, sha)
  if tag not in commits:
    raise Refused(f"There is no tag {tag}: push the tag first.")
  tips: dict[str, str | None] = {}
  def released(t: str) -> bool:
    v = _version(t, cfg.tag_prefix)
    if not v or v <= version:
      return False
    branch = version_branch(cfg, v)
    if branch not in tips:
      tips[branch] = remote_branch(git, branch)
    tip = tips[branch]
    return tip is not None and git.is_ancestor(commits[t], tip)
  git("fetch", "-q", "--no-tags", "origin", f"+refs/tags/{tag}:refs/tags/{tag}")
  commit = git("rev-parse", f"refs/tags/{tag}^{{commit}}")
  bpversion = git("show", f"{commit}:BPVERSION") if git.ok("cat-file", "-e", f"{commit}:BPVERSION") else None
  source = remote_branch(git, cfg.source)
  stable = remote_branch(git, cfg.stable)
  stable_bpversion = git("show", f"{stable}:BPVERSION") if stable and git.ok("cat-file", "-e", f"{stable}:BPVERSION") else None
  suites = gh(token, "api", "--paginate", f"repos/{REPO}/actions/runs?head_sha={commit}&event=push&per_page=100",
              "-q", ".workflow_runs[].check_suite_id")
  runs = gh(token, "api", "--paginate", f"repos/{REPO}/commits/{commit}/check-runs?per_page=100",
            "-q", ".check_runs[] | {name, status, conclusion, suite: .check_suite.id}")
  return Facts(commit=commit, bpversion=bpversion, on_source=bool(source) and git.is_ancestor(commit, source),
               version_branch=remote_branch(git, version_branch(cfg, version)), stable=stable,
               tags=[tag, *filter(released, commits)], stable_bpversion=stable_bpversion,
               not_green=not_green([json.loads(line) for line in runs.splitlines()], {int(s) for s in suites.split()}))


def release(environ, git: Git) -> int:
  cfg = config(environ)
  tag = environ["TAG"]
  version = parse_tag(tag, cfg.tag_prefix)
  facts = gather(git, cfg, tag, version, environ["GH_TOKEN"])
  updates = plan(version, facts, cfg)
  dry = environ.get("DRY_RUN") == "true"
  if stays := stable_stays(version, facts, cfg):
    print(stays)
  if not updates:
    print(f"Nothing to do: the branches are already where {tag} puts them.")
  for u in updates:
    print(f"{'would ' if dry else ''}{'create' if u.old is None else 'move'} {u.branch} {f'{u.old[:8]} -> ' if u.old else 'at '}{u.sha[:8]}")
  if updates and not dry:
    # one atomic push: all or nothing. Each lease names what the branch must still be at, so a move made since the job looked
    # (or a version branch someone created meanwhile) fails the push; the lease is also what lets bp-stable make a
    # non-fast-forward move (after a patch, the next version on bp-dev isn't a descendant of it).
    git("push", "-q", "--no-verify", "--atomic", *[f"--force-with-lease=refs/heads/{u.branch}:{u.old or ''}" for u in updates],
        push_url(environ["BP_TOKEN"], REPO), *[f"{u.sha}:refs/heads/{u.branch}" for u in updates])
  return 0


def main() -> int:
  try:
    return release(os.environ, Git("."))
  except Refused as e:
    print(f"::error::{e}")
    return 1


if __name__ == "__main__":
  sys.exit(main())
