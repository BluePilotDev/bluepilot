#!/usr/bin/env python3
"""BluePilot: fail when an upstream file differs from master without being on the hook list (#47).

Compares the end state (a PR's merge result in CI), not the files a PR touched, so the nightly sync
PR and the PR resolving a sync conflict pass as long as bp-opendbc differs from master only by hooks.
usage: bluepilot_hooks.py <master ref> [<merge result ref>, default HEAD]"""
import subprocess
import sys

HOOK_LIST = ".github/bluepilot-hooks.txt"


def git(*args: str) -> str:
  return subprocess.run(["git", *args], check=True, capture_output=True, text=True).stdout


def listed_hooks(text: str) -> set[str]:
  return {line.split("#", 1)[0].strip() for line in text.splitlines()} - {""}


def main() -> int:
  master, head = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else "HEAD")
  # the master commit the merge result contains: the nightly job moves master ahead of bp-opendbc until
  # its sync PR is merged, and upstream's new changes must not count against other PRs meanwhile
  base = git("merge-base", master, head).strip()
  # --no-renames: a moved upstream file shows as its deleted path; T: a type change (e.g. to a symlink)
  changed = set(git("diff", "-z", "--no-renames", "--name-only", "--diff-filter=MDT", base, head).split("\0")) - {""}
  listed = listed_hooks(git("show", f"{head}:{HOOK_LIST}"))

  for path in sorted(listed - changed):
    print(f"::warning file={HOOK_LIST}::{path} isn't an upstream file that differs from master: remove it from the hook list")
  unlisted = sorted(changed - listed)
  fix = "move the change under opendbc/bluepilot_lateral/, or add the file to the list with its reason"
  for path in unlisted:
    print(f"::error file={path}::{path} differs from master but isn't on the hook list ({HOOK_LIST}): {fix}")
  if not unlisted:
    print(f"ok: {len(changed)} upstream files differ from master, all on the hook list")
  return 1 if unlisted else 0


if __name__ == "__main__":
  sys.exit(main())
