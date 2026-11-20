#!/usr/bin/env python3
"""BluePilot: fail when an upstream file differs from bp-sunnypilot without being on the hook list (#224).

"Upstream" is this repo's bp-sunnypilot branch (ford-op/bp-sunnypilot's integration branch, mirrored).
Compares the end state (a PR's merge result in CI), not the files a PR touched, so a sync PR passes as
long as the work branch differs from bp-sunnypilot only by hooks. Same design as ford-op/opendbc's check.
usage: bluepilot_hooks.py <bp-sunnypilot ref> [<merge result ref>, default HEAD]"""
import subprocess
import sys

HOOK_LIST = ".github/bluepilot-hooks.txt"


def git(*args: str) -> str:
  return subprocess.run(["git", *args], check=True, capture_output=True, text=True).stdout


def listed_hooks(text: str) -> set[str]:
  return {line.split("#", 1)[0].strip() for line in text.splitlines()} - {""}


def main() -> int:
  upstream, head = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else "HEAD")
  # the bp-sunnypilot commit the merge result contains: public-sync moves the mirror ahead of the work
  # branch until the next sync is merged, and upstream's new changes must not count against other PRs meanwhile
  base = git("merge-base", upstream, head).strip()
  # --no-renames: a moved upstream file shows as its deleted path; T: a type change (e.g. to a symlink);
  # a moved submodule pointer (opendbc_repo) shows as M
  changed = set(git("diff", "-z", "--no-renames", "--name-only", "--diff-filter=MDT", base, head).split("\0")) - {""}
  listed = listed_hooks(git("show", f"{head}:{HOOK_LIST}"))

  for path in sorted(listed - changed):
    print(f"::warning file={HOOK_LIST}::{path} isn't an upstream file that differs from bp-sunnypilot: remove it from the hook list")
  unlisted = sorted(changed - listed)
  fix = "move the change into a BluePilot file (bluepilot/, openpilot/selfdrive/ui/bp/), or add the file to the list with its reason"
  for path in unlisted:
    print(f"::error file={path}::{path} differs from bp-sunnypilot but isn't on the hook list ({HOOK_LIST}): {fix}")
  if not unlisted:
    print(f"ok: {len(changed)} upstream files differ from bp-sunnypilot, all on the hook list")
  return 1 if unlisted else 0


if __name__ == "__main__":
  sys.exit(main())
