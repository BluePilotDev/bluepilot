#!/usr/bin/env bash
# BluePilot: print the hook patch, everything a sunnypilot/opendbc checkout needs besides the library
# itself (HOSTS.md, #63): every file outside opendbc/bluepilot_lateral/ that differs from master, minus
# ford-op/opendbc's own .github. Fails unless the changed files are exactly the hook list's non-.github
# entries; the rest are new files (the safety test loader).
# usage, in a ford-op/opendbc checkout: opendbc/bluepilot_lateral/tools/make_hook_patch.sh [base, default master] > hooks.patch
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
BASE=${1:-master}
SPEC=(. ':!opendbc/bluepilot_lateral' ':!.github')

changed=$(git diff --name-only --diff-filter=M "$BASE"...HEAD -- "${SPEC[@]}" | sort)
listed=$(python3 -c 'import sys; sys.path.insert(0, ".github"); from bluepilot_hooks import listed_hooks; print("\n".join(sorted(f for f in listed_hooks(open(".github/bluepilot-hooks.txt").read()) if not f.startswith(".github/"))))')
if [ "$changed" != "$listed" ]; then
  echo "make_hook_patch: the files that differ from $BASE don't match .github/bluepilot-hooks.txt (< listed, > changed):" >&2
  diff <(echo "$listed") <(echo "$changed") >&2 || true
  exit 1
fi
other=$(git diff --name-only --diff-filter=DRCTUX "$BASE"...HEAD -- "${SPEC[@]}")
if [ -n "$other" ]; then
  echo "make_hook_patch: only edits and new files can go in the patch; also changed: $other" >&2
  exit 1
fi
git diff --binary "$BASE"...HEAD -- "${SPEC[@]}"
