#!/usr/bin/env bash
# BluePilot: run the library's tests on a sunnypilot/opendbc checkout (HOSTS.md, #63), in a temporary
# clone of it with this repo's library copied in and the hook patch applied. The host checkout itself
# isn't touched. Uses this repo's Python environment; the working copy goes first on PYTHONPATH.
# usage, in a ford-op/opendbc checkout: opendbc/bluepilot_lateral/tools/test_on_host.sh <sunnypilot/opendbc checkout> [base]
set -euo pipefail
if [ $# -lt 1 ]; then
  echo "usage: $0 <sunnypilot/opendbc checkout> [base, default master]" >&2
  exit 2
fi
HOST=$(cd "$1" && pwd)
BASE=${2:-master}
TOOLS=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
LIB=$(cd "$TOOLS/.." && pwd)
REPO=$(cd "$LIB/../.." && pwd)
PY=${PYTHON:-$REPO/.venv/bin/python}
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

git clone --quiet --shared "$HOST" "$WORK/host"
git -C "$WORK/host" checkout --quiet "$(git -C "$HOST" rev-parse HEAD)"
# the library as committed, like the patch (a dirty working tree would test a mix that exists nowhere);
# archived from inside it, so a plain folder, a submodule and a subtree all work
mkdir -p "$WORK/host/opendbc/bluepilot_lateral"
git -C "$LIB" archive HEAD | tar -x -C "$WORK/host/opendbc/bluepilot_lateral"
(cd "$REPO" && "$TOOLS/make_hook_patch.sh" "$BASE") > "$WORK/hooks.patch"
git -C "$WORK/host" apply "$WORK/hooks.patch"
echo "host $(git -C "$HOST" rev-parse --short HEAD) + library + hook patch ($(grep -c '^diff --git' "$WORK/hooks.patch") files)"

cd "$WORK/host"
export PYTHONPATH="$WORK/host"
echo "== library tests"
"$PY" -m unittest discover -s opendbc/bluepilot_lateral/tests -t .
echo "== safety tests, as upstream's test.sh finds them (the BluePilot ones through the loader)"
(cd opendbc/safety/tests && "$PY" -m unittest discover -s .)
