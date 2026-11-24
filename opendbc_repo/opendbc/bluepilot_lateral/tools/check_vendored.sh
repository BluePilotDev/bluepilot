#!/usr/bin/env bash
# BluePilot: does a host's copy of the library (tools/vendor.sh) still match the release in its VERSION file (#24)?
# Lists every added, removed or changed file and fails if there is one. Ignores VERSION and Python's caches.
# usage, from a clone of this repo that has the commit: tools/check_vendored.sh <host folder>
set -euo pipefail
if [ $# -ne 1 ]; then
  echo "usage: $0 <folder>" >&2
  exit 2
fi
LIB="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
read -r TAG COMMIT < "$1/VERSION"
if ! git -C "$LIB" cat-file -e "$COMMIT^{commit}" 2>/dev/null; then
  echo "$COMMIT ($TAG) isn't in $LIB: fetch it first" >&2
  exit 2
fi
REF="$(mktemp -d)"
trap 'rm -rf "$REF"' EXIT
git -C "$LIB" archive "$COMMIT" | tar -x -C "$REF"

if diff -rq -x VERSION -x __pycache__ -x '*.pyc' "$REF" "$1"; then
  echo "$1 matches library $TAG ($COMMIT)"
else
  echo "$1 differs from library $TAG ($COMMIT): vendor a release instead of editing the copy" >&2
  exit 1
fi
