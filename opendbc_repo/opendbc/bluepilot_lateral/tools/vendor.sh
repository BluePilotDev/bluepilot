#!/usr/bin/env bash
# BluePilot: copy a release of this library into a host's folder, for a host that takes no submodules (#24).
# The folder becomes exactly the library at <ref> (files the library doesn't have are removed), minus what
# .gitattributes marks export-ignore (.github/), plus VERSION: "<tag> <commit>" ("untagged" when <ref> isn't one).
# tools/check_vendored.sh tells whether the folder still matches. Never edit inside the folder: change the
# library, release it, vendor the new tag.
# usage, from a clone of this repo: tools/vendor.sh <ref, a release tag> <host folder, e.g. .../iqdbc/bluepilot_lateral>
set -euo pipefail
if [ $# -ne 2 ]; then
  echo "usage: $0 <ref> <folder>" >&2
  exit 2
fi
LIB="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMMIT="$(git -C "$LIB" rev-parse --verify "$1^{commit}")"
TAG="$(git -C "$LIB" describe --tags --exact-match "$COMMIT" 2>/dev/null || echo untagged)"

rm -rf "$2"
mkdir -p "$2"
git -C "$LIB" archive "$COMMIT" | tar -x -C "$2"
echo "$TAG $COMMIT" > "$2/VERSION"
echo "$2: library $TAG ($COMMIT)"
