#!/usr/bin/env bash
# BluePilot: 100% line coverage of the BluePilot safety headers (#53). Upstream's test.sh only measures
# files under <pkg>/safety/, and the headers live in <pkg>/bluepilot_lateral/safety/. <pkg> is the host's
# opendbc package, whatever it's named (opendbc, iqdbc): the one the library is mounted in.
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
ROOT="$(cd "$DIR/../../../.." && pwd)"
PKG="$(basename "$(cd "$DIR/../../.." && pwd)")"
cd "$ROOT/$PKG/safety/tests"

source "$ROOT/setup.sh"

# reset coverage data, then run upstream's safety tests. The BluePilot ones come in through the host's
# loader (test_bluepilot_lateral.py) where it has one; otherwise they run here on their own.
rm -f ./libsafety/*.gcda ./libsafety/*.gcno
SAFETY_COVERAGE=1 python -m unittest discover -s .
if [ ! -f test_bluepilot_lateral.py ]; then
  SAFETY_COVERAGE=1 python -m unittest discover -s "$ROOT/$PKG/bluepilot_lateral/safety/tests" -t "$ROOT"
fi

if [ "$(uname)" = "Darwin" ]; then
  GCOV_EXEC="llvm-cov gcov"
else
  GCOV_EXEC="gcov"
fi

# Two reports: the library's safety/ford.h and the host's modes/ford.h share a file name, and one gcovr run
# over both can credit one's lines to the other. modes/ford.h is measured from <pkg>/safety, as the host's
# test.sh does; the second run deletes the coverage data.
if ! OUT="$(gcovr -r "$ROOT/$PKG/safety" --gcov-executable "$GCOV_EXEC" --fail-under-line=100 \
             -f "$ROOT/$PKG/safety/modes/ford.h")"; then
  echo -e "FAILED:\n$OUT"
  exit 1
fi
echo "$OUT"
if ! OUT="$(gcovr -r "$ROOT" --gcov-executable "$GCOV_EXEC" -d --fail-under-line=100 \
             -f "$ROOT/$PKG/bluepilot_lateral/safety/")"; then
  echo -e "FAILED:\n$OUT"
  exit 1
fi
echo "$OUT"
echo "SUCCESS: the BluePilot safety headers and modes/ford.h have 100% coverage!"
