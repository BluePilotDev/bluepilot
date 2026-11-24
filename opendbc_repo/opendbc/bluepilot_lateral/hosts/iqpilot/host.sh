#!/usr/bin/env bash
# BluePilot: IQPilot as a test host for this library (#18). Its opendbc, iqdbc, comes from IQ.Lvbs's own repo at the
# commit pinned below, with IQPilot's hooks (hooks.patch, the files they change outside the library) and this
# library at iqdbc/bluepilot_lateral/.
#   host.sh tree <dir> [ref]  <dir>/iq: the IQPilot folders iqdbc's code imports; <dir>/host: iqdbc on its own, a git
#                             repo with IQPilot as committed (first commit) and the hooks plus this library, placed by
#                             tools/vendor.sh (second). Writes <dir>/state: "not adopted" (the patch applied), "adopted"
#                             (IQPilot already carries it, or there's no patch), or "drifted: ..." (and fails): the hooks
#                             or IQPilot's copy of the library differ from what this repo has
#   host.sh test <dir>        build <dir>/host and run the library tests, IQPilot's safety suite with the BluePilot
#                             safety tests, and 100% line coverage of modes/ford.h and the library's safety headers
#   host.sh make <dir>        write hooks.patch from <dir>/host: every change outside the library since IQPilot's
#                             commit, committed or not. Change a hook: tree, edit <dir>/host, test, make.
set -euo pipefail

REPO=https://gitlvb.teallvbs.xyz/IQ.Lvbs/IQ.Pilot.git
PIN=2225bcf9f4793d2d6c0deca0f42af7ce2dd37aa0  # beta, 2026-11-13
PROJECT=artifacts/package_sources/iqdbc        # iqdbc's own project: setup.sh, pyproject.toml, uv.lock
IMPORTED="iqpilot/common iqpilot/cereal iqpilot/system iqpilot/selfdrive"  # imported by iqdbc's code and tests
EXTRA_PACKAGES="pyserial setproctitle"          # imported too, but only in IQPilot's top-level lock

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB="$(cd "$HERE/../.." && pwd)"
DIR="$(mkdir -p "$2" && cd "$2" && pwd)"

case "$1" in
tree)
  REF=${3:-$PIN}
  rm -rf "$DIR/iq" "$DIR/host"
  GIT_LFS_SKIP_SMUDGE=1 git clone -q --filter=blob:none --no-checkout "$REPO" "$DIR/iq"
  # shellcheck disable=SC2086 # a list of paths
  git -C "$DIR/iq" sparse-checkout set $PROJECT $IMPORTED
  GIT_LFS_SKIP_SMUDGE=1 git -C "$DIR/iq" checkout -q "$REF"
  # iqdbc on its own: under IQPilot's root, uv takes it for a member of IQPilot's workspace
  cp -R "$DIR/iq/$PROJECT" "$DIR/host"
  cd "$DIR/host"
  git init -q
  git add -A
  git -c user.name=ci -c user.email=ci@localhost commit -q -m "IQPilot $(git -C "$DIR/iq" rev-parse --short HEAD)"
  state="adopted"
  if [ -f "$HERE/hooks.patch" ]; then
    if git apply --check "$HERE/hooks.patch" 2>/dev/null; then
      git apply "$HERE/hooks.patch"
      state="not adopted"
    elif ! git apply --check --reverse "$HERE/hooks.patch" 2>/dev/null; then
      state="drifted: hooks.patch neither applies nor is already in IQPilot (update it to what IQPilot carries: make)"
    fi
  fi
  # IQPilot's own copy, once it carries one, has to be a release as vendored, before this library replaces it
  if [ -f iqdbc/bluepilot_lateral/VERSION ] && ! "$LIB/tools/check_vendored.sh" iqdbc/bluepilot_lateral; then
    state="drifted: IQPilot's copy of the library differs from the release in its VERSION"
  fi
  "$LIB/tools/vendor.sh" HEAD iqdbc/bluepilot_lateral
  git add -A
  git -c user.name=ci -c user.email=ci@localhost commit -q -m "hooks and library"
  echo "$state" > "$DIR/state"
  echo "IQPilot $(git -C "$DIR/iq" rev-parse --short HEAD), hooks: $state, library $(git -C "$LIB" rev-parse --short HEAD)"
  [[ "$state" != drifted* ]]
  ;;
test)
  cd "$DIR/host"
  # IQPilot's setup.sh installs mull from Cloudsmith, which has answered 402 since 2026-11-02; the same package from
  # mull's GitHub release (what setup.sh uses on macOS). With mull-runner-18 present, setup.sh skips its install.
  if [ "$(uname -s)" = Linux ] && ! command -v mull-runner-18 >/dev/null 2>&1; then
    deb="Mull-18-0.26.1-LLVM-18.1-ubuntu-$(uname -m)-24.04.deb"
    curl -sSL -o "$DIR/$deb" "https://github.com/mull-project/mull/releases/download/0.26.1/$deb"
    sudo apt-get update -qq && sudo apt-get install -y -qq clang-18 "$DIR/$deb"
    rm -f "$DIR/$deb"
  fi
  set +u
  # shellcheck disable=SC1091 # IQPilot's: tools (clang-18 and mull on Linux, for the coverage build) and the venv
  source setup.sh
  set -u
  export PYTHONPATH="$DIR/host:$DIR/iq"  # setup.sh sets it to iqdbc alone; its code imports iqpilot too
  # shellcheck disable=SC2086 # a list of packages
  uv pip install -q $EXTRA_PACKAGES
  scons -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu)"
  python -m unittest discover -s iqdbc/bluepilot_lateral/tests -t .
  # IQPilot's safety suite as its test.sh runs it (pytest: unittest also collects its base classes), the BluePilot
  # tests in the same run (its cross-mode TX sweep reads every Test* class it collects), minus IQPilot's own failures
  deselect=()
  # pytest names a test from its rootdir (iqdbc's project), not from the directory it runs in
  while read -r t; do deselect+=("--deselect=iqdbc/safety/tests/$t"); done < <(grep -v -e '^#' -e '^$' "$HERE/known_failures.txt")
  cd iqdbc/safety/tests
  rm -f libsafety/*.gcda
  pytest -q -n8 --ignore-glob='misra/*' "${deselect[@]}" . ../../bluepilot_lateral/safety/tests
  if [ "$(uname)" = "Darwin" ]; then GCOV_EXEC="/opt/homebrew/opt/llvm@18/bin/llvm-cov gcov"; else GCOV_EXEC="llvm-cov-18 gcov"; fi
  # two reports: modes/ford.h and the library's safety/ford.h share a file name (see bluepilot_coverage.sh)
  gcovr -r .. --gcov-executable "$GCOV_EXEC" --fail-under-line=100 -f "$DIR/host/iqdbc/safety/modes/ford.h"
  gcovr -r "$DIR/host" --gcov-executable "$GCOV_EXEC" -d --fail-under-line=100 -f "$DIR/host/iqdbc/bluepilot_lateral/safety/"
  ;;
make)
  git -C "$DIR/host" diff HEAD~1 -- . ':(exclude)iqdbc/bluepilot_lateral' > "$HERE/hooks.patch"
  echo "hooks.patch: $(grep -c '^diff --git' "$HERE/hooks.patch") files"
  ;;
*)
  echo "usage: $0 tree|test|make <dir> [ref]" >&2
  exit 2
  ;;
esac
