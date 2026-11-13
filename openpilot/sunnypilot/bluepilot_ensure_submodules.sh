#!/usr/bin/env bash
# BluePilot: self-heal the nested bluepilot_lateral submodule.
#
# comma's installer takes a cached-fetch path on flash.comma.ai images and runs a
# NON-recursive `git submodule update --init`: opendbc_repo is checked out but its
# nested bluepilot_lateral submodule stays empty, and the build fails with
# ModuleNotFoundError: opendbc.bluepilot_lateral.host_types. SSH/dev installs use
# tools/op.sh (--init --recursive) and never hit this. Fill it here, scoped to the
# opendbc path, before the build imports it. Fail-soft: an offline boot is no worse
# than today, which already fails the import.
set -u
cd "$(dirname "$0")/../.." || exit 0  # repo root

LIB="opendbc_repo/opendbc/bluepilot_lateral/__init__.py"
[ -f "$LIB" ] && exit 0  # present (normal case, including every boot after the first heal)

# AGNOS starts openpilot right after the filesystem (comma.service: After=local-fs.target), before
# wifi has associated, so the fetch fails fast on a cold boot: keep trying until the network is up,
# for a few minutes. Nothing starts without the library (manager imports opendbc.car.structs).
# Counted in tries, not by the clock: a cold boot starts with the wall clock behind, so HTTPS fails
# ("certificate is not yet valid") until time sync jumps it forward, which would end a clock deadline
# before the first fetch that could work.
echo "bluepilot: nested bluepilot_lateral submodule missing; fetching (waits for the network)..."
tries=$(( ${BLUEPILOT_HEAL_TIMEOUT:-180} / 5 ))
n=0
while :; do
  if git submodule update --init --recursive opendbc_repo 2>/tmp/bluepilot_heal.err && [ -f "$LIB" ]; then
    echo "bluepilot: bluepilot_lateral restored"
    exit 0
  fi
  n=$((n + 1))
  if [ "$n" -ge "$tries" ]; then break; fi
  sleep 5
done
echo "bluepilot: WARNING could not fetch bluepilot_lateral (offline?): $(tail -n1 /tmp/bluepilot_heal.err 2>/dev/null); continuing" >&2
exit 0
