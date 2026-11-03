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

echo "bluepilot: nested bluepilot_lateral submodule missing; fetching..."
n=0
while [ "$n" -lt 3 ]; do
  if git submodule update --init --recursive opendbc_repo && [ -f "$LIB" ]; then
    echo "bluepilot: bluepilot_lateral restored"
    exit 0
  fi
  n=$((n + 1))
  sleep 2
done
echo "bluepilot: WARNING could not fetch bluepilot_lateral (offline?); continuing" >&2
exit 0
