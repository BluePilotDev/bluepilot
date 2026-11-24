#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source "$DIR/launch_env.sh"

# BluePilot TRON test branch: verbose boot. Everything this script (and the build/manager) prints is
# tee'd into /data/bp_boot.log, and bp_bootlog.py shows its tail on screen until another UI needs the
# display (AGNOS updater, build spinner, manager's ui). If the build or manager dies, it comes back.
BP_BOOTLOG=/data/bp_boot.log
[ -f "$BP_BOOTLOG" ] && mv -f "$BP_BOOTLOG" "$BP_BOOTLOG.prev"
exec > >(tee -a "$BP_BOOTLOG") 2>&1
export PS4='+ $(date +%T) '
set -x
BP_BOOTLOG_PID=""
function bootlog_start {
  if [ -f /AGNOS ] && [ -z "$BP_BOOTLOG_PID" ]; then
    python3 "$DIR/openpilot/system/ui/bp_bootlog.py" "$BP_BOOTLOG" > /dev/null 2>&1 &
    BP_BOOTLOG_PID=$!
  fi
}
function bootlog_stop {
  if [ -n "$BP_BOOTLOG_PID" ]; then
    kill "$BP_BOOTLOG_PID" 2>/dev/null
    wait "$BP_BOOTLOG_PID" 2>/dev/null
    BP_BOOTLOG_PID=""
  fi
}
echo "BluePilot boot $(date) $(cat /VERSION 2>/dev/null) $(git -C "$DIR" log --oneline -1 2>/dev/null)"
bootlog_start
# End BluePilot

# BluePilot: set boot and logo images (non-interactive, safe to call repeatedly)
"$DIR/scripts/boot_logo.sh" --headless --update --force --quiet

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta
  rm -f /data/scons_cache/config.lock

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # Check if AGNOS update is required
  if [ "$(< /VERSION)" != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/openpilot/common/hardware/comma/agnos.py"
    MANIFEST="$DIR/openpilot/system/hardware/comma/agnos.json"
    if "$AGNOS_PY" --verify "$MANIFEST"; then
      sudo reboot
    fi
    bootlog_stop  # BluePilot: updater needs the display

    # BluePilot TRON test branch: flash.comma.ai now ships AGNOS 20.x, where the "magic" service owns
    # the display (/tmp/drmfd.sock). Our 19.9-era raylib UIs (updater included) can't draw there, so the
    # UI updater never shows and the device sits on magic's comma logo forever. Downgrade headless
    # instead: 19.9 vs 20.6 differ only in boot + system (bootloaders identical), same as the
    # documented headless agnos.py --swap recipe. Invisible (~5-15 min), then reboots into 19.9.
    if [ -S /tmp/drmfd.sock ] || [ -f /usr/comma/magic.py ]; then
      echo "BluePilot: AGNOS $(< /VERSION) has magic display; headless swap to $AGNOS_VERSION"
      until "$AGNOS_PY" --swap "$MANIFEST"; do
        echo "BluePilot: headless AGNOS swap failed, retrying in 10s"
        sleep 10
      done
      sudo reboot
      sleep 60
    fi
    # End BluePilot
    while true; do
      "$DIR/openpilot/common/hardware/comma/updater" "$AGNOS_PY" "$MANIFEST"
    done
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f "$DIR/.git/index.lock"

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The DIR init file has to exist, with a newer modtime than anything in
  #    the DIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${DIR}/.overlay_init" ]; then
    find "${DIR}/.git" -newer "${DIR}/.overlay_init" | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${DIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv "$DIR" /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" "$DIR"
          cd "$DIR"
          # releases before the openpilot/ move can't swap AGNOS on newer AGNOS, use ours
          [ ! -L system/hardware/tici/agnos.json ] && /data/safe_staging/old_openpilot/openpilot/common/hardware/comma/agnos.py --verify system/hardware/tici/agnos.json && sudo reboot

          echo "Restarting launch script ${LAUNCHER_LOCATION}"
          unset AGNOS_VERSION
          bootlog_stop  # BluePilot: the re-exec'd launcher starts its own
          exec "${LAUNCHER_LOCATION}"
        else
          echo "openpilot backup found, not updating"
          # TODO: restore backup? This means the updater didn't start after swapping
        fi
      fi
    fi
  fi

  # handle pythonpath
  ln -sfn "$(pwd)" /data/pythonpath
  export PYTHONPATH="$PWD"

  # submodule package symlinks for PYTHONPATH imports on device.
  # on PC these come from editable installs via pyproject.toml / uv.
  ln -sfn msgq_repo/msgq msgq
  ln -sfn opendbc_repo/opendbc opendbc
  ln -sfn rednose_repo/rednose rednose
  ln -sfn teleoprtc_repo/teleoprtc teleoprtc
  ln -sfn tinygrad_repo/tinygrad tinygrad

  # BluePilot: fill the nested bluepilot_lateral submodule if the installer's
  # non-recursive clone left it empty (see openpilot/sunnypilot/bluepilot_ensure_submodules.sh)
  bash openpilot/sunnypilot/bluepilot_ensure_submodules.sh || true
  # End BluePilot

  # BluePilot TRON test branch: opendbc_repo is a plain folder here. comma's cached-fetch installer
  # leaves the old submodule's .git pointer file inside it; drop it so git doesn't see a nested repo.
  [ -f opendbc_repo/.git ] && rm -f opendbc_repo/.git
  # End BluePilot

  # hardware specific init
  if [ -f /AGNOS ]; then
    agnos_init
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  # start manager
  cd openpilot/system/manager
  if [ ! -f "$DIR/prebuilt" ]; then
    bootlog_stop  # BluePilot: build spinner needs the display
    # BluePilot: verbose build UI (progress + live scons line + failure log). Falls back to the
    # stock build if bp_build.py can't start, so a UI bug can't leave the device unable to build.
    ./bp_build.py || ./build.py
  fi
  bootlog_stop  # BluePilot: prebuilt path skips the block above; ui needs the display
  ./manager.py
  echo "BluePilot: manager.py exited ($?)"
  bootlog_start  # BluePilot: show why

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
