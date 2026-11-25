#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source "$DIR/launch_env.sh"

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
    # BluePilot: `sudo reboot` only queues the reboot and returns. Ask until it is queued, then replace the shell, so no
    # line of this script runs after it (the loop below would start the flasher again during the shutdown, and a
    # second agnos.py --swap marks the slot just switched to unbootable)
    if "$AGNOS_PY" --verify "$MANIFEST"; then
      until sudo reboot; do sleep 5; done
      exec sleep infinity
    fi
    while true; do
      # BluePilot: the bundled updater is only a screen around `agnos.py --swap`. Where it can't start (its bundled
      # UI code needs pyray's raygui, which a newer AGNOS's pyray lacks), flash without the screen: agnos.py writes the
      # inactive slot, checks it by hash and swaps. On a failure the loop goes round again, as before; after the reboot
      # request nothing of this script runs (as above).
      if ! "$DIR/openpilot/common/hardware/comma/updater" "$AGNOS_PY" "$MANIFEST"; then
        if "$AGNOS_PY" --swap "$MANIFEST"; then
          until sudo reboot; do sleep 5; done
          exec sleep infinity
        fi
      fi
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

  # BluePilot: JetLink (jetlink_repo), and with the link on, cut the comma's draw from the USB-C port from boot
  # (its owner, jetlinkd, starts ~20 s later): the comma runs from the car harness, never the host's port.
  # "off" is a missing, empty or 0 JetlinkLink, and then the root script is never run
  ln -sfn jetlink_repo/jetlink jetlink
  if [ -f /AGNOS ] && [ -f jetlink/comma/jetlink-root.sh ] && [ -s /data/params/d/JetlinkLink ] && \
     [ "$(cat /data/params/d/JetlinkLink)" != "0" ]; then
    sudo -n bash jetlink/comma/jetlink-root.sh boot /data/params/d/JetlinkLink &
  fi
  # End BluePilot

  # BluePilot: fill the nested bluepilot_lateral submodule if the installer's
  # non-recursive clone left it empty (see openpilot/sunnypilot/bluepilot_ensure_submodules.sh)
  bash openpilot/sunnypilot/bluepilot_ensure_submodules.sh || true
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
    # BluePilot: verbose build UI (progress + live scons line + failure log). Falls back to the
    # stock build if bp_build.py can't start, so a UI bug can't leave the device unable to build.
    ./bp_build.py || ./build.py
  fi
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
