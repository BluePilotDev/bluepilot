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
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/system/hardware/tici/agnos.json"
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST
  fi
}

function fix_egl_adreno {
  # BluePilot: comma's on-screen URL installer points the GPU EGL/GLES libs at the Adreno
  # drivers. ford-op/BluePilot installs by git clone (private repo), which skips that
  # installer, so ldconfig's higher-versioned glvnd/Mesa libs win and magic.service (the
  # AGNOS display server) fails eglGetDisplay -> no UI. Re-point them like the installer does.
  local lib=/usr/lib/aarch64-linux-gnu
  if [ "$(readlink $lib/libEGL.so.1)" != "libEGL.so.1.0.0" ] || \
     [ "$(readlink $lib/libGLESv2.so.2)" != "libGLESv2.so.2.0.0" ]; then
    sudo mount -o remount,rw /
    sudo ln -sf libEGL.so.1.0.0 "$lib/libEGL.so.1"
    sudo ln -sf libGLESv2.so.2.0.0 "$lib/libGLESv2.so.2"
    sudo mount -o remount,ro /
    sudo systemctl restart magic.service 2>/dev/null || true
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The DIR init file has to exist, with a newer modtime than anything in
  #    the DIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${DIR}/.overlay_init" ]; then
    find ${DIR}/.git -newer ${DIR}/.overlay_init | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${DIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv $DIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $DIR
          cd $DIR

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
  ln -sfn $(pwd) /data/pythonpath
  export PYTHONPATH="$PWD"

  # BluePilot: Quickboot's untracked prebuilt marker is only valid for the commit that
  # created it. Source updates can add dependencies or change compiled Params; reusing the
  # marker would skip both uv sync and SCons and can prevent manager from starting. Tracked
  # release prebuilts remain authoritative and are never invalidated here.
  QUICKBOOT_COMMIT_FILE="$DIR/.quickboot_commit"
  if [ -f "$DIR/prebuilt" ] && git -C "$DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1 && \
     ! git -C "$DIR" ls-files --error-unmatch prebuilt >/dev/null 2>&1; then
    CURRENT_COMMIT="$(git -C "$DIR" rev-parse HEAD)"
    QUICKBOOT_COMMIT="$(cat "$QUICKBOOT_COMMIT_FILE" 2>/dev/null || true)"
    if [ "$QUICKBOOT_COMMIT" != "$CURRENT_COMMIT" ]; then
      echo "Quickboot build is stale; rebuilding for the current commit"
      rm -f "$DIR/prebuilt"
    fi
  fi
  # End BluePilot

  # hardware specific init
  if [ -f /AGNOS ]; then
    agnos_init
    fix_egl_adreno
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  # start manager
  cd system/manager
  if [ ! -f $DIR/prebuilt ]; then
    # BluePilot: verbose build UI (progress + live scons line + failure log). Falls back to the
    # stock build if bp_build.py can't start, so a UI bug can't leave the device unable to build.
    ./bp_build.py || ./build.py
  fi
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
