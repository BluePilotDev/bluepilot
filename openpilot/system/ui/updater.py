#!/usr/bin/env python3
import os

# BluePilot: the AGNOS update screen runs as a bundle (tools/release/pack.py) on whatever AGNOS the device has. It uses
# comma's own widgets: the sunnypilot skin's widgets import Params, a compiled library a bundle can't load from its zip
os.environ.setdefault("SUNNYPILOT_UI", "0")

from openpilot.system.ui.lib.application import gui_app  # noqa: E402
import openpilot.system.ui.tici_updater as tici_updater  # noqa: E402
import openpilot.system.ui.mici_updater as mici_updater  # noqa: E402


def main():
  if gui_app.big_ui():
    tici_updater.main()
  else:
    mici_updater.main()


if __name__ == "__main__":
  main()
