"""
BluePilot: MICI vehicle fingerprint selector (make → model).

Horizontal NavScroller pattern (same as PreferredNetworkSelectMici / WifiUIMici).
Car data from sunnypilot selfdrive/car/car_list.json (same as TICI PlatformSelector).
"""

from __future__ import annotations

import json
import os
from collections.abc import Callable

from openpilot.common.basedir import BASEDIR
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigButtonBP
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller

CAR_LIST_JSON = os.path.join(BASEDIR, "sunnypilot", "selfdrive", "car", "car_list.json")


def load_car_platforms() -> dict:
  with open(CAR_LIST_JSON) as f:
    return json.load(f)


def platform_names_for_make(platforms: dict, make: str) -> list[str]:
  names = [p for p, d in platforms.items() if d.get("make") == make]
  return sorted(names)


def makes_available(platforms: dict) -> list[tuple[str, str]]:
  """
  Every unique ``make`` from car_list.json, sorted — same set TICI uses for TreeFolder
  in PlatformSelector._show_platform_dialog (sorted unique makes, then platforms per make).
  """
  makes = sorted({d.get("make") for d in platforms.values() if d.get("make")})
  return [(m, m) for m in makes]


class VehicleMakeSelectMici(NavScroller):
  """Horizontal list of make buttons; opens model list for the chosen make."""

  def __init__(
    self,
    platforms: dict,
    on_stack_done: Callable[[], None] | None = None,
  ):
    super().__init__()
    self.set_back_callback(self._on_back)
    self._platforms = platforms
    self._on_stack_done = on_stack_done

    for display_label, make_key in makes_available(platforms):
      btn = BigButtonBP(
        display_label.lower(),
        "",
        "../../sunnypilot/selfdrive/assets/offroad/icon_vehicle.png",
      )
      btn.set_click_callback(lambda mk=make_key: self._open_models(mk))
      self._scroller.add_widget(btn)

  def _open_models(self, make_key: str):
    names = platform_names_for_make(self._platforms, make_key)
    if not names:
      cloudlog.warning(f"No platforms for make {make_key}")
      return
    panel = VehicleModelSelectMici(
      self._platforms,
      names,
      on_vehicle_set=self._after_vehicle_set,
    )
    gui_app.push_widget(panel)

  def _after_vehicle_set(self):
    """User confirmed a vehicle: pop make panel, refresh root screen."""
    gui_app.pop_widget()
    if self._on_stack_done:
      self._on_stack_done()

  def _on_back(self):
    gui_app.pop_widget()
    if self._on_stack_done:
      self._on_stack_done()


class VehicleModelSelectMici(NavScroller):
  """Horizontal list of vehicle (platform) names; confirm then write CarPlatformBundle."""

  def __init__(
    self,
    platforms: dict,
    platform_names: list[str],
    on_vehicle_set: Callable[[], None] | None = None,
  ):
    super().__init__()
    self.set_back_callback(self._on_back)
    self._platforms = platforms
    self._on_vehicle_set = on_vehicle_set

    for name in platform_names:
      display = name.lower()
      if len(display) > 42:
        display = display[:39] + "..."
      # scroll=False: TICI ConfirmDialog uses margins that break Label wrapping on narrow MICI screens
      # (near-zero width → one character per line, "eff" column from "...effect...").
      btn = BigButtonBP(display, "", "../../sunnypilot/selfdrive/assets/offroad/icon_vehicle.png", scroll=False)
      btn.set_click_callback(lambda n=name: self._ask_confirm(n))
      self._scroller.add_widget(btn)

  def _ask_confirm(self, platform_name: str):
    # MICI: use swipe slider (BigConfirmationDialog), not TICI ConfirmDialog (200px side margins).
    def on_confirmed():
      self._apply_vehicle(platform_name)

    title = (
      tr("slide to\napply now")
      if ui_state.is_offroad
      else tr("slide to\napply when offroad")
    )
    dlg = BigConfirmationDialog(
      title,
      "icons_mici/settings/car_icon.png",
      red=False,
      confirm_callback=on_confirmed,
    )
    gui_app.push_widget(dlg)

  def _apply_vehicle(self, platform_name: str):
    data = self._platforms.get(platform_name)
    if not data:
      cloudlog.error(f"Missing car_list entry for {platform_name}")
      return
    ui_state.params.put("CarPlatformBundle", {**data, "name": platform_name})
    cloudlog.info(f"MICI vehicle: set CarPlatformBundle to {platform_name}")
    gui_app.pop_widget()
    if self._on_vehicle_set:
      self._on_vehicle_set()

  def _on_back(self):
    gui_app.pop_widget()
