from __future__ import annotations

import json
import os

from openpilot.common.basedir import BASEDIR
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller

CAR_LIST_JSON = os.path.join(BASEDIR, "sunnypilot", "selfdrive", "car", "car_list.json")
VEHICLE_ICON = "../../sunnypilot/selfdrive/assets/offroad/icon_vehicle.png"


def load_car_platforms() -> dict:
  with open(CAR_LIST_JSON) as f:
    return json.load(f)


def _truncate(text: str, max_len: int = 36) -> str:
  text = text.strip()
  return text if len(text) <= max_len else text[:max_len - 3] + "..."


def _current_platform_name() -> str:
  bundle = ui_state.params.get("CarPlatformBundle")
  if not bundle:
    return ""
  name = bundle.get("name", "") if isinstance(bundle, dict) else ""
  if isinstance(name, bytes):
    name = name.decode("utf-8", errors="replace")
  return str(name).strip()


def _vehicle_status() -> tuple[str, str]:
  if bundle := ui_state.params.get("CarPlatformBundle"):
    name = bundle.get("name", "") if isinstance(bundle, dict) else ""
    if isinstance(name, bytes):
      name = name.decode("utf-8", errors="replace")
    return tr("manual selection"), _truncate(str(name))
  if ui_state.CP is not None and ui_state.CP.carFingerprint != "MOCK":
    return tr("auto fingerprint"), _truncate(str(ui_state.CP.carFingerprint))
  return tr("vehicle"), tr("unrecognized")


class VehicleModelLayoutMici(NavScroller):
  def __init__(self, platforms: dict, platform_names: list[str]):
    super().__init__()
    self._platforms = platforms

    items = []
    for platform_name in platform_names:
      button = BigButton(platform_name, "", gui_app.texture(VEHICLE_ICON, 70, 70), scroll=True)
      button.set_click_callback(lambda name=platform_name: self._confirm_selection(name))
      items.append(button)

    self._scroller.add_widgets(items)

  def _confirm_selection(self, platform_name: str):
    title = tr("slide to\napply now") if ui_state.is_offroad() else tr("slide to\napply when offroad")

    def confirm_callback():
      data = self._platforms.get(platform_name)
      if not data:
        cloudlog.error(f"Missing car_list entry for {platform_name}")
        return
      ui_state.params.put("CarPlatformBundle", {**data, "name": platform_name})
      cloudlog.info(f"MICI vehicle: set CarPlatformBundle to {platform_name}")
      gui_app.pop_widget()
      gui_app.pop_widget()

    gui_app.push_widget(BigConfirmationDialog(title, gui_app.texture("icons_mici/settings/car_icon.png", 64, 64), confirm_callback))


class VehicleMakeLayoutMici(NavScroller):
  def __init__(self, platforms: dict):
    super().__init__()
    self._platforms = platforms

    makes = sorted({data.get("make") for data in platforms.values() if data.get("make")})
    items = []
    for make in makes:
      button = BigButton(make, "", gui_app.texture(VEHICLE_ICON, 70, 70), scroll=True)
      button.set_click_callback(lambda make_name=make: self._open_models(make_name))
      items.append(button)

    self._scroller.add_widgets(items)

  def _open_models(self, make: str):
    platform_names = sorted(name for name, data in self._platforms.items() if data.get("make") == make)
    if platform_names:
      gui_app.push_widget(VehicleModelLayoutMici(self._platforms, platform_names))


class VehicleLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    try:
      self._platforms = load_car_platforms()
    except OSError as e:
      self._platforms = {}
      cloudlog.error(f"MICI vehicle: could not load {CAR_LIST_JSON}: {e}")

    self._btn_current = BigButton(tr("current vehicle"), "", gui_app.texture(VEHICLE_ICON, 70, 70), scroll=True)
    self._btn_clear = BigButton(tr("clear vehicle"), "", gui_app.texture(VEHICLE_ICON, 70, 70))
    self._btn_select = BigButton(tr("select vehicle"), "", gui_app.texture(VEHICLE_ICON, 70, 70))

    self._btn_current.set_enabled(False)
    self._btn_clear.set_click_callback(self._on_clear)
    self._btn_select.set_click_callback(self._on_select)

    self._scroller.add_widgets([self._btn_current, self._btn_clear, self._btn_select])

  def show_event(self):
    super().show_event()
    ui_state.update_params()
    self._refresh_display()

  def _update_state(self):
    super()._update_state()
    self._refresh_display()

  def _refresh_display(self):
    title, value = _vehicle_status()
    self._btn_current.set_text(title)
    self._btn_current.set_value(value)
    self._btn_clear.set_enabled(bool(ui_state.params.get("CarPlatformBundle")))
    self._btn_select.set_enabled(len(self._platforms) > 0)

  def _on_clear(self):
    if ui_state.params.get("CarPlatformBundle"):
      ui_state.params.remove("CarPlatformBundle")
    self._refresh_display()

  def _on_select(self):
    if self._platforms:
      gui_app.push_widget(VehicleMakeLayoutMici(self._platforms))
