"""BluePilot MICI settings — master layout with category sub-panels."""

from collections.abc import Callable

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import (
  BigButtonBP, BigParamControlBP, BigMultiParamToggleBP,
)
from openpilot.selfdrive.ui.bp.mici.widgets.web_server_qr_dialog import WebServerQRDialog
from openpilot.selfdrive.ui.bp.mici.widgets.preferred_network_select import PreferredNetworkSelectMici
from openpilot.selfdrive.ui.bp.mici.widgets.connect_backend_select import ConnectBackendSelectMici
from bluepilot.backend_switch import BACKEND_LABELS, BACKENDS
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.wifi_manager import WifiManager, Network
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.bp.mici.layouts.settings.vehicle_mici import VehicleLayoutMici
from openpilot.selfdrive.ui.bp.mici.layouts.settings.audio_mici import AudioLayoutMici
from openpilot.selfdrive.ui.bp.mici.layouts.settings.visuals_mici import VisualsLayoutMici
from openpilot.selfdrive.ui.bp.mici.layouts.settings.longitudinal_mici import LongitudinalLayoutMici
from openpilot.selfdrive.ui.bp.mici.layouts.settings.lateral_mici import LateralLayoutMici


class BluePilotBigButton(BigButtonBP):
  """Category button with larger label font matching MICI SettingsLayout."""

  def _get_label_font_size(self):
    return 64


class BluePilotLayoutMici(NavScroller):
  """Top-level BluePilot settings: System items inline, five sub-panels for the rest."""

  def __init__(self, back_callback: Callable[[], None]):
    super().__init__()
    self.set_back_callback(back_callback)
    self._params = Params()

    # WiFi manager for preferred network selector
    self._wifi_manager = WifiManager()
    self._wifi_manager.set_active(False)
    self._saved_networks: list[Network] = []
    self._wifi_manager.add_callbacks(networks_updated=self._on_network_updated)

    # System inline items
    self.enable_web_routes = BigParamControlBP("web routes server", "EnableWebRoutesServer")
    self.preferred_network_btn = BigButtonBP(
      tr("pref. wifi net"), "", "icons_mici/settings/network/wifi_strength_full.png", icon_size=80,
    )
    self.preferred_network_btn.set_click_callback(self._select_preferred_network)
    self.show_web_routes_qr = BigButtonBP(
      "QR code", "", "icons_mici/settings/network/wifi_strength_full.png", icon_size=80,
    )
    self.show_web_routes_qr.set_click_callback(self._show_qr_dialog)
    self.clear_model_cache = BigButtonBP(
      "clear crashed model", "", "icons_mici/settings/device/reboot.png", icon_size=80,
    )
    self.clear_model_cache.set_click_callback(self._clear_model_cache)
    self.ui_debug_log = BigParamControlBP("ui debug logging", "BPUIDebugLog")
    self.connect_backend_btn = BigButtonBP(
      tr("connect backend"), "", "icons_mici/settings/device/reboot.png", icon_size=80,
    )
    self.connect_backend_btn.set_click_callback(self._select_connect_backend)
    self.restore_dongle_btn = BigButtonBP(
      tr("restore dongle id"), "", "icons_mici/settings/device/reboot.png", icon_size=80,
    )
    self.restore_dongle_btn.set_click_callback(self._restore_dongle_id)
    self.restore_dongle_btn.set_enabled(self._has_recoverable_dongle_id)

    # Primary lateral control selector lives above the lat sub-panel
    self.primary_lateral_control = BigMultiParamToggleBP(
      "Primary Control Variable", "FordPrefLateralControl", ["curvature", "angle"],
    )

    # Sub-panels
    vehicle_panel = VehicleLayoutMici()
    vehicle_btn = BluePilotBigButton(
      tr("vehicle"), "", "icons_mici/settings/device_icon.png", icon_size=80,
    )
    vehicle_btn.set_click_callback(lambda: gui_app.push_widget(vehicle_panel))

    audio_panel = AudioLayoutMici()
    audio_btn = BluePilotBigButton(
      tr("audio"), "", "icons_mici/microphone.png", icon_size=80,
    )
    audio_btn.set_click_callback(lambda: gui_app.push_widget(audio_panel))

    visuals_panel = VisualsLayoutMici()
    visuals_btn = BluePilotBigButton(
      tr("visuals"), "", "icons_mici/settings.png", icon_size=80,
    )
    visuals_btn.set_click_callback(lambda: gui_app.push_widget(visuals_panel))

    long_panel = LongitudinalLayoutMici()
    long_btn = BluePilotBigButton(
      tr("long. tuning"), "", "icons_mici/settings/developer_icon.png", icon_size=80,
    )
    long_btn.set_click_callback(lambda: gui_app.push_widget(long_panel))

    lat_panel = LateralLayoutMici()
    lat_btn = BluePilotBigButton(
      tr("lat. tuning"), "", "icons_mici/settings/developer_icon.png", icon_size=80,
    )
    lat_btn.set_click_callback(lambda: gui_app.push_widget(lat_panel))

    self._scroller.add_widgets([
      self.enable_web_routes,
      self.show_web_routes_qr,
      self.preferred_network_btn,
      vehicle_btn,
      audio_btn,
      visuals_btn,
      self.primary_lateral_control,
      lat_btn,
      long_btn,
      self.ui_debug_log,
      self.clear_model_cache,
      self.connect_backend_btn,
      self.restore_dongle_btn,
    ])

    self._refresh_toggles = (
      ("EnableWebRoutesServer", self.enable_web_routes),
      ("BPUIDebugLog", self.ui_debug_log),
    )

    ui_state.add_offroad_transition_callback(self._update_toggles)

  # --- Lifecycle ---

  def show_event(self):
    super().show_event()
    self._update_toggles()
    self._wifi_manager.set_active(True)
    self.preferred_network_btn.set_value(self._get_preferred_network_display())
    self.connect_backend_btn.set_value(self._get_connect_backend_display())
    self.restore_dongle_btn.set_value(self._get_recoverable_dongle_id_preview())

  def hide_event(self):
    super().hide_event()
    self._wifi_manager.set_active(False)

  def _render(self, rect):
    self._wifi_manager.process_callbacks()
    self._scroller.render(rect)

  # --- WiFi helpers ---

  def _on_network_updated(self, networks: list[Network]):
    self._saved_networks = [n for n in networks if self._wifi_manager.is_connection_saved(n.ssid)]
    self.preferred_network_btn.set_enabled(len(self._saved_networks) > 0)
    self.preferred_network_btn.set_value(self._get_preferred_network_display())

    try:
      favorite_value = self._params.get("WifiFavoriteSSID")
      current_favorite = ""
      if favorite_value:
        if isinstance(favorite_value, bytes):
          current_favorite = favorite_value.decode("utf-8", errors="replace").strip("\x00")
        else:
          current_favorite = str(favorite_value).strip("\x00")
      if current_favorite:
        saved_connections = self._wifi_manager._connections  # no public accessor on WifiManager
        if current_favorite not in saved_connections:
          self._params.put("WifiFavoriteSSID", "")
          cloudlog.info(f"Cleared preferred network '{current_favorite}' - network no longer saved")
    except Exception as e:
      cloudlog.debug(f"Error checking preferred network: {e}")

  def _get_preferred_network_display(self) -> str:
    try:
      favorite_value = self._params.get("WifiFavoriteSSID")
      if favorite_value:
        if isinstance(favorite_value, bytes):
          favorite_ssid = favorite_value.decode("utf-8", errors="replace").strip("\x00")
        else:
          favorite_ssid = str(favorite_value).strip("\x00")
        if favorite_ssid:
          return favorite_ssid[:17] + "..." if len(favorite_ssid) > 20 else favorite_ssid
    except Exception:
      pass
    return tr("None")

  def _select_preferred_network(self):
    if not self._saved_networks:
      return
    panel = PreferredNetworkSelectMici(
      self._wifi_manager,
      self._saved_networks,
      on_dismiss=lambda: self.preferred_network_btn.set_value(self._get_preferred_network_display()),
    )
    gui_app.push_widget(panel)

  # --- Actions ---

  def _clear_model_cache(self):
    def do_clear():
      for key in ("ModelRunnerTypeCache", "ModelManager_ActiveBundle"):
        try:
          self._params.remove(key)
        except Exception:
          pass
      self._params.put_bool("DoReboot", True, block=False)
      cloudlog.info("BluePilot: Cleared model cache, triggered reboot")

    icon = gui_app.texture("icons_mici/settings/device/reboot.png", 64, 64)
    dialog = BigConfirmationDialog(
      tr("clear model runner cache and reboot?"),
      icon,
      confirm_callback=do_clear,
    )
    gui_app.push_widget(dialog)

  def _get_connect_backend_display(self) -> str:
    try:
      raw = self._params.get("BPConnectBackend")
      idx = int(raw) if raw not in (None, "") else 0
      if 0 <= idx < len(BACKENDS):
        return BACKEND_LABELS[BACKENDS[idx]]
    except Exception:
      pass
    return BACKEND_LABELS[BACKENDS[0]]

  def _select_connect_backend(self):
    prev_idx = 0
    try:
      raw = self._params.get("BPConnectBackend")
      prev_idx = int(raw) if raw not in (None, "") else 0
    except (TypeError, ValueError):
      prev_idx = 0

    def on_selected(idx: int):
      self.connect_backend_btn.set_value(self._get_connect_backend_display())
      if idx == prev_idx:
        return
      # Backend switch takes effect at launch, so offer a reboot right away.
      icon = gui_app.texture("icons_mici/settings/device/reboot.png", 64, 64)
      dialog = BigConfirmationDialog(
        tr("reboot to switch servers?"),
        icon,
        confirm_callback=lambda: self._params.put_bool("DoReboot", True, block=False),
      )
      gui_app.push_widget(dialog)

    panel = ConnectBackendSelectMici(
      on_selected=on_selected,
      on_dismiss=lambda: self.connect_backend_btn.set_value(self._get_connect_backend_display()),
    )
    gui_app.push_widget(panel)

  def _get_recoverable_dongle_id(self) -> str | None:
    try:
      from bluepilot.backend_switch import find_recoverable_dongle_id
      return find_recoverable_dongle_id(self._params)
    except Exception:
      return None

  def _has_recoverable_dongle_id(self) -> bool:
    return self._get_recoverable_dongle_id() is not None

  def _get_recoverable_dongle_id_preview(self) -> str:
    candidate = self._get_recoverable_dongle_id()
    if candidate is None:
      return ""
    return f"{candidate[:6]}…{candidate[-4:]}" if len(candidate) > 12 else candidate

  def _restore_dongle_id(self):
    candidate = self._get_recoverable_dongle_id()
    if candidate is None:
      return

    def do_restore():
      try:
        from bluepilot.backend_switch import restore_cached_dongle_id
        restore_cached_dongle_id(self._params, candidate)
      except Exception:
        cloudlog.exception("bp_dongle_id_recovery_ui failed")
        return
      self.restore_dongle_btn.set_value(self._get_recoverable_dongle_id_preview())
      self.restore_dongle_btn.set_enabled(self._has_recoverable_dongle_id)
      icon = gui_app.texture("icons_mici/settings/device/reboot.png", 64, 64)
      dialog = BigConfirmationDialog(
        tr("dongle id restored. reboot to apply?"),
        icon,
        confirm_callback=lambda: self._params.put_bool("DoReboot", True, block=False),
      )
      gui_app.push_widget(dialog)

    preview = self._get_recoverable_dongle_id_preview()
    icon = gui_app.texture("icons_mici/settings/device/reboot.png", 64, 64)
    dialog = BigConfirmationDialog(
      tr("restore cached dongle id {preview}?").format(preview=preview),
      icon,
      confirm_callback=do_restore,
    )
    gui_app.push_widget(dialog)

  def _show_qr_dialog(self):
    if not self._params.get_bool("EnableWebRoutesServer"):
      return
    try:
      qr_dialog = WebServerQRDialog(back_callback=gui_app.pop_widget)
      gui_app.push_widget(qr_dialog)
    except Exception as e:
      cloudlog.warning(f"Failed to show QR dialog: {e}")

  # --- State sync ---

  def _update_buttons(self):
    server_enabled = ui_state.params.get_bool("EnableWebRoutesServer")
    self.show_web_routes_qr.set_enabled(server_enabled)
    self.preferred_network_btn.set_enabled(len(self._saved_networks) > 0)
    self.preferred_network_btn.set_value(self._get_preferred_network_display())

  def _update_toggles(self):
    ui_state.update_params()
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))
    self.connect_backend_btn.set_value(self._get_connect_backend_display())
    self.restore_dongle_btn.set_value(self._get_recoverable_dongle_id_preview())
    self.restore_dongle_btn.set_enabled(self._has_recoverable_dongle_id)
    self._update_buttons()
