"""BluePilot MICI: System settings panel — WiFi, web routes, QR, model cache, debug logging."""

from collections.abc import Callable

from openpilot.common.params import Params
from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigButtonBP
from openpilot.selfdrive.ui.bp.mici.widgets.preferred_network_select import PreferredNetworkSelectMici
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui import text as text_lib
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.wifi_manager import WifiManager, Network
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog
from openpilot.system.ui.widgets.scroller import NavScroller

from openpilot.selfdrive.ui.bp.mici.widgets.web_server_qr_dialog import WebServerQRDialog


class SystemLayoutMici(NavScroller):
  def __init__(self, back_callback: Callable[[], None] | None = None):
    super().__init__()
    if back_callback is not None:
      self.set_back_callback(back_callback)
    self._params = Params()

    # WiFi manager for preferred network selector
    self._wifi_manager = WifiManager()
    self._wifi_manager.set_active(False)
    self._saved_networks: list[Network] = []
    self._wifi_manager.add_callbacks(networks_updated=self._on_network_updated)

    self.preferred_network_btn = BigButtonBP(
      tr("Preferred WiFi Network"),
      "",
      "icons_mici/settings/network/wifi_strength_full.png",
      icon_size=80,
    )
    self.preferred_network_btn.set_click_callback(self._select_preferred_network)

    self.enable_web_routes = BigButtonBP(
      tr("Enable Web Routes Server"),
      "",
      None,
    )
    self.enable_web_routes.set_click_callback(lambda: self._toggle_bool("EnableWebRoutesServer"))

    self.show_qr_btn = BigButtonBP(
      tr("Show QR Code"),
      "",
      "icons_mici/settings/network/wifi_strength_full.png",
      icon_size=80,
    )
    self.show_qr_btn.set_click_callback(self._show_qr_dialog)

    self.clear_model_btn = BigButtonBP(
      tr("Clear Crashed Model"),
      "",
      "icons_mici/settings/device/reboot.png",
      icon_size=80,
    )
    self.clear_model_btn.set_click_callback(self._clear_model_cache)

    self.ui_debug_log = BigButtonBP(
      tr("UI Debug Logging"),
      "",
      None,
    )
    self.ui_debug_log.set_click_callback(lambda: self._toggle_bool("BPUIDebugLog"))

    self._scroller.add_widgets([
      self.enable_web_routes,
      self.show_qr_btn,
      self.preferred_network_btn,
      self.ui_debug_log,
      self.clear_model_btn,
    ])

    ui_state.add_offroad_transition_callback(self._update_buttons)

  def show_event(self):
    super().show_event()
    self._update_buttons()
    self._wifi_manager.set_active(True)
    self.preferred_network_btn.set_value(self._get_preferred_network_display())

  def hide_event(self):
    super().hide_event()
    self._wifi_manager.set_active(False)

  def _render(self, rect):
    self._wifi_manager.process_callbacks()
    self._scroller.render(rect)

  def _toggle_bool(self, key: str):
    current = self._params.get_bool(key)
    self._params.put_bool(key, not current, block=False)
    self._update_buttons()

  def _update_buttons(self):
    ui_state.update_params()

    server_enabled = self._params.get_bool("EnableWebRoutesServer")
    self.enable_web_routes.set_checked(server_enabled)
    self.show_qr_btn.set_enabled(server_enabled)

    debug_enabled = self._params.get_bool("BPUIDebugLog")
    self.ui_debug_log.set_checked(debug_enabled)

    self.preferred_network_btn.set_enabled(len(self._saved_networks) > 0)
    self.preferred_network_btn.set_value(self._get_preferred_network_display())

  def _show_qr_dialog(self):
    if not self._params.get_bool("EnableWebRoutesServer"):
      return
    try:
      qr_dialog = WebServerQRDialog(back_callback=gui_app.pop_widget)
      gui_app.push_widget(qr_dialog)
    except Exception as e:
      from openpilot.common.swaglog import cloudlog
      cloudlog.warning(f"Failed to show QR dialog: {e}")

  def _clear_model_cache(self):
    def do_clear():
      try:
        self._params.remove("ModelRunnerTypeCache")
      except Exception:
        pass
      try:
        self._params.remove("ModelManager_ActiveBundle")
      except Exception:
        pass
      self._params.put_bool("DoReboot", True, block=False)
      from openpilot.common.swaglog import cloudlog
      cloudlog.info("BluePilot: Cleared model cache, triggered reboot")

    icon = gui_app.texture("icons_mici/settings/device/reboot.png", 64, 64)
    dialog = BigConfirmationDialog(
      tr("Clear crashed model runner cache and reboot?"),
      icon,
      confirm_callback=do_clear,
    )
    gui_app.push_widget(dialog)

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
        saved_connections = self._wifi_manager._connections
        if current_favorite not in saved_connections:
          self._params.put("WifiFavoriteSSID", "")
    except Exception:
      pass

  def _get_preferred_network_display(self) -> str:
    try:
      favorite_value = self._params.get("WifiFavoriteSSID")
      if favorite_value:
        if isinstance(favorite_value, bytes):
          favorite_ssid = favorite_value.decode("utf-8", errors="replace").strip("\x00")
        else:
          favorite_ssid = str(favorite_value).strip("\x00")
        if favorite_ssid:
          if len(favorite_ssid) > 20:
            return favorite_ssid[:17] + "..."
          return favorite_ssid
    except Exception:
      pass
    return tr("None")

  def _select_preferred_network(self):
    if len(self._saved_networks) == 0:
      return
    panel = PreferredNetworkSelectMici(
      self._wifi_manager,
      self._saved_networks,
      on_dismiss=lambda: self.preferred_network_btn.set_value(self._get_preferred_network_display()),
    )
    gui_app.push_widget(panel)
