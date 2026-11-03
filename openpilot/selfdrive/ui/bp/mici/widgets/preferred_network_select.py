"""
BluePilot: MICI preferred WiFi network selector.

Uses the same horizontal NavScroller + BigButton pattern as the MICI WiFi network panel
(WifiUIMici) for consistent UX. Tap a network to set it as preferred, or "None" to clear.
"""

from collections.abc import Callable

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigButtonBP
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.wifi_manager import WifiManager, Network
from openpilot.system.ui.widgets.scroller import NavScroller


def _display_ssid(ssid: str) -> str:
  """Normalize SSID for display; fallback for empty."""
  s = (ssid or "").strip()
  if not s:
    return "Hidden Network"
  return "".join(c for c in s if c.isprintable() or c in " \t") or "Hidden Network"


class PreferredNetworkSelectMici(NavScroller):
  """
  Horizontal scrolling panel of saved networks (same pattern as WifiUIMici).
  Tap a network to set it as preferred; tap "None" to clear.
  """

  def __init__(self, wifi_manager: WifiManager, saved_networks: list[Network],
               on_dismiss: Callable[[], None] | None = None):
    super().__init__()
    self.set_back_callback(self._on_back)
    self._params = Params()
    self._wifi_manager = wifi_manager
    self._saved_networks = saved_networks
    self._on_dismiss = on_dismiss

    # "None" first, then saved networks
    none_btn = BigButtonBP(tr("None"), "", "icons_mici/settings/network/wifi_strength_full.png")
    none_btn.set_click_callback(lambda: self._select(""))
    self._scroller.add_widget(none_btn)

    for network in saved_networks:
      btn = BigButtonBP(
        _display_ssid(network.ssid),
        "",
        "icons_mici/settings/network/wifi_strength_full.png"
      )
      btn.set_click_callback(lambda ssid=network.ssid: self._select(ssid))
      self._scroller.add_widget(btn)

  def _select(self, ssid: str):
    """Save selection and dismiss."""
    self._params.put("WifiFavoriteSSID", ssid)
    if ssid:
      cloudlog.info(f"Set preferred network: {ssid}")
    else:
      cloudlog.info("Cleared preferred network")
    gui_app.pop_widget()
    if self._on_dismiss:
      self._on_dismiss()

  def _on_back(self):
    """Swipe to dismiss without changing selection."""
    gui_app.pop_widget()
    if self._on_dismiss:
      self._on_dismiss()

  def show_event(self):
    super().show_event()
    self._wifi_manager.set_active(True)

  def hide_event(self):
    super().hide_event()
    self._wifi_manager.set_active(False)
