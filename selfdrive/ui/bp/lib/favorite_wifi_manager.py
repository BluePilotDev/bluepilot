"""Standalone preferred/favorite WiFi network manager for BluePilot.

Runs a background thread that periodically checks whether the user's
preferred WiFi network is available and auto-connects to it.  All
WiFi operations are delegated to a stock WifiManager instance so that
this file does not modify any upstream code.
"""

import threading
import time

from jeepney import DBusAddress
from jeepney.wrappers import Properties

from openpilot.common.swaglog import cloudlog
from openpilot.system.ui.lib.networkmanager import (
  NM, NM_ACTIVE_CONNECTION_IFACE, NM_ACCESS_POINT_IFACE,
)

try:
  from openpilot.common.params import Params
except Exception:
  Params = None

FAVORITE_CHECK_INTERVAL = 30.0   # seconds between checks
INITIAL_CHECK_DELAY = 5.0        # seconds to wait after start before first check


class FavoriteWifiManager:
  """Monitors the WifiFavoriteSSID param and auto-connects when the
  preferred network is saved in NetworkManager and in range."""

  def __init__(self, wifi_manager):
    self._wm = wifi_manager
    self._exit = False
    self._thread = threading.Thread(target=self._run, daemon=True)
    self._thread.start()

  def stop(self):
    self._exit = True

  # ---- internal ----------------------------------------------------

  def _run(self):
    last_check_time = 0.0
    startup_time = time.monotonic()
    initial_check_done = False

    while not self._exit:
      current_time = time.monotonic()

      # Short delay after startup so networks have time to scan
      if not initial_check_done:
        if current_time - startup_time < INITIAL_CHECK_DELAY:
          time.sleep(1)
          continue
        initial_check_done = True
        last_check_time = 0.0  # force immediate first check

      # Regular interval
      if current_time - last_check_time < FAVORITE_CHECK_INTERVAL:
        time.sleep(1)
        continue

      last_check_time = current_time

      try:
        self._check_once()
      except Exception as e:
        cloudlog.exception(f"Error checking favorite network: {e}")

      time.sleep(1)  # small sleep to prevent tight loop

  def _check_once(self):
    if Params is None:
      return

    favorite_ssid = self._read_favorite_ssid()
    if not favorite_ssid:
      return

    # Verify the favorite is saved in NetworkManager
    saved_connections = self._wm._get_connections()
    if favorite_ssid not in saved_connections:
      cloudlog.warning(
        f"Favorite network '{favorite_ssid}' is not saved in NetworkManager, cannot auto-connect"
      )
      return

    # Determine what we're currently connected to via NetworkManager
    current_ssid = self._get_current_wifi_ssid()

    if current_ssid == favorite_ssid:
      cloudlog.debug(f"Favorite network '{favorite_ssid}' is already connected")
      return

    # Connected to something else — try to switch
    if current_ssid and current_ssid != favorite_ssid:
      favorite_in_scan = False
      with self._wm._lock:
        for network in self._wm._networks:
          if network.ssid == favorite_ssid:
            favorite_in_scan = True
            break

      cloudlog.info(
        f"Connected to '{current_ssid}', switching to favorite "
        f"'{favorite_ssid}' (in scan: {favorite_in_scan})..."
      )
      try:
        self._wm._deactivate_connection(current_ssid)
        time.sleep(2)
        self._wm.activate_connection(favorite_ssid, block=False)
      except Exception as e:
        cloudlog.warning(f"Failed to switch to favorite network '{favorite_ssid}': {e}")

  # ---- helpers -----------------------------------------------------

  @staticmethod
  def _read_favorite_ssid() -> str:
    value = Params().get("WifiFavoriteSSID")
    if not value:
      return ""
    if isinstance(value, bytes):
      return value.decode("utf-8", errors="replace").strip("\x00")
    return str(value).strip("\x00")

  def _get_current_wifi_ssid(self) -> str | None:
    """Ask NetworkManager directly for the currently-connected WiFi SSID."""
    try:
      active_connections = self._wm._get_active_connections()
    except Exception:
      return None

    for conn_path in active_connections:
      try:
        conn_addr = DBusAddress(
          conn_path, bus_name=NM, interface=NM_ACTIVE_CONNECTION_IFACE
        )
        conn_type = (
          self._wm._router_main
          .send_and_get_reply(Properties(conn_addr).get("Type"))
          .body[0][1]
        )
        if conn_type == "802-11-wireless":
          specific_obj = (
            self._wm._router_main
            .send_and_get_reply(Properties(conn_addr).get("SpecificObject"))
            .body[0][1]
          )
          if specific_obj != "/":
            ap_addr = DBusAddress(
              specific_obj, bus_name=NM, interface=NM_ACCESS_POINT_IFACE
            )
            ssid_bytes = (
              self._wm._router_main
              .send_and_get_reply(Properties(ap_addr).get("Ssid"))
              .body[0][1]
            )
            return bytes(ssid_bytes).decode("utf-8", "replace")
      except Exception:
        continue
    return None
