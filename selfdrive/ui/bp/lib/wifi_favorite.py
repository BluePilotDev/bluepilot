"""
BluePilot: WiFi favorite network auto-connect manager.

Periodically checks if the user's preferred WiFi network (set via WifiFavoriteSSID param)
is in range and auto-connects to it if the device is connected to a different network.
"""

import threading
import time

from jeepney import DBusAddress
from jeepney.wrappers import Properties

from openpilot.common.swaglog import cloudlog
from openpilot.system.ui.lib.networkmanager import (NM, NM_ACTIVE_CONNECTION_IFACE, 
                                                    NM_ACCESS_POINT_IFACE)

try:
  from openpilot.common.params import Params
except Exception:
  Params = None

FAVORITE_CHECK_PERIOD_SECONDS = 30  # Wait 30 seconds between checks
INITIAL_CHECK_DELAY = 30.0  # Wait 30 seconds after startup before first check
SCAN_WAIT_SECONDS = 5.0  # Wait 5 seconds after requesting scan for results to populate
MIN_SIGNAL_STRENGTH = 20  # Minimum signal strength (0-100) required to auto-connect


class WifiFavoriteManager:
  """Monitors for a favorite WiFi network and auto-connects when in range."""

  def __init__(self, wifi_manager):
    self._wifi_manager = wifi_manager
    self._exit = False
    self._thread = threading.Thread(target=self._run, daemon=True)

  def start(self):
    """Start the favorite network monitoring thread."""
    self._thread.start()

  def stop(self):
    """Signal the thread to stop and wait for it."""
    self._exit = True
    if self._thread.is_alive():
      self._thread.join()

  def _run(self):
    """Main loop: scan WiFi, check for favorite network, and auto-connect."""
    startup_time = time.monotonic()
    last_check_time = 0.0
    
    while not self._exit:
      current_time = time.monotonic()
      
      # Wait 30 seconds after startup before first check
      if current_time - startup_time < INITIAL_CHECK_DELAY:
        time.sleep(1)
        continue
      
      # Wait 30 seconds between checks
      if current_time - last_check_time < FAVORITE_CHECK_PERIOD_SECONDS:
        time.sleep(1)
        continue
      
      last_check_time = current_time
      
      try:
        if Params is None:
          continue
        
        # Check if DBus is available
        if self._wifi_manager._router_main is None:
          continue
        
        params = Params()
        favorite_value = params.get("WifiFavoriteSSID")
        favorite_ssid = ""
        if favorite_value:
          if isinstance(favorite_value, bytes):
            favorite_ssid = favorite_value.decode('utf-8', errors='replace').strip('\x00')
          else:
            favorite_ssid = str(favorite_value).strip('\x00')
        
        if not favorite_ssid:
          # No favorite set, skip scanning
          continue
        
        # Verify favorite network is saved in NetworkManager
        saved_connections = self._wifi_manager._connections
        if favorite_ssid not in saved_connections:
          cloudlog.debug(f"BluePilot: Favorite network '{favorite_ssid}' is not saved in NetworkManager")
          continue
        
        # Request WiFi scan and update networks list
        cloudlog.debug(f"BluePilot: Scanning for networks (checking for favorite '{favorite_ssid}')...")
        try:
          self._wifi_manager._request_scan()
          # force=True: _update_networks() no-ops when settings UI set_active(False); preferred
          # WiFi must still populate _networks while user is on home screen / MICI nav.
          self._wifi_manager._update_networks(force=True)
          # Wait for scan results to populate
          time.sleep(SCAN_WAIT_SECONDS)
          self._wifi_manager._update_networks(force=True)
        except Exception as e:
          cloudlog.warning(f"BluePilot: Failed to scan networks: {e}")
          continue
        
        # Check NetworkManager's active connections directly
        active_connections = self._wifi_manager._get_active_connections()
        current_connected_ssid = None
        for conn_path in active_connections:
          try:
            conn_addr = DBusAddress(conn_path, bus_name=NM, interface=NM_ACTIVE_CONNECTION_IFACE)
            conn_type = self._wifi_manager._router_main.send_and_get_reply(Properties(conn_addr).get('Type')).body[0][1]
            if conn_type == '802-11-wireless':
              specific_obj_path = self._wifi_manager._router_main.send_and_get_reply(Properties(conn_addr).get('SpecificObject')).body[0][1]
              if specific_obj_path != "/":
                ap_addr = DBusAddress(specific_obj_path, bus_name=NM, interface=NM_ACCESS_POINT_IFACE)
                ap_ssid_bytes = self._wifi_manager._router_main.send_and_get_reply(Properties(ap_addr).get('Ssid')).body[0][1]
                current_connected_ssid = bytes(ap_ssid_bytes).decode("utf-8", "replace")
                break
          except Exception:
            continue
        
        # If favorite is already connected, nothing to do
        if current_connected_ssid == favorite_ssid:
          cloudlog.debug(f"BluePilot: Favorite network '{favorite_ssid}' is already connected")
          continue
        
        # Check if favorite is in scan results with sufficient signal strength
        favorite_network = None
        favorite_signal_strength = 0
        # Match WifiManager: scan results are updated under _scan_lock (not _lock — upstream rename).
        with self._wifi_manager._scan_lock:
          for network in self._wifi_manager._networks:
            if network.ssid == favorite_ssid:
              favorite_network = network
              favorite_signal_strength = network.strength
              break
        
        # Only attempt connection if favorite is visible in scan results AND has sufficient signal strength
        if favorite_network is None:
          cloudlog.debug(f"BluePilot: Favorite network '{favorite_ssid}' not in scan results (out of range)")
          continue
        
        if favorite_signal_strength < MIN_SIGNAL_STRENGTH:
          cloudlog.debug(f"BluePilot: Favorite network '{favorite_ssid}' signal strength ({favorite_signal_strength}%) below minimum ({MIN_SIGNAL_STRENGTH}%)")
          continue
        
        # Favorite is available with good signal strength - attempt connection
        if current_connected_ssid:
          cloudlog.info(f"BluePilot: Connected to '{current_connected_ssid}', switching to favorite '{favorite_ssid}' (signal: {favorite_signal_strength}%)...")
          try:
            # Disconnect from current network first
            self._wifi_manager._deactivate_connection(current_connected_ssid)
            time.sleep(2)
          except Exception as e:
            cloudlog.warning(f"BluePilot: Failed to disconnect from '{current_connected_ssid}': {e}")
            continue
        else:
          cloudlog.info(f"BluePilot: Connecting to favorite network '{favorite_ssid}' (signal: {favorite_signal_strength}%)...")
        
        # Try to activate favorite network
        try:
          self._wifi_manager.activate_connection(favorite_ssid, block=False)
        except Exception as e:
          cloudlog.warning(f"BluePilot: Failed to connect to favorite network '{favorite_ssid}': {e}")
      
      except Exception as e:
        cloudlog.exception(f"BluePilot: Error checking favorite network: {e}")
      
      # Small sleep to prevent tight loop
      time.sleep(1)
