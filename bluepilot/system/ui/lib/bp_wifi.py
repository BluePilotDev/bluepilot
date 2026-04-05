"""
BluePilot: WiFi extensions for WPA2/WPA3 cipher support and favorite network auto-connect.

Provides an extended supports_wpa bitmask that includes TKIP, CCMP, and SAE ciphers
to fix blank SSID detection on C4/MICI devices. Also manages favorite network
auto-reconnection.
"""

from openpilot.system.ui.lib.networkmanager import (
  NM_802_11_AP_SEC_PAIR_WEP40, NM_802_11_AP_SEC_PAIR_WEP104,
  NM_802_11_AP_SEC_PAIR_TKIP, NM_802_11_AP_SEC_PAIR_CCMP,
  NM_802_11_AP_SEC_GROUP_WEP40, NM_802_11_AP_SEC_GROUP_WEP104,
  NM_802_11_AP_SEC_GROUP_TKIP, NM_802_11_AP_SEC_GROUP_CCMP,
  NM_802_11_AP_SEC_KEY_MGMT_PSK, NM_802_11_AP_SEC_KEY_MGMT_SAE,
)

# Extended WPA bitmask including WPA2/WPA3 ciphers (TKIP, CCMP, SAE)
SUPPORTS_WPA_EXTENDED = (
  NM_802_11_AP_SEC_PAIR_WEP40 | NM_802_11_AP_SEC_PAIR_WEP104 |
  NM_802_11_AP_SEC_PAIR_TKIP | NM_802_11_AP_SEC_PAIR_CCMP |
  NM_802_11_AP_SEC_GROUP_WEP40 | NM_802_11_AP_SEC_GROUP_WEP104 |
  NM_802_11_AP_SEC_GROUP_TKIP | NM_802_11_AP_SEC_GROUP_CCMP |
  NM_802_11_AP_SEC_KEY_MGMT_PSK | NM_802_11_AP_SEC_KEY_MGMT_SAE
)
