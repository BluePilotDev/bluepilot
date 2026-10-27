from openpilot.selfdrive.ui.bp.onroad.network_status_bp import network_bar_count, network_mode_label


def test_network_bar_count_offline():
  assert network_bar_count(0, 4) == 0


def test_network_bar_count_tracks_strength():
  assert [network_bar_count(1, strength) for strength in range(5)] == [0, 1, 2, 4, 5]


def test_network_bar_count_clamps_invalid_values():
  assert network_bar_count(4, -1) == 0
  assert network_bar_count(4, 99) == 5


def test_network_mode_label_prefers_live_cellular_technology():
  assert network_mode_label(1, "LTE") == "WiFi"
  assert network_mode_label(4, "LTE") == "LTE"
  assert network_mode_label(4, "4G") == "4G"
  assert network_mode_label(5, "NR5G") == "5G"
  assert network_mode_label(6, "") == "ETH"


def test_network_mode_label_has_transport_fallbacks():
  assert network_mode_label(0, "") == "OFFLINE"
  assert network_mode_label(2, "") == "2G"
  assert network_mode_label(3, "") == "3G"
  assert network_mode_label(4, "") == "4G"
  assert network_mode_label(5, "") == "5G"
