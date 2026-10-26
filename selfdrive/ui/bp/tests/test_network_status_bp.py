from openpilot.selfdrive.ui.bp.onroad.network_status_bp import network_bar_count


def test_network_bar_count_offline():
  assert network_bar_count(0, 4) == 0


def test_network_bar_count_tracks_strength():
  assert [network_bar_count(1, strength) for strength in range(5)] == [0, 1, 2, 3, 4]


def test_network_bar_count_clamps_invalid_values():
  assert network_bar_count(4, -1) == 0
  assert network_bar_count(4, 99) == 4
