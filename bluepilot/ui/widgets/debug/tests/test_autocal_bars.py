"""Battery-gauge fill logic (bluepilot/ui/widgets/debug/autocal_bars.band_fill).

Regression for the 'fills full then un-fills' bug: the fill is now one monotonic
progress-toward-calibrated metric, and a calibrated ('good') band latches full.
"""
from bluepilot.ui.widgets.debug.autocal_bars import band_fill, _COLLECT_TOP, _VERIFY_TOP


def test_collect_never_reads_full():
  # Evidence alone must top out below full — the old bug filled to 1.0 on evidence, then
  # dropped when the phase moved on.
  fill, full = band_fill({"ph": "collect", "w": 100, "need": 10}, False)
  assert fill <= _COLLECT_TOP and not full


def test_collect_progresses():
  low, _ = band_fill({"ph": "collect", "w": 0, "need": 10}, False)
  mid, _ = band_fill({"ph": "collect", "w": 5, "need": 10}, False)
  assert low == 0.0 and 0.0 < mid < _COLLECT_TOP


def test_good_is_full_and_latches():
  fill, full = band_fill({"ph": "good"}, False)
  assert fill == 1.0 and full


def test_full_survives_propose_flicker():
  # Once full, a borderline flip to 'propose' must NOT drop the battery.
  fill, full = band_fill({"ph": "propose", "t": 1.1}, True)
  assert fill == 1.0 and full


def test_verify_drops_from_full_into_band():
  # A genuine new step being checked drops out of full, into [_COLLECT_TOP, _VERIFY_TOP].
  fill, full = band_fill({"ph": "verify", "vw": 0, "vneed": 6}, True)
  assert not full and _COLLECT_TOP <= fill <= _VERIFY_TOP


def test_verify_progresses_within_band():
  lo, _ = band_fill({"ph": "verify", "vw": 0, "vneed": 6}, False)
  hi, _ = band_fill({"ph": "verify", "vw": 6, "vneed": 6}, False)
  assert lo == _COLLECT_TOP and abs(hi - _VERIFY_TOP) < 1e-9


def test_collect_reset_drops_full():
  # Evidence lost (back to collect) un-latches full.
  fill, full = band_fill({"ph": "collect", "w": 0, "need": 10}, True)
  assert not full and fill == 0.0
