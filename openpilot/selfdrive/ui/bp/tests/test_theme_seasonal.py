"""Auto seasonal theme: pack-declared season.json windows, movable feasts, user-pack parity."""
import datetime
import json

from openpilot.selfdrive.ui.bp.lib import theme_pack
from openpilot.selfdrive.ui.bp.lib.theme_pack import _easter, _thanksgiving, seasonal_pack

d = datetime.date


class TestMovableFeasts:
  def test_easter_dates(self):
    # Known Easter Sundays (Gregorian)
    assert _easter(2024) == d(2024, 3, 31)
    assert _easter(2025) == d(2025, 4, 20)
    assert _easter(2026) == d(2026, 4, 5)
    assert _easter(2027) == d(2027, 3, 28)

  def test_thanksgiving_dates(self):
    # Fourth Thursday of November
    assert _thanksgiving(2024) == d(2024, 11, 28)
    assert _thanksgiving(2025) == d(2025, 11, 27)
    assert _thanksgiving(2026) == d(2026, 11, 26)


class TestSeasonalWindows:
  def test_bundled_windows(self):
    assert seasonal_pack(d(2026, 1, 3)) == "new_years"      # wraps the year boundary
    assert seasonal_pack(d(2026, 2, 14)) == "valentines_day"
    assert seasonal_pack(d(2026, 3, 17)) == "st_patricks_day"
    assert seasonal_pack(d(2026, 5, 5)) == "cinco_de_mayo"
    assert seasonal_pack(d(2026, 7, 4)) == "fourth_of_july"
    assert seasonal_pack(d(2026, 10, 31)) == "halloween_week"
    assert seasonal_pack(d(2026, 12, 25)) == "christmas_week"
    assert seasonal_pack(d(2026, 12, 30)) == "new_years"

  def test_movable_windows_and_overlap(self):
    # Easter 2026 is Apr 5, so its week (Mar 30 - Apr 6) contains April Fools;
    # the shorter April Fools window wins its own days.
    assert seasonal_pack(d(2026, 4, 1)) == "april_fools"
    assert seasonal_pack(d(2026, 4, 4)) == "easter_week"
    assert seasonal_pack(d(2026, 4, 6)) == "easter_week"
    # Thanksgiving 2026 is Nov 26: window Nov 21-28
    assert seasonal_pack(d(2026, 11, 21)) == "thanksgiving_week"
    assert seasonal_pack(d(2026, 11, 28)) == "thanksgiving_week"

  def test_off_season_is_empty(self):
    for day in (d(2026, 1, 20), d(2026, 6, 10), d(2026, 7, 22), d(2026, 9, 15), d(2026, 12, 5)):
      assert seasonal_pack(day) == ""

  def test_every_day_resolves_to_known_pack(self):
    bundled = set(theme_pack.list_packs()) | {""}
    day = d(2026, 1, 1)
    while day.year == 2026:
      assert seasonal_pack(day) in bundled
      day += datetime.timedelta(days=1)


class TestUserPackParity:
  def test_user_pack_participates_equally(self, tmp_path, monkeypatch):
    # A user-dropped pack with a season.json joins auto rotation like any bundled pack
    root = tmp_path / "team_week"
    root.mkdir()
    (root / "season.json").write_text(json.dumps({"start": "09-01", "end": "09-07"}))
    monkeypatch.setattr(theme_pack, "USER_DIR", str(tmp_path))
    assert seasonal_pack(d(2026, 9, 3)) == "team_week"
    assert seasonal_pack(d(2026, 9, 20)) == ""

  def test_user_pack_shadows_bundled_window(self, tmp_path, monkeypatch):
    # Same-name user pack overrides the bundled one entirely, window included
    root = tmp_path / "christmas_week"
    root.mkdir()
    (root / "season.json").write_text(json.dumps({"start": "12-01", "end": "12-31"}))
    monkeypatch.setattr(theme_pack, "USER_DIR", str(tmp_path))
    assert seasonal_pack(d(2026, 12, 5)) == "christmas_week"
