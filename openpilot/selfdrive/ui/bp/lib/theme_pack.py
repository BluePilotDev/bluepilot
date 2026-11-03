"""BluePilot: seasonal theme pack loader.

A theme pack is a directory of plain assets — no code:

  <pack>/colors/colors.json        RGBA entries: Path, PathEdge, LaneLines, LeadMarker
  <pack>/sounds/<name>.wav         overrides for matching alert sounds (engage.wav, disengage.wav, ...)
  <pack>/steering_wheel/wheel.png  steering wheel icon override

Packs are discovered in BUNDLED_DIR (shipped with the repo) and USER_DIR (device-local,
drop packs in over SSH). Selection is the BPThemePack string param holding the pack's
directory name; empty/missing/unknown means no theme pack, and the special value
RAD_RACER selects the built-in 8-Bit Racer code theme instead of a pack.

This module is imported by soundd as well as the UI, so pyray is only imported inside
the texture/color helpers, never at module level.
"""
import datetime
import json
import os
import time

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params

PARAM_KEY = "BPThemePack"
AUTO_PARAM_KEY = "BPThemeAutoSeasonal"
RAD_RACER = "rad_racer"  # param value: the built-in 8-Bit Racer code theme (not a pack on disk)
BUNDLED_DIR = os.path.join(BASEDIR, "openpilot", "selfdrive", "assets", "bp_themes")
USER_DIR = "/data/bp_themes"
_PARAM_POLL_S = 2.0

# colors.json keys the renderers consume (unknown keys are loaded but ignored):
# Path/PathEdge (path ribbon gradient), LaneLines, LeadMarker (vision lead chevron + info box),
# RoadEdges (road boundary lines), Accent (torque bar fill, set-speed value),
# Background (sky treatment — rendered by theme_scene.PackScene)
COLOR_KEYS = ("Path", "PathEdge", "LaneLines", "LeadMarker", "RoadEdges", "Accent", "Background")


class ThemePack:
  def __init__(self, name: str, root: str):
    self.name = name
    self.root = root
    # Raw RGBA tuples keyed by colors.json name; rl.Color built lazily in rl_colors()
    self.colors: dict[str, tuple[int, int, int, int]] = {}
    self._rl_colors: dict | None = None
    self._wheel_textures: dict[int, object] = {}

    colors_path = os.path.join(root, "colors", "colors.json")
    if os.path.isfile(colors_path):
      try:
        with open(colors_path) as f:
          data = json.load(f)
        for key, v in data.items():
          self.colors[key] = (int(v["red"]), int(v["green"]), int(v["blue"]), int(v["alpha"]))
      except (OSError, ValueError, KeyError, TypeError):
        self.colors = {}

  def sound_path(self, filename: str) -> str | None:
    p = os.path.join(self.root, "sounds", filename)
    return p if os.path.isfile(p) else None

  @property
  def wheel_png(self) -> str | None:
    p = os.path.join(self.root, "steering_wheel", "wheel.png")
    return p if os.path.isfile(p) else None

  def rl_colors(self) -> dict:
    """colors.json entries as rl.Color, built once. UI processes only."""
    if self._rl_colors is None:
      import pyray as rl
      self._rl_colors = {k: rl.Color(*v) for k, v in self.colors.items()}
    return self._rl_colors

  def wheel_texture(self, size: int):
    """Steering wheel icon as a texture resized to size x size, or None. UI processes only."""
    size = int(size)
    if size not in self._wheel_textures:
      self._wheel_textures[size] = None
      if self.wheel_png is not None:
        import pyray as rl
        img = rl.load_image(self.wheel_png)
        if img.width > 0:
          rl.image_resize(img, size, size)
          tex = rl.load_texture_from_image(img)
          rl.set_texture_filter(tex, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
          self._wheel_textures[size] = tex
        rl.unload_image(img)
    return self._wheel_textures[size]


def list_packs() -> list[str]:
  """Names of all discoverable packs, bundled first, then user packs, each sorted."""
  names: list[str] = []
  for base in (BUNDLED_DIR, USER_DIR):
    if os.path.isdir(base):
      for entry in sorted(os.listdir(base)):
        if entry not in names and os.path.isdir(os.path.join(base, entry)):
          names.append(entry)
  return names


def _resolve(name: str) -> ThemePack | None:
  for base in (USER_DIR, BUNDLED_DIR):  # user packs shadow bundled ones of the same name
    root = os.path.join(base, name)
    if os.path.isdir(root):
      return ThemePack(name, root)
  return None


def _param_value(params: Params | None = None) -> str:
  raw = (params or Params()).get(PARAM_KEY) or ""
  if isinstance(raw, bytes):
    raw = raw.decode("utf-8", errors="replace")
  return raw.strip()


def _easter(year: int) -> datetime.date:
  """Gregorian Easter Sunday (Anonymous Gregorian algorithm)."""
  a = year % 19
  b, c = divmod(year, 100)
  d, e = divmod(b, 4)
  g = (8 * b + 13) // 25
  h = (19 * a + b - d - g + 15) % 30
  i, k = divmod(c, 4)
  ell = (32 + 2 * e + 2 * i - h - k) % 7
  m = (a + 11 * h + 22 * ell) // 451
  month, day = divmod(h + ell - 7 * m + 114, 31)
  return datetime.date(year, month, day + 1)


def _thanksgiving(year: int) -> datetime.date:
  """US Thanksgiving: fourth Thursday of November."""
  first = datetime.date(year, 11, 1)
  return first + datetime.timedelta(days=(3 - first.weekday()) % 7 + 21)


_ANCHORS = {"easter": _easter, "us_thanksgiving": _thanksgiving}


def _season_window(root: str, year: int) -> tuple[datetime.date, datetime.date] | None:
  """The pack's holiday window for `year`, from <pack>/season.json, else None.

  Every discoverable pack — bundled or user-dropped — is treated the same: a pack
  participates in auto-seasonal switching iff it ships a season.json. Two forms:

    {"start": "12-18", "end": "12-27"}                       fixed MM-DD, inclusive;
                                                             start > end wraps the year
    {"anchor": "easter", "offset_start": -6, "offset_end": 1} relative to a movable
                                                             feast (easter, us_thanksgiving)
  """
  path = os.path.join(root, "season.json")
  try:
    with open(path) as f:
      data = json.load(f)
    anchor = data.get("anchor")
    if anchor is not None:
      day = _ANCHORS[anchor](year)
      return (day + datetime.timedelta(days=int(data.get("offset_start", 0))),
              day + datetime.timedelta(days=int(data.get("offset_end", 0))))
    sm, sd = (int(v) for v in data["start"].split("-"))
    em, ed = (int(v) for v in data["end"].split("-"))
    return datetime.date(year, sm, sd), datetime.date(year, em, ed)
  except (OSError, ValueError, KeyError, TypeError):
    return None


def seasonal_pack(today: datetime.date | None = None) -> str:
  """Name of the pack whose season.json window contains `today`, else ''.

  All packs are scanned equally (user packs shadow bundled ones via _resolve).
  Overlaps go to the shortest window — the more specific holiday (April Fools)
  beats the broader one it falls inside (Easter week) — then alphabetical.
  """
  today = today or datetime.date.today()
  best: tuple[int, str] | None = None
  for name in list_packs():
    pack = _resolve(name)
    if pack is None:
      continue
    for year in (today.year - 1, today.year):  # windows that wrap the year boundary
      window = _season_window(pack.root, year)
      if window is None:
        continue
      start, end = window
      if end < start:  # fixed window wrapping into the next year (e.g. 12-29 .. 01-05)
        end = end.replace(year=end.year + 1)
      if start <= today <= end:
        length = (end - start).days
        if best is None or (length, name) < best:
          best = (length, name)
  return best[1] if best else ""


def auto_seasonal_enabled(params: Params | None = None) -> bool:
  return (params or Params()).get_bool(AUTO_PARAM_KEY)


def _effective_name(params: Params | None = None) -> str:
  """Selector value, overridden by the date-matched pack while Auto Seasonal is on.

  Outside holiday windows (or if the seasonal pack is missing on disk) the manual
  selection — including Off and Rad Racer — applies unchanged.
  """
  p = params or Params()
  name = _param_value(p)
  if p.get_bool(AUTO_PARAM_KEY):
    season = seasonal_pack()
    if season and _resolve(season) is not None:
      return season
  return name


def rad_racer_active(params: Params | None = None) -> bool:
  """True when the theme selector is set to the built-in 8-Bit Racer theme.

  Rad Racer is a code theme, not a pack on disk, so it lives as a special selector value;
  while it is active get_active_pack() returns None (no pack color/sound/wheel overrides).
  """
  return _effective_name(params).lower() == RAD_RACER


def selector_entries() -> list[tuple[str, str]]:
  """(label, param value) pairs for the theme selector — the single source of truth for
  both the C3X and MICI settings pages, so the toggle behaves identically on each.
  Packs are listed in calendar order of their season windows (Jan→Dec); packs without
  a season.json follow, alphabetical."""
  year = datetime.date.today().year

  def _calendar_key(name: str):
    pack = _resolve(name)
    win = _season_window(pack.root, year) if pack is not None else None
    if win is None:
      return (1, 0, 0, name)
    return (0, win[0].month, win[0].day, name)

  return [("Off", ""), ("8-Bit Racer", RAD_RACER)] + [(name, name) for name in sorted(list_packs(), key=_calendar_key)]


_cache: dict = {"checked_at": 0.0, "name": None, "pack": None}


def get_active_pack(force: bool = False) -> ThemePack | None:
  """The currently selected pack, or None. Re-reads the param at most every 2s."""
  now = time.monotonic()
  if not force and now - _cache["checked_at"] < _PARAM_POLL_S:
    return _cache["pack"]
  _cache["checked_at"] = now

  name = _effective_name()
  if name != _cache["name"]:
    _cache["name"] = name
    _cache["pack"] = _resolve(name) if name else None
  return _cache["pack"]


def active_pack_name() -> str:
  pack = get_active_pack()
  return pack.name if pack else ""


if __name__ == "__main__":
  # CLI for scripts/launchers: python3 -m openpilot.selfdrive.ui.bp.lib.theme_pack [pack|off|rad_racer] [minimal|camera]
  # The optional second arg toggles BPHideCameraView, for previewing over rlog-only replays
  # where the camera feed is black.
  import sys
  _name = sys.argv[1] if len(sys.argv) > 1 else ""
  if _name.lower() == "off":
    _name = ""
  if _name and _name.lower() != RAD_RACER and _resolve(_name) is None:
    print(f"unknown pack '{_name}' (available: {', '.join(list_packs()) or 'none'}, or '{RAD_RACER}')")
    sys.exit(1)
  params = Params()
  # block: launchers start the UI right after this process — the value must be on disk
  params.put(PARAM_KEY, _name, block=True)
  print(f"BPThemePack = '{_name}'")
  if len(sys.argv) > 2 and sys.argv[2] in ("minimal", "camera"):
    params.put_bool("BPHideCameraView", sys.argv[2] == "minimal", block=True)
    print(f"BPHideCameraView = {sys.argv[2] == 'minimal'}")
