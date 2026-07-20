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
import json
import os
import time

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params

PARAM_KEY = "BPThemePack"
RAD_RACER = "rad_racer"  # param value: the built-in 8-Bit Racer code theme (not a pack on disk)
BUNDLED_DIR = os.path.join(BASEDIR, "selfdrive", "assets", "bp_themes")
USER_DIR = "/data/bp_themes"
_PARAM_POLL_S = 2.0

# colors.json keys the renderers consume (unknown keys are loaded but ignored):
# Path/PathEdge (path ribbon gradient), LaneLines, LeadMarker (vision lead chevron + info box),
# RoadEdges (road boundary lines), Accent (torque bar fill, set-speed value),
# Background (sky gradient when the camera view is hidden)
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


def rad_racer_active(params: Params | None = None) -> bool:
  """True when the theme selector is set to the built-in 8-Bit Racer theme.

  Rad Racer is a code theme, not a pack on disk, so it lives as a special selector value;
  while it is active get_active_pack() returns None (no pack color/sound/wheel overrides).
  """
  return _param_value(params).lower() == RAD_RACER


def selector_entries() -> list[tuple[str, str]]:
  """(label, param value) pairs for the theme selector — the single source of truth for
  both the C3X and MICI settings pages, so the toggle behaves identically on each."""
  return [("Off", ""), ("8-Bit Racer", RAD_RACER)] + [(name, name) for name in list_packs()]


_cache: dict = {"checked_at": 0.0, "name": None, "pack": None}


def get_active_pack(force: bool = False) -> ThemePack | None:
  """The currently selected pack, or None. Re-reads the param at most every 2s."""
  now = time.monotonic()
  if not force and now - _cache["checked_at"] < _PARAM_POLL_S:
    return _cache["pack"]
  _cache["checked_at"] = now

  name = _param_value()
  if name != _cache["name"]:
    _cache["name"] = name
    _cache["pack"] = _resolve(name) if name else None
  return _cache["pack"]


def active_pack_name() -> str:
  pack = get_active_pack()
  return pack.name if pack else ""


def draw_background(rect, hide_camera: bool) -> None:
  """Paint the pack's sky: a vertical gradient from Background down to near-black.

  Only drawn when the camera view is hidden (Minimal Driving View) — over live camera
  footage the background would obscure the road. UI processes only.
  """
  if not hide_camera:
    return
  pack = get_active_pack()
  if pack is None:
    return
  bg = pack.rl_colors().get("Background")
  if bg is None:
    return
  import pyray as rl
  bottom = rl.Color(int(bg.r * 0.12), int(bg.g * 0.12), int(bg.b * 0.12), 255)
  rl.draw_rectangle_gradient_v(int(rect.x), int(rect.y), int(rect.width), int(rect.height), bg, bottom)


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
