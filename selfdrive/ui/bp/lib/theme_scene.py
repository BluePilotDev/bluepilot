"""BluePilot: the theme scene system — one modular architecture for every theme kind.

A *scene* is what a theme draws around the road view: a background pass (before the
model/road render) and a foreground pass (after the HUD, always under alerts). Three
kinds exist behind one interface:

  - None:           plain UI (no theme selected)
  - RadRacerScene:  the built-in 8-Bit Racer code theme. The scene object carries the
                    capability flags (replaces_road / replaces_hud); the actual pixel
                    work stays in rad_racer_theme.py, invoked by the host views exactly
                    as before — this adapter unifies SELECTION and dispatch, not drawing.
  - PackScene:      data-driven scenes for seasonal packs. Each pack may ship a
                    scene.json describing particle layers (falling snow, rising hearts,
                    fluttering bats, firework bursts), static decor (string lights,
                    garlands, bunting) and a sky treatment. Packs remain pure assets.

Host views call active_scene() every frame (internally cached, param-polled by
theme_pack) and use the scene's capabilities + draw passes. The model renderers keep
their own rad-racer road-swap flag (same param, same cadence) — replaces_road() here
mirrors it for the view layer.

scene.json (version 1) — all coordinates are FRACTIONS of the view rect, sizes in px
at 1080p (scaled by rect height). Unknown/invalid layers are dropped; a missing or
malformed scene.json means a colors-only pack (previous behavior). Two intensity
contexts: Minimal Driving View (camera hidden) gets full counts/alpha; over live
camera counts are scaled by camera_count_scale and camera_alpha applies.

This module is UI-only but must import without pyray (unit tests parse and simulate
scenes on any machine); pyray is imported inside draw methods only.
"""
import json
import math
import os
import random
import zlib

from openpilot.selfdrive.ui.bp.lib import theme_pack

# Hard budgets — a malformed (or malicious user-) scene.json cannot exceed these.
MAX_PARTICLES_MINIMAL = 60
MAX_PARTICLES_CAMERA = 24
MAX_BURST_POOL = 64
MAX_DECOR = 12
_SPEED_REF_MS = 30.0          # vEgo giving full speed_scale effect

_MODES = ("fall", "rise", "wander", "flutter")
_ANCHORS = ("top_edge", "top_left", "top_right", "bottom_left", "bottom_right")
_IDLES = ("none", "twinkle", "pulse", "sway")


def _clampf(v, lo, hi, default):
  try:
    return max(lo, min(hi, float(v)))
  except (TypeError, ValueError):
    return default


def _rng_range(spec, lo, hi, default):
  """[a, b] pair clamped; scalars accepted as [v, v]."""
  if isinstance(spec, (int, float)):
    spec = [spec, spec]
  if not (isinstance(spec, (list, tuple)) and len(spec) == 2):
    spec = default
  a = _clampf(spec[0], lo, hi, default[0])
  b = _clampf(spec[1], lo, hi, default[1])
  return (min(a, b), max(a, b))


def _resolve_tint(t, colors):
  """'#RRGGBB' | [r,g,b] | 'colors:Key' -> (r, g, b); white on anything invalid."""
  try:
    if isinstance(t, str) and t.startswith("colors:"):
      c = colors.get(t[7:])
      return (c[0], c[1], c[2]) if c else (255, 255, 255)
    if isinstance(t, str) and t.startswith("#") and len(t) == 7:
      return (int(t[1:3], 16), int(t[3:5], 16), int(t[5:7], 16))
    if isinstance(t, (list, tuple)) and len(t) >= 3:
      return (int(t[0]) & 255, int(t[1]) & 255, int(t[2]) & 255)
  except (TypeError, ValueError):
    pass
  return (255, 255, 255)


class _ParticleLayer:
  def __init__(self, d, colors):
    from openpilot.selfdrive.ui.bp.lib.theme_scene_assets import SPRITE_NAMES
    self.sprite = str(d.get("sprite", ""))
    if self.sprite not in SPRITE_NAMES:
      raise ValueError(f"unknown sprite {self.sprite!r}")
    self.mode = str(d.get("mode", "fall"))
    if self.mode not in _MODES:
      raise ValueError(f"unknown mode {self.mode!r}")
    self.count = int(_clampf(d.get("count", 30), 1, MAX_PARTICLES_MINIMAL, 30))
    self.camera_count_scale = _clampf(d.get("camera_count_scale", 0.4), 0.0, 1.0, 0.4)
    self.size_px = _rng_range(d.get("size_px"), 4, 64, (10, 22))
    self.fall_speed = _rng_range(d.get("fall_speed"), 0.005, 0.5, (0.05, 0.10))
    self.sway_amp = _rng_range(d.get("sway_amp"), 0.0, 0.08, (0.004, 0.015))
    self.sway_freq_hz = _rng_range(d.get("sway_freq_hz"), 0.02, 2.0, (0.15, 0.5))
    self.spin_dps = _rng_range(d.get("spin_dps"), -360, 360, (0.0, 0.0))
    self.tints = [_resolve_tint(t, colors) for t in (d.get("tints") or ["#FFFFFF"])][:8] or [(255, 255, 255)]
    self.minimal_alpha = int(_clampf(d.get("minimal_alpha", 225), 0, 255, 225))
    self.camera_alpha = int(_clampf(d.get("camera_alpha", 110), 0, 255, 110))
    self.speed_scale = _clampf(d.get("speed_scale", 0.5), 0.0, 2.0, 0.5)
    self.depth_split = _clampf(d.get("depth_split", 0.6), 0.0, 1.0, 0.6)
    self.blend_additive = d.get("blend") == "additive"
    # per-particle state: [x, y, size, speed, phase, spin_dps, angle, tint_idx, seed]
    self.particles: list[list[float]] = []

  def seed(self, rng):
    self.particles = []
    for _ in range(self.count):
      self.particles.append([
        rng.random(), rng.random(),
        rng.uniform(*self.size_px),
        rng.uniform(*self.fall_speed),
        rng.uniform(0.0, 2.0 * math.pi),
        rng.uniform(*self.spin_dps),
        rng.uniform(0.0, 360.0),
        rng.randrange(len(self.tints)),
        rng.random(),
      ])

  def step(self, dt, speed_mult):
    two_pi = 2.0 * math.pi
    for p in self.particles:
      freq = self.sway_freq_hz[0] + (self.sway_freq_hz[1] - self.sway_freq_hz[0]) * p[8]
      p[4] = (p[4] + two_pi * freq * dt) % two_pi
      p[6] = (p[6] + p[5] * dt) % 360.0
      v = p[3] * speed_mult
      if self.mode == "fall":
        p[1] += v * dt
        if p[1] > 1.06:
          p[1] -= 1.12
      elif self.mode == "rise":
        p[1] -= v * dt
        if p[1] < -0.06:
          p[1] += 1.12
      elif self.mode == "flutter":
        p[0] += v * dt * (1.0 if p[8] > 0.5 else -1.0)
        if p[0] > 1.08:
          p[0] -= 1.16
        elif p[0] < -0.08:
          p[0] += 1.16
      else:  # wander
        p[0] += math.cos(p[4] * 0.7) * v * dt * 0.6
        p[1] += math.sin(p[4]) * v * dt * 0.45
        p[0] = p[0] % 1.0
        p[1] = p[1] % 1.0


class _BurstLayer:
  """Firework bursts: periodic radial explosions from random points in a region."""

  def __init__(self, d, colors):
    self.period_s = _rng_range(d.get("period_s"), 0.4, 10.0, (1.5, 3.0))
    self.per_burst = int(_clampf(d.get("particles_per_burst"), 6, 32, 24))
    self.radial_speed = _rng_range(d.get("radial_speed"), 0.02, 0.6, (0.10, 0.22))
    self.gravity = _clampf(d.get("gravity", 0.12), 0.0, 1.0, 0.12)
    self.fade_s = _rng_range(d.get("fade_s"), 0.3, 4.0, (1.0, 1.6))
    r = d.get("region") or [0.1, 0.05, 0.8, 0.35]
    self.region = tuple(_clampf(r[i] if isinstance(r, (list, tuple)) and len(r) == 4 else None,
                                0.0, 1.0, (0.1, 0.05, 0.8, 0.35)[i]) for i in range(4))
    self.tints = [_resolve_tint(t, colors) for t in (d.get("tints") or ["#FFD700", "#FFFFFF"])][:8]
    self.minimal_alpha = int(_clampf(d.get("minimal_alpha", 235), 0, 255, 235))
    self.camera_alpha = int(_clampf(d.get("camera_alpha", 100), 0, 255, 100))
    self.blend_additive = d.get("blend", "additive") == "additive"
    self.camera_count_scale = _clampf(d.get("camera_count_scale", 0.5), 0.0, 1.0, 0.5)
    # pool entries: [x, y, vx, vy, age, ttl, tint_idx, size]
    self.pool: list[list[float]] = []
    self._next_burst = 0.0

  def seed(self, rng):
    self.pool = []
    self._next_burst = rng.uniform(*self.period_s) * 0.5

  def step(self, dt, rng, minimal):
    self._next_burst -= dt
    if self._next_burst <= 0.0:
      self._next_burst = rng.uniform(*self.period_s)
      n = self.per_burst if minimal else max(4, int(self.per_burst * self.camera_count_scale))
      if len(self.pool) + n <= MAX_BURST_POOL:
        cx = self.region[0] + rng.random() * self.region[2]
        cy = self.region[1] + rng.random() * self.region[3]
        tint = rng.randrange(len(self.tints))
        for _ in range(n):
          ang = rng.uniform(0.0, 2.0 * math.pi)
          spd = rng.uniform(*self.radial_speed)
          self.pool.append([cx, cy, math.cos(ang) * spd, math.sin(ang) * spd,
                            0.0, rng.uniform(*self.fade_s), tint,
                            rng.uniform(9.0, 18.0)])
    alive = []
    for p in self.pool:
      p[4] += dt
      if p[4] < p[5]:
        p[0] += p[2] * dt
        p[3] += self.gravity * dt
        p[1] += p[3] * dt
        alive.append(p)
    self.pool = alive


class _DecorLayer:
  """Static (or gently idling) decoration: tiled edge sprites or corner pieces."""

  def __init__(self, d, colors):
    from openpilot.selfdrive.ui.bp.lib.theme_scene_assets import SPRITE_NAMES
    self.sprite = str(d.get("sprite", ""))
    if self.sprite not in SPRITE_NAMES:
      raise ValueError(f"unknown sprite {self.sprite!r}")
    self.anchor = str(d.get("anchor", "top_edge"))
    if self.anchor not in _ANCHORS:
      raise ValueError(f"unknown anchor {self.anchor!r}")
    self.size_frac = _clampf(d.get("size_frac", 0.04), 0.01, 0.20, 0.04)
    self.spacing_frac = _clampf(d.get("spacing_frac", 0.06), 0.02, 0.5, 0.06)
    self.droop_frac = _clampf(d.get("droop_frac", 0.025), 0.0, 0.12, 0.025)
    self.count = int(_clampf(d.get("count", 1), 1, MAX_DECOR, 1))
    self.tints = [_resolve_tint(t, colors) for t in (d.get("tints") or ["#FFFFFF"])][:8]
    self.alpha_minimal = int(_clampf(d.get("alpha_minimal", 220), 0, 255, 220))
    self.alpha_camera = int(_clampf(d.get("alpha_camera", 90), 0, 255, 90))
    self.idle = str(d.get("idle", "none"))
    if self.idle not in _IDLES:
      self.idle = "none"
    self.phase = 0.0

  def step(self, dt):
    self.phase = (self.phase + dt) % 1000.0


class SceneSpec:
  def __init__(self, sky_tint_alpha, particles, bursts, decor, fg):
    self.sky_camera_tint_alpha = sky_tint_alpha
    self.particles = particles
    self.bursts = bursts
    self.decor = decor
    self.particles_over_hud = bool(fg.get("particles_over_hud", True))
    self.corner_accents = bool(fg.get("corner_accents", False))
    self.corner_alpha = int(_clampf(fg.get("corner_alpha", 55), 0, 160, 55))


def load_scene_spec(path, colors=None) -> SceneSpec | None:
  """Parse + validate a scene.json. Returns None (colors-only pack) on any file-level
  problem; individual invalid layers are dropped."""
  colors = colors or {}
  try:
    with open(path) as f:
      d = json.load(f)
    if int(d.get("version", 0)) != 1:
      return None
  except (OSError, ValueError, TypeError):
    return None

  sky = d.get("sky") or {}
  sky_tint = int(_clampf(sky.get("camera_tint_alpha", 40), 0, 120, 40))
  particles, bursts, decor = [], [], []
  total = 0
  for layer in (d.get("layers") or []):
    if not isinstance(layer, dict):
      continue
    try:
      t = layer.get("type", "particles")
      if t == "particles":
        pl = _ParticleLayer(layer, colors)
        if total + pl.count > MAX_PARTICLES_MINIMAL:
          pl.count = max(0, MAX_PARTICLES_MINIMAL - total)
        total += pl.count
        if pl.count > 0:
          particles.append(pl)
      elif t == "burst":
        if len(bursts) < 2:
          bursts.append(_BurstLayer(layer, colors))
      elif t == "decor":
        if len(decor) < 4:
          decor.append(_DecorLayer(layer, colors))
    except (ValueError, TypeError, KeyError):
      continue  # drop invalid layer, keep the rest
  return SceneSpec(sky_tint, particles, bursts, decor, d.get("foreground") or {})


# ---------------------------------------------------------------------------- scene kinds
class SceneBase:
  def replaces_road(self) -> bool:
    return False

  def replaces_hud(self) -> bool:
    return False

  def draw_background(self, rect, minimal: bool) -> None:
    pass

  def draw_foreground(self, rect, minimal: bool) -> None:
    pass


class RadRacerScene(SceneBase):
  """Capability adapter for the built-in 8-Bit Racer code theme. Drawing stays in
  rad_racer_theme.py / the host views; this object routes the dispatch decisions
  that used to be ad-hoc `rad_racer_active()` checks."""

  def replaces_road(self) -> bool:
    return True

  def replaces_hud(self) -> bool:
    return True   # TICI renders its full scene; MICI hosts draw inline as before


class PackScene(SceneBase):
  def __init__(self, pack, spec: SceneSpec):
    self.pack = pack
    self.spec = spec
    self._rng = random.Random(zlib.crc32(pack.name.encode()))
    for layer in spec.particles:
      layer.seed(self._rng)
    for b in spec.bursts:
      b.seed(self._rng)
    self._stepped_this_frame = False

  # ---------------- sim
  def _v_ego(self) -> float:
    try:
      from openpilot.selfdrive.ui.ui_state import ui_state
      if ui_state.sm is not None and ui_state.sm.valid.get('carState', False):
        return max(0.0, float(ui_state.sm['carState'].vEgo))
    except Exception:
      pass
    return 0.0

  def _step(self, minimal: bool):
    from openpilot.system.ui.lib.application import gui_app
    dt = 1.0 / max(1, gui_app.target_fps)
    speed_mult = 1.0
    v = self._v_ego()
    for layer in self.spec.particles:
      m = 1.0 + layer.speed_scale * min(1.0, v / _SPEED_REF_MS)
      layer.step(dt, m * speed_mult)
    for b in self.spec.bursts:
      b.step(dt, self._rng, minimal)
    for dl in self.spec.decor:
      dl.step(dt)

  # ---------------- draw helpers
  def _draw_particles(self, rect, minimal: bool, front_pass: bool):
    import pyray as rl
    from openpilot.selfdrive.ui.bp.lib.theme_scene_assets import build_sprite
    scale = rect.height / 1080.0
    for layer in self.spec.particles:
      n_total = layer.count if minimal else max(1, int(layer.count * layer.camera_count_scale))
      split = int(round(n_total * layer.depth_split))
      lo, hi = (split, n_total) if front_pass else (0, split)
      if lo >= hi:
        continue
      alpha = layer.minimal_alpha if minimal else layer.camera_alpha
      tex = build_sprite(layer.sprite)
      tex2 = build_sprite("bat2") if layer.sprite == "bat" else None
      if layer.blend_additive:
        rl.begin_blend_mode(rl.BlendMode.BLEND_ADDITIVE)
      src = rl.Rectangle(0, 0, tex.width, tex.height)
      for p in layer.particles[lo:hi]:
        sway = math.sin(p[4]) * (layer.sway_amp[0] + (layer.sway_amp[1] - layer.sway_amp[0]) * p[8])
        x = rect.x + ((p[0] + sway) % 1.0) * rect.width
        y = rect.y + p[1] * rect.height
        # back-pass particles render smaller + dimmer for depth
        size = p[2] * scale * (1.0 if front_pass else 0.72)
        a = int(alpha * (1.0 if front_pass else 0.7))
        tint = layer.tints[int(p[7])]
        t = tex2 if (tex2 is not None and p[4] > math.pi) else tex
        dst = rl.Rectangle(x, y, size, size)
        rl.draw_texture_pro(t, src, dst, rl.Vector2(size / 2, size / 2), p[6],
                            rl.Color(tint[0], tint[1], tint[2], a))
      if layer.blend_additive:
        rl.end_blend_mode()

  def _draw_bursts(self, rect, minimal: bool):
    import pyray as rl
    from openpilot.selfdrive.ui.bp.lib.theme_scene_assets import build_sprite
    scale = rect.height / 1080.0
    for b in self.spec.bursts:
      if not b.pool:
        continue
      alpha = b.minimal_alpha if minimal else b.camera_alpha
      tex = build_sprite("spark")
      src = rl.Rectangle(0, 0, tex.width, tex.height)
      if b.blend_additive:
        rl.begin_blend_mode(rl.BlendMode.BLEND_ADDITIVE)
      for p in b.pool:
        fade = max(0.0, 1.0 - p[4] / p[5])
        a = int(alpha * fade * fade)
        if a <= 2:
          continue
        size = p[7] * scale * (0.6 + 0.4 * fade)
        tint = b.tints[int(p[6]) % len(b.tints)]
        dst = rl.Rectangle(rect.x + p[0] * rect.width, rect.y + p[1] * rect.height, size, size)
        rl.draw_texture_pro(tex, src, dst, rl.Vector2(size / 2, size / 2), 0.0,
                            rl.Color(tint[0], tint[1], tint[2], a))
      if b.blend_additive:
        rl.end_blend_mode()

  def _draw_decor(self, rect, minimal: bool):
    import pyray as rl
    from openpilot.selfdrive.ui.bp.lib.theme_scene_assets import build_sprite
    for dl in self.spec.decor:
      alpha = dl.alpha_minimal if minimal else dl.alpha_camera
      tex = build_sprite(dl.sprite)
      src = rl.Rectangle(0, 0, tex.width, tex.height)
      size = dl.size_frac * rect.height
      idle_a = 1.0
      if dl.idle == "pulse":
        idle_a = 0.75 + 0.25 * math.sin(dl.phase * 2.0)
      if dl.anchor == "top_edge":
        # tiled along the top with a gentle catenary droop; twinkle alternates brightness
        n = max(2, int(rect.width / (dl.spacing_frac * rect.width)))
        for i in range(n + 1):
          fx = i / n
          droop = math.sin(fx * math.pi) * dl.droop_frac
          x = rect.x + fx * rect.width
          y = rect.y + droop * rect.height + size * 0.4
          a = alpha * idle_a
          if dl.idle == "twinkle":
            a *= 0.55 + 0.45 * math.sin(dl.phase * 3.0 + i * 1.7) ** 2
          tint = dl.tints[i % len(dl.tints)]
          rot = math.degrees(math.atan2(math.cos(fx * math.pi) * dl.droop_frac * math.pi, 1.0 / max(n, 1))) if dl.sprite == "garland" else 0.0
          dst = rl.Rectangle(x, y, size, size)
          rl.draw_texture_pro(tex, src, dst, rl.Vector2(size / 2, size / 2), rot,
                              rl.Color(tint[0], tint[1], tint[2], int(a)))
      else:
        corner_size = size * 2.2
        margin = corner_size * 0.42
        pos = {
          "top_left": (rect.x + margin, rect.y + margin, -18.0),
          "top_right": (rect.x + rect.width - margin, rect.y + margin, 18.0),
          "bottom_left": (rect.x + margin, rect.y + rect.height - margin, 195.0),
          "bottom_right": (rect.x + rect.width - margin, rect.y + rect.height - margin, 165.0),
        }[dl.anchor]
        tint = dl.tints[0]
        dst = rl.Rectangle(pos[0], pos[1], corner_size, corner_size)
        rl.draw_texture_pro(tex, src, dst, rl.Vector2(corner_size / 2, corner_size / 2), pos[2],
                            rl.Color(tint[0], tint[1], tint[2], int(alpha * idle_a)))

  def _draw_corner_accents(self, rect, minimal: bool):
    import pyray as rl
    colors = self.pack.rl_colors()
    acc = colors.get("Accent") or colors.get("Background")
    if acc is None:
      return
    a = self.spec.corner_alpha if minimal else int(self.spec.corner_alpha * 0.55)
    w = rect.width * 0.13
    h = rect.height * 0.10
    c = rl.Color(acc.r, acc.g, acc.b, a)
    fade = rl.Color(acc.r, acc.g, acc.b, 0)
    x0, y0, x1 = rect.x, rect.y, rect.x + rect.width
    rl.draw_triangle(rl.Vector2(x0, y0), rl.Vector2(x0, y0 + h), rl.Vector2(x0 + w, y0), c)
    rl.draw_triangle(rl.Vector2(x0 + w, y0), rl.Vector2(x0, y0 + h), rl.Vector2(x0 + w * 1.6, y0 + h * 0.35), fade)
    rl.draw_triangle(rl.Vector2(x1, y0), rl.Vector2(x1 - w, y0), rl.Vector2(x1, y0 + h), c)
    rl.draw_triangle(rl.Vector2(x1 - w, y0), rl.Vector2(x1 - w * 1.6, y0 + h * 0.35), rl.Vector2(x1, y0 + h), fade)

  # ---------------- passes
  def draw_background(self, rect, minimal: bool) -> None:
    import pyray as rl
    bg = self.pack.rl_colors().get("Background")
    if bg is not None:
      if minimal:
        bottom = rl.Color(int(bg.r * 0.12), int(bg.g * 0.12), int(bg.b * 0.12), 255)
        rl.draw_rectangle_gradient_v(int(rect.x), int(rect.y), int(rect.width), int(rect.height), bg, bottom)
      elif self.spec.sky_camera_tint_alpha > 0:
        rl.draw_rectangle(int(rect.x), int(rect.y), int(rect.width), int(rect.height),
                          rl.Color(bg.r, bg.g, bg.b, self.spec.sky_camera_tint_alpha))
    self._step(minimal)
    self._draw_particles(rect, minimal, front_pass=False)

  def draw_foreground(self, rect, minimal: bool) -> None:
    if self.spec.corner_accents:
      self._draw_corner_accents(rect, minimal)
    self._draw_decor(rect, minimal)
    if self.spec.particles_over_hud:
      self._draw_particles(rect, minimal, front_pass=True)
    self._draw_bursts(rect, minimal)


# ---------------------------------------------------------------------------- dispatcher
_scene_cache: dict = {"key": None, "scene": None, "checked_at": 0.0}
_POLL_S = 2.0


def _default_spec() -> SceneSpec:
  """Colors-only packs (no scene.json): behave exactly like the pre-scene loader —
  gradient sky in Minimal Driving View, nothing over camera, no layers."""
  return SceneSpec(0, [], [], [], {"particles_over_hud": False, "corner_accents": False})


def active_scene() -> SceneBase | None:
  """The scene for the current theme selection, or None. Cheap per-frame call: the
  selection is re-checked at most every _POLL_S; scenes are rebuilt (and reseeded)
  only when the selection changes."""
  import time
  now = time.monotonic()
  if now - _scene_cache["checked_at"] < _POLL_S:
    return _scene_cache["scene"]
  _scene_cache["checked_at"] = now

  if theme_pack.rad_racer_active():
    key = theme_pack.RAD_RACER
  else:
    pack = theme_pack.get_active_pack()
    key = pack.name if pack else None

  if key != _scene_cache["key"]:
    _scene_cache["key"] = key
    scene = None
    if key == theme_pack.RAD_RACER:
      scene = RadRacerScene()
    elif key is not None:
      pack = theme_pack.get_active_pack()
      if pack is not None:
        spec = load_scene_spec(os.path.join(pack.root, "scene.json"), pack.colors)
        scene = PackScene(pack, spec if spec is not None else _default_spec())
    _scene_cache["scene"] = scene
  return _scene_cache["scene"]
