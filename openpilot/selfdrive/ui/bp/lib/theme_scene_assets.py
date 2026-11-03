"""BluePilot: procedurally rendered sprites for theme-pack scenes.

Every sprite is generated from 2D signed-distance fields in numpy at load time —
no bundled images, smooth antialiased edges, proper shading. Most sprites render
GRAYSCALE + alpha so a single texture serves every pack (per-particle color via
draw_texture_pro tint); a few sprites (garland, poppy, flags, the hero mascots)
bake their own colors because they are multi-color by nature.

The field math is GL-free and unit-testable (render_sprite returns numpy arrays);
only build_sprite touches pyray, uploading via an in-memory PNG.
"""
import math
import struct
import zlib
from functools import lru_cache

import numpy as np

SPRITE_SIZE = 96          # default raster size (px); crisp down to ~14 px on screen
_EDGE = 1.5               # antialias edge width in pixels


# ---------------------------------------------------------------------------- field helpers
def _grid(size: int):
  """Coordinate grid in [-1, 1] x [-1, 1], y increasing downward."""
  ax = np.linspace(-1.0, 1.0, size, dtype=np.float32)
  x, y = np.meshgrid(ax, ax)
  return x, y


def _aa(sdf: np.ndarray, size: int) -> np.ndarray:
  """Signed distance -> coverage alpha [0..1] with a smooth edge."""
  px = 2.0 / size
  return np.clip(0.5 - sdf / (_EDGE * px * 2.0), 0.0, 1.0).astype(np.float32)


def _circle(x, y, cx, cy, r):
  return np.hypot(x - cx, y - cy) - r


def _ellipse(x, y, cx, cy, rx, ry):
  # scaled-space approximation; adequate for shading/AA at sprite scale
  return (np.hypot((x - cx) / rx, (y - cy) / ry) - 1.0) * min(rx, ry)


def _segment(x, y, ax_, ay, bx, by, w):
  px, py = x - ax_, y - ay
  bx_, by_ = bx - ax_, by - ay
  h = np.clip((px * bx_ + py * by_) / (bx_ * bx_ + by_ * by_ + 1e-9), 0.0, 1.0)
  return np.hypot(px - bx_ * h, py - by_ * h) - w


def _rot(x, y, ang):
  c, s = math.cos(ang), math.sin(ang)
  return c * x - s * y, s * x + c * y


def _smin(a, b, k=0.06):
  """Smooth union — organic blends between lobes."""
  h = np.clip(0.5 + 0.5 * (b - a) / k, 0.0, 1.0)
  return b + (a - b) * h - k * h * (1.0 - h)


# ---------------------------------------------------------------------------- sprites
def _snowflake(size: int, seed: int = 0):
  """Six-fold dendrite: folded polar space -> one stem + angled side branches."""
  x, y = _grid(size)
  r = np.hypot(x, y)
  ang = np.arctan2(y, x)
  # fold into one 30-degree sector (6-fold + mirror symmetry)
  sector = math.pi / 3.0
  a = np.abs(((ang + sector / 2.0) % sector) - sector / 2.0)
  fx, fy = r * np.cos(a), r * np.sin(a)

  rng = np.random.default_rng(11 + seed)
  # strokes sized to survive 20-40px on-screen draws — thin dendrites vanish there
  stem_w = 0.060 + 0.010 * rng.random()
  d = _segment(fx, fy, 0.02, 0.0, 0.92, 0.0, stem_w)
  # side branches: at each radius, a V pair angled forward, shrinking outward
  for br in (0.30, 0.52, 0.72):
    ln = (0.30 - 0.22 * br) * (0.9 + 0.2 * rng.random())
    bx = br + ln * math.cos(1.05)
    by = ln * math.sin(1.05)
    d = np.minimum(d, _segment(fx, fy, br, 0.0, bx, by, stem_w * 0.85))
  d = np.minimum(d, _circle(fx, fy, 0.0, 0.0, 0.13))       # hub
  d = np.minimum(d, _circle(fx, fy, 0.92, 0.0, 0.075))     # tip crystal
  alpha = _aa(d, size)
  # crystalline shading: brighter core, faint sparkle at hub/tips
  v = 0.82 + 0.18 * np.clip(1.0 - r, 0.0, 1.0)
  return v, alpha


def _heart(size: int):
  """Classic implicit heart, glossy: rim shade + upper-left light."""
  x, y = _grid(size)
  hx, hy = x * 1.25, -y * 1.25 + 0.30
  f = (hx * hx + hy * hy - 0.75) ** 3 - hx * hx * hy * hy * hy
  # convert implicit value to a pseudo-distance for AA
  d = f / (np.abs(np.gradient(f)[0]) + np.abs(np.gradient(f)[1]) + 1e-6) * (2.0 / size)
  alpha = _aa(d.astype(np.float32), size)
  light = np.clip(0.85 - 0.35 * np.hypot(x + 0.35, y + 0.35), 0.35, 1.0)
  rim = np.clip(-d * size * 0.5, 0.0, 1.0)  # darker just inside the edge
  v = np.clip(light * (0.75 + 0.25 * rim), 0.0, 1.0)
  # specular dot
  v = np.maximum(v, np.clip(1.0 - np.hypot((x + 0.30) * 4.0, (y + 0.42) * 4.0), 0.0, 1.0))
  return v.astype(np.float32), alpha


def _vesica(x, y, width, length):
  """Leaf/petal body: intersection of two offset circles, stretched."""
  r = (width * width + length * length) / (2.0 * width)
  return np.maximum(_circle(x, y, r - width, 0.0, r), _circle(x, y, -(r - width), 0.0, r))


def _leaf(size: int):
  """Pointed leaf: vesica body tip-up, midrib to the tip, veins clipped inside,
  stem attached at the base and flowing into the midrib."""
  x, y = _grid(size)
  lx, ly = _rot(x, y, -0.30)
  body = _vesica((ly + 0.05) * 1.05, lx * 1.05, 0.36, 0.80) * 0.9
  stem = _segment(lx, ly, 0.0, 0.72, -0.06, 1.00, 0.044)
  stem = np.minimum(stem, _segment(lx, ly, 0.0, 0.72, 0.0, 0.55, 0.044))
  alpha = np.maximum(_aa(body, size), _aa(stem, size))
  inside = body < 0
  v = 0.58 + 0.42 * np.clip(0.5 - ly * 0.65, 0.0, 1.0)
  rib = _segment(lx, ly, 0.0, -0.78, 0.0, 0.72, 0.020)
  v = np.where((rib < 0) & inside, v * 0.52, v)
  for t, ln in ((-0.50, 0.20), (-0.18, 0.28), (0.16, 0.30), (0.45, 0.24)):
    for sgn in (-1.0, 1.0):
      vein = _segment(lx, ly, 0.0, t, sgn * ln, t - ln * 0.9, 0.014)
      v = np.where((vein < 0) & inside, v * 0.70, v)
  v = np.where(stem < 0, 0.42, v)
  return v.astype(np.float32), alpha


def _petal(size: int):
  x, y = _grid(size)
  bend = 0.18 * (y * y - 0.3)          # soft banana curve
  body = _vesica((y) * 1.05, (x - bend) * 1.05, 0.30, 0.82) * 0.9
  alpha = _aa(body, size)
  v = 0.66 + 0.34 * np.clip(0.55 - y * 0.75, 0.0, 1.0)
  edge_hi = np.clip(-body * size * 0.25, 0.0, 1.0)
  v = np.clip(v * (0.85 + 0.15 * edge_hi), 0.0, 1.0)
  crease = _segment(x - bend, y, 0.0, -0.75, 0.0, 0.65, 0.016)
  v = np.where(crease < 0, v * 0.82, v)
  return v.astype(np.float32), alpha


def _clover(size: int):
  """Three heart-shaped leaflets 120 degrees apart, notched, stem attached."""
  x, y = _grid(size)
  cy = y + 0.10
  d = None
  for k in range(3):
    lx, ly = _rot(x, cy, k * 2.0 * math.pi / 3.0)
    lobe = _smin(_circle(lx, ly, -0.155, -0.40, 0.235), _circle(lx, ly, 0.155, -0.40, 0.235), 0.07)
    notch = _segment(lx, ly, 0.0, -0.62, 0.0, -0.40, 0.045)   # heart cleft
    lobe = np.maximum(lobe, -notch)
    d = lobe if d is None else np.minimum(d, lobe)
  # stem starts INSIDE the lobe junction and arcs out
  stem = np.minimum(_segment(x, cy, 0.0, -0.05, 0.06, 0.45, 0.048),
                    _segment(x, cy, 0.06, 0.45, 0.22, 0.80, 0.048))
  alpha = np.maximum(_aa(d, size), _aa(stem, size))
  v = np.full_like(x, 0.85)
  for k in range(3):
    lx, ly = _rot(x, cy, k * 2.0 * math.pi / 3.0)
    crease = _segment(lx, ly, 0.0, -0.14, 0.0, -0.58, 0.011)
    v = np.where((crease < 0) & (d < 0), v * 0.60, v)
  v = np.where((stem < 0) & (d > 0), 0.55, v)
  return v.astype(np.float32), alpha


def _bat(size: int, flap_up: bool):
  """Iconic bat: two wing humps on top, scalloped trailing edge below, ears + head.
  Frames differ by wing tilt about the shoulders."""
  x, y = _grid(size)
  ax = np.abs(x)
  tilt = -0.30 if flap_up else 0.18
  wx, wy = _rot(ax - 0.08, y, tilt)
  # top humps (wing leading edge)
  humps = np.minimum(_circle(wx, wy, 0.30, 0.02, 0.30), _circle(wx, wy, 0.64, 0.10, 0.22))
  # membrane fills down to a trailing edge...
  membrane = np.maximum(humps - 0.10, wy - 0.30)
  # ...scalloped by bites from below
  for i, bx in enumerate((0.16, 0.46, 0.74)):
    membrane = np.maximum(membrane, -_circle(wx, wy, bx, 0.46, 0.155 - 0.015 * i))
  membrane = np.maximum(membrane, wx - 0.90)
  body = _ellipse(x, y, 0.0, 0.06, 0.115, 0.21)
  head = _circle(x, y, 0.0, -0.16, 0.105)
  # ears: short tapered strokes from the head top to points
  ear = np.minimum(_segment(ax, y, 0.055, -0.22, 0.105, -0.385, 0.052),
                   _segment(ax, y, 0.075, -0.30, 0.105, -0.385, 0.030))
  d = np.minimum(np.minimum(membrane, body), np.minimum(head, ear))
  alpha = _aa(d.astype(np.float32), size)
  v = np.full_like(x, 0.88)
  for bx in (0.20, 0.50):
    ridge = _segment(wx, wy, 0.04, 0.0, bx + 0.18, 0.34, 0.011)
    v = np.where((ridge < 0) & (d < 0), v * 0.72, v)
  return v.astype(np.float32), alpha


def _glow(size: int, core: float, falloff: float):
  """Radial glow (firefly / spark core) — built for additive blending."""
  x, y = _grid(size)
  r = np.hypot(x, y)
  core_term = np.clip(1.0 - r / core, 0.0, 1.0) if core > 1e-6 else 0.0
  alpha = np.clip(np.exp(-(r / falloff) ** 2) + core_term, 0.0, 1.0)
  return np.ones_like(x), alpha.astype(np.float32)


def _spark(size: int):
  """Four-point star + glow."""
  x, y = _grid(size)
  d = np.minimum(_segment(x, y, -0.7, 0.0, 0.7, 0.0, 0.045),
                 _segment(x, y, 0.0, -0.7, 0.0, 0.7, 0.045))
  dx, dy = _rot(x, y, math.pi / 4)
  d = np.minimum(d, np.minimum(_segment(dx, dy, -0.4, 0.0, 0.4, 0.0, 0.03),
                               _segment(dx, dy, 0.0, -0.4, 0.0, 0.4, 0.03)))
  star = _aa(d, size)
  _, glow = _glow(size, 0.10, 0.45)
  alpha = np.clip(star + glow * 0.6, 0.0, 1.0)
  return np.ones_like(x), alpha.astype(np.float32)


def _bulb(size: int):
  """String-light bulb hanging point-down: cap fused to the glass, specular
  highlight, restrained halo."""
  x, y = _grid(size)
  glass = _smin(_circle(x, y, 0.0, 0.20, 0.30), _segment(x, y, 0.0, -0.22, 0.0, 0.16, 0.14), 0.10)
  cap = np.maximum(np.abs(x) - 0.15, np.abs(y + 0.30) - 0.13).astype(np.float32)
  alpha = np.maximum(_aa(glass, size), _aa(cap, size))
  r_glow = np.hypot(x, (y - 0.14) * 0.9)
  alpha = np.clip(alpha + np.exp(-(r_glow / 0.55) ** 2) * 0.30, 0.0, 1.0)
  v = np.where(cap < 0, 0.32, 0.90)
  ribs = np.abs(((y + 0.30) * 12.0) % 2.0 - 1.0) < 0.35
  v = np.where((cap < 0) & ribs, 0.45, v)
  spec = np.clip(1.0 - np.hypot((x + 0.11) * 5.5, (y - 0.08) * 4.0), 0.0, 1.0)
  v = np.clip(v + spec * 0.9, 0.0, 1.3)
  return v.astype(np.float32), alpha


def _pennant(size: int):
  """Crisp bunting triangle, bordered, subtle sheen."""
  x, y = _grid(size)
  # triangle: top edge y=-0.8 between x±0.55, apex (0, 0.85)
  d = np.maximum(y * 0 + (-0.8 - y), np.abs(x) * 1.55 + (y + 0.8) * 0.52 - 0.86)
  d = np.maximum(d, -(0.85 - y))
  alpha = _aa(d.astype(np.float32), size)
  v = 0.78 + 0.22 * np.clip(-x + 0.3, 0.0, 1.0) * 0.5
  border = np.clip(-d * size * 0.35, 0.0, 1.0)
  v = np.clip(v * (0.7 + 0.3 * border), 0.0, 1.0)
  return v.astype(np.float32), alpha


def _garland_tile(size: int):
  """Colored: a lush horizontal pine sprig — dense two-row needle fan on a branch,
  berry cluster with highlights — stamped along curves by the engine."""
  rng = np.random.default_rng(9)
  x, y = _grid(size)
  branch = _segment(x, y, -0.9, 0.02, 0.9, -0.02, 0.030)
  d = branch.copy()
  for i in range(34):
    t = -0.85 + 1.7 * (i / 33.0)
    side = 1.0 if i % 2 == 0 else -1.0
    ang = side * (1.05 + rng.uniform(-0.15, 0.15)) - 0.42
    ln = 0.30 + rng.uniform(-0.05, 0.07)
    seg = _segment(x, y, t, 0.0, t + ln * 0.45, side * ln * 0.85, 0.030)
    d = np.minimum(d, seg)
    seg2 = _segment(x, y, t + 0.04, 0.0, t + 0.04 + ln * 0.55, side * ln * 0.6, 0.024)
    d = np.minimum(d, seg2)
  alpha = _aa(d, size)
  rgb = np.zeros((size, size, 3), dtype=np.float32)
  tone = (0.70 + 0.30 * rng.random((size, size))).astype(np.float32)
  rgb[..., 0] = 0.14 * tone
  rgb[..., 1] = 0.46 * tone
  rgb[..., 2] = 0.20 * tone
  rgb[branch < 0] = (0.24, 0.16, 0.08)
  for bx, by in ((0.30, 0.06), (0.42, -0.08), (0.36, 0.20)):
    berry = _circle(x, y, bx, by, 0.080)
    rgb[berry < 0] = (0.80, 0.15, 0.15)
    rgb[_circle(x, y, bx - 0.026, by - 0.026, 0.028) < 0] = (1.0, 0.58, 0.58)
    alpha = np.maximum(alpha, _aa(berry, size))
  return rgb, alpha


def _tri_sdf(x, y, p0, p1, p2):
  """Signed distance-ish field of a triangle (inside < 0). CCW vertex order."""
  d = None
  pts = (p0, p1, p2)
  for i in range(3):
    a, b = pts[i], pts[(i + 1) % 3]
    nx, ny = b[1] - a[1], -(b[0] - a[0])
    ln = math.hypot(nx, ny) or 1.0
    sd = (x - a[0]) * (nx / ln) + (y - a[1]) * (ny / ln)
    d = sd if d is None else np.maximum(d, sd)
  return d


def _star5(size: int):
  """Five-point star, point up: union of 5 point-triangles over an inner pentagon."""
  x, y = _grid(size)
  outer, inner = 0.88, 0.36
  op = [(outer * math.sin(a), -outer * math.cos(a)) for a in (i * 2 * math.pi / 5 for i in range(5))]
  ip = [(inner * math.sin(a + math.pi / 5), -inner * math.cos(a + math.pi / 5)) for a in (i * 2 * math.pi / 5 for i in range(5))]
  d = None
  for i in range(5):
    t = _tri_sdf(x, y, op[i], ip[i], ip[(i - 1) % 5])
    d = t if d is None else np.minimum(d, t)
  pent = None  # inner pentagon: intersection of its 5 edge half-planes
  for i in range(5):
    a, b = ip[i], ip[(i + 1) % 5]
    nx, ny = b[1] - a[1], -(b[0] - a[0])
    ln = math.hypot(nx, ny) or 1.0
    sd = (x - a[0]) * (nx / ln) + (y - a[1]) * (ny / ln)
    pent = sd if pent is None else np.maximum(pent, sd)
  d = np.minimum(d, pent)
  alpha = _aa(d, size)
  # beveled shading: ridges from center to the points
  r = np.hypot(x, y)
  v = 0.78 + 0.22 * np.clip(1.0 - r * 0.9, 0.0, 1.0)
  v = np.maximum(v, np.clip(1.0 - np.hypot((x + 0.2) * 3.0, (y + 0.28) * 3.0), 0.0, 1.0))
  return v.astype(np.float32), alpha


def _orb(size: int):
  """Faceted glowing sphere — the New Year's drop ball. Tinted at draw time."""
  x, y = _grid(size)
  r = np.hypot(x, y)
  disc = _aa(r - 0.60, size)
  glow = np.exp(-((r - 0.60).clip(0) / 0.30) ** 2) * 0.55
  alpha = np.clip(disc + glow, 0.0, 1.0)
  lattice = 0.68 + 0.32 * (np.sin(x * 21.0) * np.sin(y * 21.0)) ** 2
  v = np.where(disc > 0.5, lattice * (1.0 - 0.25 * np.clip(r / 0.60, 0.0, 1.0) ** 2), 1.0)
  v = np.maximum(v, np.clip(1.0 - np.hypot((x + 0.22) * 3.4, (y + 0.22) * 3.4), 0.0, 1.0))
  return v.astype(np.float32), alpha


def _poppy(size: int):
  """Colored: remembrance poppy — four round overlapping scarlet petals, dark center."""
  x, y = _grid(size)
  petals = None
  edge = None
  for k in range(4):
    a = k * math.pi / 2 + math.pi / 4
    c = _circle(x, y, 0.36 * math.cos(a), 0.36 * math.sin(a), 0.44)
    petals = c if petals is None else np.minimum(petals, c)
    edge = np.abs(c) if edge is None else np.minimum(edge, np.abs(c))
  alpha = _aa(petals, size)
  r = np.hypot(x, y)
  shade = np.clip(1.0 - edge * 1.8, 0.55, 1.0) * (0.80 + 0.20 * np.clip(1.0 - r, 0.0, 1.0))
  rgb = np.zeros((size, size, 3), dtype=np.float32)
  rgb[..., 0] = 0.90 * shade
  rgb[..., 1] = 0.13 * shade
  rgb[..., 2] = 0.10 * shade
  center = _circle(x, y, 0.0, 0.0, 0.17)
  rgb[center < 0] = (0.10, 0.07, 0.06)
  ring = (np.abs(r - 0.20) < 0.035)
  rgb[ring & (petals < 0)] = (0.30, 0.22, 0.10)   # stamen ring
  return rgb, np.clip(alpha, 0.0, 1.0)


def _sleigh(size: int):
  """Colored: Santa's sleigh in side profile — red body, curled prow, gold runner."""
  x, y = _grid(size)
  red = (0.72, 0.10, 0.12)
  dark_red = (0.50, 0.06, 0.08)
  gold = (0.85, 0.68, 0.25)
  rgb = np.zeros((size, size, 3), dtype=np.float32)
  alpha = np.zeros((size, size), dtype=np.float32)
  body = np.maximum(_ellipse(x, y, 0.05, 0.05, 0.62, 0.34), y - 0.28)   # tub, flat floor
  back = _ellipse(x, y, 0.52, -0.12, 0.16, 0.30)                        # high back rest
  body = np.minimum(body, np.maximum(back, y - 0.28))
  prow = _segment(x, y, -0.52, 0.10, -0.72, -0.30, 0.075)               # curling front
  prow = np.minimum(prow, _circle(x, y, -0.72, -0.34, 0.095))
  shape = np.minimum(body, prow)
  alpha = np.maximum(alpha, _aa(shape, size))
  rgb[shape < 1] = red
  rgb[(shape < 0) & (y < -0.05)] = dark_red
  rim = (np.abs(shape) < 0.035) & (y < 0.1)
  rgb[rim] = gold
  runner = _segment(x, y, -0.62, 0.42, 0.58, 0.42, 0.035)               # gold runner
  runner = np.minimum(runner, _circle(x, y, -0.66, 0.36, 0.06))
  strut1 = _segment(x, y, -0.30, 0.28, -0.30, 0.42, 0.03)
  strut2 = _segment(x, y, 0.34, 0.28, 0.34, 0.42, 0.03)
  for part in (runner, strut1, strut2):
    alpha = np.maximum(alpha, _aa(part, size))
    rgb[part < 0] = gold
  return np.clip(rgb, 0.0, 1.0), np.clip(alpha, 0.0, 1.0)


def _ghost(size: int):
  """Colored: friendly sheet ghost — wavy hem, side-glance eyes."""
  x, y = _grid(size)
  rgb = np.zeros((size, size, 3), dtype=np.float32)
  head = _circle(x, y, 0.0, -0.30, 0.42)
  taper = 0.42 - 0.10 * np.clip(y + 0.30, 0.0, 1.0)   # sheet narrows toward the hem
  sides = np.maximum(np.abs(x) - taper, y + 0.30)
  hem = y - (0.52 + 0.10 * np.cos(x * 11.0))          # wavy bottom edge
  body = np.maximum(np.minimum(head, sides), hem)
  alpha = _aa(body, size)
  shade = 0.86 + 0.14 * np.clip(-(x + y) * 0.6, 0.0, 1.0)
  for ch in range(3):
    rgb[..., ch] = np.where(body < 1, shade, 0.0)
  rgb[..., 2] = np.where(body < 1, np.minimum(shade + 0.04, 1.0), 0.0)  # cool cast
  for ex in (-0.10, 0.16):
    eye = _ellipse(x, y, ex, -0.30, 0.065, 0.10)
    rgb[eye < 0] = (0.12, 0.12, 0.18)
  mouth = _ellipse(x, y, 0.04, -0.08, 0.05, 0.07)
  rgb[mouth < 0] = (0.12, 0.12, 0.18)
  return np.clip(rgb, 0.0, 1.0), np.clip(alpha, 0.0, 1.0)


def _turkey(size: int):
  """Colored: cartoon turkey — banded fan tail, round body, wattle."""
  x, y = _grid(size)
  rgb = np.zeros((size, size, 3), dtype=np.float32)
  alpha = np.zeros((size, size), dtype=np.float32)
  fan_bands = ((0.62, (0.72, 0.18, 0.10)), (0.50, (0.85, 0.48, 0.12)), (0.38, (0.92, 0.72, 0.22)))
  ang = np.arctan2(-(y + 0.10), x)
  in_fan_ang = (ang > 0.15) & (ang < math.pi - 0.15)
  r = np.hypot(x, y + 0.10)
  for radius, col in fan_bands:
    band = in_fan_ang & (r < radius)
    rgb[band] = col
    alpha = np.maximum(alpha, band.astype(np.float32))
  body = _ellipse(x, y, 0.0, 0.22, 0.34, 0.30)
  alpha = np.maximum(alpha, _aa(body, size))
  rgb[body < 0] = (0.42, 0.26, 0.14)
  head = _circle(x, y, 0.24, -0.10, 0.13)
  neck = _segment(x, y, 0.16, 0.14, 0.24, -0.08, 0.09)
  for part in (neck, head):
    alpha = np.maximum(alpha, _aa(part, size))
    rgb[part < 0] = (0.48, 0.30, 0.16)
  beak = _tri_sdf(x, y, (0.35, -0.12), (0.46, -0.07), (0.35, -0.03))
  alpha = np.maximum(alpha, _aa(beak.astype(np.float32), size))
  rgb[beak < 0] = (0.95, 0.75, 0.25)
  wattle = _ellipse(x, y, 0.33, 0.02, 0.045, 0.09)
  alpha = np.maximum(alpha, _aa(wattle, size))
  rgb[wattle < 0] = (0.80, 0.15, 0.15)
  eye = _circle(x, y, 0.27, -0.13, 0.028)
  rgb[eye < 0] = (0.08, 0.08, 0.08)
  return np.clip(rgb, 0.0, 1.0), np.clip(alpha, 0.0, 1.0)


def _bunny(size: int):
  """Colored: hopping bunny silhouette — long ears back, cotton tail."""
  x, y = _grid(size)
  rgb = np.zeros((size, size, 3), dtype=np.float32)
  alpha = np.zeros((size, size), dtype=np.float32)
  fur = (0.88, 0.86, 0.90)
  body = _ellipse(x, y, -0.05, 0.18, 0.40, 0.28)
  head = _circle(x, y, 0.34, -0.06, 0.19)
  ear1 = _ellipse(*_rot(x - 0.28, y + 0.34, -0.55), 0.0, 0.0, 0.07, 0.30)
  ear2 = _ellipse(*_rot(x - 0.16, y + 0.30, -0.85), 0.0, 0.0, 0.07, 0.28)
  leg = _ellipse(x, y, -0.28, 0.36, 0.20, 0.12)
  for part in (body, head, ear1, ear2, leg):
    alpha = np.maximum(alpha, _aa(part, size))
    rgb[part < 0] = fur
  inner = _ellipse(*_rot(x - 0.28, y + 0.34, -0.55), 0.0, 0.02, 0.035, 0.20)
  rgb[inner < 0] = (0.95, 0.72, 0.78)
  tail = _circle(x, y, -0.46, 0.10, 0.09)
  alpha = np.maximum(alpha, _aa(tail, size))
  rgb[tail < 0] = (0.98, 0.98, 1.0)
  eye = _circle(x, y, 0.40, -0.10, 0.028)
  rgb[eye < 0] = (0.10, 0.10, 0.12)
  nose = _circle(x, y, 0.52, -0.02, 0.025)
  rgb[nose < 0] = (0.90, 0.55, 0.60)
  return np.clip(rgb, 0.0, 1.0), np.clip(alpha, 0.0, 1.0)


def _winged_heart(size: int):
  """Colored: red heart with white wings — valentine courier."""
  x, y = _grid(size)
  rgb = np.zeros((size, size, 3), dtype=np.float32)
  alpha = np.zeros((size, size), dtype=np.float32)
  for sgn in (-1.0, 1.0):     # wings: three stacked feather arcs each side
    for k, (wy, wl) in enumerate(((-0.18, 0.44), (-0.06, 0.36), (0.06, 0.26))):
      wing = _ellipse(*_rot(x * sgn - 0.42, y + 0.10 - wy * 0.3, 0.45 + k * 0.12), 0.0, wy, wl, 0.09)
      alpha = np.maximum(alpha, _aa(wing, size))
      rgb[wing < 0] = (0.94, 0.94, 0.97)
  hx, hy = x * 1.9, -y * 1.9 + 0.25
  f = (hx * hx + hy * hy - 0.75) ** 3 - hx * hx * hy * hy * hy
  heart = np.where(f < 0, -1.0, 1.0).astype(np.float32)
  am = _aa(heart, size)
  alpha = np.maximum(alpha, am)
  hr = np.clip(0.80 - 0.25 * np.hypot(x + 0.10, y + 0.12), 0.35, 1.0)
  mask = am > 0.5
  rgb[mask] = 0.0
  rgb[..., 0] = np.where(mask, 0.85 * hr + 0.15, rgb[..., 0])
  rgb[..., 1] = np.where(mask, 0.10 * hr, rgb[..., 1])
  rgb[..., 2] = np.where(mask, 0.14 * hr, rgb[..., 2])
  return np.clip(rgb, 0.0, 1.0), np.clip(alpha, 0.0, 1.0)


def _pot_gold(size: int):
  """Colored: leprechaun's pot with a mound of coins."""
  x, y = _grid(size)
  rgb = np.zeros((size, size, 3), dtype=np.float32)
  alpha = np.zeros((size, size), dtype=np.float32)
  pot = np.maximum(_circle(x, y, 0.0, 0.10, 0.46), -(y - -0.08))    # bowl below rim line
  rim = _ellipse(x, y, 0.0, -0.08, 0.50, 0.09)
  mound = _ellipse(x, y, 0.0, -0.16, 0.40, 0.16)
  alpha = np.maximum.reduce([_aa(pot, size), _aa(rim, size), _aa(mound, size)])
  shade = 0.30 + 0.14 * np.clip(-(x + y), 0.0, 1.0)
  for ch in range(3):
    rgb[..., ch] = np.where(pot < 1, shade, rgb[..., ch])
  rgb[..., 2] = np.where(pot < 1, shade + 0.05, rgb[..., 2])  # cool iron cast
  rgb[rim < 0] = (0.24, 0.24, 0.30)
  gold_shade = (0.88 + 0.12 * np.clip(-(x * 0.5 + y), 0.0, 1.0))
  gm = mound < 0
  rgb[..., 0] = np.where(gm, 0.92 * gold_shade, rgb[..., 0])
  rgb[..., 1] = np.where(gm, 0.74 * gold_shade, rgb[..., 1])
  rgb[..., 2] = np.where(gm, 0.22 * gold_shade, rgb[..., 2])
  rng = np.random.default_rng(3)
  for _ in range(7):    # coin glints
    cx, cy = rng.uniform(-0.30, 0.30), rng.uniform(-0.28, -0.10)
    coin = _circle(x, y, cx, cy, 0.045)
    rgb[coin < 0] = (1.0, 0.90, 0.45)
  return np.clip(rgb, 0.0, 1.0), np.clip(alpha, 0.0, 1.0)


def _rocket(size: int):
  """Colored: firework rocket climbing — red nose, white body, flame."""
  x, y = _grid(size)
  rx, ry = _rot(x, y, 0.5)     # tilted like it's climbing forward
  rgb = np.zeros((size, size, 3), dtype=np.float32)
  alpha = np.zeros((size, size), dtype=np.float32)
  body = _ellipse(rx, ry, 0.0, 0.05, 0.16, 0.46)
  alpha = np.maximum(alpha, _aa(body, size))
  rgb[body < 0] = (0.93, 0.93, 0.96)
  nose = ry + 0.28
  nm = (body < 0) & (nose < 0)
  rgb[nm] = (0.78, 0.14, 0.16)
  for sgn in (-1.0, 1.0):
    fin = _tri_sdf(rx * sgn, ry, (0.10, 0.28), (0.30, 0.55), (0.10, 0.50))
    alpha = np.maximum(alpha, _aa(fin.astype(np.float32), size))
    rgb[fin < 0] = (0.78, 0.14, 0.16)
  window = _circle(rx, ry, 0.0, -0.05, 0.07)
  rgb[window < 0] = (0.35, 0.55, 0.85)
  flame = _ellipse(rx, ry, 0.0, 0.62, 0.09, 0.14)
  fg = np.exp(-((np.hypot(rx, ry - 0.62) / 0.22) ** 2)) * 0.9
  alpha = np.maximum(alpha, np.maximum(_aa(flame, size), fg))
  fm = flame < 0
  rgb[..., 0] = np.where(fm, 1.0, np.where(fg > 0.15, np.maximum(rgb[..., 0], fg), rgb[..., 0]))
  rgb[..., 1] = np.where(fm, 0.78, np.where(fg > 0.15, np.maximum(rgb[..., 1], fg * 0.6), rgb[..., 1]))
  rgb[..., 2] = np.where(fm, 0.25, rgb[..., 2])
  return np.clip(rgb, 0.0, 1.0), np.clip(alpha, 0.0, 1.0)


def _sombrero(size: int):
  """Colored: sombrero — wide swooping brim, tall crown, festive band."""
  x, y = _grid(size)
  rgb = np.zeros((size, size, 3), dtype=np.float32)
  alpha = np.zeros((size, size), dtype=np.float32)
  straw = (0.88, 0.72, 0.38)
  brim = _ellipse(x, y + 0.06 * np.cos(x * 4.0), 0.0, 0.22, 0.72, 0.14)
  crown = np.maximum(_ellipse(x, y, 0.0, 0.02, 0.30, 0.38), y - 0.22)
  tip = _ellipse(x, y, 0.0, -0.34, 0.16, 0.09)
  for part in (brim, crown, tip):
    alpha = np.maximum(alpha, _aa(part, size))
    rgb[part < 0] = straw
  shade = (crown < 0) & (x > 0.10)
  rgb[shade] = (0.78, 0.60, 0.28)
  band = (crown < 0) & (np.abs(y - 0.12) < 0.06)
  bx = ((x * 6.0).astype(int) % 2 == 0)
  rgb[band & bx] = (0.75, 0.15, 0.18)
  rgb[band & ~bx] = (0.15, 0.50, 0.30)
  edge = (np.abs(brim) < 0.03)
  rgb[edge] = (0.75, 0.15, 0.18)
  return np.clip(rgb, 0.0, 1.0), np.clip(alpha, 0.0, 1.0)


def _jester_hat(size: int):
  """Colored: three-pronged jester hat with belled tips."""
  x, y = _grid(size)
  rgb = np.zeros((size, size, 3), dtype=np.float32)
  alpha = np.zeros((size, size), dtype=np.float32)
  prongs = (((-0.62, -0.44), (0.55, 0.10, 0.65), (-0.02, 0.10)),
            ((0.0, -0.62), (0.90, 0.70, 0.20), (0.0, 0.05)),
            ((0.62, -0.40), (0.16, 0.62, 0.60), (0.02, 0.10)))
  base_y = 0.30
  for (tipx, tipy), col, (bx_off, _) in prongs:
    prong = _tri_sdf(x, y, (tipx, tipy), (bx_off + 0.30, base_y), (bx_off - 0.30, base_y))
    alpha = np.maximum(alpha, _aa(prong.astype(np.float32), size))
    rgb[prong < 0] = col
    bell = _circle(x, y, tipx, tipy, 0.075)
    alpha = np.maximum(alpha, _aa(bell, size))
    rgb[bell < 0] = (0.95, 0.85, 0.40)
  brim = _ellipse(x, y, 0.0, base_y + 0.04, 0.58, 0.10)
  alpha = np.maximum(alpha, _aa(brim, size))
  rgb[brim < 0] = (0.55, 0.20, 0.60)
  return np.clip(rgb, 0.0, 1.0), np.clip(alpha, 0.0, 1.0)


def _usflag(size: int):
  """Colored: American flag on a short pole, gently waving. 13 stripes, starred canton."""
  x, y = _grid(size)
  red = (0.76, 0.12, 0.16)
  white = (0.93, 0.93, 0.96)
  navy = (0.13, 0.18, 0.38)
  rgb = np.zeros((size, size, 3), dtype=np.float32)
  alpha = np.zeros((size, size), dtype=np.float32)
  # pole down the left edge
  pole = _segment(x, y, -0.78, -0.72, -0.78, 0.85, 0.035)
  alpha = np.maximum(alpha, _aa(pole, size))
  rgb[pole < 0] = (0.62, 0.65, 0.72)
  rgb[_circle(x, y, -0.78, -0.78, 0.06) < 0] = (0.85, 0.72, 0.35)   # gold finial
  alpha = np.maximum(alpha, _aa(_circle(x, y, -0.78, -0.78, 0.06), size))
  # flag body: rows waved horizontally by a sine in y
  wave = 0.06 * np.sin(y * 7.0)
  fx = x - wave                     # flag-local x
  in_flag = (fx > -0.74) & (fx < 0.72) & (y > -0.72) & (y < 0.22)
  fy = (y + 0.72) / 0.94            # 0 at flag top .. 1 at bottom
  stripe = (fy * 13).astype(int) % 2
  for ch, (rv, wv) in enumerate(zip(red, white, strict=True)):
    rgb[..., ch] = np.where(in_flag, np.where(stripe == 0, rv, wv), rgb[..., ch])
  canton = in_flag & (fx < -0.15) & (fy < 7.0 / 13.0)
  for ch, cv in enumerate(navy):
    rgb[..., ch] = np.where(canton, cv, rgb[..., ch])
  # star dots on a staggered grid inside the canton
  gx = ((fx + 0.74) * 14.0).astype(int)
  gy = (fy * 14.0).astype(int)
  dots = canton & ((gx + gy) % 2 == 0) & (np.abs((fx + 0.74) * 14.0 - gx - 0.5) < 0.30) \
         & (np.abs(fy * 14.0 - gy - 0.5) < 0.30)
  for ch, wv in enumerate(white):
    rgb[..., ch] = np.where(dots, wv, rgb[..., ch])
  alpha = np.maximum(alpha, in_flag.astype(np.float32))
  # gentle shading down the waves for cloth depth
  shade = np.where(in_flag, 0.88 + 0.12 * np.cos(y * 7.0), 1.0)
  rgb *= shade[..., None]
  return np.clip(rgb, 0.0, 1.0), np.clip(alpha, 0.0, 1.0)


# ---------------------------------------------------------------------------- registry
def render_sprite(name: str, size: int = SPRITE_SIZE):
  """(rgb float32 [h,w,3], alpha float32 [h,w]) in 0..1 — GL-free, unit-testable."""
  if name.startswith("snowflake"):
    variant = int(name[-1]) if name[-1].isdigit() else 0
    v, a = _snowflake(size, variant)
  elif name == "heart":
    v, a = _heart(size)
  elif name == "leaf":
    v, a = _leaf(size)
  elif name == "petal":
    v, a = _petal(size)
  elif name == "clover":
    v, a = _clover(size)
  elif name == "bat":
    v, a = _bat(size, True)
  elif name == "bat2":
    v, a = _bat(size, False)
  elif name == "spark":
    v, a = _spark(size)
  elif name == "bulb":
    v, a = _bulb(size)
  elif name == "pennant":
    v, a = _pennant(size)
  elif name == "star5":
    v, a = _star5(size)
  elif name == "orb":
    v, a = _orb(size)
  elif name == "garland":
    return _garland_tile(size)
  elif name == "poppy":
    return _poppy(size)
  elif name == "usflag":
    return _usflag(size)
  elif name == "sleigh":
    return _sleigh(size)
  elif name == "ghost":
    return _ghost(size)
  elif name == "turkey":
    return _turkey(size)
  elif name == "bunny":
    return _bunny(size)
  elif name == "winged_heart":
    return _winged_heart(size)
  elif name == "pot_gold":
    return _pot_gold(size)
  elif name == "rocket":
    return _rocket(size)
  elif name == "sombrero":
    return _sombrero(size)
  elif name == "jester_hat":
    return _jester_hat(size)
  else:
    raise KeyError(name)
  rgb = np.repeat(np.clip(v, 0.0, 1.0)[..., None], 3, axis=2)
  return rgb, a




# ---------------------------------------------------------------------------- backdrops
BACKDROP_STYLES = ("pines", "skyline", "hills", "mesas", "spooky", "zigzag",
                   "hearts", "eggs", "clover_hills", "farmland", "capitol", "flags")


def render_backdrop(style: str, width: int = 768, height: int = 160, seed: int = 3):
  """Horizon silhouette band (rgb, alpha), tileable horizontally. White/grey shades —
  tinted at draw time from the pack palette. GL-free."""
  rng = np.random.default_rng(seed + hash(style) % 1000)
  xs = np.arange(width)
  prof = np.zeros(width, dtype=np.float32)
  flag_poles: list[tuple[int, float]] = []

  if style == "pines":
    x = 0
    while x < width:
      w = int(rng.uniform(18, 42))
      h = rng.uniform(0.45, 0.95)
      for i in range(w):
        if x + i < width:
          prof[x + i] = max(prof[x + i], h * (1.0 - abs(i - w / 2) / (w / 2)))
      x += int(w * rng.uniform(0.55, 0.8))
  elif style == "skyline":
    x = 0
    while x < width:
      w = int(rng.uniform(24, 70))
      h = rng.uniform(0.25, 0.95)
      prof[x:x + w] = h
      x += w + int(rng.uniform(2, 10))
  elif style == "hills":
    for f, a in ((1.0, 0.45), (2.3, 0.25), (4.7, 0.12)):
      ph = rng.uniform(0, 2 * math.pi)
      prof += a * (0.5 + 0.5 * np.sin(2 * math.pi * f * xs / width + ph))
    prof = np.clip(prof, 0.05, 1.0)
  elif style == "mesas":
    prof[:] = 0.12
    x = 0
    while x < width:
      w = int(rng.uniform(60, 140))
      h = rng.uniform(0.4, 0.85)
      ramp = int(w * 0.18)
      for i in range(w):
        if x + i >= width:
          break
        t = 1.0
        if i < ramp:
          t = i / ramp
        elif i > w - ramp:
          t = (w - i) / ramp
        prof[x + i] = max(prof[x + i], h * t)
      x += w + int(rng.uniform(40, 110))
    placed = 0  # saguaros only in the flats between mesas: flat-topped trunk + two arms
    for _ in range(24):
      if placed >= 5:
        break
      cx = int(rng.uniform(12, width - 12))
      if float(np.max(prof[cx - 10:cx + 11])) > 0.20:
        continue
      placed += 1
      base = 0.12
      h = rng.uniform(0.34, 0.46)
      for off, frac, hw in ((0, 1.0, 2), (-6, 0.58, 1), (6, 0.66, 1)):
        for i in range(-hw, hw + 1):
          xx = cx + off + i
          if 0 <= xx < width:
            round_top = 1.0 - 0.12 * (abs(i) / max(hw, 1)) ** 2
            prof[xx] = max(prof[xx], base + h * frac * round_top)
  elif style == "spooky":
    for f, a in ((0.8, 0.35), (2.1, 0.18)):
      ph = rng.uniform(0, 2 * math.pi)
      prof += a * (0.5 + 0.5 * np.sin(2 * math.pi * f * xs / width + ph))
    for _ in range(6):  # bare gnarled trees: tapered trunk + stepped branch tiers
      x = int(rng.uniform(10, width - 24))
      h = rng.uniform(0.55, 0.95)
      base = float(prof[x])
      for i in range(-3, 4):  # trunk tapers upward
        if 0 <= x + i < width:
          prof[x + i] = max(prof[x + i], base + h * 0.55 * (1.0 - abs(i) / 4.5))
      for tier, frac in ((1, 0.42), (2, 0.30)):  # branch tiers reach sideways
        span = int(6 + tier * 4)
        for sgn in (-1, 1):
          bx = x + sgn * span
          for i in range(-2, 3):
            if 0 <= bx + i < width:
              prof[bx + i] = max(prof[bx + i], base + h * frac * (1.0 - abs(i) / 3.5))
  elif style == "hearts":
    # heart summits: two round lobes with a soft cleft, clear gaps between hearts
    x = 0
    while x < width:
      w = int(rng.uniform(110, 150))
      h = rng.uniform(0.5, 0.85)
      c = w / 2.0
      for i in range(w):
        if x + i >= width:
          break
        t = (i - c) / c
        m = 0.0
        for lc in (-0.42, 0.42):        # big round lobes, wide overlap at the middle
          d2 = 1.0 - ((t - lc) / 0.56) ** 2
          if d2 > 0:
            m = max(m, math.sqrt(d2))
        cleft = 1.0 - 0.30 * max(0.0, 1.0 - (abs(t) / 0.20) ** 2)   # dip only near center
        prof[x + i] = max(prof[x + i], h * m * cleft)
      x += int(w * rng.uniform(1.15, 1.45))
  elif style == "eggs":
    # a clutch of giant eggs standing on the horizon, overlapping
    x = 0
    while x < width:
      w = int(rng.uniform(70, 130))
      h = rng.uniform(0.45, 0.9)
      c = w / 2.0
      for i in range(w):
        if x + i >= width:
          break
        t = (i - c) / c
        d2 = 1.0 - t * t
        if d2 > 0:
          prof[x + i] = max(prof[x + i], h * d2 ** 0.72)
      x += int(w * rng.uniform(0.55, 0.75))
  elif style == "clover_hills":
    # triple-lobed shamrock mounds: tall center lobe, two lower shoulders
    x = 0
    while x < width:
      w = int(rng.uniform(110, 160))
      h = rng.uniform(0.42, 0.78)
      c = w / 2.0
      for i in range(w):
        if x + i >= width:
          break
        t = (i - c) / c
        m = 0.0
        for lc, lh, lr in ((-0.58, 0.70, 0.42), (0.0, 1.0, 0.46), (0.58, 0.70, 0.42)):
          d2 = 1.0 - ((t - lc) / lr) ** 2
          if d2 > 0:
            m = max(m, lh * math.sqrt(d2))
        prof[x + i] = max(prof[x + i], h * m)
      x += int(w * rng.uniform(0.82, 1.05))
  elif style == "farmland":
    # gentle field swells, haystack domes, and two barns with silos
    for f, a in ((0.7, 0.18), (1.9, 0.10)):
      ph = rng.uniform(0, 2 * math.pi)
      prof += a * (0.5 + 0.5 * np.sin(2 * math.pi * f * xs / width + ph))
    prof += 0.08
    for _ in range(5):
      x0 = int(rng.uniform(0, width - 44))
      w = int(rng.uniform(24, 40))
      h = rng.uniform(0.14, 0.24)
      c = w / 2.0
      base = float(np.max(prof[x0:x0 + w]))
      for i in range(w):
        t = (i - c) / c
        d2 = 1.0 - t * t
        if d2 > 0:
          prof[x0 + i] = max(prof[x0 + i], base + h * math.sqrt(d2))
    for bx in (int(width * 0.20), int(width * 0.64)):
      bw = int(rng.uniform(54, 68))
      body = rng.uniform(0.26, 0.32)
      base = float(np.max(prof[bx:bx + bw])) * 0.5
      c = bw / 2.0
      for i in range(bw):
        if bx + i >= width:
          break
        t = abs(i - c) / c
        prof[bx + i] = max(prof[bx + i], base + body + 0.24 * (1.0 - t))  # straight gable roof
      sx = bx + bw + 5   # silo: slim flat-capped tower
      for i in range(11):
        if sx + i >= width:
          break
        t = abs(i - 5.0) / 5.0
        prof[sx + i] = max(prof[sx + i], base + body + 0.22 + 0.05 * (1.0 - t * t))
  elif style == "capitol":
    # low government skyline, central rotunda dome, obelisk monument
    x = 0
    while x < width:
      w = int(rng.uniform(30, 66))
      h = rng.uniform(0.20, 0.42)
      prof[x:x + w] = h
      x += w + int(rng.uniform(4, 12))
    cx, dw, dh, base = width // 2, 116, 0.30, 0.34
    for i in range(dw):
      xx = cx - dw // 2 + i
      t = (i - dw / 2.0) / (dw / 2.0)
      d2 = 1.0 - t * t
      if 0 <= xx < width and d2 > 0:
        prof[xx] = max(prof[xx], base + dh * math.sqrt(d2))
    for i in range(-4, 5):   # lantern atop the dome
      if 0 <= cx + i < width:
        prof[cx + i] = max(prof[cx + i], base + dh + 0.11 * (1.0 - abs(i) / 5.0))
    ox = width // 4          # obelisk: tapered shaft, pyramidion tip
    for i in range(-7, 8):
      if 0 <= ox + i < width:
        t = abs(i) / 7.0
        prof[ox + i] = max(prof[ox + i], 0.86 * (1.0 - t * t * 0.55))
  elif style == "flags":
    # memorial hillside: soft lawn swells and a rank of flagpoles (flags stamped below)
    for f, a in ((0.9, 0.14), (2.2, 0.07)):
      ph = rng.uniform(0, 2 * math.pi)
      prof += a * (0.5 + 0.5 * np.sin(2 * math.pi * f * xs / width + ph))
    prof += 0.06
    for k in range(4):
      px = int(width * (0.10 + 0.26 * k) + rng.uniform(-8, 8))
      h = rng.uniform(0.74, 0.88)
      flag_poles.append((px, h))
      for i in range(-1, 2):
        if 0 <= px + i < width:
          prof[px + i] = max(prof[px + i], h)
  else:  # zigzag
    period = 48
    prof = 0.15 + 0.7 * np.abs(((xs / period) % 2.0) - 1.0).astype(np.float32)

  # solid ground base so the band fills its rect instead of stretching empty sky
  prof = 0.26 + 0.74 * np.clip(prof, 0.0, 1.0) * 0.92
  yy = np.linspace(1.0, 0.0, height, dtype=np.float32)[:, None]   # row 0 = texture top
  # opaque below the profile line, soft 2px edge at the top of the silhouette
  alpha = np.clip((prof[None, :] - yy) * height * 0.5, 0.0, 1.0)
  # mild depth shading, kept bright enough to survive dark-tint packs
  v = (0.72 + 0.28 * yy).astype(np.float32) * np.ones((height, width), dtype=np.float32)

  if style in ("skyline", "capitol"):
    # sparse lit windows: full-white texels read as the tint at full brightness
    win_rng = np.random.default_rng(7)
    fill = alpha > 0.6
    for wy in range(6, height - 3, 9):
      for wx in range(4, width - 3, 7):
        if fill[wy, wx] and fill[wy + 2, wx + 1] and win_rng.random() < 0.28:
          v[wy:wy + 2, wx:wx + 2] = 1.0
  elif style == "flags":
    # Self-colored (pack tint should be #FFFFFF): slate hillside, US flags on the poles.
    hill = np.array([0.30, 0.38, 0.55], dtype=np.float32)
    red = np.array([0.76, 0.12, 0.16], dtype=np.float32)
    white = np.array([0.93, 0.93, 0.96], dtype=np.float32)
    navy = np.array([0.13, 0.18, 0.38], dtype=np.float32)
    rgb_f = v[..., None] * hill[None, None, :]
    for px, raw_h in flag_poles:
      top = 0.26 + 0.74 * min(raw_h, 1.0) * 0.92   # same remap as the profile
      fl = int(width * 0.058)             # flag length
      fh = 0.13                           # flag height in profile units
      for row in range(height):
        ry = float(yy[row, 0])
        if top - fh <= ry <= top - 0.015:
          rel = (top - 0.015 - ry) / (fh - 0.015)          # 0 at flag top .. 1 at bottom
          wave = int(2.5 * math.sin(row * 0.9 + px))
          x0, x1 = px + 2, min(width - 1, px + 2 + fl + wave)
          if x1 <= x0:
            continue
          stripe = red if int(rel * 7) % 2 == 0 else white  # 7 visible stripes
          rgb_f[row, x0:x1] = stripe
          alpha[row, x0:x1] = 1.0
          if rel < 0.54:                                    # canton with star dots
            c1 = min(x1, x0 + int(fl * 0.42))
            rgb_f[row, x0:c1] = navy
            if row % 2 == 0:
              for sx in range(x0 + 1, c1 - 1, 3):
                rgb_f[row, sx] = white
      # pole picked out in light grey, only above the hill surface beside it
      hill_ref = float(prof[min(px + 5, width - 1)])
      pole_rows = (alpha[:, px] > 0.5) & (yy[:, 0] > hill_ref + 0.01)
      for i in range(-1, 2):
        if 0 <= px + i < width:
          rgb_f[pole_rows, px + i] = np.array([0.62, 0.65, 0.72], dtype=np.float32)
    rgb = np.clip(rgb_f, 0.0, 1.0)
    return rgb, alpha.astype(np.float32)

  rgb = np.repeat(v[..., None], 3, axis=2)
  return rgb, alpha.astype(np.float32)


def render_celestial(kind: str, size: int = 192):
  """(rgb, alpha): big soft moon (with craters) or sun disc with glow — tinted at draw."""
  x, y = _grid(size)
  r = np.hypot(x, y)
  if kind == "moon":
    disc = _aa(r - 0.55, size)
    v = np.full((size, size), 0.95, dtype=np.float32)
    rng = np.random.default_rng(5)
    for _ in range(6):
      cx, cy = rng.uniform(-0.35, 0.35), rng.uniform(-0.35, 0.35)
      cr = rng.uniform(0.05, 0.14)
      crater = np.hypot(x - cx, y - cy) < cr
      v = np.where(crater, v * 0.82, v)
    glow = np.exp(-((r - 0.55).clip(0) / 0.35) ** 2) * 0.45
    alpha = np.clip(disc + glow, 0.0, 1.0)
    v = np.where(disc > 0.5, v, 1.0)
  else:  # sun
    core = _aa(r - 0.42, size)
    glow = np.exp(-((r - 0.42).clip(0) / 0.5) ** 2) * 0.6
    alpha = np.clip(core + glow, 0.0, 1.0)
    v = np.ones((size, size), dtype=np.float32)
  rgb = np.repeat(v[..., None] if v.ndim == 2 else v, 3, axis=2)
  return rgb, alpha.astype(np.float32)


def render_rainbow(width: int = 640, height: int = 320):
  """(rgb, alpha): a soft arc rainbow, self-colored."""
  ax = np.linspace(-1.0, 1.0, width, dtype=np.float32)
  ay = np.linspace(0.0, 1.0, height, dtype=np.float32)
  x, y = np.meshgrid(ax, ay)
  # arc centered below the strip's bottom edge so the visible part is a proper rainbow
  r = np.hypot(x * 1.05, (1.0 - y) * 0.9 + 0.12)
  bands = [(0.98, (230, 60, 60)), (0.925, (240, 150, 50)), (0.87, (245, 220, 80)),
           (0.815, (90, 200, 90)), (0.76, (80, 140, 230)), (0.705, (150, 90, 200))]
  rgb = np.zeros((height, width, 3), dtype=np.float32)
  alpha = np.zeros((height, width), dtype=np.float32)
  for r0, c in bands:
    band = np.clip(1.0 - np.abs(r - r0) / 0.028, 0.0, 1.0)
    for i in range(3):
      rgb[..., i] = np.where(band > alpha, c[i] / 255.0, rgb[..., i])
    alpha = np.maximum(alpha, band * 0.85)
  # legs dissolve into horizon mist instead of hard-clipping at the strip's bottom edge
  fade = np.clip((1.0 - y) / 0.26, 0.0, 1.0)
  alpha *= fade * fade * (3.0 - 2.0 * fade)
  return rgb, alpha


SPRITE_NAMES = frozenset({
  "snowflake", "heart", "leaf", "petal", "clover", "bat", "bat2", "spark",
  "bulb", "pennant", "garland", "star5", "orb", "poppy", "usflag",
  "sleigh", "ghost", "turkey", "bunny", "winged_heart", "pot_gold", "rocket",
  "sombrero", "jester_hat",
})


def _png_bytes(rgb: np.ndarray, alpha: np.ndarray) -> bytes:
  """Minimal in-memory PNG encoder (RGBA8) — avoids per-pixel GL image writes."""
  h, w = alpha.shape
  rgba = np.empty((h, w, 4), dtype=np.uint8)
  rgba[..., :3] = np.clip(rgb * 255.0, 0, 255).astype(np.uint8)
  rgba[..., 3] = np.clip(alpha * 255.0, 0, 255).astype(np.uint8)
  raw = b"".join(b"\x00" + rgba[i].tobytes() for i in range(h))

  def chunk(tag: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data))

  ihdr = struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0)
  return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
          + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


@lru_cache(maxsize=None)
def build_backdrop(style: str):
  """Backdrop band as a texture (bilinear). Requires GL."""
  import pyray as rl
  rgb, a = render_backdrop(style)
  png = _png_bytes(rgb, a)
  img = rl.load_image_from_memory(".png", png, len(png))
  tex = rl.load_texture_from_image(img)
  rl.unload_image(img)
  rl.set_texture_filter(tex, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
  return tex


@lru_cache(maxsize=None)
def build_celestial(kind: str):
  import pyray as rl
  rgb, a = render_celestial(kind)
  png = _png_bytes(rgb, a)
  img = rl.load_image_from_memory(".png", png, len(png))
  tex = rl.load_texture_from_image(img)
  rl.unload_image(img)
  rl.set_texture_filter(tex, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
  return tex


@lru_cache(maxsize=None)
def build_rainbow():
  import pyray as rl
  rgb, a = render_rainbow()
  png = _png_bytes(rgb, a)
  img = rl.load_image_from_memory(".png", png, len(png))
  tex = rl.load_texture_from_image(img)
  rl.unload_image(img)
  rl.set_texture_filter(tex, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
  return tex


@lru_cache(maxsize=None)
def build_sprite(name: str, size: int = SPRITE_SIZE):
  """Sprite as an rl.Texture (bilinear — these are smooth art, not pixel art).
  Requires an active GL context. Cached per (name, size)."""
  import pyray as rl
  rgb, a = render_sprite(name, size)
  png = _png_bytes(rgb, a)
  img = rl.load_image_from_memory(".png", png, len(png))
  tex = rl.load_texture_from_image(img)
  rl.unload_image(img)
  rl.set_texture_filter(tex, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
  return tex
