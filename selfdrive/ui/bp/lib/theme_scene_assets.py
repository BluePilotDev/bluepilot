"""BluePilot: procedurally rendered sprites for theme-pack scenes.

Every sprite is generated from 2D signed-distance fields in numpy at load time —
no bundled images, smooth antialiased edges, proper shading. Most sprites render
GRAYSCALE + alpha so a single texture serves every pack (per-particle color via
draw_texture_pro tint); a few decor sprites (pumpkin, garland tile) bake their own
colors because they are multi-color by nature.

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
  stem_w = 0.035 + 0.008 * rng.random()
  d = _segment(fx, fy, 0.02, 0.0, 0.92, 0.0, stem_w)
  # side branches: at each radius, a V pair angled forward, shrinking outward
  for br in (0.30, 0.52, 0.72):
    ln = (0.30 - 0.22 * br) * (0.9 + 0.2 * rng.random())
    bx = br + ln * math.cos(1.05)
    by = ln * math.sin(1.05)
    d = np.minimum(d, _segment(fx, fy, br, 0.0, bx, by, stem_w * 0.75))
  d = np.minimum(d, _circle(fx, fy, 0.0, 0.0, 0.10))       # hub
  d = np.minimum(d, _circle(fx, fy, 0.92, 0.0, 0.055))     # tip crystal
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
  stem = _segment(lx, ly, 0.0, 0.72, -0.06, 1.00, 0.030)
  stem = np.minimum(stem, _segment(lx, ly, 0.0, 0.72, 0.0, 0.55, 0.030))
  alpha = np.maximum(_aa(body, size), _aa(stem, size))
  inside = body < 0
  v = 0.58 + 0.42 * np.clip(0.5 - ly * 0.65, 0.0, 1.0)
  rib = _segment(lx, ly, 0.0, -0.78, 0.0, 0.72, 0.014)
  v = np.where((rib < 0) & inside, v * 0.52, v)
  for t, ln in ((-0.50, 0.20), (-0.18, 0.28), (0.16, 0.30), (0.45, 0.24)):
    for sgn in (-1.0, 1.0):
      vein = _segment(lx, ly, 0.0, t, sgn * ln, t - ln * 0.9, 0.009)
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
  crease = _segment(x - bend, y, 0.0, -0.75, 0.0, 0.65, 0.010)
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
  stem = np.minimum(_segment(x, cy, 0.0, -0.05, 0.06, 0.45, 0.032),
                    _segment(x, cy, 0.06, 0.45, 0.22, 0.80, 0.032))
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


def _pumpkin(size: int):
  """Colored: ribbed body, stem, carved face with candle glow."""
  x, y = _grid(size)
  body = None
  rib_edge = None
  for cx in (-0.42, -0.21, 0.0, 0.21, 0.42):
    e = _ellipse(x, y, cx * 0.8, 0.12, 0.34, 0.55)
    body = e if body is None else np.minimum(body, e)
    rib_edge = np.abs(e) if rib_edge is None else np.minimum(rib_edge, np.abs(e))
  alpha = _aa(body, size)
  stem = _segment(x, y, 0.02, -0.55, 0.14, -0.82, 0.07)
  alpha = np.maximum(alpha, _aa(stem, size))

  # carved face: triangle eyes + zigzag mouth
  def tri(cx, cy, s):
    return np.maximum(np.abs(x - cx) * 1.7 - (y - cy + s) * 1.0, (y - cy) - s * 0.4)

  face = np.minimum(tri(-0.22, -0.05, 0.16), tri(0.22, -0.05, 0.16))
  mouth = _segment(x, y, -0.34, 0.32, -0.17, 0.24, 0.045)
  for a_, b_ in ((-0.17, 0.24), (0.0, 0.34), (0.17, 0.24)):
    pass
  mouth = np.minimum(mouth, _segment(x, y, -0.17, 0.24, 0.0, 0.34, 0.045))
  mouth = np.minimum(mouth, _segment(x, y, 0.0, 0.34, 0.17, 0.24, 0.045))
  mouth = np.minimum(mouth, _segment(x, y, 0.17, 0.24, 0.34, 0.32, 0.045))
  carved = np.minimum(face, mouth)

  rgb = np.zeros((size, size, 3), dtype=np.float32)
  shade = np.clip(1.0 - rib_edge * 2.2, 0.55, 1.0) * np.clip(0.95 - (y + 0.1) * 0.25, 0.6, 1.0)
  rgb[..., 0] = 0.94 * shade
  rgb[..., 1] = 0.47 * shade
  rgb[..., 2] = 0.10 * shade
  stem_m = (stem < 0)
  rgb[stem_m] = (0.38, 0.45, 0.20)
  carved_m = (carved < 0) & (body < 0)
  rgb[carved_m] = (1.0, 0.85, 0.35)   # candle glow through the cuts
  return rgb, np.clip(alpha, 0.0, 1.0)


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
  elif name == "firefly":
    v, a = _glow(size, 0.12, 0.5)
  elif name == "spark":
    v, a = _spark(size)
  elif name == "bulb":
    v, a = _bulb(size)
  elif name == "pennant":
    v, a = _pennant(size)
  elif name == "pumpkin":
    return _pumpkin(size)
  elif name == "garland":
    return _garland_tile(size)
  else:
    raise KeyError(name)
  rgb = np.repeat(np.clip(v, 0.0, 1.0)[..., None], 3, axis=2)
  return rgb, a


SPRITE_NAMES = frozenset({
  "snowflake", "snowflake1", "snowflake2", "heart", "leaf", "petal", "clover",
  "bat", "bat2", "firefly", "spark", "bulb", "pennant", "pumpkin", "garland",
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
