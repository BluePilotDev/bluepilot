"""
Rad Racer 8-bit theme assets: NES palette, 5x7 pixel font, and pixel-art sprites.

All art is original pixel art inspired by late-80s NES racing games. Sprites and
glyphs are built once as raylib textures (point-filtered so scaling stays crisp)
the first time they are requested, which must happen after the GL window exists.
"""
import pyray as rl

# --- NES-style palette (sampled from reference footage) ---
PURPLE = rl.Color(108, 36, 255, 255)        # gauge cluster body
PURPLE_LIGHT = rl.Color(144, 36, 255, 255)  # cluster accents / wedges
PURPLE_DARK = rl.Color(64, 16, 152, 255)    # cluster shading
GREEN = rl.Color(72, 216, 85, 255)          # HUD text green
GREEN_ROAD = rl.Color(72, 216, 0, 255)      # road line green
TEAL = rl.Color(72, 216, 170, 255)          # sign faces / regen
TAN = rl.Color(252, 216, 170, 255)          # city windows / highlights
BLUE = rl.Color(72, 72, 255, 255)           # buildings / bridge
RED = rl.Color(216, 40, 0, 255)             # ego car body
RED_DARK = rl.Color(124, 8, 0, 255)         # ego car shading
TAIL_RED = rl.Color(176, 32, 0, 255)        # taillights — slightly darker than body red
YELLOW = rl.Color(252, 188, 60, 255)        # lead car body (yellow/teal variants)
BLACK = rl.Color(0, 0, 0, 255)
WHITE = rl.Color(236, 236, 236, 255)
GRAY = rl.Color(116, 116, 116, 255)

# --- 5x7 pixel font ('X' = lit pixel) ---
_FONT_5X7 = {
  '0': ["XXXXX", "X...X", "X..XX", "X.X.X", "XX..X", "X...X", "XXXXX"],
  '1': ["..X..", ".XX..", "..X..", "..X..", "..X..", "..X..", "XXXXX"],
  '2': ["XXXXX", "....X", "....X", "XXXXX", "X....", "X....", "XXXXX"],
  '3': ["XXXXX", "....X", "....X", ".XXXX", "....X", "....X", "XXXXX"],
  '4': ["X...X", "X...X", "X...X", "XXXXX", "....X", "....X", "....X"],
  '5': ["XXXXX", "X....", "X....", "XXXXX", "....X", "....X", "XXXXX"],
  '6': ["XXXXX", "X....", "X....", "XXXXX", "X...X", "X...X", "XXXXX"],
  '7': ["XXXXX", "....X", "....X", "...X.", "..X..", "..X..", "..X.."],
  '8': ["XXXXX", "X...X", "X...X", "XXXXX", "X...X", "X...X", "XXXXX"],
  '9': ["XXXXX", "X...X", "X...X", "XXXXX", "....X", "....X", "XXXXX"],
  'A': [".XXX.", "X...X", "X...X", "XXXXX", "X...X", "X...X", "X...X"],
  'B': ["XXXX.", "X...X", "X...X", "XXXX.", "X...X", "X...X", "XXXX."],
  'C': [".XXXX", "X....", "X....", "X....", "X....", "X....", ".XXXX"],
  'D': ["XXXX.", "X...X", "X...X", "X...X", "X...X", "X...X", "XXXX."],
  'E': ["XXXXX", "X....", "X....", "XXXX.", "X....", "X....", "XXXXX"],
  'F': ["XXXXX", "X....", "X....", "XXXX.", "X....", "X....", "X...."],
  'G': [".XXXX", "X....", "X....", "X..XX", "X...X", "X...X", ".XXXX"],
  'H': ["X...X", "X...X", "X...X", "XXXXX", "X...X", "X...X", "X...X"],
  'I': ["XXXXX", "..X..", "..X..", "..X..", "..X..", "..X..", "XXXXX"],
  'J': ["....X", "....X", "....X", "....X", "X...X", "X...X", ".XXX."],
  'K': ["X...X", "X..X.", "X.X..", "XX...", "X.X..", "X..X.", "X...X"],
  'L': ["X....", "X....", "X....", "X....", "X....", "X....", "XXXXX"],
  'M': ["X...X", "XX.XX", "X.X.X", "X.X.X", "X...X", "X...X", "X...X"],
  'N': ["X...X", "XX..X", "X.X.X", "X..XX", "X...X", "X...X", "X...X"],
  'O': [".XXX.", "X...X", "X...X", "X...X", "X...X", "X...X", ".XXX."],
  'P': ["XXXX.", "X...X", "X...X", "XXXX.", "X....", "X....", "X...."],
  'Q': [".XXX.", "X...X", "X...X", "X...X", "X.X.X", "X..X.", ".XX.X"],
  'R': ["XXXX.", "X...X", "X...X", "XXXX.", "X.X..", "X..X.", "X...X"],
  'S': [".XXXX", "X....", "X....", ".XXX.", "....X", "....X", "XXXX."],
  'T': ["XXXXX", "..X..", "..X..", "..X..", "..X..", "..X..", "..X.."],
  'U': ["X...X", "X...X", "X...X", "X...X", "X...X", "X...X", ".XXX."],
  'V': ["X...X", "X...X", "X...X", "X...X", "X...X", ".X.X.", "..X.."],
  'W': ["X...X", "X...X", "X...X", "X.X.X", "X.X.X", "XX.XX", "X...X"],
  'X': ["X...X", "X...X", ".X.X.", "..X..", ".X.X.", "X...X", "X...X"],
  'Y': ["X...X", "X...X", ".X.X.", "..X..", "..X..", "..X..", "..X.."],
  'Z': ["XXXXX", "....X", "...X.", "..X..", ".X...", "X....", "XXXXX"],
  '/': ["....X", "....X", "...X.", "..X..", ".X...", "X....", "X...."],
  '-': [".....", ".....", ".....", "XXXXX", ".....", ".....", "....."],
  '.': [".....", ".....", ".....", ".....", ".....", ".XX..", ".XX.."],
  ':': [".....", ".XX..", ".XX..", ".....", ".XX..", ".XX..", "....."],
  '>': ["X....", "XX...", "XXX..", "XXXX.", "XXX..", "XX...", "X...."],
  '<': ["....X", "...XX", "..XXX", ".XXXX", "..XXX", "...XX", "....X"],
  ' ': [".....", ".....", ".....", ".....", ".....", ".....", "....."],
}

GLYPH_W = 5
GLYPH_H = 7
GLYPH_SPACING = 1  # blank columns between glyphs, in font pixels

# --- Sprites: rows of palette-key characters, '.' = transparent ---
# Ego car (rear view, wide sports car). Keys: R=red, D=dark red, K=black, L=tail light
_EGO_CAR = [
  "......RRRRRRRRRRRR......",
  ".....RRKKKKKKKKKKRR.....",
  ".....RKKKKKKKKKKKKR.....",
  ".....RKKKKKKKKKKKKR.....",
  "....RRRRRRRRRRRRRRRR....",
  "..RRRRRRRRRRRRRRRRRRRR..",
  ".RRRRRRRRRRRRRRRRRRRRRR.",
  ".RLLRDDDDDDDDDDDDDDRLLR.",
  ".RLLRDDDDDDDDDDDDDDRLLR.",
  ".RRRRRRRRRRRRRRRRRRRRRR.",
  ".KKKKKRRRRRRRRRRRRKKKKK.",
  ".KKKKKDDDDDDDDDDDDKKKKK.",
  ".KKKKK............KKKKK.",
  "..KKK..............KKK..",
]
_EGO_CMAP = {'R': RED, 'D': RED_DARK, 'K': BLACK, 'L': TAIL_RED}

# Lead car (rear view, compact). Keys: B=body, S=body shade, K=tires/window, L=tail light
_LEAD_CAR = [
  "....BBBBBBBB....",
  "...BBKKKKKKBB...",
  "...BBKKKKKKBB...",
  "..BBBBBBBBBBBB..",
  ".BBBBBBBBBBBBBB.",
  ".BLLBBBBBBBBLLB.",
  ".BBBBBBBBBBBBBB.",
  ".BSSSSSSSSSSSSB.",
  ".KKKK......KKKK.",
  ".KKKK......KKKK.",
]

# Roadside sign: teal face with tan border (pole drawn separately at render time)
_SIGN = [
  "TTTTTTTTTTTT",
  "TAAAAAAAAAAT",
  "TAAKKAAKKAAT",
  "TAAAAAAAAAAT",
  "TAAKKKKKKAAT",
  "TAAAAAAAAAAT",
  "TTTTTTTTTTTT",
]
_SIGN_CMAP = {'T': TAN, 'A': TEAL, 'K': BLACK}


def _color_tuple(c: rl.Color) -> tuple[int, int, int, int]:
  return (c.r, c.g, c.b, c.a)


def build_texture(rows: list[str], cmap: dict[str, rl.Color]) -> rl.Texture:
  """Build a point-filtered texture from a pixel map. Requires an active GL context."""
  w, h = len(rows[0]), len(rows)
  img = rl.gen_image_color(w, h, rl.Color(0, 0, 0, 0))
  for y, row in enumerate(rows):
    for x, ch in enumerate(row):
      if ch != '.':
        rl.image_draw_pixel(img, x, y, cmap[ch])
  tex = rl.load_texture_from_image(img)
  rl.unload_image(img)
  rl.set_texture_filter(tex, rl.TextureFilter.TEXTURE_FILTER_POINT)
  return tex


def build_ego_car_texture() -> rl.Texture:
  return build_texture(_EGO_CAR, _EGO_CMAP)


def build_lead_car_texture(body: rl.Color, shade: rl.Color) -> rl.Texture:
  cmap = {'B': body, 'S': shade, 'K': BLACK, 'L': TAIL_RED}
  return build_texture(_LEAD_CAR, cmap)


def build_sign_texture() -> rl.Texture:
  return build_texture(_SIGN, _SIGN_CMAP)


def build_skyline_texture(width: int = 512, height: int = 56, seed: int = 1986) -> rl.Texture:
  """Procedural pixel city skyline: blue buildings with tan windows and a suspension bridge."""
  import random
  rng = random.Random(seed)
  img = rl.gen_image_color(width, height, rl.Color(0, 0, 0, 0))

  def px(x, y, c):
    if 0 <= x < width and 0 <= y < height:
      rl.image_draw_pixel(img, x, y, c)

  # Buildings across the strip
  x = 0
  while x < width:
    bw = rng.randint(5, 14)
    bh = rng.randint(10, height - 8)
    gap = rng.randint(1, 6)
    top = height - bh
    for by in range(top, height):
      for bx in range(x, min(x + bw, width)):
        px(bx, by, BLUE)
    # Windows: sparse tan pixels on a grid
    for by in range(top + 1, height - 1, 2):
      for bx in range(x + 1, min(x + bw - 1, width), 2):
        if rng.random() < 0.55:
          px(bx, by, TAN)
    # Occasional antenna
    if bh > height * 0.6 and rng.random() < 0.4:
      ax = x + bw // 2
      for ay in range(max(0, top - 4), top):
        px(ax, ay, BLUE)
      px(ax, max(0, top - 5), TAN)
    x += bw + gap

  # Suspension bridge silhouette over a section of the skyline
  bx0, bx1 = int(width * 0.55), int(width * 0.85)
  deck_y = height - 10
  for bx in range(bx0, bx1):
    px(bx, deck_y, BLUE)
    px(bx, deck_y + 1, BLUE)
  for tower_x in (bx0 + 6, bx1 - 7):
    for ty in range(deck_y - 9, deck_y + 6):
      px(tower_x, ty, BLUE)
      px(tower_x + 1, ty, BLUE)
    px(tower_x, deck_y - 10, TAN)
  # Catenary cables: simple V shapes between towers
  span = (bx1 - 7) - (bx0 + 6)
  for i in range(span):
    bx = bx0 + 6 + i
    t = i / max(1, span - 1)
    sag = int(8 * (4 * (t - 0.5) * (t - 0.5)))  # parabola: 0 at center, 8 at towers
    px(bx, deck_y - 1 - sag, TAN)

  # Ground glow line of city lights
  for gx in range(width):
    if rng.random() < 0.35:
      px(gx, height - 1, TAN)
    if rng.random() < 0.12:
      px(gx, height - 2, WHITE)

  tex = rl.load_texture_from_image(img)
  rl.unload_image(img)
  rl.set_texture_filter(tex, rl.TextureFilter.TEXTURE_FILTER_POINT)
  return tex


class PixelFont:
  """5x7 pixel font rendered from per-glyph textures with nearest-neighbor scaling."""

  def __init__(self):
    self._glyphs: dict[str, rl.Texture] = {}
    white = {'X': WHITE}
    for ch, rows in _FONT_5X7.items():
      if ch == ' ':
        continue
      self._glyphs[ch] = build_texture(rows, white)

  @staticmethod
  def measure(text: str, px: float) -> float:
    """Width of text in screen pixels, where px is the size of one font pixel."""
    n = len(text)
    if n == 0:
      return 0.0
    return (n * GLYPH_W + (n - 1) * GLYPH_SPACING) * px

  def draw(self, text: str, x: float, y: float, px: float, color: rl.Color):
    """Draw text with top-left origin. Unknown characters render as blanks."""
    cx = x
    step = (GLYPH_W + GLYPH_SPACING) * px
    for ch in text.upper():
      tex = self._glyphs.get(ch)
      if tex is not None:
        src = rl.Rectangle(0, 0, GLYPH_W, GLYPH_H)
        dst = rl.Rectangle(cx, y, GLYPH_W * px, GLYPH_H * px)
        rl.draw_texture_pro(tex, src, dst, rl.Vector2(0, 0), 0.0, color)
      cx += step

  def draw_centered(self, text: str, center_x: float, y: float, px: float, color: rl.Color):
    self.draw(text, center_x - self.measure(text, px) / 2, y, px, color)
