"""
BluePilot: the rendered world, a drawn road scene for the comma 3X Minimal Driving View.

The model's road edges, lane lines and path are drawn from a camera above and just behind you, not through the
road camera's calibration, over a dark sky and ground with distance fog. Pavement seams and roadside posts pass at
your speed, and the lead is a car on the road. Your own vehicle is not drawn: the path starts at the bottom of the
view, where you are.

BPDrivingScene picks it (0 plain, 1 rendered) while BPHideCameraView is on; the BPScene* params turn its parts on
and off. ModelRendererBP owns the model data and the lead smoothing and calls draw() once per frame.

Model frame throughout: x forward from the device (m), y right (m), heights above the road (m). The road is drawn
flat: model heights are ignored.
"""
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np
import pyray as rl

from openpilot.selfdrive.ui.bp.onroad.lane_change_bp import LaneChangeOverlay, draw_ribbons
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient

DRIVING_SCENE_RENDERED = 1  # BPDrivingScene value that picks the rendered world (0 is the plain view)

# Chase camera, in the model frame
CAM_X = -8.0              # m behind the device
CAM_HEIGHT = 7.0          # m above the road: puts a lead under 100 ft below the lead overlay, which then sits
                          # at the top center of the view (ChevronMetricsBP's inverted mode)
FOCAL = 600.0 / 540.0     # focal length, as a fraction of the view height
HORIZON = 0.28            # horizon, as a fraction of the view height from its top
NEAR_X = 2.0              # m: just past the bottom edge of the view

# Depth samples the road, lines and path are drawn at (denser near, where they are large on screen)
XS = np.array([NEAR_X, 4.0, 6.0, 9.0, 12.0, 16.0, 20.0, 25.0, 31.0, 38.0, 46.0, 55.0, 66.0, 79.0,
               94.0, 111.0, 130.0, 152.0, 176.0, 200.0], dtype=np.float32)

CENTER_FROM_PROB, CENTER_FULL_PROB = 0.3, 0.6  # your lane's lines start to set the lane center, then set it alone
REF_IDX = int(np.searchsorted(XS, 10.0))  # the depth sample a lane-center jump is judged at

FOG_START, FOG_END, FOG_MAX = 30.0, 260.0, 0.85

SEAM_SPACING, SEAM_DEPTH, SEAM_MAX_X = 15.0, 0.2, 150.0
POST_SPACING, POST_OFFSET, POST_MAX_X = 40.0, 0.7, 180.0
DASH_PERIOD, DASH_LEN = 12.0, 3.0
LANE_BEYOND_M = 2.5       # m of road beyond a line that makes it a line between lanes (drawn dashed when asked)
LINE_HALF_W = 0.075
PATH_HALF_W = 0.9
LANE_LINE_MIN_PROB = 0.4   # a line this sure is drawn in full and holds the road wide enough for it
LANE_LINE_FADE_FROM = 0.1   # below this a line is not drawn; between the two it fades
EDGE_TAU = 0.8            # s: how slowly the road's width follows the model's road edges
CENTER_TAU = 0.15         # s: how slowly your lane's center follows the model (short: moving across shows at once)
LINE_TAU = 0.3            # s: how slowly a lane line's distance from your lane's center follows the model
PROB_TAU = 0.5            # s: how slowly a lane line fades in and out
CENTER_JUMP_M = 1.2       # m: a lane-center move this big in one frame is a lane change or a lost line: start over


SKY_TOP = (0, 0, 0)       # near black, as the plain Minimal Driving View is
SKY_BOTTOM = (18, 20, 24)
FOG = (18, 20, 24)
GROUND = (6, 7, 9)
ROAD = (44, 48, 54)
LINE = (228, 232, 238)
SEAM = (255, 255, 255)
POST = (150, 156, 164)
REFLECTOR_LEFT = (255, 196, 40)
REFLECTOR_RIGHT = (240, 240, 240)
CAR = (128, 136, 148)
GLASS = (22, 26, 32)
TIRE = (12, 12, 14)
TAIL = (215, 34, 42)
LEAD_PAD_RADAR = (0, 134, 233)
LEAD_PAD_VISION = (218, 202, 37)


def fog_t(x) -> np.ndarray:
  """How far toward the fog color a point x meters ahead is drawn (0 near, FOG_MAX far)."""
  return np.clip((np.asarray(x, dtype=np.float32) - FOG_START) / (FOG_END - FOG_START), 0.0, 1.0) * FOG_MAX


def fogged(color: tuple, x: float, alpha: int = 255) -> rl.Color:
  t = float(fog_t(x))
  return rl.Color(int(color[0] + (FOG[0] - color[0]) * t), int(color[1] + (FOG[1] - color[1]) * t),
                  int(color[2] + (FOG[2] - color[2]) * t), int(alpha))


def mix(a: tuple, b: tuple, t: float) -> tuple:
  return tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(3))


@dataclass(frozen=True)
class ChaseCamera:
  """Projects model-frame points into a view rectangle from behind and above your vehicle."""
  x: float
  y: float
  width: float
  height: float

  @property
  def focal(self) -> float:
    return FOCAL * self.height

  @property
  def horizon_y(self) -> float:
    return self.y + HORIZON * self.height

  @property
  def center_x(self) -> float:
    return self.x + self.width / 2

  def project(self, xf, yr, h=0.0) -> np.ndarray:
    """(N, 2) screen points for forward distances xf, right offsets yr and heights h (scalars broadcast)."""
    xf, yr, h = np.broadcast_arrays(np.asarray(xf, dtype=np.float32), np.asarray(yr, dtype=np.float32),
                                    np.asarray(h, dtype=np.float32))
    d = np.maximum(xf - CAM_X, 0.5)
    return np.stack([self.center_x + self.focal * yr / d, self.horizon_y + self.focal * (CAM_HEIGHT - h) / d], axis=-1)

  def y_fraction(self, xf: float) -> float:
    """Screen height of the road x meters ahead, as a fraction of the view from its top."""
    return float((self.project(xf, 0.0)[1] - self.y) / self.height)


@dataclass
class SceneModel:
  """The model output the scene draws: each line as (x, y) samples, plus how far ahead to draw."""
  lane_lines: list = field(default_factory=list)      # 4 arrays (N, >=2): x forward, y right
  lane_line_probs: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.float32))
  road_edges: list = field(default_factory=list)      # 2 arrays: left, right
  path: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float32))
  max_x: float = 150.0        # m of road drawn ahead
  path_max_x: float = 100.0   # m of path drawn ahead (the plan's length)


@dataclass
class SceneLead:
  d_rel: float      # m ahead
  y_right: float    # m right of the device
  is_radar: bool


@dataclass
class SceneOptions:
  road_motion: bool = True
  dashed_lines: bool = False
  lead_vehicle: bool = True


def line_y(line: np.ndarray, xs: np.ndarray) -> np.ndarray:
  """A model line's lateral offset at xs; held flat behind its first point and past its last."""
  return np.interp(xs, line[:, 0], line[:, 1]).astype(np.float32)


Edges = tuple[np.ndarray, np.ndarray]
PathPainter = Callable[[np.ndarray], None]


def has_road_edges(model: SceneModel) -> bool:
  return len(model.road_edges) >= 2 and all(e.shape[0] >= 2 for e in model.road_edges[:2])


def road_bounds(model: SceneModel, xs: np.ndarray) -> Edges | None:
  """Left and right edge of the drawn road at xs: the road edges, widened to hold every drawn lane line."""
  if not has_road_edges(model):
    return None
  left, right = line_y(model.road_edges[0], xs), line_y(model.road_edges[1], xs)
  for i, ln in enumerate(model.lane_lines):
    if ln.shape[0] >= 2 and model.lane_line_probs[i] >= LANE_LINE_MIN_PROB:
      ys = line_y(ln, xs)
      left, right = np.minimum(left, ys - 0.3), np.maximum(right, ys + 0.3)
  if not np.all(right - left > 1.0):
    return None
  return left, right


def edge_bounds(model: SceneModel, xs: np.ndarray) -> Edges | None:
  """The road's edges at xs as they are, for a model SceneSmoother has already widened and steadied."""
  if not has_road_edges(model):
    return None
  left, right = line_y(model.road_edges[0], xs), line_y(model.road_edges[1], xs)
  return (left, right) if np.all(right - left > 1.0) else None


def dashed_lines(model: SceneModel, at_x: float = 20.0) -> list[bool]:
  """Which lane lines have another lane's width of road beyond them (lines between lanes).

  The model does not say whether a line is painted dashed; this is the guess the dashed option draws from. The
  outermost line on each side, with the road edge close beyond it, stays solid.
  """
  out = [False] * len(model.lane_lines)
  if not has_road_edges(model):
    return out
  xs = np.array([at_x], dtype=np.float32)
  edge_left, edge_right = float(line_y(model.road_edges[0], xs)[0]), float(line_y(model.road_edges[1], xs)[0])
  for i, ln in enumerate(model.lane_lines):
    if ln.shape[0] < 2:
      continue
    y = float(line_y(ln, xs)[0])
    beyond = (y - edge_left) if i < 2 else (edge_right - y)
    out[i] = beyond > LANE_BEYOND_M
  return out


def lane_center(lines: list, probs: np.ndarray, path: np.ndarray, xs: np.ndarray) -> np.ndarray:
  """Your lane's center at xs: midway between your lane's lines (sampled at xs) when both are seen, your path when
  they are not, and a blend of the two in between, so a line going unsure moves the center smoothly."""
  fallback = line_y(path, xs) if path.shape[0] >= 2 else np.zeros_like(xs)
  if len(lines) < 3 or lines[1] is None or lines[2] is None:
    return fallback
  w = float(np.clip((min(probs[1], probs[2]) - CENTER_FROM_PROB) / (CENTER_FULL_PROB - CENTER_FROM_PROB), 0.0, 1.0))
  return w * (lines[1] + lines[2]) / 2 + (1.0 - w) * fallback


class SceneSmoother:
  """Steadies the model's lines and edges from frame to frame, without making the road lag when you move.

  Every line and both road edges are kept as their distance from your lane's center, and those distances are
  filtered: lane lines over LINE_TAU, the road's edges over EDGE_TAU. A width change eases in; a move across the
  road shows almost at once, because the center itself is filtered only over the short CENTER_TAU. The road is widened to hold the lane
  lines before it is filtered, so an outer line coming and going does not snap it wider and narrower. Each line's
  confidence is filtered too, so lines fade in and out instead of popping. The path is
  steadied the same way as the lane lines. A sudden jump of the lane center (the
  model handing your lane over in a lane change) starts everything over from the new frame.
  """

  def __init__(self, line_tau: float = LINE_TAU, edge_tau: float = EDGE_TAU, prob_tau: float = PROB_TAU):
    self.line_tau, self.edge_tau, self.prob_tau = line_tau, edge_tau, prob_tau
    self._lines: list = []                # per lane line: its distance from the lane center at XS, or None
    self._edges: Edges | None = None      # the road's edges as distances from the lane center at XS
    self._probs: np.ndarray | None = None
    self._path: np.ndarray | None = None  # the path's distance from the lane center, at the path's own points
    self._center: np.ndarray | None = None  # your lane's center at XS, filtered
    self._center_ref = 0.0

  def smooth(self, model: SceneModel, dt: float) -> SceneModel:
    xs = XS
    n = len(model.lane_lines)
    probs = np.zeros(n, dtype=np.float32)
    probs[:min(n, len(model.lane_line_probs))] = np.asarray(model.lane_line_probs, dtype=np.float32)[:n]
    lines = [line_y(ln, xs) if ln.shape[0] >= 2 else None for ln in model.lane_lines]
    filtered_probs = probs if self._probs is None or self._probs.shape != probs.shape else self._probs
    center = lane_center(lines, filtered_probs, model.path, xs)
    center_ref = float(center[REF_IDX])
    bounds = road_bounds(model, xs)  # widened by this frame's lines, then filtered with the edges

    has_path = model.path.shape[0] > 0
    path_off = model.path[:, 1] - np.interp(model.path[:, 0], xs, center) if has_path else np.empty(0, dtype=np.float32)

    restart = (self._probs is None or self._center is None or self._probs.shape != probs.shape or len(self._lines) != n or
               abs(center_ref - self._center_ref) > CENTER_JUMP_M)
    self._center_ref = center_ref
    if restart:
      self._center = center
      self._probs = probs
      self._lines = [None if y is None else y - center for y in lines]
      self._edges = None if bounds is None else (bounds[0] - center, bounds[1] - center)
      self._path = path_off
    else:
      assert self._center is not None and self._probs is not None
      self._center = _ema(self._center, center, dt, CENTER_TAU)
      self._probs = _ema(self._probs, probs, dt, self.prob_tau)
      for i, y in enumerate(lines):
        prev = self._lines[i]
        self._lines[i] = None if y is None else (y - center if prev is None else _ema(prev, y - center, dt, self.line_tau))
      if bounds is None:
        self._edges = None
      elif self._edges is None:
        self._edges = (bounds[0] - center, bounds[1] - center)
      else:
        self._edges = (_ema(self._edges[0], bounds[0] - center, dt, self.edge_tau),
                       _ema(self._edges[1], bounds[1] - center, dt, self.edge_tau))
      same = self._path is not None and self._path.shape == path_off.shape
      self._path = _ema(self._path, path_off, dt, self.line_tau) if same else path_off

    drawn_center = self._center

    def at(offset: np.ndarray) -> np.ndarray:
      return np.stack([xs, drawn_center + offset], axis=-1)

    empty = np.empty((0, 2), dtype=np.float32)
    path = model.path.copy()
    if has_path:
      path[:, 1] = np.interp(path[:, 0], xs, drawn_center) + self._path
    return SceneModel(
      lane_lines=[empty if off is None else at(off) for off in self._lines],
      lane_line_probs=self._probs.copy(),
      road_edges=[] if self._edges is None else [at(self._edges[0]), at(self._edges[1])],
      path=path, max_x=model.max_x, path_max_x=model.path_max_x,
    )


def _ema(prev: np.ndarray, new: np.ndarray, dt: float, tau: float) -> np.ndarray:
  return prev + (dt / (tau + dt)) * (new - prev)


def repeating(phase: float, spacing: float, offset: float, x_min: float, x_max: float) -> np.ndarray:
  """Positions spaced `spacing` apart that slide toward you as `phase` (meters traveled) grows."""
  first = x_min + ((offset - phase - x_min) % spacing)  # the nearest one at or past x_min
  return np.arange(first, x_max, spacing, dtype=np.float32)


class RenderedWorld:
  """Draws the rendered world into a view rectangle. One per ModelRendererBP."""

  def __init__(self):
    self._phase = 0.0  # meters traveled, for the seams, posts and dashes
    self._dt = 0.05
    self._smoother = SceneSmoother()

  def advance(self, v_ego: float, dt: float, motion: bool) -> None:
    self._dt = dt
    if motion:
      self._phase = (self._phase + max(0.0, v_ego) * dt) % 1200.0

  @staticmethod
  def camera(rect: rl.Rectangle) -> ChaseCamera:
    return ChaseCamera(rect.x, rect.y, rect.width, rect.height)

  def lead_anchor(self, rect: rl.Rectangle, d_rel: float, y_right: float) -> tuple[float, float]:
    """Where the lead car's rear meets the road, on screen: the point the lead's distance box hangs from, as the
    chevron's apex does in the camera view (the box is drawn below it)."""
    p = self.camera(rect).project(d_rel, y_right, 0.0)
    return float(p[0]), float(p[1])

  def draw(self, rect: rl.Rectangle, model: SceneModel, leads: list[SceneLead], opts: SceneOptions,
           paint_path: PathPainter | None = None, path_half_w: float = PATH_HALF_W,
           lane_change: LaneChangeOverlay | None = None) -> None:
    """paint_path draws the path polygon (screen points, left edge near to far, then right edge far to near), so
    the path can look as it does in the camera view; without it the path is a plain translucent ribbon."""
    cam = self.camera(rect)
    rl.rl_disable_backface_culling()
    try:
      self._draw_sky_and_ground(cam)
      max_x = float(np.clip(model.max_x, 10.0, XS[-1]))
      xs = XS[XS <= max_x]
      model = self._smoother.smooth(model, self._dt)
      bounds = edge_bounds(model, xs)
      if bounds is not None:
        self._draw_road(rect, cam, xs, bounds)
        self._draw_seams(cam, model)
      if lane_change is not None:
        draw_ribbons(rect, lane_change.shapes(near_x=NEAR_X, far_x=max_x),
                     lambda p: cam.project(p[:, 0], p[:, 1]).astype(np.float32))
      lead_d = min((ld.d_rel for ld in leads), default=None) if opts.lead_vehicle else None
      ribbon = self.path_ribbon(cam, model, max_x, lead_d, path_half_w)
      if ribbon is not None:
        if paint_path is not None:
          paint_path(ribbon)
        else:
          draw_polygon(rect, ribbon, rl.Color(255, 255, 255, 40))
      self._draw_lane_lines(rect, cam, model, xs, max_x, opts.dashed_lines)
      if bounds is not None:
        self._draw_posts(cam, model)
      if opts.lead_vehicle:
        for ld in sorted(leads, key=lambda ld: -ld.d_rel):
          self.draw_lead_vehicle(cam, ld)
    finally:
      rl.rl_enable_backface_culling()

  # --- background --------------------------------------------------------------------------------------------

  @staticmethod
  def _draw_sky_and_ground(cam: ChaseCamera) -> None:
    x, w = int(cam.x), int(cam.width)
    top, horizon, bottom = int(cam.y), int(cam.horizon_y), int(cam.y + cam.height)
    rl.draw_rectangle_gradient_v(x, top, w, horizon - top + 1, rl.Color(*SKY_TOP, 255), rl.Color(*SKY_BOTTOM, 255))
    rl.draw_rectangle_gradient_v(x, horizon, w, bottom - horizon, fogged(GROUND, FOG_END), rl.Color(*GROUND, 255))

  @staticmethod
  def _fog_gradient(cam: ChaseCamera, color: tuple, alpha_near: float, alpha_far: float) -> Gradient:
    """Vertical gradient that matches distance fog on the flat road: far (top) to near (bottom)."""
    samples = [FOG_END, 120.0, 60.0, FOG_START, NEAR_X]
    top, bottom = cam.y_fraction(FOG_END), 1.0
    colors, stops = [], []
    for i, x in enumerate(samples):
      a = alpha_far + (alpha_near - alpha_far) * (i / (len(samples) - 1))
      colors.append(fogged(color, x, int(255 * a)))
      stops.append((cam.y_fraction(x) - top) / max(bottom - top, 1e-3))
    return Gradient(start=(0.0, top), end=(0.0, bottom), colors=colors, stops=stops)

  def _draw_road(self, rect, cam: ChaseCamera, xs: np.ndarray, bounds) -> None:
    left, right = bounds
    ribbon = np.vstack([cam.project(xs, left), cam.project(xs, right)[::-1]])
    draw_polygon(rect, ribbon, gradient=self._fog_gradient(cam, ROAD, 1.0, 1.0))

  def _draw_seams(self, cam: ChaseCamera, model: SceneModel) -> None:
    for x in repeating(self._phase, SEAM_SPACING, 3.0, NEAR_X, min(SEAM_MAX_X, model.max_x)):
      xx = np.array([x, x + SEAM_DEPTH], dtype=np.float32)
      bounds = edge_bounds(model, xx)
      if bounds is None:
        continue
      left, right = bounds
      quad = cam.project([x, x, x + SEAM_DEPTH, x + SEAM_DEPTH], [left[0], right[0], right[1], left[1]])
      self._quad(quad, rl.Color(*SEAM, int(255 * 0.05 * (1.0 - float(fog_t(x)) / FOG_MAX))))

  @staticmethod
  def path_ribbon(cam: ChaseCamera, model: SceneModel, max_x: float, lead_d: float | None,
                  half_w: float) -> np.ndarray | None:
    """The path as a screen polygon, from the bottom of the view to its end (short of the lead car). It keeps the
    model's own path points, so a color per point (the camera view's acceleration colors) still lines up."""
    path = model.path
    if path.shape[0] < 2:
      return None
    end = min(max_x, model.path_max_x)
    if lead_d is not None:
      end = min(end, lead_d - 1.0)
    if end <= NEAR_X + 0.5:
      return None
    inside = path[(path[:, 0] > NEAR_X) & (path[:, 0] < end)]
    xs = np.concatenate([[NEAR_X], inside[:, 0], [end]]).astype(np.float32)
    ys = np.concatenate([line_y(path, np.array([NEAR_X], dtype=np.float32)), inside[:, 1],
                         line_y(path, np.array([end], dtype=np.float32))]).astype(np.float32)
    return np.vstack([cam.project(xs, ys - half_w), cam.project(xs, ys + half_w)[::-1]]).astype(np.float32)

  def _draw_lane_lines(self, rect, cam: ChaseCamera, model: SceneModel, xs: np.ndarray, max_x: float,
                       dashed_option: bool) -> None:
    dashed = dashed_lines(model) if dashed_option else [False] * len(model.lane_lines)
    for i, ln in enumerate(model.lane_lines):
      prob = float(model.lane_line_probs[i]) if i < len(model.lane_line_probs) else 0.0
      fade = float(np.clip((prob - LANE_LINE_FADE_FROM) / (LANE_LINE_MIN_PROB - LANE_LINE_FADE_FROM), 0.0, 1.0))
      if ln.shape[0] < 2 or fade <= 0.0:
        continue
      alpha = float(np.clip(prob * 1.1, 0.5, 0.95)) * (1.0 if i in (1, 2) else 0.7) * fade
      if dashed[i]:
        for a0 in repeating(self._phase, DASH_PERIOD, -2.0, NEAR_X - DASH_LEN, max_x):
          a, b = max(NEAR_X, float(a0)), min(max_x, float(a0) + DASH_LEN)
          if b <= a:
            continue
          seg = np.array([a, b], dtype=np.float32)
          ys = line_y(ln, seg)
          quad = cam.project([a, a, b, b], [ys[0] - LINE_HALF_W, ys[0] + LINE_HALF_W, ys[1] + LINE_HALF_W, ys[1] - LINE_HALF_W])
          self._quad(quad, fogged(LINE, a, int(255 * alpha)))
      else:
        ys = line_y(ln, xs)
        ribbon = np.vstack([cam.project(xs, ys - LINE_HALF_W), cam.project(xs, ys + LINE_HALF_W)[::-1]])
        draw_polygon(rect, ribbon, gradient=self._fog_gradient(cam, LINE, alpha, alpha * 0.6))

  def _draw_posts(self, cam: ChaseCamera, model: SceneModel) -> None:
    xs = repeating(self._phase, POST_SPACING, 10.0, NEAR_X, min(POST_MAX_X, model.max_x))
    if xs.size == 0:
      return
    bounds = road_bounds(model, xs)
    if bounds is None:
      return
    left, right = bounds
    for i in range(len(xs) - 1, -1, -1):  # far to near
      x = float(xs[i])
      for y, refl in ((float(left[i]) - POST_OFFSET, REFLECTOR_LEFT), (float(right[i]) + POST_OFFSET, REFLECTOR_RIGHT)):
        self._face(cam, x, y - 0.06, y + 0.06, 0.0, 1.05, fogged(POST, x))
        self._face(cam, x - 0.01, y - 0.05, y + 0.05, 0.82, 0.98, fogged(refl, x))

  # --- lead car ------------------------------------------------------------------------------------------------

  def draw_lead_vehicle(self, cam: ChaseCamera, lead: SceneLead) -> None:
    d, y = float(lead.d_rel), float(lead.y_right)
    if d < NEAR_X + 2.0 or d > 250.0:
      return
    body, light = fogged(CAR, d), fogged(mix(CAR, (255, 255, 255), 0.2), d)
    dark, glass = fogged(mix(CAR, (0, 0, 0), 0.5), d), fogged(GLASS, d)
    pad = LEAD_PAD_RADAR if lead.is_radar else LEAD_PAD_VISION
    self._top(cam, d - 0.4, d + 4.9, y - 1.25, y + 1.25, 0.0, rl.Color(*pad, 128))
    self._top(cam, d - 0.1, d + 4.6, y - 1.0, y + 1.0, 0.0, rl.Color(0, 0, 0, 115))
    self._top(cam, d + 1.0, d + 3.0, y - 0.8, y + 0.8, 1.48, light)
    self._poly4(cam.project([d, d, d + 0.5, d + 0.5], [y - 0.92, y + 0.92, y + 0.9, y - 0.9], [1.05, 1.05, 1.08, 1.08]), light)
    self._poly4(cam.project([d + 0.5, d + 0.5, d + 1.0, d + 1.0], [y - 0.84, y + 0.84, y + 0.78, y - 0.78],
                            [1.08, 1.08, 1.48, 1.48]), glass)
    self._face(cam, d + 0.3, y - 0.95, y - 0.62, 0.0, 0.34, fogged(TIRE, d))
    self._face(cam, d + 0.3, y + 0.62, y + 0.95, 0.0, 0.34, fogged(TIRE, d))
    self._face(cam, d, y - 0.95, y + 0.95, 0.3, 1.05, body)
    self._face(cam, d - 0.05, y - 0.97, y + 0.97, 0.22, 0.42, dark)
    self._face(cam, d - 0.02, y - 0.93, y - 0.6, 0.76, 0.95, fogged(TAIL, d))
    self._face(cam, d - 0.02, y + 0.6, y + 0.93, 0.76, 0.95, fogged(TAIL, d))
    self._face(cam, d - 0.03, y - 0.24, y + 0.24, 0.46, 0.62, fogged((210, 212, 216), d))

  # --- primitives --------------------------------------------------------------------------------------------

  @staticmethod
  def _quad(p: np.ndarray, color: rl.Color) -> None:
    """Quad given as near-left, near-right, far-right, far-left (any consistent loop works)."""
    rl.draw_triangle_strip([(float(p[0][0]), float(p[0][1])), (float(p[1][0]), float(p[1][1])),
                            (float(p[3][0]), float(p[3][1])), (float(p[2][0]), float(p[2][1]))], 4, color)

  _poly4 = _quad

  def _face(self, cam: ChaseCamera, x: float, y0: float, y1: float, h0: float, h1: float, color: rl.Color) -> None:
    """Upright rectangle facing you at distance x, from y0 to y1 and h0 to h1."""
    self._quad(cam.project([x, x, x, x], [y0, y1, y1, y0], [h0, h0, h1, h1]), color)

  def _top(self, cam: ChaseCamera, x0: float, x1: float, y0: float, y1: float, h: float, color: rl.Color) -> None:
    """Flat rectangle at height h, from x0 to x1 and y0 to y1."""
    self._quad(cam.project([x0, x0, x1, x1], [y0, y1, y1, y0], h), color)
