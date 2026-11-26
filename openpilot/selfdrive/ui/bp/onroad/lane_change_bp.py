"""
BluePilot: the lane change animation, for the comma 3X camera view and the rendered world.

While openpilot waits to change lanes (preLaneChange), the lane you are moving into pulses, red while the blind spot
on that side is occupied. Once the change starts, the lane fills and a row of chevrons lights up in sequence toward
it. Once you cross the line, the highlight (now on your own lane) fades out.

The lane is held by its two lines: the one you cross and the next one out. The model numbers its lane lines from
your lane, so once you are over the line it renumbers them: your old lane's line becomes the new lane's, and the
lane you moved into becomes lines 1 and 2. That moment is found from your lane's center jumping toward the side you
moved to, or from your lane's lines coming back (their confidence drops near zero while you straddle the line).
The held lines then follow the new numbers, steadied, so the highlight stays on the same lane.

This module keeps the state and builds the shapes in the model frame (x forward, y right, z down, in m). Each view
projects and draws them: ModelRendererBP through the road camera, RenderedWorld through its chase camera.
BPLaneChangeAnimation turns it on.
"""
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
import pyray as rl

from openpilot.cereal import log
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

LANE_CHANGE_BLUE = (40, 150, 255)    # the lane you are moving into
LANE_CHANGE_BLOCKED = (255, 70, 50)  # ... while the blind spot on that side is occupied
LANE_FILL_ALPHA = 0.30
CHEVRON_ALPHA = 0.95

PULSE_HZ = 1.4         # while waiting to start, about a turn signal's rate
SWEEP_S = 0.9          # one pass of the chevrons across the lane
LEVEL_TAU = 0.12       # s: fade in and out
LINE_TAU = 0.2         # s: how slowly the held lines follow the model
MODEL_DT = 0.05        # s between model frames
CROSSED_FADE_S = 1.2   # s: the highlight fades out over this after you cross
CENTER_JUMP_FRAC = 0.3  # a jump of your lane's center by this fraction of a lane, toward the side: you crossed
JUMP_WINDOW = 10        # model frames (0.5 s) the jump may take
LOST_PROB, BACK_PROB = 0.2, 0.4  # your lane's lines are lost while you straddle the line, and back once over it
MIN_LANE_M = 1.5       # m: narrower than this is not a lane
DEFAULT_LANE_M = 3.6   # m: the lane's width when the next line out is not seen
LANE_NEAR_X, LANE_FAR_X = 0.0, 60.0
REF_X = 10.0           # m ahead: where lines are matched from frame to frame
CHEVRON_X = 12.0       # m ahead: where the row of chevrons sits
CHEVRON_COUNT = 3
CHEVRON_LEN = 0.8      # m across the lane, tip to tail
CHEVRON_DEPTH = 4.0    # m along the road, arm end to arm end (long: the road is seen at a shallow angle)
CHEVRON_ARM_W = 0.9    # m: arm thickness, along the road
LINE_INSET = 0.12      # m: keep the fill off the painted lines


@dataclass
class Ribbon:
  """A strip between two edges sampled at the same depths: (N,3) each, near to far, in the model frame."""
  left: np.ndarray
  right: np.ndarray
  color: tuple[int, int, int, int]
  fade: bool = True  # the far end fades out


class HeldLine:
  """One lane line held by position: the model line nearest to where it was, steadied."""

  def __init__(self, line: np.ndarray):
    self.raw = line
    self.ref = _y(line, REF_X)

  def follow(self, line: np.ndarray, dt: float) -> None:
    self.raw = line
    self.ref += (dt / (LINE_TAU + dt)) * (_y(line, REF_X) - self.ref)

  def points(self, xs: np.ndarray) -> np.ndarray:
    """(N,2): y and z at xs, the model's shape moved to the steadied position."""
    yz = _at(self.raw, xs)
    yz[:, 0] += self.ref - _y(self.raw, REF_X)
    return yz


class LaneChangeOverlay:
  """Follows openpilot's lane change and builds the animation's shapes."""

  def __init__(self):
    self.direction = 0      # -1 left, +1 right, 0 none
    self.waiting = False    # preLaneChange: the lane change waits to start
    self.changing = False   # laneChangeStarting
    self.blocked = False    # the blind spot on the lane change side is occupied
    self.crossed = False    # you are over the line, in the new lane
    self.level = 0.0        # 0..1: how visible the whole animation is
    self._t = 0.0           # animation clock (s)
    self._since_cross = 0.0
    self._cross: HeldLine | None = None  # the line you cross
    self._outer: HeldLine | None = None  # the next line out: the new lane's far side
    self._width = DEFAULT_LANE_M
    self._centers: list[float] = []      # your lane's center at REF_X, recent model frames
    self._lines_lost = False

  @property
  def visible(self) -> bool:
    return self.level > 0.01 and self.direction != 0 and self._cross is not None

  def update(self, state, direction, blindspot_left: bool, blindspot_right: bool, lines: list[np.ndarray] | None,
             probs, dt: float) -> None:
    """state, direction: modelV2.meta.laneChangeState and laneChangeDirection. lines, probs: the four lane lines,
    (N,3) points with x increasing, and their confidence, from a new model frame; None when no new frame arrived."""
    active = state in (LaneChangeState.preLaneChange, LaneChangeState.laneChangeStarting)
    side = -1 if direction == LaneChangeDirection.left else 1 if direction == LaneChangeDirection.right else 0
    ok = lines is not None and len(lines) == 4 and all(ln.shape[0] >= 2 for ln in lines)
    if active and side != 0:
      if side != self.direction or not (self.waiting or self.changing):
        self._start(side, lines if ok else None)
      self.waiting = state == LaneChangeState.preLaneChange
      self.changing = state == LaneChangeState.laneChangeStarting
      self.blocked = blindspot_left if side < 0 else blindspot_right
    else:
      self.waiting = self.changing = False

    if ok and self.direction != 0:
      assert lines is not None
      if self._cross is None:
        self._start(self.direction, lines)
      else:
        self._follow(lines, probs, MODEL_DT)

    self._t += dt
    if self.crossed:
      self._since_cross += dt
    target = 0.0
    if self.waiting or self.changing:
      target = max(0.0, 1.0 - self._since_cross / CROSSED_FADE_S) if self.crossed else 1.0
    self.level += (dt / (LEVEL_TAU + dt)) * (target - self.level)
    if self.level <= 0.01 and not (self.waiting or self.changing):
      self.direction = 0
      self._cross = self._outer = None

  def _start(self, side: int, lines: list[np.ndarray] | None) -> None:
    self.direction = side
    self.crossed = False
    self._since_cross = 0.0
    self._t = 0.0
    self._cross = self._outer = None
    self._centers = []
    self._lines_lost = False
    if lines is None:
      return
    a, b = (1, 0) if side < 0 else (2, 3)  # your lane's line on that side, and the next one out
    self._cross, self._outer = HeldLine(lines[a]), HeldLine(lines[b])
    width = side * (self._outer.ref - self._cross.ref)
    self._width = width if width > MIN_LANE_M else DEFAULT_LANE_M

  def _follow(self, lines: list[np.ndarray], probs, dt: float) -> None:
    assert self._cross is not None
    s = self.direction
    center = (_y(lines[1], REF_X) + _y(lines[2], REF_X)) / 2
    if self.changing and not self.crossed:
      jumped = any(s * (center - c) > CENTER_JUMP_FRAC * self._width for c in self._centers)
      p = min(float(probs[1]), float(probs[2])) if probs is not None and len(probs) >= 3 else 1.0
      self._lines_lost = self._lines_lost or p < LOST_PROB
      if jumped or (self._lines_lost and p > BACK_PROB):
        self.crossed = True
        self._since_cross = 0.0
    self._centers = (self._centers + [center])[-JUMP_WINDOW:]
    if self.crossed:  # the lane you moved into is yours now: lines 1 and 2
      a, b = (2, 1) if s < 0 else (1, 2)
    else:
      a, b = (1, 0) if s < 0 else (2, 3)
    self._cross.follow(lines[a], dt)
    if self._outer is None:
      self._outer = HeldLine(lines[b])
    else:
      self._outer.follow(lines[b], dt)

  def shapes(self, near_x: float = LANE_NEAR_X, far_x: float = LANE_FAR_X) -> list[Ribbon]:
    """The animation's shapes for this frame, in the order they are drawn: the lane fill, then the chevrons."""
    if not self.visible:
      return []
    assert self._cross is not None
    far = min(far_x, float(self._cross.raw[-1, 0]))
    if far <= near_x + 1.0:
      return []
    xs = np.linspace(near_x, far, 16, dtype=np.float32)
    cross = self._cross.points(xs)
    if self._outer is not None:
      outer = self._outer.points(xs)
    else:
      outer = cross.copy()
      outer[:, 0] += self.direction * self._width
    left, right = np.minimum(cross[:, 0], outer[:, 0]), np.maximum(cross[:, 0], outer[:, 0])
    if not np.all(right - left > MIN_LANE_M):
      return []
    z = (cross[:, 1] + outer[:, 1]) / 2
    rgb = LANE_CHANGE_BLOCKED if self.blocked and not self.crossed else LANE_CHANGE_BLUE

    pulse = self._pulse() if self.waiting else 1.0
    fill_a = int(255 * LANE_FILL_ALPHA * self.level * pulse)
    out = [Ribbon(_pts(xs, left + LINE_INSET, z), _pts(xs, right - LINE_INSET, z), (*rgb, fill_a))]

    if not self.crossed and CHEVRON_X < far:
      l0, r0 = float(np.interp(CHEVRON_X, xs, left)), float(np.interp(CHEVRON_X, xs, right))
      out += self._chevrons(l0, r0, float(np.interp(CHEVRON_X, xs, z)), rgb)
    return out

  def _pulse(self) -> float:
    return 0.35 + 0.65 * float(np.sin(np.pi * PULSE_HZ * self._t)) ** 2

  def _chevrons(self, left: float, right: float, z: float, rgb) -> list[Ribbon]:
    """A row of chevrons across the lane, pointing the way you are moving. Each lights up in turn toward the side
    you are moving to; while the lane change waits to start they pulse together."""
    s = self.direction
    x0, half, w = CHEVRON_X, CHEVRON_DEPTH / 2, CHEVRON_ARM_W
    out = []
    for i in range(CHEVRON_COUNT):
      k = i if s > 0 else CHEVRON_COUNT - 1 - i  # k = 0 lights up first
      if self.waiting:
        lit = self._pulse()
      else:
        phase = (self._t / SWEEP_S - k / CHEVRON_COUNT) % 1.0
        lit = 0.25 + 0.75 * max(0.0, 1.0 - 2.5 * phase)
      alpha = int(255 * CHEVRON_ALPHA * self.level * lit)
      if alpha <= 0:
        continue
      yc = left + (right - left) * (i + 0.5) / CHEVRON_COUNT
      tip, tail = yc + s * CHEVRON_LEN / 2, yc - s * CHEVRON_LEN / 2
      for sign in (1.0, -1.0):  # the far arm, then the near arm: each from its end to the tip
        a = np.array([[x0 + sign * half, tail, z], [x0, tip, z]], dtype=np.float32)
        b = np.array([[x0 + sign * (half - w), tail, z], [x0 - sign * w, tip, z]], dtype=np.float32)
        out.append(Ribbon(a, b, (*rgb, alpha), fade=False))
    return out


Projector = Callable[[np.ndarray], np.ndarray]  # (N,3) model-frame points to (N,2) screen points, NaN where hidden


def draw_ribbons(rect: rl.Rectangle, ribbons: list[Ribbon], project: Projector) -> None:
  """Draw the animation's shapes with a view's projection. Depths one side cannot show are dropped."""
  for rb in ribbons:
    left, right = project(rb.left), project(rb.right)
    keep = np.all(np.isfinite(left), axis=1) & np.all(np.isfinite(right), axis=1)
    if int(np.sum(keep)) < 2:
      continue
    left, right = left[keep], right[keep]
    points = np.vstack([left, right[::-1]]).astype(np.float32)
    color = rl.Color(*rb.color)
    if not rb.fade or rect.height <= 0:
      draw_polygon(rect, points, color)
      continue
    near = float((left[0, 1] + right[0, 1]) / 2 - rect.y) / rect.height
    far = float((left[-1, 1] + right[-1, 1]) / 2 - rect.y) / rect.height
    if abs(near - far) < 1e-3:
      draw_polygon(rect, points, color)
      continue
    faded = rl.Color(rb.color[0], rb.color[1], rb.color[2], int(rb.color[3] * 0.15))
    draw_polygon(rect, points, gradient=Gradient(start=(0.0, near), end=(0.0, far), colors=[color, faded], stops=[0.0, 1.0]))


def _y(line: np.ndarray, x: float) -> float:
  return float(np.interp(x, line[:, 0], line[:, 1]))


def _at(line: np.ndarray, xs: np.ndarray) -> np.ndarray:
  """A lane line's y (and z when it has one) at xs: (N,2) of y, z."""
  y = np.interp(xs, line[:, 0], line[:, 1])
  z = np.interp(xs, line[:, 0], line[:, 2]) if line.shape[1] > 2 else np.zeros_like(xs)
  return np.stack([y, z], axis=-1).astype(np.float32)


def _pts(xs: np.ndarray, ys: np.ndarray, zs: np.ndarray) -> np.ndarray:
  return np.stack([xs, ys, zs], axis=-1).astype(np.float32)
