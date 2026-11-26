"""
BluePilot: steady numbers for the lead overlay (the camera view's boxes and the rendered world's tag).

A vision lead's distance moves 1-2 m from one frame to the next, so the raw readout changes nearly every frame
(route 0000001a: the ft value about 15 times a second, the mph value about 9). The values are filtered first, then
each shown number is held until the filtered value has moved most of a display step away from it, and the distance
step grows with the distance (no 1 ft digits on a car 150 ft ahead). A new lead, or one that jumps (a cut-in, or the
lead changing to another car), shows its own numbers at once instead of sliding over from the old ones.
"""
from openpilot.common.constants import CV

D_TAU = 0.5          # s: distance filter
V_TAU = 0.8          # s: relative speed filter
HOLD = 0.8           # a shown number changes once the value is this many display steps away from it
JUMP_M, JUMP_FRAC = 6.0, 0.25  # a distance change this big in one update is another lead: start over
GAP_STEP = 0.1       # s
FT_PER_M = 3.28084


def distance_step(value: float, metric: bool) -> float:
  """Display step for a distance in display units: finer near, coarser far."""
  if metric:
    return 1.0 if value < 50.0 else 5.0
  return 1.0 if value < 50.0 else 5.0 if value < 200.0 else 10.0


class Held:
  """A number shown in fixed steps that changes only once its value has moved HOLD steps away."""

  def __init__(self):
    self.shown: float | None = None

  def update(self, value: float, step: float) -> float:
    if self.shown is None or abs(value - self.shown) >= HOLD * step:
      self.shown = round(value / step) * step
    return self.shown

  def reset(self) -> None:
    self.shown = None


class LeadReadout:
  """Filtered, held distance, speed and time gap for one lead."""

  def __init__(self):
    self.present = False
    self._d: float | None = None
    self._v_rel = 0.0
    self._dist, self._speed, self._gap = Held(), Held(), Held()

  def update(self, present: bool, d_rel: float, v_rel: float, dt: float) -> None:
    if not present:
      self.present = False
      return
    if not self.present or self._d is None or abs(d_rel - self._d) > max(JUMP_M, JUMP_FRAC * self._d):
      self._d, self._v_rel = d_rel, v_rel
      for h in (self._dist, self._speed, self._gap):
        h.reset()
    else:
      self._d += (dt / (D_TAU + dt)) * (d_rel - self._d)
      self._v_rel += (dt / (V_TAU + dt)) * (v_rel - self._v_rel)
    self.present = True

  @property
  def d_rel(self) -> float:
    return self._d if self._d is not None else 0.0

  def distance(self, metric: bool) -> float:
    """Distance in m or ft, held."""
    val = max(0.0, self.d_rel) * (1.0 if metric else FT_PER_M)
    return self._dist.update(val, distance_step(self._dist.shown if self._dist.shown is not None else val, metric))

  def speed(self, v_ego: float, metric: bool) -> float:
    """The lead's speed in km/h or mph, held."""
    val = max(0.0, (self._v_rel + v_ego) * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH))
    return self._speed.update(val, 1.0)

  def gap(self, v_ego: float) -> float | None:
    """Time gap in s, held; None when there is none to show (stopped, or 200 s or more)."""
    val = self.d_rel / v_ego if (self.d_rel > 0 and v_ego > 0) else 0.0
    if not 0 < val < 200:
      self._gap.reset()
      return None
    return self._gap.update(val, GAP_STEP)

  def lines(self, v_ego: float, metric: bool, distance: bool = True, speed: bool = True, gap: bool = True) -> list[str]:
    """The readout's text lines, in the order the overlay shows them."""
    out = []
    if distance:
      out.append(f"{self.distance(metric):.0f} {'m' if metric else 'ft'}")
    if speed:
      out.append(f"{self.speed(v_ego, metric):.0f} {'km/h' if metric else 'mph'}")
    if gap:
      g = self.gap(v_ego)
      out.append(f"{g:.1f} s" if g is not None else "---")
    return out
