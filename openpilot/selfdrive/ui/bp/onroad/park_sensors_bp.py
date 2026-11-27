"""
BluePilot: the park sensors overlay for the comma 3X onroad view.

Ford's park aid reports eight zones (four across the front bumper, four across the rear) on the camera bus
(ParkAid_Aud_Warn_Stat 0x3AA, ParkAid_Aud_Warn_Stat2 0x3AB). Each zone is a step, not a distance: 15 is clear,
lower is closer, 0 is off. The truck turns them on below about 11 mph and off above about 12.5 mph (2025 F-150
routes), so this shows only while they are on and something is in range.

The overlay is a small top-down truck in the view's lower right corner, front up, with three bands around each
zone: the band at the object's distance lights (yellow far, amber nearer, red closest) and the others stay faint.
The car does not say how many centimeters a step is, so the bands are steps too.

BPParkSensors turns it on (off by default). ParkSensorsOverlay reads carStateBP.parkAid and the setting, and
AugmentedRoadViewBP draws it in the camera view, the minimal view and the rendered world.
"""
import time
from dataclasses import dataclass, field

import pyray as rl

ZONE_OFF, ZONE_CLEAR = 0, 15
NEAR_MAX, MID_MAX = 4, 9    # zone steps: 1-4 closest (red), 5-9 nearer (amber), 10-14 far (yellow)

FAR_COLOR = (255, 214, 64)
MID_COLOR = (255, 140, 30)
NEAR_COLOR = (255, 56, 48)
IDLE_BAND = (255, 255, 255, 22)   # the bands not lit: faint, to show where each zone looks
PANEL = (10, 12, 16, 150)
TRUCK_FILL = (58, 62, 70)
TRUCK_LINE = (200, 205, 212)
GLASS = (30, 34, 42)

# Layout at scale 1.0 (comma 3X px)
TRUCK_W, TRUCK_H = 104.0, 290.0
TRUCK_R = 0.32             # corner roundness
RING_INSET = 64.0          # px from the bumper to each ring set's center, inside the truck
BAND_R = (92.0, 116.0, 140.0)  # inner radius of the near, mid and far band
BAND_W = 18.0
SECTOR_GAP = 3.0           # degrees between zones
FRONT_SPAN = (212.0, 328.0)  # degrees, raylib's (0 right, 90 down): across the front, left corner to right corner
REAR_SPAN = (32.0, 148.0)    # across the rear, right corner to left corner
PAD = 24.0
MARGIN = 40.0              # px from the view's right and bottom edges
DEV_UI_RIGHT_W = 230.0     # the developer UI's right column (184 px plus its margins)

STATUS_ENABLED = 1         # the module's front/rear status: 0 disabled, 1 enabled, 3 faulted
HOLD_S = 1.5               # s the overlay stays after the last near zone: the zones come at 5 Hz and flicker at a band's edge
FADE_IN_S, FADE_OUT_S = 0.15, 0.4
PARAM_REFRESH_S = 1.0


@dataclass(frozen=True)
class ParkZones:
  """Zone steps, left to right as seen from the driver's seat: corner, center, center, corner."""
  front: tuple[int, ...] = (ZONE_OFF,) * 4
  rear: tuple[int, ...] = (ZONE_OFF,) * 4
  front_ok: bool = False   # the front sensors are enabled (not off or faulted)
  rear_ok: bool = False

  @property
  def active(self) -> bool:
    return (self.front_ok and any(z != ZONE_OFF for z in self.front)) or \
           (self.rear_ok and any(z != ZONE_OFF for z in self.rear))

  @property
  def something_near(self) -> bool:
    return any(band(z) > 0 for z in (self.front if self.front_ok else ())) or \
           any(band(z) > 0 for z in (self.rear if self.rear_ok else ()))


def band(zone: int) -> int:
  """Which band a zone step lights: 0 none (off or clear), 1 far, 2 nearer, 3 closest."""
  if zone <= ZONE_OFF or zone >= ZONE_CLEAR:
    return 0
  return 3 if zone <= NEAR_MAX else 2 if zone <= MID_MAX else 1


BAND_COLORS = {1: FAR_COLOR, 2: MID_COLOR, 3: NEAR_COLOR}


@dataclass
class Layout:
  """Where the widget's parts sit for one view and scale."""
  x: float
  y: float
  w: float
  h: float
  cx: float
  front_c: rl.Vector2 = field(default_factory=lambda: rl.Vector2(0, 0))
  rear_c: rl.Vector2 = field(default_factory=lambda: rl.Vector2(0, 0))
  truck: rl.Rectangle = field(default_factory=lambda: rl.Rectangle(0, 0, 0, 0))
  s: float = 1.0


def layout(rect: rl.Rectangle, scale: float = 1.0) -> Layout:
  s = scale
  reach = (BAND_R[2] + BAND_W) * s - RING_INSET * s  # how far the far band reaches past the bumper
  w = 2 * (BAND_R[2] + BAND_W) * s + 2 * PAD * s
  h = TRUCK_H * s + 2 * reach + 2 * PAD * s
  x = rect.x + rect.width - MARGIN - w
  y = rect.y + rect.height - MARGIN - h
  cx = x + w / 2
  truck = rl.Rectangle(cx - TRUCK_W * s / 2, y + PAD * s + reach, TRUCK_W * s, TRUCK_H * s)
  return Layout(x, y, w, h, cx, rl.Vector2(cx, truck.y + RING_INSET * s),
                rl.Vector2(cx, truck.y + truck.height - RING_INSET * s), truck, s)


def _sectors(span: tuple[float, float]) -> list[tuple[float, float]]:
  a0, a1 = span
  step = (a1 - a0) / 4
  return [(a0 + i * step + SECTOR_GAP / 2, a0 + (i + 1) * step - SECTOR_GAP / 2) for i in range(4)]


def zone_arcs(lo: Layout, zones: ParkZones) -> list[tuple[rl.Vector2, tuple[float, float], int, bool]]:
  """(ring center, sector in degrees, zone step, sensors on) for the eight zones, front left to right, then rear
  left to right. raylib's angles run clockwise from the right, so across the rear the right corner comes first."""
  front = zip(_sectors(FRONT_SPAN), zones.front, strict=True)
  rear = reversed(list(zip(_sectors(REAR_SPAN), reversed(zones.rear), strict=True)))
  return [(lo.front_c, sec, z, zones.front_ok) for sec, z in front] + [(lo.rear_c, sec, z, zones.rear_ok) for sec, z in rear]


def _draw_zone_bands(center: rl.Vector2, sector: tuple[float, float], zone: int, ok: bool, s: float, a: float) -> None:
  lit = band(zone) if ok else 0
  for i, r in enumerate(BAND_R):  # i: 0 near, 1 mid, 2 far
    level = 3 - i
    inner, outer = r * s, (r + BAND_W) * s
    if lit == level:
      c = BAND_COLORS[level]
      rl.draw_ring(center, inner, outer, sector[0], sector[1], 24, rl.Color(*c, int(255 * a)))
    else:
      rl.draw_ring(center, inner, outer, sector[0], sector[1], 24, rl.Color(*IDLE_BAND[:3], int(IDLE_BAND[3] * a)))


def _draw_truck(lo: Layout, a: float) -> None:
  t, s = lo.truck, lo.s
  rl.draw_rectangle_rounded(t, TRUCK_R, 12, rl.Color(*TRUCK_FILL, int(255 * a)))
  rl.draw_rectangle_rounded_lines_ex(t, TRUCK_R, 12, 3 * s, rl.Color(*TRUCK_LINE, int(230 * a)))
  # windshield, cab roof and bed: the front is up
  inset = 12 * s
  glass = rl.Rectangle(t.x + inset, t.y + 0.22 * t.height, t.width - 2 * inset, 0.12 * t.height)
  rl.draw_rectangle_rounded(glass, 0.4, 8, rl.Color(*GLASS, int(255 * a)))
  roof = rl.Rectangle(t.x + inset, t.y + 0.36 * t.height, t.width - 2 * inset, 0.20 * t.height)
  rl.draw_rectangle_rounded_lines_ex(roof, 0.25, 8, 2 * s, rl.Color(*TRUCK_LINE, int(120 * a)))
  bed = rl.Rectangle(t.x + inset, t.y + 0.60 * t.height, t.width - 2 * inset, 0.34 * t.height)
  rl.draw_rectangle_rounded_lines_ex(bed, 0.12, 8, 2 * s, rl.Color(*TRUCK_LINE, int(120 * a)))
  # mirrors
  for sx in (-1, 1):
    mx = t.x - 10 * s if sx < 0 else t.x + t.width
    rl.draw_rectangle_rounded(rl.Rectangle(mx, t.y + 0.24 * t.height, 10 * s, 18 * s), 0.5, 4, rl.Color(*TRUCK_FILL, int(255 * a)))


def draw_park_sensors(rect: rl.Rectangle, zones: ParkZones, alpha: float = 1.0, scale: float = 1.0) -> None:
  a = max(0.0, min(1.0, alpha))
  if a <= 0.0:
    return
  lo = layout(rect, scale)
  rl.draw_rectangle_rounded(rl.Rectangle(lo.x, lo.y, lo.w, lo.h), 0.12, 12, rl.Color(*PANEL[:3], int(PANEL[3] * a)))
  for center, sector, zone, ok in zone_arcs(lo, zones):
    _draw_zone_bands(center, sector, zone, ok, lo.s, a)
  _draw_truck(lo, a)


def zones_from_park_aid(park_aid) -> ParkZones:
  """carStateBP.parkAid as zones; nothing while its frames are missing."""
  if not park_aid.dataAvailable:
    return ParkZones()
  front, rear = list(park_aid.frontZones), list(park_aid.rearZones)
  if len(front) != 4 or len(rear) != 4:
    return ParkZones()
  return ParkZones(tuple(front), tuple(rear),
                   park_aid.frontStatus == STATUS_ENABLED, park_aid.rearStatus == STATUS_ENABLED)


class ParkSensorsOverlay:
  """The setting, the zones from carStateBP and the fade. update() once per frame, then render()."""
  def __init__(self, params=None):
    self._params = params
    self._enabled = False
    self._param_checked = -1e9
    self._zones = ParkZones()
    self._shown = ParkZones()   # what is drawn: the last zones with something near, held through the fade
    self._last_near = -1e9
    self._alpha = 0.0
    self._t = None

  def _refresh_param(self, now: float) -> None:
    if now - self._param_checked < PARAM_REFRESH_S:
      return
    self._param_checked = now
    if self._params is None:
      from openpilot.common.params import Params
      self._params = Params()
    try:
      self._enabled = bool(self._params.get_bool("BPParkSensors"))
    except Exception:
      self._enabled = False

  def update(self, sm, started_frame: int, now: float | None = None) -> None:
    now = time.monotonic() if now is None else now
    dt = 0.0 if self._t is None else max(0.0, min(0.1, now - self._t))
    self._t = now
    self._refresh_param(now)

    zones = ParkZones()
    if self._enabled and sm.recv_frame.get("carStateBP", 0) > started_frame:
      zones = zones_from_park_aid(sm["carStateBP"].parkAid)
    self._zones = zones
    if zones.something_near:
      self._last_near = now
      self._shown = zones
    elif zones.active and now - self._last_near < HOLD_S:
      self._shown = zones   # the sensors still on: show them clearing

    target = 1.0 if self._enabled and now - self._last_near < HOLD_S else 0.0
    step = dt / (FADE_IN_S if target > self._alpha else FADE_OUT_S)
    self._alpha = min(target, self._alpha + step) if target > self._alpha else max(target, self._alpha - step)

  @property
  def visible(self) -> bool:
    return self._alpha > 0.0

  def render(self, rect: rl.Rectangle, dev_ui_right: bool = False, dev_ui_bottom: float = 0.0, scale: float = 1.0) -> None:
    if not self.visible:
      return
    r = rl.Rectangle(rect.x, rect.y, rect.width - (DEV_UI_RIGHT_W if dev_ui_right else 0.0), rect.height - dev_ui_bottom)
    draw_park_sensors(r, self._shown, self._alpha, scale)
