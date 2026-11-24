"""
BluePilot: shared harness for the Ford 4-signal lateral safety tests.

Not a test module (no test_ prefix). Provides:
  * BPFordLateralModel: a Python mirror of the BluePilot curvature checks in
    opendbc/bluepilot_lateral/safety/lateral.h (ford_bp_curvature_rate_lookup_check) and the lateral
    acceleration cap, computed with the same float32 arithmetic and rounding as the C, so the
    tests can assert exact CAN-unit boundaries on both x86 and ARM runners.
  * BPFordTestCase: unittest base that borrows upstream's Ford test class for its message
    builders (composition, so upstream's tests are not inherited) and sets the BP_LATERAL safety
    param bit before ford_init.
"""
import re
import unittest
from pathlib import Path

import numpy as np

from ....car.structs import CarParams
from ....car.ford.fordcan import CanBus
from ....car.ford.values import FordSafetyFlags
from ...can import fordcan_ext
from ...core.values_ext import FordSafetyFlagsSP
from ....safety.tests.libsafety import libsafety_py
from ....safety.tests.common import CANPackerSafety
from ....safety.tests import test_ford  # module import keeps its test classes out of discovery here
from ...tests import host

f32 = np.float32


def c_int(v) -> int:
  """C (int) cast: truncation toward zero."""
  return int(np.trunc(float(v)))


# --- constants mirrored from the C (opendbc/safety/modes/ford.h, opendbc/bluepilot_lateral/safety/*.h, lateral.h) ---
CURVATURE_TO_CAN = 50000.0          # FORD_LIMITS .curvature_to_can
FORD_CURVATURE_MAX = 0.02           # rad/m, FORD_CURVATURE_MAX in ford_declarations.h
MAX_CURVATURE_CAN = c_int(f32(FORD_CURVATURE_MAX) * f32(CURVATURE_TO_CAN))  # FORD_LIMITS .max_curvature, 1000
MAX_CURVATURE_ERROR_CAN = 100       # FORD_LIMITS(_, 100)
MAX_CURVATURE_ERROR_CAN_PINION = 150  # FORD_BP_STEERING_LIMITS_PINION / FORD_CANFD_STEERING_LIMITS_PINION
CURVATURE_ERROR_MIN_SPEED = 10.0    # m/s
VEHICLE_SPEED_FACTOR = 1000.0
RATE_LOOKUP_X = (5., 16., 25.)                 # m/s, FORD_LIMITS curvature_rate_{up,down}_lookup
RATE_LOOKUP_UP_Y = (0.0025, 0.0014, 0.00018)   # rad/m per frame, curvature_rate_up_lookup
RATE_LOOKUP_DOWN_Y = (0.002525, 0.001414, 0.0001818)  # curvature_rate_down_lookup: 1% looser than up (#20)
ISO_LATERAL_ACCEL = 3.0
EARTH_G = 9.81
AVERAGE_ROAD_ROLL = 0.06
BP_MAX_LATERAL_ACCEL = float(f32(ISO_LATERAL_ACCEL) - f32(EARTH_G) * f32(AVERAGE_ROAD_ROLL))  # lateral.h, ~2.41 m/s^2

PATH_ANGLE_TO_CAN = 2000.0          # FORD_PATH_ANGLE_LIMITS .angle_deg_to_can (1 / 5e-4 rad)
FORD_PATH_ANGLE_MAX = 0.25          # rad, curvature mode
FORD_DBC_PATH_ANGLE_MIN = -0.5      # rad, angle mode: the DBC's own range
FORD_DBC_PATH_ANGLE_MAX = 0.5235
PATH_ANGLE_MAX_CAN = c_int(f32(FORD_PATH_ANGLE_MAX) * f32(PATH_ANGLE_TO_CAN))          # 500
PATH_ANGLE_DBC_MIN_CAN = c_int(f32(FORD_DBC_PATH_ANGLE_MIN) * f32(PATH_ANGLE_TO_CAN))  # -1000
PATH_ANGLE_DBC_MAX_CAN = c_int(f32(FORD_DBC_PATH_ANGLE_MAX) * f32(PATH_ANGLE_TO_CAN))  # 1047, truncated like the C's (int) cast
PATH_ANGLE_LOOKUP_X = (10., 15., 25.)
PATH_ANGLE_LOOKUP_Y = (0.0561, 0.04335, 0.00918)   # rad per frame

PATH_OFFSET_TO_CAN = 100.0          # FORD_PATH_OFFSET_LIMITS .angle_deg_to_can (1 / 0.01 m)
FORD_PATH_OFFSET_MAX = 1.0          # m
PATH_OFFSET_MAX_CAN = c_int(f32(FORD_PATH_OFFSET_MAX) * f32(PATH_OFFSET_TO_CAN))  # 100
PATH_OFFSET_LOOKUP_X = (5., 15., 25.)
PATH_OFFSET_LOOKUP_Y = (0.05, 0.025, 0.01)          # m per frame

CURVATURE_RATE_TO_CAN_CANFD = 1000000.0   # LatCtlCrv_NoRate2_Actl wire resolution, 1e-6 1/m^2
CURVATURE_RATE_TO_CAN_CAN = 4000000.0     # LatCtlCurv_NoRate_Actl wire resolution, 2.5e-7 1/m^2

LATCTL_CURVATURE_SIGNAL_MAX = 0.02094  # rad/m, LatCtlCurv_No_Actl range [-0.02|0.02094] in the DBC
CURVATURE_SIGNAL_MAX_CAN = c_int(f32(LATCTL_CURVATURE_SIGNAL_MAX) * f32(CURVATURE_TO_CAN))  # 1047: the packer cannot go further
FORMER_LATCH_WINDOW_FRAMES = 70      # > the 60 frames the removed reset-bypass latch covered (#9)
SMALL_ANGLE = 0.01                  # rad: a small non-zero path_angle; probes hold it constant, so its ROC never decides


def c_interpolate(xs, ys, x) -> np.float32:
  """safety_interpolate() from opendbc/safety/helpers.h, in float32."""
  x = f32(x)
  xs = [f32(v) for v in xs]
  ys = [f32(v) for v in ys]
  ret = ys[-1]
  if x <= xs[0]:
    ret = ys[0]
  else:
    for i in range(len(xs) - 1):
      if x < xs[i + 1]:
        x0, y0 = xs[i], ys[i]
        dx = f32(max(xs[i + 1] - x0, f32(0.0001)))
        dy = f32(ys[i + 1] - y0)
        ret = f32(f32(dy * f32(x - x0)) / dx) + y0
        break
  return f32(ret)


def c_fudged_speed(speed_ms: float, offset: float) -> np.float32:
  """(vehicle_speed.x / VEHICLE_SPEED_FACTOR) + offset, as the C computes it: the sample is an
  int in mm/s, the division and the offset happen in double, and the result is rounded to
  float once when passed to safety_interpolate. speed_ms comes from libsafety's getters,
  which are the sample / 1000 rounded to float32; the integer is recovered exactly."""
  sample = round(float(speed_ms) * VEHICLE_SPEED_FACTOR)
  return f32(sample / VEHICLE_SPEED_FACTOR + offset)


class BPFordLateralModel:
  """Mirror of the BP curvature path in ford_bp_curvature_cmd_checks + ford_bp_curvature_rate_lookup_check.

  Speeds are what the C sees: vehicle_speed.min / max / values[0] in m/s (read back from
  libsafety after the rx messages so packer rounding is included).
  """

  def __init__(self, canfd: bool, max_curvature_error: int = MAX_CURVATURE_ERROR_CAN):
    self.limit_lateral_accel = canfd  # FORD_LIMITS(true, …) only for CAN FD
    self.max_curvature_error = max_curvature_error

  @staticmethod
  def rate_delta(speed_min: float, up: bool = True) -> int:
    # (safety_interpolate(lookup, (vehicle_speed.min / F) - 1.) * curvature_to_can) + 1.  -> int
    fudged = c_fudged_speed(speed_min, -1.0)
    table = RATE_LOOKUP_UP_Y if up else RATE_LOOKUP_DOWN_Y
    return c_int(float(f32(c_interpolate(RATE_LOOKUP_X, table, fudged) * f32(CURVATURE_TO_CAN))) + 1.0)

  @staticmethod
  def rate_delta_relaxed(speed_max: float, up: bool = True) -> int:
    fudged = c_fudged_speed(speed_max, 1.0)
    table = RATE_LOOKUP_UP_Y if up else RATE_LOOKUP_DOWN_Y
    return c_int(float(f32(c_interpolate(RATE_LOOKUP_X, table, fudged) * f32(CURVATURE_TO_CAN))) - 1.0)

  @staticmethod
  def accel_cap(speed_min: float) -> int:
    # BP_MAX_LATERAL_ACCEL / (fudged_speed^2) * curvature_to_can + 1, fudged = max(v.min - 1, 1)
    fudged = f32(max(float(c_fudged_speed(speed_min, -1.0)), 1.0))
    max_curv = f32(f32(BP_MAX_LATERAL_ACCEL) / f32(fudged * fudged))
    return c_int(float(f32(max_curv * f32(CURVATURE_TO_CAN))) + 1.0)

  def bounds(self, desired_last: int, meas_min: int, meas_max: int,
             speed_min: float, speed_max: float, speed_now: float) -> tuple[int, int]:
    """(lowest, highest) allowed desired curvature in CAN units for the next frame."""
    delta_up = self.rate_delta(speed_min, up=True)
    delta_down = self.rate_delta(speed_min, up=False)
    highest = desired_last + (delta_up if desired_last > 0 else delta_down)
    lowest = desired_last - (delta_down if desired_last >= 0 else delta_up)
    if self.max_curvature_error and speed_now > CURVATURE_ERROR_MIN_SPEED:
      relaxed_up = self.rate_delta_relaxed(speed_max, up=True)
      relaxed_down = self.rate_delta_relaxed(speed_max, up=False)
      lowest_err = meas_min - self.max_curvature_error - 1
      highest_err = meas_max + self.max_curvature_error + 1
      if desired_last > highest_err:
        delta = relaxed_down if desired_last >= 0 else relaxed_up
        highest = max(desired_last - delta, highest_err)
      elif desired_last < lowest_err:
        delta = relaxed_down if desired_last <= 0 else relaxed_up
        lowest = min(desired_last + delta, lowest_err)
      else:
        highest = min(highest, highest_err)
        lowest = max(lowest, lowest_err)
      lowest = max(-MAX_CURVATURE_CAN, min(MAX_CURVATURE_CAN, lowest))
      highest = max(-MAX_CURVATURE_CAN, min(MAX_CURVATURE_CAN, highest))
    return lowest, highest

  def cap(self, speed_min: float) -> int:
    """The largest |curvature| the checks accept at this speed: the lateral-accel cap on CAN FD, the
    0.02 hard limit otherwise (FORD_LIMITS .limit_lateral_acceleration)."""
    return min(self.accel_cap(speed_min), MAX_CURVATURE_CAN) if self.limit_lateral_accel else MAX_CURVATURE_CAN

  def allowed(self, desired: int, desired_last: int, meas_min: int, meas_max: int,
              speed_min: float, speed_max: float, speed_now: float) -> bool:
    """Whether a non-zero desired curvature passes the BP curvature checks (controls allowed, steer enabled)."""
    if abs(desired) > MAX_CURVATURE_CAN:
      return False
    lowest, highest = self.bounds(desired_last, meas_min, meas_max, speed_min, speed_max, speed_now)
    if desired > highest or desired < lowest:
      return False
    if self.limit_lateral_accel:
      cap = self.accel_cap(speed_min)
      if desired > cap or desired < -cap:
        return False
    return True


_NUM = r"[-+0-9.eE]+"
_GEOMETRY_ROW = re.compile(rf"\{{\.slip_factor = (?P<slip>{_NUM})f, \.steer_ratio = (?P<sr>{_NUM})f, \.wheelbase = (?P<wb>{_NUM})f\}},\s*// (?P<idx>\d+):")


def pinion_geometry_table() -> dict[int, tuple[float, float, float]]:
  """ford_pinion_geometry from opendbc/bluepilot_lateral/safety/ford.h as {index: (slip_factor, steer_ratio,
  wheelbase)}. Read from source: libsafety exposes no getters for it and adding some would touch
  upstream's harness."""
  bp = Path(__file__).resolve().parents[1]
  src = (bp / "ford.h").read_text()
  body = src[src.index("ford_pinion_geometry[FORD_PINION_GEOMETRY_ROWS] = {"):]
  body = body[:body.index("};")]
  matches = list(_GEOMETRY_ROW.finditer(body))
  # every initializer must parse (a row in a spelling the regex misses would otherwise vanish),
  # the comment index must be the row's position, and the count must be the declared size
  assert len(matches) == body.count("{.slip_factor"), "a geometry row did not parse"
  rows = {}
  for pos, m in enumerate(matches):
    assert int(m["idx"]) == pos, f"row {pos} is labeled {m['idx']}"
    rows[pos] = (float(m["slip"]), float(m["sr"]), float(m["wb"]))
  declared = re.search(r"#define FORD_PINION_GEOMETRY_ROWS (\d+)U", (bp / "ford_declarations.h").read_text())
  assert declared and int(declared[1]) == len(rows), "FORD_PINION_GEOMETRY_ROWS does not match the table"
  return rows


def angle_roc_delta(xs, ys, scale: float, speed_min: float) -> int:
  """path_angle / path_offset per-frame ROC: interpolate(lookup, v.min - 1) * scale + 1."""
  fudged = c_fudged_speed(speed_min, -1.0)
  return c_int(float(f32(c_interpolate(xs, ys, fudged) * f32(scale))) + 1.0)


class BPFordTestCase(unittest.TestCase):
  """Base for BluePilot Ford lateral tests. Subclasses set CANFD and may set PARAM_SP."""

  CANFD = True
  PARAM_SP = FordSafetyFlagsSP.BP_LATERAL

  def setUp(self):
    cls = test_ford.TestFordCANFDStockSafety if self.CANFD else test_ford.TestFordLongitudinalSafety
    self.stock = cls("test_steer_allowed")  # upstream's Ford TestCase, for its message builders only
    self.stock.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.stock.safety = libsafety_py.libsafety
    self.safety = self.stock.safety
    self.reinit()
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.model = BPFordLateralModel(self.CANFD)
    self.set_meas(0, 15.0)

  def tearDown(self):
    # stock classes in the same process must not inherit the bit
    host.set_safety_param(self.safety, 0)
    super().tearDown()  # the host's own resets (IQ.Pilot's AOL state)

  def reinit(self):
    """A safety-mode init with the BP bit set (ford_init reads the bit)."""
    host.set_safety_param(self.safety, self.PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, host.ford_param(FordSafetyFlags.CANFD if self.CANFD else 0, self.PARAM_SP))

  # --- wrappers over upstream's builders ---
  def tx(self, msg) -> bool:
    return self.stock._tx(msg)

  def rx(self, msg) -> bool:
    return self.stock._rx(msg)

  def lat(self, enabled: bool, path_offset=0.0, path_angle=0.0, curvature=0.0, curvature_rate=0.0, increment_timer=True):
    return self.stock._lat_ctl_msg(enabled, path_offset, path_angle, curvature, curvature_rate, increment_timer=increment_timer)

  def set_meas(self, curvature: float, speed: float):
    self.stock._reset_curvature_measurement(curvature, speed)

  def set_prev_curvature_can(self, can: int):
    self.safety.set_desired_curvature_last(can)

  def speeds(self) -> tuple[float, float]:
    return float(self.safety.get_vehicle_speed_min()), float(self.safety.get_vehicle_speed_max())

  def meas(self) -> tuple[int, int]:
    return int(self.safety.get_curvature_meas_min()), int(self.safety.get_curvature_meas_max())

  def lka_bp_status_msg(self, angle_mode_engaged: bool, shadow_curvature: float):
    """Lane_Assist_Data1 with BluePilot's angle-mode side channel (can/LANE_ASSIST_DATA1.md), built by
    the real packer, so every angle-mode test runs Python's packing through the panda's decode."""
    addr, dat, bus = fordcan_ext.create_lka_msg(self.stock.packer, CanBus(fingerprint={}), False, None, angle_mode_engaged,
                                                 shadow_curvature)
    return libsafety_py.make_CANPacket(addr, bus, bytes(dat))
