#!/usr/bin/env python3
"""
BluePilot Ford lateral: the pinion-sourced curvature measurement (STEER_ANGLE_CURVATURE).

With bit 0 of the SP safety param set (and BP_LATERAL, which it requires), the panda measures
curvature from the PSCM steering pinion angle through the per-platform geometry table in
bluepilot_lateral/safety/ford.h instead of the RCM yaw rate, and widens the error band to 150 CAN units.
The geometry index rides bits 1-4. Ported from BluePilot's earlier vendored tests onto the
BP matrix classes; the geometry-table check reads the C table from source (no libsafety
getters), so no upstream harness file is touched.
"""
import unittest
from types import SimpleNamespace

import numpy as np

from ....car.ford.values import CAR
from ....car.vehicle_model import VehicleModel, calc_slip_factor
from ...core.values_ext import (
  FordSafetyFlagsSP, FORD_HOST_ONLY_PLATFORMS, FORD_NO_PINION_PLATFORMS, FORD_PINION_GEOMETRY_INDEX, FORD_PINION_GEOMETRY_SHIFT,
  FORD_PINION_WHEELBASE,
)
from ...core.carcontroller_ext import pinion_vehicle_model, scale_tire_stiffness_per_mass
from ....car.ford.interface import CarInterface
from .ford_bluepilot_common import (
  CURVATURE_TO_CAN, CURVATURE_ERROR_MIN_SPEED, MAX_CURVATURE_ERROR_CAN_PINION, pinion_geometry_table,
)
from . import test_ford_bluepilot_matrix as matrix


def pinion_param_sp(geometry_index: int) -> int:
  return int(FordSafetyFlagsSP.BP_LATERAL) | int(FordSafetyFlagsSP.STEER_ANGLE_CURVATURE) | (geometry_index << FORD_PINION_GEOMETRY_SHIFT)


GEOMETRY = pinion_geometry_table()  # the C table, so the platform mixins can never drift from it


class BPPinionMixin:
  """Pinion-angle measurement source. The geometry comes from the C table row for GEOMETRY_INDEX;
  TestFordBPPinionGeometryTable checks that row against CarSpecs."""

  MAX_CURVATURE_ERROR_CAN_BP = MAX_CURVATURE_ERROR_CAN_PINION
  MAX_CURVATURE_ERROR = MAX_CURVATURE_ERROR_CAN_PINION / CURVATURE_TO_CAN  # the host's own error-band tests (IQ.Pilot's test_ford)
  GEOMETRY_INDEX = 0
  PINION_SLIP_FACTOR, PINION_STEER_RATIO, PINION_WHEELBASE = GEOMETRY[0]
  cnt_pinion = 0

  def __init_subclass__(cls, **kwargs):
    # a platform sets GEOMETRY_INDEX only; the geometry and the safety param follow from it
    super().__init_subclass__(**kwargs)
    cls.PINION_SLIP_FACTOR, cls.PINION_STEER_RATIO, cls.PINION_WHEELBASE = GEOMETRY[cls.GEOMETRY_INDEX]
    cls.PARAM_SP = pinion_param_sp(cls.GEOMETRY_INDEX)

  def _curvature_factor(self, speed: float) -> float:
    speed = max(speed, 0.1)
    return 1. / (1. - (self.PINION_SLIP_FACTOR * (speed ** 2))) / self.PINION_WHEELBASE

  def _curvature_to_pinion_angle_deg(self, curvature: float, speed: float) -> float:
    # inverse of ford_bp_rx_hook: curvature = angle_rad * curvature_factor(speed) / steer_ratio
    return float(np.degrees(curvature * self.PINION_STEER_RATIO / self._curvature_factor(speed)))

  def _pinion_quant_tol(self, speed: float) -> int:
    # 0.1 deg DBC quantization in curvature CAN units at this speed, +2 for float rounding
    return int(np.radians(0.1) * self._curvature_factor(speed) / self.PINION_STEER_RATIO * CURVATURE_TO_CAN) + 2

  def _pinion_msg(self, curvature: float, speed: float, quality_flag=True, bus=0):
    values = {"StePinComp_An_Est": self._curvature_to_pinion_angle_deg(curvature, speed),
              "StePinCompAnEst_D_Qf": 3 if quality_flag else 0,
              "StePinAn_No_Cnt": self.cnt_pinion % 16}
    self.__class__.cnt_pinion += 1
    return self.packer.make_can_msg_safety("SteeringPinion_Data", bus, values)

  def _reset_curvature_measurement(self, curvature, speed):
    # 14 frames, not 6: after a counter discontinuity (rejected bad-QF frames still advance the
    # python-side counter) the rx counter check drops frames until it re-syncs
    for _ in range(14):
      self._rx(self._speed_msg(speed))
      self._rx(self._speed_msg_2(speed))  # ford_bp_curvature_cmd_checks cross-checks the two speed sources
      self._rx(self._pinion_msg(curvature, speed))

  def test_rx_hook(self):
    # checksum, counter and quality flag checks: the stock matrix plus the pinion message
    for quality_flag in (True, False):
      for msg_type in ("speed", "speed_2", "yaw", "pinion"):
        self.safety.set_controls_allowed(True)
        for _ in range(10):
          if msg_type == "speed":
            msg = self._speed_msg(0, quality_flag=quality_flag)
          elif msg_type == "speed_2":
            msg = self._speed_msg_2(0, quality_flag=quality_flag)
          elif msg_type == "yaw":
            msg = self._yaw_rate_msg(0, 0, quality_flag=quality_flag)
          else:
            msg = self._pinion_msg(0, 0, quality_flag=quality_flag)
          self.assertEqual(quality_flag, self._rx(msg))
          self.assertEqual(quality_flag, self.safety.get_controls_allowed())

        # corrupt the checksum byte; not checked on the 2nd speed source or the pinion message
        # (unknown OEM checksum: integrity is counter + quality flag)
        msg[0].data[3] = 0
        should_rx = msg_type in ("speed_2", "pinion") and quality_flag
        self.assertEqual(should_rx, self._rx(msg))
        self.assertEqual(should_rx, self.safety.get_controls_allowed())

  def test_angle_measurements(self):
    """The rx hook converts pinion angle to curvature through the geometry row, within the
    signal's 0.1 deg quantization."""
    for speed in np.arange(0.5, 40, 0.5):
      for curvature in np.arange(0, 0.02 * 2, 2e-3):
        self._rx(self._speed_msg(speed))
        for c in (curvature, -curvature, 0, 0, 0, 0):
          self._rx(self._pinion_msg(c, speed))
        tol = self._pinion_quant_tol(speed)
        self.assertAlmostEqual(self.safety.get_curvature_meas_min(), round(-curvature * CURVATURE_TO_CAN), delta=tol)
        self.assertAlmostEqual(self.safety.get_curvature_meas_max(), round(curvature * CURVATURE_TO_CAN), delta=tol)
        self._rx(self._pinion_msg(0, speed))
        self.assertAlmostEqual(self.safety.get_curvature_meas_min(), round(-curvature * CURVATURE_TO_CAN), delta=tol)
        self.assertAlmostEqual(self.safety.get_curvature_meas_max(), 0, delta=tol)
        self._rx(self._pinion_msg(0, speed))
        self.assertAlmostEqual(self.safety.get_curvature_meas_min(), 0, delta=tol)
        self.assertAlmostEqual(self.safety.get_curvature_meas_max(), 0, delta=tol)

  def test_pinion_quality_flag_gates_measurement(self):
    speed = CURVATURE_ERROR_MIN_SPEED + 5
    self._reset_curvature_measurement(0.005, speed)
    before = self.safety.get_curvature_meas_max()
    self.assertGreater(before, 0)
    for _ in range(6):
      self.assertFalse(self._rx(self._pinion_msg(0, speed, quality_flag=False)))
    self.assertEqual(self.safety.get_curvature_meas_max(), before)

  def test_pinion_other_bus_ignored(self):
    # only the main bus pinion frame feeds the measurement (its rx check entry is bus 0; ford_bp_rx_hook also matches the bus)
    speed = CURVATURE_ERROR_MIN_SPEED + 5
    self._reset_curvature_measurement(0.005, speed)
    before = self.safety.get_curvature_meas_max()
    for _ in range(6):
      self._rx(self._pinion_msg(-0.005, speed, bus=2))
    self.assertEqual(self.safety.get_curvature_meas_max(), before)

  def test_pinion_sign_convention(self):
    """A command matching the measured sign passes the error band; a sign-inverted command (the
    broken-yaw failure mode this source exists for) is blocked above the gate speed."""
    speed = CURVATURE_ERROR_MIN_SPEED + 5
    curvature = 0.005  # well above the 150-unit band
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self.safety.set_controls_allowed(True)
        self._reset_curvature_measurement(sign * curvature, speed)
        self._set_prev_desired_angle(sign * curvature)
        self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, sign * curvature, 0)))
        self._set_prev_desired_angle(-sign * curvature)
        self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, -sign * curvature, 0)))

  def test_pinion_check_inert_below_gate_speed(self):
    self.safety.set_controls_allowed(True)
    speed = CURVATURE_ERROR_MIN_SPEED - 2
    self._reset_curvature_measurement(0.005, speed)
    inverted = -0.005
    self._set_prev_desired_angle(inverted)
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, inverted, 0)))


class FordExplorerPinionGeometry:  # GEOMETRY_INDEX only; BPPinionMixin.__init_subclass__ fills in the rest
  """FORD_EXPLORER_MK6, the on-road-validated primary platform."""
  GEOMETRY_INDEX = 5


class FordBroncoSportPinionGeometry:  # GEOMETRY_INDEX only; BPPinionMixin.__init_subclass__ fills in the rest
  """FORD_BRONCO_SPORT_MK1, smallest wheelbase in the table."""
  GEOMETRY_INDEX = 1


class FordF150PinionGeometry:  # GEOMETRY_INDEX only; BPPinionMixin.__init_subclass__ fills in the rest
  """FORD_F_150_MK14, largest wheelbase in the table."""
  GEOMETRY_INDEX = 8


class TestFordBPPinionLongitudinalSafety(FordExplorerPinionGeometry, BPPinionMixin, matrix.TestFordBPLongitudinalSafety):
  pass


class TestFordBPPinionCANFDStockSafety(FordExplorerPinionGeometry, BPPinionMixin, matrix.TestFordBPCANFDStockSafety):
  pass


class TestFordBPPinionCANFDLongitudinalSafety(FordExplorerPinionGeometry, BPPinionMixin, matrix.TestFordBPCANFDLongitudinalSafety):
  pass


class TestFordBPPinionBroncoSportSafety(FordBroncoSportPinionGeometry, BPPinionMixin, matrix.TestFordBPLongitudinalSafety):
  pass


class TestFordBPPinionF150Safety(FordF150PinionGeometry, BPPinionMixin, matrix.TestFordBPCANFDLongitudinalSafety):
  pass


class TestFordBPPinionGeometryTable(unittest.TestCase):
  """The C geometry table must match CarSpecs + calc_slip_factor(VehicleModel(CP)) for every
  platform in FORD_PINION_GEOMETRY_INDEX, so it cannot rot as platforms change. Where BluePilot
  deliberately uses another wheelbase (FORD_PINION_WHEELBASE), the row uses that one, and so does
  the Python pinion path."""

  def _assert_row_matches_carspecs(self, car, idx):
    specs = car.config.specs
    wheelbase = FORD_PINION_WHEELBASE.get(car, specs.wheelbase)
    center_to_front = wheelbase * specs.centerToFrontRatio
    front, rear = scale_tire_stiffness_per_mass(wheelbase, center_to_front, specs.tireStiffnessFactor)
    # both VehicleModel signatures, as pinion_vehicle_model builds it (slip factor doesn't read inertia)
    CP = SimpleNamespace(wheelbase=wheelbase, steerRatio=specs.steerRatio, centerToFront=center_to_front,
                         tireStiffnessFrontPerMass=front, tireStiffnessRearPerMass=rear, rotationalInertiaPerMass=1.0,
                         mass=1.0, tireStiffnessFront=front, tireStiffnessRear=rear, rotationalInertia=1.0, steerRatioRear=0.)
    slip_factor = calc_slip_factor(VehicleModel(CP))

    slip, sr, wb = GEOMETRY[idx]
    self.assertAlmostEqual(sr, specs.steerRatio, places=3, msg=str(car))
    self.assertAlmostEqual(wb, wheelbase, places=3, msg=str(car))
    self.assertAlmostEqual(slip, slip_factor, delta=abs(slip_factor) * 1e-4, msg=str(car))

  def test_geometry_matches_carspecs(self):
    rows = GEOMETRY
    # the index rides bits 1-4 of current_safety_param_sp; growing past 15 would silently
    # disable the firmware side while the control side still enables
    self.assertLessEqual(len(rows) - 1, 15)
    # every supported platform has a row unless it can't take the pinion source (FORD_NO_PINION_PLATFORMS), every
    # Python index entry points at a C row. A row for a platform this host's CAR doesn't have
    # (FORD_HOST_ONLY_PLATFORMS) has no CarSpecs here
    self.assertEqual({n for n in FORD_PINION_GEOMETRY_INDEX if n in set(CAR)}, set(CAR) - FORD_NO_PINION_PLATFORMS,
                     "every Ford platform needs a geometry row")
    self.assertTrue(set(FORD_PINION_GEOMETRY_INDEX) - set(CAR) <= FORD_HOST_ONLY_PLATFORMS)
    self.assertTrue(set(FORD_PINION_GEOMETRY_INDEX.values()) <= set(rows) - {0})
    self.assertEqual(len(set(FORD_PINION_GEOMETRY_INDEX.values())), len(FORD_PINION_GEOMETRY_INDEX), "duplicate index")

    for name, idx in FORD_PINION_GEOMETRY_INDEX.items():
      if name in set(CAR):
        self._assert_row_matches_carspecs(CAR(name), idx)

  def test_f150_uses_the_average_wheelbase(self):
    """#21: F-150s with lane centering come in 3.68 and 3.99 m wheelbases and nothing tells them
    apart, so pinion mode uses the average on both sides: the C row and the Python model."""
    self.assertEqual(FORD_PINION_WHEELBASE[CAR.FORD_F_150_MK14], 3.84)
    self.assertAlmostEqual(GEOMETRY[FORD_PINION_GEOMETRY_INDEX[CAR.FORD_F_150_MK14]][2], 3.84, places=3)
    CP = CarInterface.get_non_essential_params(CAR.FORD_F_150_MK14)
    VM = pinion_vehicle_model(CP)
    self.assertAlmostEqual(VM.l, 3.84, places=3)
    self.assertAlmostEqual(VM.aF / VM.l, CP.centerToFront / CP.wheelbase, places=6)  # the platform's weight split is kept

  def test_other_platforms_use_carspecs(self):
    for car in FORD_PINION_GEOMETRY_INDEX:
      if car in FORD_PINION_WHEELBASE or car not in set(CAR):
        continue
      CP = CarInterface.get_non_essential_params(car)
      with self.subTest(car=car):
        self.assertAlmostEqual(pinion_vehicle_model(CP).l, CP.wheelbase, places=6)

  def test_invalid_index_row_is_inert(self):
    self.assertEqual(GEOMETRY[0], (0.0, 1.0, 1.0))


if __name__ == "__main__":
  unittest.main()
