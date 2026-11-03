import numpy as np
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.ui.ui_state import ui_state


DEFAULT_MAX_LAT_ACCEL_BP = 3.0  # m/s^2
ROLL_COMPENSATION_MAX_WEIGHT = 0.5


class TorqueBarStateBP:
  """Shared BluePilot torque bar state update math.

  Renderers stay separate between TICI and MICI, but the torque estimate should
  stay in one BP-owned place so upstream math changes are easier to absorb.
  """

  roll_compensation_max_weight = ROLL_COMPENSATION_MAX_WEIGHT
  default_max_lateral_accel = DEFAULT_MAX_LAT_ACCEL_BP

  def _update_torque_filter_bp(self) -> None:
    if ui_state.sm['controlsState'].lateralControlState.which() == 'angleState':
      controls_state = ui_state.sm['controlsState']
      car_state = ui_state.sm['carState']
      live_parameters = ui_state.sm['liveParameters']
      car_control = ui_state.sm['carControl']

      actual_lateral_accel = controls_state.curvature * car_state.vEgo ** 2
      desired_lateral_accel = controls_state.desiredCurvature * car_state.vEgo ** 2
      accel_diff = (desired_lateral_accel - actual_lateral_accel)

      roll_compensation = live_parameters.roll * ACCELERATION_DUE_TO_GRAVITY * self._roll_compensation_weight_bp(car_state.vEgo)
      lateral_acceleration = actual_lateral_accel - roll_compensation
      max_lateral_acceleration = ui_state.CP.maxLateralAccel if ui_state.CP else self.default_max_lateral_accel

      if not car_control.latActive:
        self._torque_filter.update(0.0)
      else:
        self._torque_filter.update(np.clip((lateral_acceleration + accel_diff) / max_lateral_acceleration, -1, 1))
    else:
      self._torque_filter.update(-ui_state.sm['carOutput'].actuatorsOutput.torque)

  def _roll_compensation_weight_bp(self, v_ego: float) -> float:
    # Roll is less accurate near standstill, so reduce its effect at low speed.
    # Full roll compensation makes crowned straightaways look like steering bias.
    return np.interp(v_ego, [5, 15], [0.0, self.roll_compensation_max_weight])
