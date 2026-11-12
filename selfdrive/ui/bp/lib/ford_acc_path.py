"""Blended path feedback; the selected base shader/gradient stays intact."""

import math
import time


class FordAccPathFeedback:
  def __init__(self):
    self.value = 0.0
    self.last_time = time.monotonic()

  def update(self, enabled, sm, openpilot_longitudinal):
    now = time.monotonic()
    dt = min(max(now - self.last_time, 0.0), 0.1)
    self.last_time = now
    if (not enabled or openpilot_longitudinal or
        not sm.all_checks(['carStateBP', 'carState', 'carControl'])):
      self.value = 0.0
      return 0.0
    cs = sm['carState']
    feedback = sm['carStateBP'].fordAcc
    if (not feedback.available or not cs.cruiseState.enabled or cs.gasPressed or
        cs.brakePressed or sm['carControl'].longActive):
      self.value = 0.0
      return 0.0
    target = float(feedback.feedback)
    if not math.isfinite(target):
      self.value = 0.0
      return 0.0
    self.value += (max(-1.0, min(1.0, target)) - self.value) * (1 - math.exp(-dt / 0.25))
    return self.value


def draw_ford_acc_feedback(renderer, sm, points, opacity=1.0):
  # Imports stay here so signal handling and gating can be tested without a display.
  import pyray as rl
  from openpilot.selfdrive.ui.ui_state import ui_state
  from openpilot.system.ui.lib.shader_polygon import Gradient, draw_polygon

  if not hasattr(renderer, '_ford_acc_feedback'):
    renderer._ford_acc_feedback = FordAccPathFeedback()
  value = renderer._ford_acc_feedback.update(
    renderer._ford_acc_path_enabled, sm, getattr(ui_state.CP, 'openpilotLongitudinalControl', True),
  )
  if abs(value) < 0.01 or not points.size or renderer._rad_racer:
    return
  rgb = (235, 85, 40) if value < 0 else (95, 235, 175)
  alpha = int((155 if value < 0 else 125) * abs(value) * max(0.0, min(1.0, opacity)))
  draw_polygon(renderer._rect, points, gradient=Gradient(
    start=(0.0, 1.0), end=(0.0, 0.0),
    colors=[rl.Color(*rgb, alpha), rl.Color(*rgb, int(alpha * 0.8)), rl.Color(*rgb, 0)], stops=[0.0, 0.55, 1.0],
  ))
