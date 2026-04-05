import numpy as np
import pyray as rl
from openpilot.selfdrive.ui.mici.onroad.model_renderer import ModelRenderer, THROTTLE_COLORS, NO_THROTTLE_COLORS
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
# BluePilot: Rainbow shader moved to BP module after upstream removal
from openpilot.bluepilot.ui.lib.bp_shaders import draw_rainbow_polygon

class ModelRendererBP(ModelRenderer):
  def __init__(self):
    super().__init__()
    self._rainbow_v = 20

  def _update_state(self):
    super()._update_state()
    sm = ui_state.sm

    if ui_state.rainbow_path:
      v= sm['carState'].vEgo
      self._rainbow_v = np.clip(v, 2.5, 35) / 30

  def _draw_path(self, sm):
    if ui_state.rainbow_path:
      draw_rainbow_polygon(self._rect, self._path.projected_points, rainbow_v=self._rainbow_v)
    else:
      super()._draw_path(sm)