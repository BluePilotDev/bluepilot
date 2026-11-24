from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.bp.onroad.torque_bar_state_bp import TorqueBarStateBP


class TorqueBarBP(TorqueBarStateBP, TorqueBar):
  """BluePilot Mici torque bar with shared BP torque-state math.

  Rendering stays inherited from Mici, while the update math is shared with the
  TICI BP renderer.
  """

  def _update_state(self):
    if self._demo:
      return

    try:
      self._update_torque_filter_bp()
    except (KeyError, AttributeError):
      pass
