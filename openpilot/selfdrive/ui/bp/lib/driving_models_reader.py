"""
BluePilot: the Driving Models view from the device's live state, for both homes: TICI's card
(bluepilot/ui/widgets/model_info.py) and MICI's page (selfdrive/ui/bp/mici/layouts/driving_models_page.py).

Reads JetLink's status and the link's USB speed (jetlink_status.StatusReader, at most 5 Hz), the chestnut's state and
the model manager's chestnut pick (a params read, at most once a second), and the on-device model's name
(modelManagerSP.activeBundle), and hands them to driving_models.view, which decides what the card shows.
"""
import time

from openpilot.common.params import Params
from openpilot.selfdrive.ui.bp.lib import driving_models, jetlink_status
from openpilot.selfdrive.ui.ui_state import ui_state

# the chestnut pick's name: a params read, once a second at most
CHESTNUT_NAME_PERIOD_S = 1.0


class ViewReader:
  def __init__(self, params: Params | None = None, clock=time.monotonic):
    self._params = params if params is not None else Params()
    self._clock = clock
    self._reader = jetlink_status.StatusReader(clock=clock, speed=True)
    self._chestnut_name: str | None = None
    self._chestnut_name_at = -float('inf')
    self.view = driving_models.view(None)

  def update(self) -> driving_models.View:
    status = self._reader.get()
    chestnut_state = getattr(ui_state.chestnut_state, 'value', 'disconnected')
    if not jetlink_status.shown(status) and chestnut_state != 'disconnected':
      self._refresh_chestnut_name()
    self.view = driving_models.view(self._active_model_name(), status, self._reader.speed(), chestnut_state,
                                    self._chestnut_name)
    return self.view

  def _refresh_chestnut_name(self):
    now = self._clock()
    if now - self._chestnut_name_at < CHESTNUT_NAME_PERIOD_S:
      return
    self._chestnut_name_at = now
    try:
      from openpilot.sunnypilot.models.helpers import get_selected_bundle
      bundle = get_selected_bundle(self._params, "chestnut")
      self._chestnut_name = bundle.displayName if bundle is not None and bundle.displayName else None
    except Exception:
      self._chestnut_name = None

  @staticmethod
  def _active_model_name() -> str | None:
    try:
      model_manager = ui_state.sm["modelManagerSP"]
      if model_manager.activeBundle.ref:
        return model_manager.activeBundle.displayName
    except Exception:
      pass
    return None
