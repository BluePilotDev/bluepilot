"""C4 DCam trigger submenu using the native compact settings layout."""

from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigParamControlBP
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.scroller import NavScroller


class DcamParamControlBP(BigParamControlBP):
  def _width_hint(self) -> int:
    # Reserve the toggle pill's space so longer DCam titles never run under it.
    return int(self._rect.width - self.LABEL_HORIZONTAL_PADDING * 2 - self._txt_enabled_toggle.width)


class DcamTriggersLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    self.set_back_callback(gui_app.pop_widget)
    for title, description, key in (
      ('When Turn Signal Active', 'Show DCam view when turn signal active', 'BPCroppedDcamTurnSignal'),
      ('When BSM Active', 'Show DCam view when vehicle Blind-Spot Monitoring System active', 'BPCroppedDcamBSM'),
    ):
      option = DcamParamControlBP(title, key)
      option.set_value(description)
      option._sub_label.set_font_size(24)
      option.set_enabled(lambda params=option.params: params.get_bool('BPCroppedDcam'))
      self._scroller.add_widgets([option])
