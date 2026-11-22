"""BluePilot MICI: JetLink settings panel — the big-model link selector and the charge-phone toggle, offroad only.

Placed as its own sub-panel until the MICI menu redesign settles where it lives."""

from collections.abc import Callable

from openpilot.selfdrive.ui.bp.lib import jetlink_settings, jetlink_status
from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigMultiParamToggleBP, BigParamControlBP
from openpilot.selfdrive.ui.mici.widgets.button import BigMultiToggle, GreyBigButton
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets.scroller import NavScroller


class JetlinkLinkToggle(BigMultiParamToggleBP):
  """JetlinkLink as Off / USB. A value the selector does not offer (iOS) shows Off; tapping writes only the modes
  offered, and only offroad."""

  def _load_value(self):
    index = jetlink_settings.selected_index(self._params)
    self.set_value(self._options[max(index, 0)])

  def _handle_mouse_release(self, mouse_pos):
    if not ui_state.is_offroad():
      return
    # advance the option on screen (BigMultiToggle), but store through the offroad-only writer, not
    # BigMultiParamToggle's put of the raw index
    BigMultiToggle._handle_mouse_release(self, mouse_pos)
    if not jetlink_settings.write_link(self._params, self._options.index(self.value), ui_state.is_offroad()):
      self._load_value()


class JetlinkStatusCard(GreyBigButton):
  """The link's state: the chip as the title, the port, progress and the ADB note under it. Not tappable."""

  def __init__(self):
    super().__init__("jetlink", "")
    self._reader = jetlink_status.StatusReader()
    self._shown: tuple[str, str] | None = None

  def _update_state(self):
    super()._update_state()
    status = self._reader.get()
    text = f"jetlink: {jetlink_status.chip(status).lower()}"
    value = "\n".join(jetlink_status.detail_lines(status)[:3])
    if (text, value) != self._shown:
      self._shown = (text, value)
      self.set_text(text)
      self.set_value(value)


class JetlinkLayoutMici(NavScroller):
  def __init__(self, back_callback: Callable[[], None] | None = None):
    super().__init__()
    if back_callback is not None:
      self.set_back_callback(back_callback)

    self.link = JetlinkLinkToggle("big model link", jetlink_settings.LINK_PARAM, list(jetlink_settings.LINK_LABELS.values()))
    self.link.set_enabled(lambda: ui_state.is_offroad())
    self.charge_phone = BigParamControlBP("jetlink: charge phone", jetlink_settings.CHARGE_PHONE_PARAM)
    self.charge_phone.set_enabled(lambda: ui_state.is_offroad())

    self.status = JetlinkStatusCard()

    self._scroller.add_widgets([
      self.link,
      self.status,
      self.charge_phone,
    ])
    ui_state.add_offroad_transition_callback(self._update_buttons)

  def _update_buttons(self) -> None:
    self.link._load_value()
    self.charge_phone.refresh()

  def show_event(self):
    super().show_event()
    self._update_buttons()
