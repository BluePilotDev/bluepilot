"""BluePilot MICI Audio settings panel."""

from collections.abc import Callable

from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigMultiParamToggleBP, BigParamControlBP
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller


class AudioLayoutMici(NavScroller):
  def __init__(self, back_callback: Callable[[], None] | None = None):
    super().__init__()
    if back_callback is not None:
      self.set_back_callback(back_callback)

    self.use_custom_sounds = BigParamControlBP(
      "Use Custom Engage/Disengage Sounds",
      "BPUseCustomSounds",
      toggle_callback=self._on_custom_sounds_toggled,
    )
    self.custom_sound_selection = BigMultiParamToggleBP(
      "Engage/Disengage Sound",
      "BPCustSoundsSelection",
      ["Comma 4", "Comma 3x", "Tesla"],
      select_callback=self._on_sound_selection_changed,
    )

    self._scroller.add_widgets([
      self.use_custom_sounds,
      self.custom_sound_selection,
    ])
    self._update_buttons()

  def _on_custom_sounds_toggled(self, enabled: bool) -> None:
    self.custom_sound_selection.set_enabled(enabled)

  def _on_sound_selection_changed(self, _selection: str) -> None:
    self._prompt_sound_reboot()

  def _prompt_sound_reboot(self) -> None:
    icon = gui_app.texture("icons_mici/settings/device/reboot.png", 64, 64)
    dialog = BigConfirmationDialog(
      tr("For these sound changes to take effect, you will need to reboot your device.\n\nTHIS REBOOT WILL DISENGAGE OPENPILOT.\n\nSwipe to reboot now, or go back to reboot later."),  # noqa: E501
      icon,
      confirm_callback=lambda: self.use_custom_sounds.params.put_bool("DoReboot", True, block=False),
    )
    gui_app.push_widget(dialog)

  def _update_buttons(self) -> None:
    self.use_custom_sounds.refresh()
    self.custom_sound_selection._load_value()
    self.custom_sound_selection.set_enabled(self.use_custom_sounds.params.get_bool("BPUseCustomSounds"))

  def show_event(self):
    super().show_event()
    self._update_buttons()
