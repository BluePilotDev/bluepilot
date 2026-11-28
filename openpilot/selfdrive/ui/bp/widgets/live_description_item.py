"""BluePilot: a ListItem whose description may change while it is open.

Stock ListItem measures its height only when its description is opened or closed, so a description that grows while
open (JetLink's status, read 5 times a second) draws over the rows below it. This one measures again whenever the
open description's text changes."""
from collections.abc import Callable

from openpilot.system.ui.widgets.list_view import ITEM_PADDING, ITEM_TEXT_VALUE_COLOR, ListItem, TextAction


class LiveDescriptionItem(ListItem):
  def _update_state(self):
    changed = self.description != self._prev_description
    super()._update_state()
    if changed and self.description_visible:
      self._rect.height = self.get_item_height(self._font, int(self._rect.width - ITEM_PADDING * 2))


def live_text_item(title: str | Callable[[], str], value: str | Callable[[], str],
                   description: str | Callable[[], str] | None = None) -> LiveDescriptionItem:
  """text_item(), with a description that may change while it is open."""
  return LiveDescriptionItem(title=title, description=description, action_item=TextAction(text=value, color=ITEM_TEXT_VALUE_COLOR))
