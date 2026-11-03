import pyray as rl
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.system.ui.lib.application import FontWeight
from openpilot.selfdrive.ui.mici.layouts.home import MiciHomeLayout

class BpMiciHomeLayout(MiciHomeLayout):
    def __init__(self):
      super().__init__()
      self._openpilot_label = UnifiedLabel("bluepilot", font_size=96, text_color=rl.Color(0, 0, 255, int(255 * 0.9)), font_weight=FontWeight.AUDIOWIDE, max_width=480, wrap_text=False)