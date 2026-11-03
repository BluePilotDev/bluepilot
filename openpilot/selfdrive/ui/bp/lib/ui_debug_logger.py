"""
BluePilot UI Debug Logger — transition-only logging for diagnosing onroad rendering issues.

Only logs when state changes (visible→invisible, data becoming unavailable, etc).
Zero cost when disabled (single bool check). Toggled via BPUIDebugLog param.

Usage:
  from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log

  bp_ui_log.visibility("HybridGauge", visible=False, reason="dataAvailable=false")
  bp_ui_log.state("HudRenderer", "brakes_on", True)
  bp_ui_log.scissor("PowerFlowGauge", "begin", x=100, y=200, w=300, h=400)
  bp_ui_log.event("AugmentedRoadView", "scissor reset triggered")

On device: journalctl -t ui -f | grep BP_UI
"""

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog


class BPUIDebugLogger:
  """Singleton logger that only emits output on state transitions."""

  _instance = None

  def __new__(cls):
    if cls._instance is None:
      cls._instance = super().__new__(cls)
      cls._instance._initialized = False
    return cls._instance

  def __init__(self):
    if self._initialized:
      return
    self._initialized = True
    self._enabled = False
    self._params = Params()
    self._frame_count = 0
    self._param_check_interval = 300  # Check param every ~300 frames (~5s at 60fps)

    # State tracking dicts for transition detection
    self._visibility_state: dict[str, bool] = {}
    self._value_state: dict[str, object] = {}

    # Initial param check
    self._refresh_enabled()

  def _refresh_enabled(self):
    try:
      self._enabled = self._params.get_bool("BPUIDebugLog")
    except Exception:
      self._enabled = False

  def tick(self):
    """Call once per frame to periodically refresh the enabled param."""
    self._frame_count += 1
    if self._frame_count >= self._param_check_interval:
      self._frame_count = 0
      self._refresh_enabled()

  @property
  def enabled(self) -> bool:
    return self._enabled

  def visibility(self, component: str, visible: bool, reason: str = ""):
    """Log only when a component's visibility changes."""
    if not self._enabled:
      return
    prev = self._visibility_state.get(component)
    if prev == visible:
      return
    self._visibility_state[component] = visible
    state_str = "VISIBLE" if visible else "HIDDEN"
    reason_str = f" ({reason})" if reason else ""
    cloudlog.warning(f"BP_UI [{component}] {state_str}{reason_str}")

  def state(self, component: str, key: str, new_value):
    """Log only when a tracked value changes."""
    if not self._enabled:
      return
    full_key = f"{component}.{key}"
    prev = self._value_state.get(full_key)
    if prev == new_value:
      return
    self._value_state[full_key] = new_value
    cloudlog.warning(f"BP_UI [{component}] {key}: {prev} -> {new_value}")

  def scissor(self, component: str, action: str, x: int = 0, y: int = 0, w: int = 0, h: int = 0):
    """Log scissor begin/end/reset operations."""
    if not self._enabled:
      return
    if action == "begin":
      cloudlog.warning(f"BP_UI [{component}] scissor begin x={x} y={y} w={w} h={h}")
    else:
      cloudlog.warning(f"BP_UI [{component}] scissor {action}")

  def event(self, component: str, message: str):
    """Log a one-shot event (errors, unexpected conditions)."""
    if not self._enabled:
      return
    cloudlog.warning(f"BP_UI [{component}] {message}")


# Module-level singleton
bp_ui_log = BPUIDebugLogger()
