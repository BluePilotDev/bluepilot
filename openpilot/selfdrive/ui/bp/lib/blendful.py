from openpilot.selfdrive.ui.ui_state import ui_state


def blendful_running() -> bool:
  """Blendful's toggle is on and a tinygrad model runs (the case controlsd uses): Lanefull then reaches the car code
  as off in both modes (#285), so the settings screens show both Lanefull toggles off and locked. The saved settings
  are not written. Call on a state change, not per frame: it reads a param."""
  return ui_state.params.get_bool("Blendful") and ui_state.active_runner_tinygrad
