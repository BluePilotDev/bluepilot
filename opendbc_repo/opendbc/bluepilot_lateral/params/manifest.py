"""
BluePilot: every setting the lateral library reads, in one place.

Each Setting says what the host stores (key, type, default: its params_keys.h line), what the user
sees (label, description, range: the BluePilot UI's text and limits), and how the library reads it
(at startup, every frame or both; clamped to its range or not). The library reads every setting
through `value` (per frame, from the CarControlSP.params ParamStore) or `startup_value` (get_car's
params dict), with one rule: a published, parseable value is used; missing, empty or unparsable
gives the default.

`python -m <pkg>.bluepilot_lateral.params.manifest` prints the lines for a host's params_keys.h (<pkg>: the host's opendbc package).
"""
from dataclasses import dataclass
from typing import Literal

STARTUP, PER_FRAME, BOTH = "startup", "per-frame", "both"


@dataclass(frozen=True)
class Setting:
  key: str
  type: type
  default: bool | int | float
  read: Literal["startup", "per-frame", "both"]
  label: str
  description: str
  range: tuple[float, float] | None = None  # the UI's limits
  clamp: bool = False  # the library clamps to range (the angle-mode factors only, as BluePilot does)


SETTINGS: tuple[Setting, ...] = (
  Setting("disable_BP_lat_UI", bool, False, BOTH, "Disable BP Lateral Control",
          "Disable BluePilot lateral control."),
  Setting("enable_human_turn_detection_curv", bool, True, PER_FRAME, "Enable Human Turn Detection",
          "Enable detection of human-initiated turns."),
  Setting("enable_lane_positioning_curv", bool, False, PER_FRAME, "Enable Lane Positioning",
          "Enable custom lane positioning controls."),
  Setting("enable_lane_full_mode_curv", bool, False, PER_FRAME, "Enable Lanefull Mode",
          "Enable lanefull mode for lane positioning."),
  Setting("enable_lane_positioning_ang", bool, False, PER_FRAME, "Enable Lane Positioning",
          "Nudge the vehicle toward true lane-line center (plus optional bias) while in angle-primary control."),
  Setting("FordPrefLateralControl", int, 0, PER_FRAME, "Primary Control Variable",
          "Curvature matches the existing strategy. Angle uses path_angle as the main actuator (in development).",
          range=(0, 1)),
  # an on/off toggle in the UI, stored as INT
  Setting("custom_profile_curv", int, 0, PER_FRAME, "Use Custom Tuning Profile",
          "Enable custom tuning profile settings.", range=(0, 1)),
  Setting("lane_change_factor_high_curv", float, 0.85, PER_FRAME, "Lane Change Factor High",
          "Scales steering during a lane change (curvature control). Values <1.0 reduce it.", range=(0.5, 1.0)),
  Setting("pc_blend_ratio_high_C_UI_curv", float, 0.4, PER_FRAME, "Predicted Curvature Blend Ratio High",
          "Adjust the high curvature blend ratio (0.0-1.0).", range=(0.0, 1.0)),
  Setting("pc_blend_ratio_low_C_UI_curv", float, 0.4, PER_FRAME, "Predicted Curvature Blend Ratio Low",
          "Adjust the low curvature blend ratio (0.0-1.0).", range=(0.0, 1.0)),
  Setting("custom_path_offset_curv", float, 0.0, PER_FRAME, "In-Lane Offset",
          "Adjust the in-lane offset (-0.5 to 0.5).", range=(-0.5, 0.5)),
  Setting("LC_PID_gain_UI_curv", float, 1.0, PER_FRAME, "Centering PID gain",
          "PID gain for the curvature-mode centering controller. Applies whenever lane positioning is on.",
          range=(0.0, 50.0)),
  Setting("FordLowSpeedFactor_ang", float, 1.0, PER_FRAME, "Low Speed Adjustment Factor",
          "Scales the low-speed steering response in angle mode. Adjust for personal feel. Default 1.0.",
          range=(0.5, 1.5), clamp=True),
  Setting("FordHighSpeedFactor_ang", float, 1.0, PER_FRAME, "High Speed Adjustment Factor",
          "Scales the high-speed steering response in angle mode. Adjust for personal feel. Default 1.0.",
          range=(0.5, 1.5), clamp=True),
  Setting("FordHighSpeedDampening_ang", float, 1.0, PER_FRAME, "High Speed Low Curve Adjustment Factor",
          "Tune adjustment factor for low curve straightaways (highways) at high speeds. If oversteering, reduce. " +
          "If understeering, increase", range=(0.25, 1.25), clamp=True),
  Setting("lane_change_factor_high_ang", float, 1.0, PER_FRAME, "Lane Change Factor High",
          "Scales steering during a lane change (angle control). Values >1.0 boost it.", range=(0.85, 1.5), clamp=True),
  Setting("custom_path_offset_ang", float, 0.0, PER_FRAME, "In-Lane Offset",
          "Adjust the in-lane offset (-0.5 to 0.5).", range=(-0.5, 0.5), clamp=True),
  Setting("lane_centering_strength_ang", float, 0.25, PER_FRAME, "Lane Centering Strength",
          "How much authority the lane centering trim has vs. the model's own path (0.0-1.0).",
          range=(0.0, 1.0), clamp=True),
  Setting("FordPrefSteerAngleCurvature", bool, False, STARTUP, "Use Pinion Yaw Sensor",
          "Measures how the car is turning from the steering pinion angle sensor instead of a faulty RCM yaw sensor " +
          '(symptoms: "Turn Exceeds Steering Limit" warnings, weak curve tracking, "Service AdvanceTrac"). Check with ' +
          "tools/ford_yaw_health_check.py. Applies the next time the car starts. Not available on the Edge."),
)

_BY_KEY = {s.key: s for s in SETTINGS}

# what a host publishes in CarControlSP.params every frame (values_ext.BP_LATERAL_PARAMS)
PER_FRAME_KEYS = tuple(s.key for s in SETTINGS if s.read != STARTUP)


def _parse(setting: Setting, raw):
  """The setting's value from what the host gave, or None if there is nothing usable."""
  if isinstance(raw, (bytes, bytearray)):
    raw = raw.decode("utf-8", errors="replace")
  if raw is None or raw == "":
    return None
  if setting.type is bool:
    if isinstance(raw, (bool, int)):
      return bool(raw)
    text = str(raw).strip().lower()
    return True if text in ("1", "true") else False if text in ("0", "false") else None
  try:
    v = setting.type(raw)
  except (TypeError, ValueError):
    return None
  if setting.clamp and setting.range is not None:
    v = setting.type(min(max(v, setting.range[0]), setting.range[1]))
  return v


def default(key: str):
  return _BY_KEY[key].default


def value(params, key: str):
  """A per-frame setting from the host's ParamStore (built from this frame's (key, value) pairs)."""
  setting = _BY_KEY[key]
  assert setting.read != STARTUP, f"{key} is read at startup only: hosts don't publish it per frame"
  v = _parse(setting, params.get(key))
  return setting.default if v is None else v


def startup_value(params_dict: dict, key: str):
  """A startup setting from get_car's params dict (values as Params returned them)."""
  setting = _BY_KEY[key]
  assert setting.read != PER_FRAME, f"{key} is read per frame only: hosts don't pass it at startup"
  v = _parse(setting, params_dict.get(key))
  return setting.default if v is None else v


def params_keys_lines() -> list[str]:
  """A host's params_keys.h entries for every setting."""
  types = {bool: "BOOL", int: "INT", float: "FLOAT"}

  def text(s):
    return ("1" if s.default else "0") if s.type is bool else str(s.default)
  return [f'{{"{s.key}", {{PERSISTENT | BACKUP, {types[s.type]}, "{text(s)}"}}}},' for s in SETTINGS]


if __name__ == "__main__":
  print("\n".join(params_keys_lines()))
