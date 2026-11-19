"""
BluePilot: Publishes BP-specific messages from card.py.

Centralizes controllerStateBP and carStateBP message publishing so that
card.py only needs a single function call instead of inline BP blocks.
"""

import dataclasses
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.bluepilot.selfdrive.car.ford_lateral_support import record_degraded

_SETTINGS_INTERVAL = 5.0  # re-read params at most every 5 s
_settings_last_read: float = 0.0
_settings_cache: dict = {}
_degraded_reason: str = ""  # the health check's last reason handed to record_degraded


def _get_bool(p: Params, key: str, default: bool = False) -> bool:
  try:
    return p.get_bool(key)
  except Exception:
    return default


def _get_float(p: Params, key: str, default: float = 0.0) -> float:
  try:
    v = p.get(key, return_default=True)
    if v in (None, b"", ""):
      return default
    if isinstance(v, bytes):
      v = v.decode("utf-8", errors="replace").strip("\x00")
    return float(v)
  except Exception:
    return default


def _get_int(p: Params, key: str, default: int = 0) -> int:
  try:
    v = p.get(key, return_default=True)
    if v in (None, b"", ""):
      return default
    if isinstance(v, bytes):
      v = v.decode("utf-8", errors="replace").strip("\x00")
    return max(0, min(255, int(float(v))))
  except Exception:
    return default


def _get_str(p: Params, key: str, default: str = "") -> str:
  try:
    v = p.get(key, return_default=True)
    if v is None:
      return default
    return v.decode("utf-8", errors="replace").strip("\x00") if isinstance(v, bytes) else str(v).strip()
  except Exception:
    return default


def _refresh_settings_cache() -> dict:
  """BluePilot-menu settings snapshot -- see custom.capnp ControllerStateBP for the field-by-field
  param-key mapping and the field-retirement convention. Keep this dict's keys in sync with the
  capnp/structs.py bms* field names (1:1, same order, same grouping comments)."""
  p = Params()
  return {
    # --- System ---
    "bmsUiDebugLogging":               _get_bool(p, "BPUIDebugLog"),
    "bmsConnectBackend":               _get_int(p, "BPConnectBackend", 0),
    "bmsWebRoutesServerEnabled":       _get_bool(p, "EnableWebRoutesServer"),
    "bmsPreferredWifiNetwork":         _get_str(p, "WifiFavoriteSSID"),
    # --- Vehicle ---
    "bmsShowBlueCruiseUiOnCluster":    _get_bool(p, "send_hands_free_cluster_msg"),
    "bmsTwelveVBatteryLimit":          _get_float(p, "vbatt_pause_charging", 11.8),
    # --- Visuals ---
    "bmsHideOnroadBorder":             _get_bool(p, "BPHideOnroadBorder"),
    "bmsDisableLaneLineStatusColor":   _get_bool(p, "BPDisableLaneLineStatusColor"),
    "bmsMinimalDrivingView":           _get_bool(p, "BPHideCameraView"),
    "bmsEightBitRacerTheme":           _get_str(p, "BPThemePack").strip().lower() == "rad_racer",
    "bmsRainbowLaneLines":             _get_bool(p, "BPRainbowLines"),
    "bmsShowBlindspotOverlay":         _get_bool(p, "ShowBlindspotOverlay", True),
    "bmsShowBrakeStatus":              _get_bool(p, "ShowBrakeStatus"),
    "bmsShowConfidenceBall":           _get_bool(p, "BPShowConfidenceBall", True),
    "bmsAnimateSteeringWheel":         _get_bool(p, "BPAnimateSteeringWheel", True),
    "bmsWheelIconStyle":               _get_int(p, "BPSteeringWheelIconStyle", 0),
    "bmsShowRadarLeadOverlay":         _get_bool(p, "FordPrefShowRadarLeadOverlay", True),
    "bmsRadarOverlaySize":             _get_int(p, "FordPrefRadarOverlaySize", 1),
    "bmsShowHybridBatteryStatus":      _get_bool(p, "FordPrefHybridBatteryStatus"),
    "bmsShowHybridPowerFlow":          _get_bool(p, "FordPrefHybridPowerFlow"),
    "bmsHybridDriveGaugeSize":         _get_int(p, "FordPrefHybridDriveGaugeSize", 1),
    "bmsHybridGaugeStyle":             _get_int(p, "FordPrefGaugeStyle", 0),
    "bmsHybridPowerFlowStyleRound":    _get_bool(p, "FordPrefHybridPowerFlowAlternate"),
    "bmsLowerRightDisplay":            _get_int(p, "mici_complication", 0),
    "bmsRainbowMode":                  _get_bool(p, "RainbowMode"),
    "bmsHideOnroadFade":               _get_bool(p, "mici_hide_onroad_fade"),
    # --- Longitudinal Tuning ---
    "bmsBypassBpLongitudinalControl":  _get_bool(p, "disable_BP_long_UI"),
    "bmsDisableDownhillCompensation":  _get_bool(p, "disable_downhill_comp_UI"),
    "bmsDisableFordRadarVisionOnly":   _get_bool(p, "disable_ford_radar_UI"),
    # --- Lateral Tuning ---
    "bmsDisableBpLateralControl":      _get_bool(p, "disable_BP_lat_UI"),
    "bmsDisableLaneChangeUnderSpeed":  _get_bool(p, "BlinkerPauseLaneChange"),
    "bmsMinimumSpeedToPauseLaneChange": _get_int(p, "BlinkerMinLateralControlSpeed", 20),
    "bmsShowLateralControlMode":       _get_bool(p, "BpShowLateralControl"),
  }


def publish_controller_state_bp(CI, pm):
  """Publish controllerStateBP from the BluePilot car controller's status() (every Ford on bp-full)."""
  global _settings_last_read, _settings_cache, _degraded_reason
  status = getattr(getattr(CI.CC, "bp", None), "status", None)
  if status is None:
    return
  st = status()
  cs_bp_send = messaging.new_message('controllerStateBP')
  cs_bp_send.valid = True
  cs_bp = cs_bp_send.controllerStateBP
  # BluePilot: the lateral controller's status, field for field (custom.capnp ControllerStateBP.Lateral)
  for field, value in dataclasses.asdict(st).items():
    setattr(cs_bp.lateral, field, value)
  # the mode the controller ran, straight off the car controller (not Params)
  LateralMode = custom.ControllerStateBP.LateralMode
  cs_bp.activeLateralMode = LateralMode.curvature if st.active else LateralMode.openpilot
  # the health check's verdict, kept for the settings pages when it changes
  if st.degradedReason != _degraded_reason:
    _degraded_reason = st.degradedReason
    record_degraded(Params(), _degraded_reason)

  # BluePilot: settings snapshot -- refreshed at most every _SETTINGS_INTERVAL s so Params
  # reads don't add latency to every card.py tick.
  now = time.monotonic()
  if not _settings_cache or now - _settings_last_read >= _SETTINGS_INTERVAL:
    try:
      _settings_cache = _refresh_settings_cache()
    except Exception:
      pass
    _settings_last_read = now
  if _settings_cache:
    for field, value in _settings_cache.items():
      setattr(cs_bp, field, value)

  # BluePilot: fingerprint info -- plain attribute reads on CarParams, no Params round-trip
  # needed, so no caching required (fingerprint never changes after startup).
  CP = getattr(CI, "CP", None)
  if CP is not None:
    cs_bp.bmsFingerprintForced = bool(getattr(CP, "fingerprintSource", None) == "fixed")
    cs_bp.bmsFingerprint = str(getattr(CP, "carFingerprint", "") or "")

  pm.send('controllerStateBP', cs_bp_send)


def publish_car_state_bp(CI, pm, can_valid):
  """Publish carStateBP (hybrid gauges, brake lights) from the BluePilot car state; nothing before its first update."""
  car_state_bp = getattr(getattr(CI.CS, "bp", None), "car_state_bp", None)
  if not car_state_bp:
    return
  cs_bp_send = messaging.new_message('carStateBP')
  cs_bp_send.valid = can_valid
  cs_bp_send.carStateBP = car_state_bp
  pm.send('carStateBP', cs_bp_send)
