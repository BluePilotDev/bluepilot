"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json

from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.car.sync_sunnylink_params import CAR_LIST_JSON_OUT

ONROAD_BRIGHTNESS_MIGRATION_VERSION: str = "1.0"
ONROAD_BRIGHTNESS_TIMER_MIGRATION_VERSION: str = "1.0"

# index → seconds mapping for OnroadScreenOffTimer (SSoT)
ONROAD_BRIGHTNESS_TIMER_VALUES = {0: 3, 1: 5, 2: 7, 3: 10, 4: 15, 5: 30, **{i: (i - 5) * 60 for i in range(6, 16)}}
VALID_TIMER_VALUES = set(ONROAD_BRIGHTNESS_TIMER_VALUES.values())


def _migrate_car_platform_bundle(_params):
  bundle = _params.get("CarPlatformBundle")
  if bundle is None:
    return

  old_platform = bundle.get("platform")
  if not old_platform:
    return

  from opendbc.car.fingerprints import MIGRATION  # lazy: avoids heavy import at module level
  if old_platform not in MIGRATION:
    return

  new_platform = str(MIGRATION[old_platform])

  with open(CAR_LIST_JSON_OUT) as f:
    car_list = json.load(f)

  candidates = [(k, v) for k, v in car_list.items() if v.get("platform") == new_platform]
  if candidates:
    old_model = bundle.get("model")
    key, data = next(((k, v) for k, v in candidates if v.get("model") == old_model), candidates[0])
    bundle = {**data, "name": key}
  else:
    bundle["platform"] = new_platform

  _params.put("CarPlatformBundle", bundle, block=True)
  cloudlog.info(f"params_migration: CarPlatformBundle migrated {old_platform!r} -> {new_platform!r}")


BP_LATERAL_SCHEME_PARAMS_MIGRATION_VERSION: str = "1"

# (old key, new key) -- old keys stay declared in common/params_keys.h (harmless orphans) so their
# stored values are still readable here. lane_change_factor_high intentionally has no _ang entry:
# the old curvature-tuned default (0.85) is the wrong direction for angle mode, so _ang just takes
# its own fresh params_keys.h default instead of inheriting a stale value.
_BP_LATERAL_SCHEME_PARAM_RENAMES = (
  ("enable_human_turn_detection", "enable_human_turn_detection_curv"),
  ("lane_change_factor_high", "lane_change_factor_high_curv"),
  ("pc_blend_ratio_high_C_UI", "pc_blend_ratio_high_C_UI_curv"),
  ("pc_blend_ratio_low_C_UI", "pc_blend_ratio_low_C_UI_curv"),
  ("enable_lane_positioning", "enable_lane_positioning_curv"),
  ("custom_path_offset", "custom_path_offset_curv"),
  ("enable_lane_full_mode", "enable_lane_full_mode_curv"),
  ("custom_profile", "custom_profile_curv"),
  ("LC_PID_gain_UI", "LC_PID_gain_UI_curv"),
  ("FordAngleLowSpeedFactor", "FordLowSpeedFactor_ang"),
  ("FordAngleHighSpeedFactor", "FordHighSpeedFactor_ang"),
)


def _migrate_bp_lateral_scheme_params(_params):
  # Marker is a STRING param (like the OnroadScreenOff*Migrated flags above): the original BOOL
  # declaration made put("1") raise a type error inside the except below, so the marker never
  # stuck and this re-ran (and re-seeded, clobbering user-tuned values) on every boot.
  if _params.get("BPLateralSchemeParamsMigratedV1") == BP_LATERAL_SCHEME_PARAMS_MIGRATION_VERSION:
    return

  try:
    for old_key, new_key in _BP_LATERAL_SCHEME_PARAM_RENAMES:
      # Never overwrite a value that has already been written (by a previous migration run or by
      # the user tuning the new key) -- makes re-runs harmless, and lets in-field devices that hit
      # the every-boot re-seed keep whatever they have now instead of taking one final clobber.
      if _params.get(new_key) is not None:
        cloudlog.info(f"params_migration: {new_key} already set, not re-seeding")
        continue
      old_val = _params.get(old_key, return_default=True)
      _params.put(new_key, old_val, block=True)
      cloudlog.info(f"params_migration: seeded {new_key} from {old_key} ({old_val})")

    _params.put("BPLateralSchemeParamsMigratedV1", BP_LATERAL_SCHEME_PARAMS_MIGRATION_VERSION, block=True)
    cloudlog.info("params_migration: BP lateral scheme param split complete")
  except Exception as e:
    cloudlog.exception(f"Error migrating BP lateral scheme params: {e}")


def _migrate_bp_tesla_theme(_params):
  """Retire the separate Tesla Dark selector without disabling existing devices."""
  value = _params.get("BPThemePack")
  if isinstance(value, bytes):
    value = value.decode("utf-8", errors="replace")
  if isinstance(value, str) and value.strip().lower() == "tesla_dark":
    _params.put("BPThemePack", "tesla", block=True)
    cloudlog.info("params_migration: merged Tesla Dark into automatic Tesla theme")


def run_migration(_params):
  # migrate OnroadScreenOffBrightness
  if _params.get("OnroadScreenOffBrightnessMigrated") != ONROAD_BRIGHTNESS_MIGRATION_VERSION:
    try:
      val = _params.get("OnroadScreenOffBrightness", return_default=True)
      if val >= 2:  # old: 5%, new: Screen Off
        new_val = val + 1
        _params.put("OnroadScreenOffBrightness", new_val, block=True)
        log_str = f"Successfully migrated OnroadScreenOffBrightness from {val} to {new_val}."
      else:
        log_str = "Migration not required for OnroadScreenOffBrightness."

      _params.put("OnroadScreenOffBrightnessMigrated", ONROAD_BRIGHTNESS_MIGRATION_VERSION, block=True)
      cloudlog.info(log_str + f" Setting OnroadScreenOffBrightnessMigrated to {ONROAD_BRIGHTNESS_MIGRATION_VERSION}")
    except Exception as e:
      cloudlog.exception(f"Error migrating OnroadScreenOffBrightness: {e}")

  # migrate OnroadScreenOffTimer
  if _params.get("OnroadScreenOffTimerMigrated") != ONROAD_BRIGHTNESS_TIMER_MIGRATION_VERSION:
    try:
      val = _params.get("OnroadScreenOffTimer", return_default=True)
      if val not in VALID_TIMER_VALUES:
        _params.put("OnroadScreenOffTimer", 15, block=True)
        log_str = f"Successfully migrated OnroadScreenOffTimer from {val} to 15 (default)."
      else:
        log_str = "Migration not required for OnroadScreenOffTimer."

      _params.put("OnroadScreenOffTimerMigrated", ONROAD_BRIGHTNESS_TIMER_MIGRATION_VERSION, block=True)
      cloudlog.info(log_str + f" Setting OnroadScreenOffTimerMigrated to {ONROAD_BRIGHTNESS_TIMER_MIGRATION_VERSION}")
    except Exception as e:
      cloudlog.exception(f"Error migrating OnroadScreenOffTimer: {e}")

  _migrate_car_platform_bundle(_params)

  # BluePilot: split lateral-tuning params by control scheme (curvature vs angle)
  _migrate_bp_lateral_scheme_params(_params)

  # BluePilot: Tesla now selects Light/Dark automatically from camera exposure.
  _migrate_bp_tesla_theme(_params)
