"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Per-bug regression tests for the Raylib-vs-schema parity audit. Each test
isolates one of the gating bugs that the design-overhaul branch fixes so a
future regression is loud and obvious. These tests are intentionally narrow
and additive — they do not replace the broader test_settings_schema.py.
"""
from __future__ import annotations

import json
import os
from typing import Any

import pytest

from openpilot.sunnypilot.sunnylink.tools.generate_settings_schema import (
  DEFINITION_PATH,
  TORQUE_VERSIONS_PATH,
  _build_torque_options,
  _load_torque_versions,
  generate_schema,
)


SCHEMA_VALIDATOR_PATH = os.path.join(os.path.dirname(DEFINITION_PATH), "settings_ui.schema.json")


def _walk_items(schema: dict[str, Any]):
  """Yield every item dict from the schema."""
  def _yield(item: dict[str, Any]):
    yield item
    for sub in item.get("sub_items", []):
      yield from _yield(sub)

  for panel in schema.get("panels", []):
    for section in panel.get("sections", []):
      for item in section.get("items", []):
        yield from _yield(item)
      for sp in section.get("sub_panels", []):
        for item in sp.get("items", []):
          yield from _yield(item)
    for item in panel.get("items", []):
      yield from _yield(item)
    for sp in panel.get("sub_panels", []):
      for item in sp.get("items", []):
        yield from _yield(item)
  for brand in schema.get("vehicle_settings", {}).values():
    items = brand.get("items", []) if isinstance(brand, dict) else brand
    for item in items:
      yield from _yield(item)


def _find_item(schema: dict[str, Any], key: str) -> dict[str, Any] | None:
  for item in _walk_items(schema):
    if item.get("key") == key:
      return item
  return None


def _find_section(schema: dict[str, Any], panel_id: str, section_id: str) -> dict[str, Any] | None:
  for panel in schema.get("panels", []):
    if panel.get("id") != panel_id:
      continue
    for section in panel.get("sections", []):
      if section.get("id") == section_id:
        return section
  return None


def _flatten_rule_types(rules: list[dict[str, Any]] | None) -> set[str]:
  out: set[str] = set()

  def _walk(rule: dict[str, Any]) -> None:
    out.add(rule.get("type", ""))
    if rule.get("type") == "not" and "condition" in rule:
      _walk(rule["condition"])
    elif rule.get("type") in ("any", "all"):
      for c in rule.get("conditions", []):
        _walk(c)

  for rule in rules or []:
    _walk(rule)
  return out


def _references_capability_field(rules: list[dict[str, Any]] | None, field: str) -> bool:
  found = False

  def _walk(rule: dict[str, Any]) -> None:
    nonlocal found
    if rule.get("type") == "capability" and rule.get("field") == field:
      found = True
    elif rule.get("type") == "not" and "condition" in rule:
      _walk(rule["condition"])
    elif rule.get("type") in ("any", "all"):
      for c in rule.get("conditions", []):
        _walk(c)

  for rule in rules or []:
    _walk(rule)
  return found


@pytest.fixture(scope="module")
def schema():
  return generate_schema()


class TestMadsBrandGates:
  def test_mads_main_cruise_has_brand_gate(self, schema):
    """MadsMainCruiseAllowed must gate on brand and tesla_has_vehicle_bus."""
    item = _find_item(schema, "MadsMainCruiseAllowed")
    assert item is not None
    assert _references_capability_field(item.get("enablement"), "brand")
    assert _references_capability_field(item.get("enablement"), "tesla_has_vehicle_bus")

  def test_mads_unified_engagement_has_brand_gate(self, schema):
    """MadsUnifiedEngagementMode must mirror MadsMainCruiseAllowed brand-gate."""
    item = _find_item(schema, "MadsUnifiedEngagementMode")
    assert item is not None
    assert _references_capability_field(item.get("enablement"), "brand")
    assert _references_capability_field(item.get("enablement"), "tesla_has_vehicle_bus")


class TestTestManeuversSection:
  def test_lateral_maneuver_mode_in_test_maneuvers(self, schema):
    section = _find_section(schema, "developer", "test_maneuvers")
    assert section is not None, "developer.test_maneuvers section missing"
    keys = {item["key"] for item in section.get("items", [])}
    assert "LateralManeuverMode" in keys
    assert "LongitudinalManeuverMode" in keys

  def test_test_maneuvers_section_requires_attestation(self, schema):
    section = _find_section(schema, "developer", "test_maneuvers")
    assert section is not None
    assert section.get("attestation_required") is True

  def test_test_maneuvers_section_visibility_gate(self, schema):
    section = _find_section(schema, "developer", "test_maneuvers")
    assert section is not None
    visibility = section.get("visibility")
    assert visibility, "test_maneuvers must have visibility gate"
    vis_refs = json.dumps(visibility)
    assert "is_development" in vis_refs
    assert "is_sp_release" in vis_refs
    enablement = section.get("enablement") or []
    enable_refs = json.dumps(enablement)
    assert "ShowAdvancedControls" in enable_refs, \
      "test_maneuvers must gate ShowAdvancedControls via enablement"


class TestBluePilotVehicleVisuals:
  def test_tesla_is_a_theme_option_not_a_standalone_toggle(self, schema):
    assert _find_item(schema, "BPTeslaStyleMode") is None
    item = _find_item(schema, "BPThemePack")
    assert item is not None
    values = [option["value"] for option in item["options"]]
    assert {"", "rad_racer", "tesla"} <= set(values)
    assert "tesla_dark" not in values

  def test_rainbow_lane_lines_ordered_with_visual_toggles(self, schema):
    items = schema["vehicle_settings"]["ford"]["items"]
    keys = [item["key"] for item in items]
    assert "BPRainbowLines" in keys
    assert keys.index("BPHideCameraView") < keys.index("BPRainbowLines") < keys.index("ShowBlindspotOverlay")

    item = _find_item(schema, "BPRainbowLines")
    assert item is not None
    assert item["title"] == "[Visuals] Rainbow Lane Lines"
    assert item["description"] == "Inner lane lines become rainbow colored when longitudinal control is active."


class TestBluePilotLateralSchemeSplit:
  """The Ford lateral-tuning params are split by control scheme (_curv/_ang suffixes) and every
  scheme-specific item must be visibility-gated on FordPrefLateralControl so the remote UI never
  offers a control that the other scheme silently ignores (e.g. In-Lane Offset in angle mode)."""

  OLD_DEAD_KEYS = ("lane_change_factor_high", "custom_path_offset",
                   "FordAngleLowSpeedFactor", "FordAngleHighSpeedFactor")

  @staticmethod
  def _mode_gate(item) -> int | None:
    for rule in item.get("visibility") or []:
      if rule.get("type") == "param" and rule.get("key") == "FordPrefLateralControl":
        return rule.get("equals")
    return None

  def test_no_pre_split_keys_remain(self, schema):
    for key in self.OLD_DEAD_KEYS:
      assert _find_item(schema, key) is None, f"pre-split param '{key}' still referenced in schema"

  def test_scheme_suffixed_items_gate_on_matching_mode(self, schema):
    items = schema["vehicle_settings"]["ford"]["items"]
    suffixed = [item for item in items if item["key"].endswith(("_curv", "_ang"))]
    assert suffixed, "expected _curv/_ang lateral items in the ford section"
    for item in suffixed:
      expected_mode = 0 if item["key"].endswith("_curv") else 1
      assert self._mode_gate(item) == expected_mode, \
        f"{item['key']} must be visibility-gated on FordPrefLateralControl == {expected_mode}"

  def test_curvature_group_is_complete(self, schema):
    keys = {item["key"] for item in schema["vehicle_settings"]["ford"]["items"]}
    expected = {"enable_human_turn_detection_curv", "lane_change_factor_high_curv",
                "enable_lane_positioning_curv", "custom_path_offset_curv",
                "enable_lane_full_mode_curv", "custom_profile_curv",
                "pc_blend_ratio_high_C_UI_curv", "pc_blend_ratio_low_C_UI_curv",
                "LC_PID_gain_UI_curv"}
    assert expected <= keys, f"missing curvature items: {expected - keys}"

  def test_in_lane_offset_requires_lane_positioning(self, schema):
    item = _find_item(schema, "custom_path_offset_curv")
    assert item is not None
    refs = json.dumps(item.get("enablement") or [])
    assert "enable_lane_positioning_curv" in refs

  def test_disable_toggle_and_mode_selector_lead_the_section(self, schema):
    keys = [item["key"] for item in schema["vehicle_settings"]["ford"]["items"]]
    lateral_keys = [k for k in keys if k in ("disable_BP_lat_UI", "FordPrefLateralControl")]
    assert lateral_keys == ["disable_BP_lat_UI", "FordPrefLateralControl"]
    assert keys.index("disable_BP_lat_UI") < keys.index("FordLowSpeedFactor_ang")
    assert keys.index("disable_BP_lat_UI") < keys.index("enable_human_turn_detection_curv")

  def test_high_speed_low_curve_adjustment_contract(self, schema):
    items = schema["vehicle_settings"]["ford"]["items"]
    keys = [item["key"] for item in items]
    item = _find_item(schema, "FordHighSpeedDampening_ang")
    assert item is not None
    assert keys.index("FordHighSpeedDampening_ang") == keys.index("FordHighSpeedFactor_ang") + 1
    assert item["title"] == "[Lateral Tuning] High Speed Low Curve Adjustment Factor (Angle)"
    expected_description = " ".join((
      "Tune adjustment factor for low curve straightaways (highways) at high speeds.",
      "If oversteering, reduce. If understeering, increase",
    ))
    assert item["description"] == expected_description
    assert (item["min"], item["max"], item["step"]) == (0.75, 1.25, 0.01)
    assert self._mode_gate(item) == 1


class TestValidator:
  def test_validator_accepts_real_json(self):
    """settings_ui.json validates against settings_ui.schema.json."""
    jsonschema = pytest.importorskip("jsonschema")
    with open(DEFINITION_PATH) as f:
      data = json.load(f)
    with open(SCHEMA_VALIDATOR_PATH) as f:
      validator = json.load(f)
    jsonschema.validate(instance=data, schema=validator)


class TestTorqueOptionGeneration:
  def test_torque_versions_match_generated_options(self, schema):
    versions = _load_torque_versions()
    assert versions, "latcontrol_torque_versions.json must have at least one version"
    expected = _build_torque_options(versions)
    item = _find_item(schema, "TorqueControlTune")
    assert item is not None, "TorqueControlTune item must be present"
    assert item.get("options") == expected

  def test_torque_versions_path_resolves(self):
    assert os.path.exists(TORQUE_VERSIONS_PATH), (
      f"latcontrol_torque_versions.json not found at {TORQUE_VERSIONS_PATH}"
    )


class TestReleaseBranchGates:
  @pytest.mark.parametrize("key", [
    "EnableGithubRunner",
    "QuickBootToggle",
  ])
  def test_sp_dev_items_gate_on_is_sp_release(self, schema, key):
    """sunnypilot dev items must hide on sunnypilot release branches (is_sp_release gate)."""
    item = _find_item(schema, key)
    assert item is not None, f"{key} not found in schema"
    rules = (item.get("visibility") or []) + (item.get("enablement") or [])
    assert _references_capability_field(rules, "is_sp_release"), f"{key} missing is_sp_release gate"


class TestSpuriousOffroadGatesDropped:
  def test_disengage_on_accelerator_has_no_offroad_only(self, schema):
    item = _find_item(schema, "DisengageOnAccelerator")
    assert item is not None
    assert "offroad_only" not in _flatten_rule_types(item.get("enablement"))

  def test_dynamic_experimental_has_no_offroad_only(self, schema):
    item = _find_item(schema, "DynamicExperimentalControl")
    assert item is not None
    assert "offroad_only" not in _flatten_rule_types(item.get("enablement"))


class TestNotEngagedReplacement:
  @pytest.mark.parametrize("key", [
    "AlphaLongitudinalEnabled",
    "ToyotaEnforceStockLongitudinal",
    "ToyotaStopAndGoHack",
  ])
  def test_offroad_only_replaced_with_not_engaged(self, schema, key):
    """These items should use not_engaged, not offroad_only."""
    item = _find_item(schema, key)
    assert item is not None, f"{key} not found"
    rule_types = _flatten_rule_types(item.get("enablement"))
    assert "offroad_only" not in rule_types, f"{key} still uses offroad_only"
    assert "not_engaged" in rule_types, f"{key} missing not_engaged"
