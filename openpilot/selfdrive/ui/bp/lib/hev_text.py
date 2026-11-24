"""Display text for the hybrid power-flow mode and engine-on reason (Cluster_HEV_Data2)."""
from opendbc.sunnypilot.car.ford.bluepilot.carstate_ext import HEV_ENGINE_ON_REASONS, HEV_POWER_FLOW_MODES


def hev_power_flow_text(mode_value) -> str:
  return HEV_POWER_FLOW_MODES.get(int(mode_value), "Unknown")


def hev_engine_on_reason_text(reason_value) -> str:
  return HEV_ENGINE_ON_REASONS.get(int(reason_value), "Unknown")
