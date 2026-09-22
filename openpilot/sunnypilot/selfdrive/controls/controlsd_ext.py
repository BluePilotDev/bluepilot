"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom

from opendbc.car import structs
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0


class ControlsExt(ModelStateBase):
  def __init__(self, CP: structs.CarParams, params: Params):
    ModelStateBase.__init__(self)
    self.CP = CP
    self.params = params
    self._param_update_time: float = 0.0
    self._bp_lateral_params: list[dict] = []  # BluePilot: refreshed in get_params_sp, published every frame
    self._bp_unknown_keys_logged: set[str] = set()
    self.blinker_pause_lateral = BlinkerPauseLateral()
    # BluePilot: read once here so the very first carControlSP already carries the UI params
    # (disable_BP_lat_UI in particular); get_params_sp refreshes them on PARAMS_UPDATE_PERIOD
    if self.CP.brand == 'ford':
      self._bp_lateral_params = self.read_lateral_params()

    cloudlog.info("controlsd_ext is waiting for CarParamsSP")
    self.CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
    cloudlog.info("controlsd_ext got CarParamsSP")

    self.sm_services_ext = ['radarState', 'selfdriveStateSP']
    self.pm_services_ext = ['carControlSP']

  def initialize_lateral_control(self, lac, CI, dt):
    enforce_torque_control = self.params.get_bool("EnforceTorqueControl")
    torque_versions = self.params.get("TorqueControlTune")
    if not enforce_torque_control:
      if self.CP.lateralTuning.which() == 'torque':
        return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)  # FIXME-SP: revert when upstream fixes tuning issues with v1
      return lac

    if torque_versions == 0.0:  # v0
      return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)
    else:
      return lac

  def get_params_sp(self, sm: messaging.SubMaster) -> None:
    if time.monotonic() - self._param_update_time > PARAMS_UPDATE_PERIOD:
      self.blinker_pause_lateral.get_params()

      if self.CP.lateralTuning.which() == 'torque':
        self.lat_delay = get_lat_delay(self.params, sm["lateralDelay"].lateralDelay)

      if self.CP.brand == 'ford':
        self._bp_lateral_params = self.read_lateral_params()

      self._param_update_time = time.monotonic()

  def get_lat_active(self, sm: messaging.SubMaster) -> bool:
    if self.blinker_pause_lateral.update(sm['carState']):
      return False

    ss_sp = sm['selfdriveStateSP']
    if ss_sp.mads.available:
      return bool(ss_sp.mads.active)

    # MADS not available, use stock state to engage
    return bool(sm['selfdriveState'].active)

  @staticmethod
  def get_lead_data(_lead, src: log.RadarState.LeadData) -> None:
    _lead.dRel = src.dRel
    _lead.yRel = src.yRel
    _lead.vRel = src.vRel
    _lead.aRel = src.deprecated.aRel
    _lead.vLead = src.vLead
    _lead.dPath = src.deprecated.dPath
    _lead.vLat = src.deprecated.vLat
    _lead.vLeadK = src.vLeadK
    _lead.aLeadK = src.aLeadK
    _lead.fcw = src.deprecated.fcw
    _lead.status = src.present
    _lead.aLeadTau = src.aLeadTau
    _lead.modelProb = src.modelProb
    _lead.radar = src.radar
    _lead.radarTrackId = src.radarTrackId

  def state_control_ext(self, sm: messaging.SubMaster) -> custom.CarControlSP:
    CC_SP = custom.CarControlSP.new_message()

    self.get_lead_data(CC_SP.leadOne, sm['radarState'].leadOne)
    self.get_lead_data(CC_SP.leadTwo, sm['radarState'].leadTwo)

    # MADS state
    mads_src = sm['selfdriveStateSP'].mads
    CC_SP.mads.state = mads_src.state
    CC_SP.mads.enabled = mads_src.enabled
    CC_SP.mads.active = mads_src.active
    CC_SP.mads.available = mads_src.available

    # ICBM state
    icbm_src = sm['selfdriveStateSP'].intelligentCruiseButtonManagement
    CC_SP.intelligentCruiseButtonManagement.state = icbm_src.state
    CC_SP.intelligentCruiseButtonManagement.sendButton = icbm_src.sendButton
    CC_SP.intelligentCruiseButtonManagement.vTarget = icbm_src.vTarget

    # BluePilot: per-frame inputs for the Ford 4-signal lateral in opendbc (other brands: left empty)
    if self.CP.brand == 'ford':
      self.fill_lateral_inputs(CC_SP.lateralInputs, sm)
      CC_SP.params = self._bp_lateral_params

    return CC_SP

  def read_lateral_params(self) -> list[dict]:
    """BluePilot: UI-tunable lateral params for the opendbc lateral, as CarControlSP.params entries.
    Only keys that are set on the device are published; the lateral applies its own defaults for
    the rest, exactly as it did reading Params directly. Refreshed on the PARAMS_UPDATE_PERIOD
    throttle, not every frame, so a UI change reaches the lateral within that period."""
    # imported here, not at module level: the key list lives in the opendbc submodule and only
    # Ford needs it, so a submodule on another layout must not break controlsd for other brands
    from opendbc.car.bluepilot.ford.values_ext import BP_LATERAL_PARAMS
    entries = []
    for key in BP_LATERAL_PARAMS:
      try:
        value = self.params.get(key)
      except UnknownKeyName:
        # key list (submodule) and params_keys.h (this repo) drifted; skip rather than kill controlsd
        if key not in self._bp_unknown_keys_logged:
          cloudlog.warning(f"BluePilot lateral param {key} not registered in params_keys.h; skipping")
          self._bp_unknown_keys_logged.add(key)
        continue
      if value is None:
        continue
      # same wire format Params writes: bools as "1"/"0" (checked first: bool is an int subclass)
      encoded = (b"1" if value else b"0") if isinstance(value, bool) else str(value).encode()
      entries.append({"key": key, "value": encoded})
    return entries

  @staticmethod
  def fill_lateral_inputs(li, sm: messaging.SubMaster) -> None:
    md = sm['modelV2']
    lp = sm['vehicleParameters']
    # valid = both sources seen AND every field below filled from a full-shape modelV2 (33-point
    # arrays, 3+ lane lines/probs/stds). On an invalid frame nothing is written and the lateral
    # keeps its previous inputs, as it did when a SubMaster message was stale. The opendbc side
    # re-checks array lengths before use (lateral_inputs_complete), so valid is a hint, not a promise.
    li.valid = bool(sm.seen['modelV2'] and sm.seen['vehicleParameters']
                    and len(md.laneLines) >= 3 and len(md.laneLineProbs) >= 3 and len(md.laneLineStds) >= 3)
    if not li.valid:
      return
    li.pathYawRate = list(md.orientationRate.z)
    li.pathX = list(md.position.x)
    li.pathY = list(md.position.y)
    li.laneLineLeftX = list(md.laneLines[1].x)
    li.laneLineLeftY = list(md.laneLines[1].y)
    li.laneLineRightX = list(md.laneLines[2].x)
    li.laneLineRightY = list(md.laneLines[2].y)
    li.laneLineProbs = list(md.laneLineProbs)
    li.laneLineStds = list(md.laneLineStds)
    li.laneChangeState = int(md.meta.laneChangeState.raw)
    li.laneChangeDirection = int(md.meta.laneChangeDirection.raw)
    li.steerRatio = lp.steerRatio
    li.stiffnessFactor = lp.stiffnessFactor
    li.roll = lp.roll
    li.angleOffsetDeg = lp.angleOffsetDeg
    li.lateralDelay = sm['lateralDelay'].lateralDelay

  @staticmethod
  def publish_ext(CC_SP: custom.CarControlSP, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    cc_sp_send = messaging.new_message('carControlSP')
    cc_sp_send.valid = sm['carState'].canValid
    cc_sp_send.carControlSP = CC_SP

    pm.send('carControlSP', cc_sp_send)

  def run_ext(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    CC_SP = self.state_control_ext(sm)
    self.publish_ext(CC_SP, sm, pm)
