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
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_LATERAL_ACCEL_NO_ROLL  # BluePilot
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0


BP_PARAMS_REFRESH_PERIOD = 0.5  # seconds


# BluePilot: the panda's modes that run no car's safety (panda board/main.c is_car_safety_mode)
NON_CAR_SAFETY_MODELS = ('silent', 'noOutput', 'elm327', 'allOutput')

class ControlsExt(ModelStateBase):
  def __init__(self, CP: structs.CarParams, params: Params):
    ModelStateBase.__init__(self)
    self.CP = CP
    self.params = params
    self._param_update_time: float = 0.0
    self._bp_unknown_keys_logged: set[str] = set()  # BluePilot: see read_params
    self.blinker_pause_lateral = BlinkerPauseLateral()

    cloudlog.info("controlsd_ext is waiting for CarParamsSP")
    self.CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
    cloudlog.info("controlsd_ext got CarParamsSP")

    # BluePilot: pandaStates for the Ford lateral, controllerStateBP for the steering module's own limit
    self.sm_services_ext = ['radarState', 'selfdriveStateSP', 'pandaStates', 'controllerStateBP']
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

    # BluePilot: per-frame inputs for a car that asks for them (the library sets lateralInputsRequired
    # for every Ford), and the BPx settings; other cars leave them empty
    if self.CP_SP.lateralInputsRequired:
      self.fill_lateral_inputs(CC_SP.lateralInputs, sm)
      CC_SP.params = self.published_params()

    return CC_SP

  def published_params(self) -> list[dict]:
    """The settings published to the car code, re-read from Params every BP_PARAMS_REFRESH_PERIOD.
    Reading them on every 10 ms frame cost controlsd about 1 ms of CPU per frame on a comma four."""
    now = time.monotonic()
    cached = getattr(self, "_bp_params_cache", None)
    if cached is None or now - getattr(self, "_bp_params_cache_time", 0.0) > BP_PARAMS_REFRESH_PERIOD:
      cached = self.read_bpx_params()  # BluePilot: the lateral library reads no setting while driving
      self._bp_params_cache = cached
      self._bp_params_cache_time = now
    return cached

  # BluePilot: BPx settings for the Ford car code on bp-full
  def read_bpx_params(self) -> list[dict]:
    """BluePilot (BPx): the longitudinal and cluster settings the Ford car code on bp-full reads. Only
    on BluePilot: bp-sunnypilot's opendbc doesn't have these lists."""
    try:
      from openpilot.common.bluepilot import is_bluepilot
    except ImportError:
      return []
    if not is_bluepilot():
      return []
    try:
      from opendbc.sunnypilot.car.ford.bluepilot.hud_ext import HUD_PARAMS
      from opendbc.sunnypilot.car.ford.bluepilot.longitudinal_ext import BPX_LONG_PARAMS
    except ImportError:  # an opendbc without the BPx car code: it has no reader for these either
      return []
    return self.read_params([*BPX_LONG_PARAMS, *HUD_PARAMS])

  def read_params(self, keys: list[str]) -> list[dict]:
    """BluePilot: settings for the opendbc car code, as CarControlSP.params entries. A key never
    written on the device publishes its params_keys.h default, so the car code runs the value the
    settings UI shows (#6 in ford-op/bg-opendbc). Callers go through published_params(), which caches them."""
    entries = []
    for key in keys:
      try:
        value = self.params.get(key, return_default=True)
      except UnknownKeyName:
        # key list (submodule) and params_keys.h (this repo) drifted; skip rather than kill controlsd
        if key not in self._bp_unknown_keys_logged:
          cloudlog.warning(f"BluePilot param {key} not registered in params_keys.h; skipping")
          self._bp_unknown_keys_logged.add(key)
        continue
      if value is None:
        continue
      # same wire format Params writes: bools as "1"/"0" (checked first: bool is an int subclass)
      encoded = (b"1" if value else b"0") if isinstance(value, bool) else str(value).encode()
      entries.append({"key": key, "value": encoded})
    return entries

  @staticmethod
  def lateral_accel_limit(sm: messaging.SubMaster) -> float | None:
    """BluePilot: the most curvature * v^2 the Ford lateral library reports the steering module, or the
    panda, takes at this speed, m/s^2; None when nothing is reported. The plan's clip holds it beside
    openpilot's own roll-compensated limit: the module's and the panda's limits are on the command,
    so a banked road doesn't widen them. A request above it is clipped there, where it counts as the
    steering reaching its limit, instead of silently in the car controller."""
    limit = sm['controllerStateBP'].lateral.lateralAccelLimit if sm.seen['controllerStateBP'] else 0.
    return limit if limit > 0. else None

  @staticmethod
  def fill_lateral_inputs(li, sm: messaging.SubMaster) -> None:
    # what the panda allows and has blocked: the controller sends nothing the panda would refuse, and
    # counts what it refused anyway. Filled whether or not the model fields below are.
    panda_states = sm['pandaStates']
    if sm.seen['pandaStates'] and len(panda_states) > 0:
      # the pandas that run the car's mode: a second panda the car isn't on runs noOutput (car/ford/interface.py)
      car_pandas = [ps for ps in panda_states if str(ps.safetyModel) not in NON_CAR_SAFETY_MODELS]
      li.pandaLateralAllowed = all(ps.controlsAllowed or ps.controlsAllowedLateral for ps in (car_pandas or panda_states))
      li.safetyTxBlocked = sum(ps.safetyTxBlocked for ps in panda_states)
      # the panda takes no steering frame for a while after its safety mode is set: no panda in a car's mode, or no
      # current panda state (pandad restarting), tells the controller to wait that out once the car's mode is back
      li.pandaCarSafety = bool(sm.alive['pandaStates']) and len(car_pandas) > 0

    md = sm['modelV2']
    lp = sm['vehicleParameters']
    # valid = both sources seen AND every field below filled from a full-shape modelV2 (33-point
    # arrays, 3+ lane lines/probs/stds). On an invalid frame nothing is written and the lateral
    # keeps its previous inputs, as it did when a SubMaster message was stale. The opendbc side
    # re-checks array lengths and that every value is finite before use (lateral_inputs_complete),
    # so valid is a hint, not a promise.
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
