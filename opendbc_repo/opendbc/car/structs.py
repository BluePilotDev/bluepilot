from dataclasses import dataclass as _dataclass, field, is_dataclass
from enum import Enum, StrEnum as _StrEnum, auto
from typing import dataclass_transform, get_origin

import os
import capnp
from opendbc.car.common.basedir import BASEDIR

# TODO: remove car from cereal/__init__.py and always import from opendbc
try:
  from cereal import car
except ImportError:
  capnp.remove_import_hook()
  car = capnp.load(os.path.join(BASEDIR, "car.capnp"))

CarState = car.CarState
RadarData = car.RadarData
CarControl = car.CarControl
CarParams = car.CarParams

CarStateT = capnp.lib.capnp._StructModule
RadarDataT = capnp.lib.capnp._StructModule
CarControlT = capnp.lib.capnp._StructModule
CarParamsT = capnp.lib.capnp._StructModule

# sunnypilot structs

AUTO_OBJ = object()


def auto_field():
  return AUTO_OBJ


@dataclass_transform()
def auto_dataclass(cls=None, /, **kwargs):
  cls_annotations = cls.__dict__.get('__annotations__', {})
  for name, typ in cls_annotations.items():
    current_value = getattr(cls, name)
    if current_value is AUTO_OBJ:
      origin_typ = get_origin(typ) or typ
      if isinstance(origin_typ, str):
        raise TypeError(f"Forward references are not supported for auto_field: '{origin_typ}'. Use a default_factory with lambda instead.")
      elif origin_typ in (int, float, str, bytes, list, tuple, bool) or is_dataclass(origin_typ):
        setattr(cls, name, field(default_factory=origin_typ))
      elif issubclass(origin_typ, Enum):  # first enum is the default
        setattr(cls, name, field(default=next(iter(origin_typ))))
      else:
        raise TypeError(f"Unsupported type for auto_field: {origin_typ}")

  # TODO: use slots, this prevents accidentally setting attributes that don't exist
  return _dataclass(cls, **kwargs)


class StrEnum(_StrEnum):
  @staticmethod
  def _generate_next_value_(name, *args):
    # auto() defaults to name.lower()
    return name


@auto_dataclass
class CarParamsSP:
  flags: int = auto_field()        # flags for car specific quirks
  safetyParam: int = auto_field()  # flags for custom safety flags
  pcmCruiseSpeed: bool = auto_field()
  intelligentCruiseButtonManagementAvailable: bool = auto_field()
  enableGasInterceptor: bool = auto_field()

  neuralNetworkLateralControl: 'CarParamsSP.NeuralNetworkLateralControl' = field(default_factory=lambda: CarParamsSP.NeuralNetworkLateralControl())

  @auto_dataclass
  class NeuralNetworkLateralControl:
    model: 'CarParamsSP.NeuralNetworkLateralControl.Model' = field(default_factory=lambda: CarParamsSP.NeuralNetworkLateralControl.Model())
    fuzzyFingerprint: bool = auto_field()

    @auto_dataclass
    class Model:
      path: str = auto_field()
      name: str = auto_field()


@auto_dataclass
class ModularAssistiveDrivingSystem:
  state: 'ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState' = field(
    default_factory=lambda: ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState.disabled
  )
  enabled: bool = auto_field()
  active: bool = auto_field()
  available: bool = auto_field()

  class ModularAssistiveDrivingSystemState(StrEnum):
    disabled = auto()
    paused = auto()
    enabled = auto()
    softDisabling = auto()
    overriding = auto()


@auto_dataclass
class IntelligentCruiseButtonManagement:
  state: 'IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState' = field(
    default_factory=lambda: IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState.inactive
  )
  sendButton: 'IntelligentCruiseButtonManagement.SendButtonState' = field(
    default_factory=lambda: IntelligentCruiseButtonManagement.SendButtonState.none
  )
  vTarget: float = auto_field()

  class IntelligentCruiseButtonManagementState(StrEnum):
    inactive = auto()
    preActive = auto()
    increasing = auto()
    decreasing = auto()
    holding = auto()

  class SendButtonState(StrEnum):
    none = auto()
    increase = auto()
    decrease = auto()


@auto_dataclass
class LeadData:
  dRel: float = auto_field()
  yRel: float = auto_field()
  vRel: float = auto_field()
  aRel: float = auto_field()
  vLead: float = auto_field()
  dPath: float = auto_field()
  vLat: float = auto_field()
  vLeadK: float = auto_field()
  aLeadK: float = auto_field()
  fcw: bool = auto_field()
  status: bool = auto_field()
  aLeadTau: float = auto_field()
  modelProb: float = auto_field()
  radar: bool = auto_field()
  radarTrackId: int = auto_field()

  aLeadDEPRECATED: float = auto_field()


@auto_dataclass
class CarControlSP:
  mads: 'ModularAssistiveDrivingSystem' = field(default_factory=lambda: ModularAssistiveDrivingSystem())
  params: list['CarControlSP.Param'] = auto_field()
  leadOne: 'LeadData' = field(default_factory=lambda: LeadData())
  leadTwo: 'LeadData' = field(default_factory=lambda: LeadData())
  intelligentCruiseButtonManagement: 'IntelligentCruiseButtonManagement' = field(default_factory=lambda: IntelligentCruiseButtonManagement())

  @auto_dataclass
  class Param:
    key: str = auto_field()
    value: bytes = auto_field()
    type: 'CarControlSP.ParamType' = field(
      default_factory=lambda: CarControlSP.ParamType.string
    )

  class ParamType(StrEnum):
    string = auto()
    bool = auto()
    int = auto()
    float = auto()
    time = auto()
    json = auto()
    bytes = auto()


@auto_dataclass
class CarStateSP:
  speedLimit: float = auto_field()


# BluePilot: ControllerStateBP for lateral uncertainty (angleState vehicles)
@auto_dataclass
class ControllerStateBP:
  lateralUncertainty: float = 0.0
  angleRateLimited: bool = False       # angle mode: path_angle soft-ROC clip bit this frame
  curvatureRateLimited: bool = False   # sim: equivalent curvature would be rate-limited by lateral_curv_ext
  curvatureDeviationLimited: bool = False  # current_curvature error-clip constrained the command this frame
  humanTurnLateralPaused: bool = False  # angle mode: lateral forced inactive (mode 0) during a manual turn
  stallBlipActive: bool = False  # angle mode: brief mode-0 pulse resetting PSCM authority after a post-override stall
  angleSaturated: bool = False  # angle mode: PSCM authority limit or DBC clamp modified this frame's command

  # BluePilot: full BluePilot-menu settings snapshot -- see custom.capnp ControllerStateBP for
  # field-by-field param-key mapping and the field-retirement convention.
  # --- System ---
  bmsUiDebugLogging: bool = False
  bmsConnectBackend: int = 0
  bmsWebRoutesServerEnabled: bool = False
  bmsPreferredWifiNetwork: str = ""
  # --- Vehicle ---
  bmsShowBlueCruiseUiOnCluster: bool = False
  bmsTwelveVBatteryLimit: float = 11.8
  # --- Visuals ---
  bmsHideOnroadBorder: bool = False
  bmsDisableLaneLineStatusColor: bool = False
  bmsMinimalDrivingView: bool = False
  bmsEightBitRacerTheme: bool = False
  bmsRainbowLaneLines: bool = False
  bmsShowBlindspotOverlay: bool = True
  bmsShowBrakeStatus: bool = False
  bmsShowConfidenceBall: bool = True
  bmsAnimateSteeringWheel: bool = True
  bmsWheelIconStyle: int = 0
  bmsShowRadarLeadOverlay: bool = True
  bmsRadarOverlaySize: int = 1
  bmsShowHybridBatteryStatus: bool = False
  bmsShowHybridPowerFlow: bool = False
  bmsHybridDriveGaugeSize: int = 1
  bmsHybridGaugeStyle: int = 0
  bmsHybridPowerFlowStyleRound: bool = False
  bmsLowerRightDisplay: int = 0
  bmsRainbowMode: bool = False
  bmsHideOnroadFade: bool = False
  # --- Longitudinal Tuning ---
  bmsBypassBpLongitudinalControl: bool = False
  bmsDisableDownhillCompensation: bool = False
  bmsDisableFordRadarVisionOnly: bool = False
  # --- Lateral Tuning ---
  bmsDisableBpLateralControl: bool = False
  bmsPrimaryControlVariable: int = 0
  bmsDisableLaneChangeUnderSpeed: bool = False
  bmsMinimumSpeedToPauseLaneChange: int = 20
  bmsShowLateralControlMode: bool = False
  # --- Angle Tuning ---
  # NOTE: a field must be declared HERE to survive convert_to_capnp — the publisher's
  # setattr on an undeclared name is silently dropped at conversion, publishing the capnp
  # default instead. That gap muted the auto-cal telemetry on every build until 2026-07-22.
  bmsAngleAutoCalibrate: bool = False
  bmsAngleAutoCalState: str = ""
  bmsLowSpeedAdjustmentFactor: float = 1.0
  bmsHighSpeedAdjustmentFactor: float = 1.0
  bmsLaneChangeFactorHighAngle: float = 1.0
  # --- Curvature Tuning ---
  bmsEnableHumanTurnDetection: bool = True
  bmsLaneChangeFactorHighCurvature: float = 0.85
  bmsEnableLanePositioning: bool = False
  bmsInLaneOffset: float = 0.0
  bmsEnableLanefullMode: bool = False
  bmsUseCustomTuningProfile: bool = False
  bmsPredictedCurvatureBlendRatioHigh: float = 0.4
  bmsPredictedCurvatureBlendRatioLow: float = 0.4
  bmsCenteringPidGain: float = 3.0
  # --- Fingerprint ---
  bmsFingerprintForced: bool = False
  bmsFingerprint: str = ""
