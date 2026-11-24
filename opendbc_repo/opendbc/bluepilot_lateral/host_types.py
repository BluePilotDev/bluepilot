"""BluePilot: the types a host hands the library and reads back from it (HOSTS.md).

Standard library only: opendbc/car/structs.py imports LateralInputs for CarControlSP, so an import of
anything in opendbc here would be an import cycle."""
from dataclasses import dataclass, field

# The model's prediction grid: every path array in LateralInputs has MODEL_PATH_POINTS entries at these
# times, openpilot's ModelConstants.T_IDXS (t = 10 * (i / 32)^2). The core interpolates over it.
MODEL_PATH_POINTS = 33
MODEL_T_IDXS = [10.0 * (i / 32) ** 2 for i in range(MODEL_PATH_POINTS)]


@dataclass
class LateralInputs:
  """Per-frame model / vehicle inputs for the Ford 4-signal lateral control. The host fills them from
  modelV2 / vehicleParameters / lateralDelay, so opendbc never subscribes to messaging."""
  valid: bool = False                     # producer saw model + vehicle params and filled every field below
  pathYawRate: list[float] = field(default_factory=list)  # modelV2.orientationRate.z, 33 points over T_IDXS
  pathX: list[float] = field(default_factory=list)        # modelV2.position.x
  pathY: list[float] = field(default_factory=list)        # modelV2.position.y
  laneLineLeftX: list[float] = field(default_factory=list)   # modelV2.laneLines[1]
  laneLineLeftY: list[float] = field(default_factory=list)
  laneLineRightX: list[float] = field(default_factory=list)  # modelV2.laneLines[2]
  laneLineRightY: list[float] = field(default_factory=list)
  laneLineProbs: list[float] = field(default_factory=list)   # 4 entries
  laneLineStds: list[float] = field(default_factory=list)
  laneChangeState: int = 0      # modelV2.meta.laneChangeState
  laneChangeDirection: int = 0  # modelV2.meta.laneChangeDirection
  steerRatio: float = 0.0       # vehicleParameters
  stiffnessFactor: float = 0.0
  roll: float = 0.0
  angleOffsetDeg: float = 0.0
  lateralDelay: float = 0.0     # lateralDelay.lateralDelay


@dataclass
class LateralStatus:
  """This frame's values for a BluePilot UI, from BluePilotLateral.status(). The names are the ones
  BP7's controllerStateBP publisher reads off its car controller; the four angle-mode values read
  False outside angle mode, as BP7 sets them."""
  lateralUncertainty: float = 0.0          # from whichever strategy ran last
  angleRateLimited: bool = False           # angle mode: the path_angle soft-ROC clip bit this frame
  curvatureRateLimited: bool = False       # angle mode: the equivalent curvature would have been rate-limited
  curvatureDeviationLimited: bool = False  # either mode: the current-curvature error clip constrained the command
  humanTurnLateralPaused: bool = False     # angle mode: lateral sent inactive during a manual turn
  stallBlipActive: bool = False            # angle mode: the brief mode-0 pulse after a post-override stall
  disable_BP_lat_UI: bool = False          # BluePilot lateral off: the UI toggle, or the panda bit not set at init
  primary_lateral_control: int = 0         # 0 = curvature, 1 = angle
