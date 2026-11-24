/**
 * BluePilot Ford lateral-control extensions: state and implementations.
 * See ford_declarations.h in this directory for the declarations.
 */

#pragma once

#include "../safety/ford_declarations.h"
#include "../safety/lateral.h"

// ===============================
// Global Variables
// ===============================

bool ford_bp_lateral = false;  // SP safety-param bit 5: BP 4-signal lateral checks instead of stock
bool ford_bp_pinion_curvature = false;
const AngleSteeringParams *ford_bp_pinion_params = &ford_pinion_geometry[0];

// BluePilot: per-platform geometry for pinion-angle -> curvature conversion (the optional
// angle_meas source), selected by the 4-bit geometry index in current_safety_param_sp
// bits 1-4. Row order and values must match FORD_PINION_GEOMETRY_INDEX in
// bluepilot_lateral/core/values_ext.py (enforced by test_ford_bluepilot_pinion.py's
// geometry test against CarSpecs + calc_slip_factor(VehicleModel(CP)), with the wheelbase
// overrides in FORD_PINION_WHEELBASE applied).
// Index 0 is reserved as invalid; ford_bp_init disables the feature outright on a zero or
// out-of-range index so a half-configured param can never select the wrong geometry.
// FORD_EDGE_MK2 (ALT_STEER_ANGLE: relative pinion angle + learned offset) is unsupported
// and deliberately absent.
const AngleSteeringParams ford_pinion_geometry[FORD_PINION_GEOMETRY_ROWS] = {
  {.slip_factor = 0.0f, .steer_ratio = 1.0f, .wheelbase = 1.0f},                  // 0: invalid
  {.slip_factor = -0.00062819555f, .steer_ratio = 17.7f, .wheelbase = 2.670f},    // 1: FORD_BRONCO_SPORT_MK1
  {.slip_factor = -0.00061892325f, .steer_ratio = 16.7f, .wheelbase = 2.710f},    // 2: FORD_ESCAPE_MK4
  {.slip_factor = -0.00061892325f, .steer_ratio = 16.7f, .wheelbase = 2.710f},    // 3: FORD_ESCAPE_MK4_5
  {.slip_factor = -0.00045454798f, .steer_ratio = 17.0f, .wheelbase = 3.690f},    // 4: FORD_EXPEDITION_MK4
  {.slip_factor = -0.00055447339f, .steer_ratio = 16.8f, .wheelbase = 3.025f},    // 5: FORD_EXPLORER_MK6
  {.slip_factor = -0.00062121569f, .steer_ratio = 15.0f, .wheelbase = 2.700f},    // 6: FORD_FOCUS_MK4
  {.slip_factor = -0.00045331952f, .steer_ratio = 16.9f, .wheelbase = 3.700f},    // 7: FORD_F_150_LIGHTNING_MK1
  {.slip_factor = -0.00043679222f, .steer_ratio = 17.0f, .wheelbase = 3.840f},    // 8: FORD_F_150_MK14 (average wheelbase, #21)
  {.slip_factor = -0.00054528036f, .steer_ratio = 17.0f, .wheelbase = 3.076f},    // 9: FORD_MAVERICK_MK1
  {.slip_factor = -0.00058852001f, .steer_ratio = 14.8f, .wheelbase = 2.850f},    // 10: FORD_MONDEO_MK5
  {.slip_factor = -0.00056209187f, .steer_ratio = 17.0f, .wheelbase = 2.984f},    // 11: FORD_MUSTANG_MACH_E_MK1
  {.slip_factor = -0.00051293030f, .steer_ratio = 17.0f, .wheelbase = 3.270f},    // 12: FORD_RANGER_MK2
};

bool ford_bp_angle_mode_engaged = false;
int16_t ford_bp_shadow_curvature_raw = 0;  // wire units, scale 1e-6 1/m (LANE_ASSIST_DATA1.md)

int desired_path_angle_last = 0;
int desired_path_offset_last = 0;


// ===============================
// Function Implementations
// ===============================

static inline bool path_angle_cmd_checks(int desired_path_angle, bool steer_control_enabled, const AngleSteeringLimits limits) {
  bool violation = false;

  if (steer_control_enabled) {
    float speed = ((float)vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;

    int delta_path_angle_roc = (safety_interpolate(limits.angle_rate_up_lookup, speed) * limits.angle_deg_to_can) + 1.;

    int highest_desired_path_angle = desired_path_angle_last + delta_path_angle_roc;
    int lowest_desired_path_angle = desired_path_angle_last - delta_path_angle_roc;

    violation |= safety_max_limit_check(desired_path_angle, highest_desired_path_angle, lowest_desired_path_angle);
    FORD_BP_DBG("path_angle_cmd_checks 1: desired_path_angle: %d desired_path_angle_last: %d highest_desired_path_angle: %d lowest_desired_path_angle: %d violation: %d \n",
                desired_path_angle, desired_path_angle_last, highest_desired_path_angle, lowest_desired_path_angle, (int)violation);
  }
  desired_path_angle_last = desired_path_angle;

  if (!steer_control_enabled) {
    violation |= (desired_path_angle != 0);
  }
  FORD_BP_DBG("path_angle_cmd_checks 2: violation: %d \n", (int)violation);

  return violation;
}

static inline bool path_offset_cmd_checks(int desired_path_offset, bool steer_control_enabled, const AngleSteeringLimits limits) {
  bool violation = false;

  if (steer_control_enabled) {
    float speed = ((float)vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;

    int delta_path_offset_roc = (safety_interpolate(limits.angle_rate_up_lookup, speed) * limits.angle_deg_to_can) + 1.;

    int highest_desired_path_offset = desired_path_offset_last + delta_path_offset_roc;
    int lowest_desired_path_offset = desired_path_offset_last - delta_path_offset_roc;

    violation |= safety_max_limit_check(desired_path_offset, highest_desired_path_offset, lowest_desired_path_offset);
    FORD_BP_DBG("path_offset_cmd_checks 1: desired_path_offset: %d desired_path_offset_last: %d highest_desired_path_offset: %d lowest_desired_path_offset: %d violation: %d \n",
                desired_path_offset, desired_path_offset_last, highest_desired_path_offset, lowest_desired_path_offset, (int)violation);

  }
  desired_path_offset_last = desired_path_offset;

  if (!steer_control_enabled) {
    violation |= (desired_path_offset != 0);
  }
  FORD_BP_DBG("path_offset_cmd_checks 2: violation: %d \n", (int)violation);

  return violation;
}

// BluePilot: angle mode has no "current path_angle" measurement to check the command against,
// unlike curvature mode, which compares desired_curvature against angle_meas (measured curvature,
// from yaw rate). Without this, a large deviation between commanded path_angle and the car's ACTUAL
// curvature -- e.g. a pothole or driver override kicking the wheel -- would go unchecked: path_angle's
// own ROC only bounds how fast the *command* changes, not how far it may sit from reality.
//
// Deliberately narrower than steer_angle_cmd_checks: no rate-of-change enforcement here, and no
// shared state (desired_angle_last) with curvature mode. path_angle already has its own dedicated,
// tuned ROC (path_angle_cmd_checks / FORD_PATH_ANGLE_LIMITS); imposing a second, curvature-tuned ROC
// on shadow_curvature -- which isn't an actuator, just a cross-check value -- would risk spurious
// blocks unrelated to path_angle's actual behavior (confirmed on real hardware 2026-07-10: doing
// this via steer_angle_cmd_checks caused blocks at low speed from shadow_curvature jumping frame to
// frame with nothing driving it toward path_angle's own smooth ROC). This is a pure per-frame
// proximity check: does this frame's steering intent make physical sense given where the car is.
// BluePilot: enforce_angle_error is gone from the struct; the check is unconditional now because
// its only caller, ford_lmc_checks, passes the base of a BluePilot limit set (FORD_LIMITS, above
// ford_bp_lmc_tx), all of which set it true pre-sync.
static inline bool ford_shadow_curvature_error_check(int desired_curvature, bool steer_control_enabled,
                                               const BP_HOST_CURVATURE_LIMITS limits) {
  bool violation = false;
  if (steer_control_enabled &&
      ((vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR) > limits.curvature_error_min_speed)) {
    int lowest_allowed = BP_HOST_CURVATURE_STATE.meas.min - limits.max_curvature_error - 1;
    int highest_allowed = BP_HOST_CURVATURE_STATE.meas.max + limits.max_curvature_error + 1;
    violation = safety_max_limit_check(desired_curvature, highest_allowed, lowest_allowed);
  }
  return violation;
}

// BluePilot: shared curvature/curvature_rate/path_offset/path_angle command checks for
// LateralMotionControl (CAN) and LateralMotionControl2 (CAN FD), in signed CAN units. limits is the
// curvature limit set ford_bp_lmc_tx picked for the message and the angle_meas source.
// dbg_prefix labels the FORD_BP_DBG output ("CAN Out" / "CANFD Out").
static inline bool ford_lmc_checks(int desired_curvature, int desired_curvature_rate, int desired_path_offset, int desired_path_angle,
                            bool steer_control_enabled, const FordBpCurvatureLimits *limits, const char *dbg_prefix) {
  // dbg_prefix is only read inside FORD_BP_DBG, which expands to a no-op unless FORD_BP_DEBUG is defined for the
  // debug build, making the parameter otherwise unused.
  SAFETY_UNUSED(dbg_prefix);

  // PathAngle rate limits
  static const AngleSteeringLimits FORD_PATH_ANGLE_LIMITS = {
    .max_angle = 1000,
    // 0.0005
    .angle_deg_to_can = 2000,        // 1 / (2e-5) rad to can
    // Mirror lateral_angle_ext.py _soft_roc: interp(v_ego, [9,10,15,25], [0.055,0.055,0.0425,0.009])
    // rad/call, scaled x1.02 so panda is 2% LOOSER than the Python control and never blocks LMC2.
    // lookup_t is fixed at 3 points; Python's 9 & 10 m/s nodes are both 0.055 (flat top), so {10,15,25}
    // reproduces the curve exactly and speeds <10 clamp to the first point. The +1 CAN unit and the
    // speed-1 fudge in path_angle_cmd_checks add extra headroom on top of the 2%.
    // BluePilot: LMC2 is only sent once per CarControllerParams.STEER_STEP (5) = 20Hz, not 100Hz --
    // _soft_roc's y-values (and this mirror) are per-call, not per-100Hz-tick; see lateral_angle_ext.py.
    .angle_rate_up_lookup = {
      .x = {10., 15., 25.},
      .y = {0.0561, 0.04335, 0.00918}
    },
    .angle_rate_down_lookup = {
      .x = {10., 15., 25.},
      .y = {0.0561, 0.04335, 0.00918}
    },
    .frequency = 20U,               // Hz -- LateralMotionControl/LateralMotionControl2 @ 20Hz (matches
                                    // actual STEER_STEP=5 cadence; was 100U, a stale leftover from an
                                    // abandoned 100Hz-cadence experiment. Currently unread by
                                    // path_angle_cmd_checks (only angle_rate_up/down_lookup matter),
                                    // but corrected for consistency/documentation and in case a future
                                    // rt_angle_rate_limit_check() wiring starts consuming it.
  };

  // PathOffset rate limits
  static const AngleSteeringLimits FORD_PATH_OFFSET_LIMITS = {
    .max_angle = 100,               // 1.0 meter in CAN units (100 * 0.01)
    .angle_deg_to_can = 100,        // 1 / (0.01) meter to can
    .angle_rate_up_lookup = {
      .x = {5., 15., 25.},
      .y = {0.05, 0.025, 0.01}     // Slower rate limits for path offset
    },
    .angle_rate_down_lookup = {
      .x = {5., 15., 25.},
      .y = {0.05, 0.025, 0.01}     // Slower rate limits for path offset
    },
    .frequency = 20U,               // Hz - 20Hz message rate
  };

  const BP_HOST_CURVATURE_LIMITS *base = &limits->base;
  bool violation = false;

  // Check curvature value limits (already converted to signed CAN units by the caller). Note:
  // curvature's wire scale (curvature_to_can) is the same for CAN and CAN FD, so this is safe to
  // read off the (possibly pinion-widened) limits struct regardless of bus.
  int curvature_min_can = (int)(FORD_CURVATURE_MIN * base->curvature_to_can);
  int curvature_max_can = (int)(FORD_CURVATURE_MAX * base->curvature_to_can);
  violation |= (desired_curvature < curvature_min_can) || (desired_curvature > curvature_max_can);
  FORD_BP_DBG("%s: desired_curvature: %d, curvature_min_can: %d, curvature_max_can: %d, violation: %d\n",
              dbg_prefix, desired_curvature, curvature_min_can, curvature_max_can, (int)violation);

  // Curvature rate must be 0, like stock Ford: BluePilot always sends 0 (#22)
  violation |= (desired_curvature_rate != 0);
  FORD_BP_DBG("%s: desired_curvature_rate: %d, violation: %d\n", dbg_prefix, desired_curvature_rate, (int)violation);

  // Check path offset value limits
  int path_offset_min_can = (int)(FORD_PATH_OFFSET_MIN * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
  int path_offset_max_can = (int)(FORD_PATH_OFFSET_MAX * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
  violation |= (desired_path_offset < path_offset_min_can) || (desired_path_offset > path_offset_max_can);
  FORD_BP_DBG("%s: desired_path_offset: %d, path_offset_min_can: %d, path_offset_max_can: %d, violation: %d\n",
              dbg_prefix, desired_path_offset, path_offset_min_can, path_offset_max_can, (int)violation);

  // Check path angle value limits. Angle mode uses path_angle as the actuator and may swing to
  // the full DBC range, corroborated by ford_bp_angle_mode_engaged so a frame can't unlock this
  // wider range by merely setting curvature to 0. Curvature mode always keeps the tight cap,
  // including at curvature == 0 (straight driving, or the reset/human-turn frame) -- path_angle
  // there only trims and amplifies wound-up curvature, never needs the wide range.
  float path_angle_min_phys = ford_bp_angle_mode_engaged ? FORD_DBC_PATH_ANGLE_MIN : FORD_PATH_ANGLE_MIN;
  float path_angle_max_phys = ford_bp_angle_mode_engaged ? FORD_DBC_PATH_ANGLE_MAX : FORD_PATH_ANGLE_MAX;
  const float path_angle_min_scaled = path_angle_min_phys * FORD_PATH_ANGLE_LIMITS.angle_deg_to_can;
  const float path_angle_max_scaled = path_angle_max_phys * FORD_PATH_ANGLE_LIMITS.angle_deg_to_can;
  int path_angle_min_can = (int)path_angle_min_scaled;
  int path_angle_max_can = (int)path_angle_max_scaled;
  violation |= (desired_path_angle < path_angle_min_can) || (desired_path_angle > path_angle_max_can);
  FORD_BP_DBG("%s: desired_path_angle: %d, path_angle_min_can: %d, path_angle_max_can: %d, violation: %d\n",
              dbg_prefix, desired_path_angle, path_angle_min_can, path_angle_max_can, (int)violation);

  // Check the curvature command and steer_control_enabled. At curvature 0 (angle mode, where
  // path_angle does the steering, and curvature mode's reset frame) ford_bp_curvature_cmd_checks skips
  // its value checks itself and keeps the speed cross-check, message-rate check and controls gate;
  // path_angle keeps its own checks regardless.
  violation |= ford_bp_curvature_cmd_checks(desired_curvature, steer_control_enabled, limits);
  FORD_BP_DBG("%s: 1. desired_curvature violation: %d\n", dbg_prefix, (int)violation);

  // Angle mode's own deviation-only check against shadow_curvature, once angle mode is confirmed
  // engaged. If desired_curvature == 0 but angle mode is NOT confirmed, this is skipped -- that's
  // ordinary curvature mode at zero (straight driving or the reset/human-turn frame), which needs
  // no shadow-curvature check; it's still bounded by the tight path_angle range above and
  // steer_control_enabled's own checks.
  if ((desired_curvature == 0) && ford_bp_angle_mode_engaged) {
    const float shadow_curvature_scaled = (float)ford_bp_shadow_curvature_raw * FORD_BP_LKA_SHADOW_RAW_TO_CAN;
    int shadow_curvature_can = (int)shadow_curvature_scaled;
    violation |= ford_shadow_curvature_error_check(shadow_curvature_can, steer_control_enabled, *base);
  }

  // Check path angle rate of change limits
  violation |= path_angle_cmd_checks(desired_path_angle, steer_control_enabled, FORD_PATH_ANGLE_LIMITS);
  FORD_BP_DBG("%s: 2. desired_path_angle violation: %d\n", dbg_prefix, (int)violation);

  // Check path offset rate of change limits
  violation |= path_offset_cmd_checks(desired_path_offset, steer_control_enabled, FORD_PATH_OFFSET_LIMITS);
  FORD_BP_DBG("%s: 3. desired_path_offset violation: %d\n", dbg_prefix, (int)violation);


  return violation;
}

// BluePilot: the curvature limit sets ford_bp_lmc_tx picks from: the host's BP_HOST_CURVATURE_LIMITS as
// .base plus the measured rate tables, checked by ford_bp_curvature_cmd_checks instead of upstream's
// ISO-jerk-derived delta. The values are the pre-sync macro's. max_curv_err is 100 (0.002) on the stock
// yaw-sourced angle_meas path and 150 (0.003) on the pinion-sourced path (STEER_ANGLE_CURVATURE):
// firmware uses the raw pinion angle, with no roll/alignment-offset compensation (the Python layer
// compensates via liveParameters). ford_bp_curvature_cmd_checks assumes max_steer_power 0 and
// inactive_curvature_is_zero true rather than reading them, and base sets only the fields the library
// reads, so it initializes upstream's CurvatureSteeringLimits and a host's narrower struct alike.
#define FORD_LIMITS(limit_lateral_accel, max_curv_err) {                                         \
  .base = {                                                                                      \
    .max_curvature = 1000,          /* 0.02 curvature */                                         \
    .curvature_to_can = 50000,      /* 1 / (2e-5) rad to can */                                  \
    .frequency = 20U,               /* LateralMotionControl / LateralMotionControl2 @ 20 Hz */   \
    .max_curvature_error = (max_curv_err),                                                       \
    /* no blending at low speed due to lack of torque wind-up and inaccurate current curvature */\
    .curvature_error_min_speed = 10.0,  /* m/s */                                                \
  },                                                                                             \
  /* Down table 1% looser than up (#20); both >= the Python control's tables (values_ext) */     \
  .curvature_rate_up_lookup = {                                                                  \
    {5., 16., 25.},                                                                              \
    {0.0025f, 0.0014f, 0.00018f}                                                                 \
  },                                                                                             \
  .curvature_rate_down_lookup = {                                                                \
    {5., 16., 25.},                                                                              \
    {0.002525f, 0.001414f, 0.0001818f}                                                           \
  },                                                                                             \
  .limit_lateral_acceleration = (limit_lateral_accel),                                           \
}

// BluePilot: LateralMotionControl (CAN) or LateralMotionControl2 (CAN FD) with BP lateral on. Both
// messages carry the same four signals; ford_tx_hook decodes the raw values, and this converts them
// to signed CAN units (physical = raw * scale - offset), picks the limit set for the message and the
// angle_meas source, and runs ford_lmc_checks.
static inline bool ford_bp_lmc_tx(bool canfd, unsigned int raw_curvature, unsigned int raw_curvature_rate,
                                  unsigned int raw_path_offset, unsigned int raw_path_angle, bool steer_control_enabled) {
  // static inside the one function that reads them (MISRA 8.9)
  static const FordBpCurvatureLimits FORD_BP_STEERING_LIMITS = FORD_LIMITS(false, 100);
  static const FordBpCurvatureLimits FORD_BP_STEERING_LIMITS_PINION = FORD_LIMITS(false, 150);
  static const FordBpCurvatureLimits FORD_CANFD_STEERING_LIMITS = FORD_LIMITS(true, 100);
  static const FordBpCurvatureLimits FORD_CANFD_STEERING_LIMITS_PINION = FORD_LIMITS(true, 150);

  const FordBpCurvatureLimits *limits;
  unsigned int inactive_curvature_rate;
  const char *dbg_prefix;
  if (canfd) {
    limits = ford_bp_pinion_curvature ? &FORD_CANFD_STEERING_LIMITS_PINION : &FORD_CANFD_STEERING_LIMITS;
    inactive_curvature_rate = FORD_CANFD_INACTIVE_CURVATURE_RATE;
    dbg_prefix = "CANFD Out";
  } else {
    limits = ford_bp_pinion_curvature ? &FORD_BP_STEERING_LIMITS_PINION : &FORD_BP_STEERING_LIMITS;
    inactive_curvature_rate = FORD_INACTIVE_CURVATURE_RATE;
    dbg_prefix = "CAN Out";
  }

  int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;
  int desired_curvature_rate = raw_curvature_rate - inactive_curvature_rate;
  int desired_path_offset = raw_path_offset - FORD_INACTIVE_PATH_OFFSET;
  int desired_path_angle = raw_path_angle - FORD_INACTIVE_PATH_ANGLE;
  return ford_lmc_checks(desired_curvature, desired_curvature_rate, desired_path_offset, desired_path_angle,
                         steer_control_enabled, limits, dbg_prefix);
}

// BluePilot: angle_mode_engaged and shadow_curvature ride in Lane_Assist_Data1 bits with no DBC
// signal mapped to them (layout: bluepilot_lateral/can/LANE_ASSIST_DATA1.md), read out of
// the frame being sent. A separate CAN message (0x5F0)
// was tried first: the panda never receives its own TX, and an RX check for it tripped
// safety_tick's lagging check and blocked controls car-wide (2026-07-09).
static inline void ford_bp_lka_tx(const CANPacket_t *msg) {
  ford_bp_angle_mode_engaged = (msg->data[FORD_BP_LKA_ANGLE_MODE_BYTE] & FORD_BP_LKA_ANGLE_MODE_MASK) != 0U;
  unsigned int shadow_curvature_raw = (msg->data[FORD_BP_LKA_SHADOW_MSB_BYTE] << 8) | msg->data[FORD_BP_LKA_SHADOW_LSB_BYTE];
  ford_bp_shadow_curvature_raw = (int16_t)shadow_curvature_raw;
}

// BluePilot: optional angle_meas source -- measured curvature from the steering pinion angle (PSCM)
// via the vehicle model, for vehicles whose RCM broadcasts implausible yaw (sign-inverted vs
// IMU/steering geometry) while its quality flag still reads OK. The pinion angle was validated
// against the comma IMU (corr +0.99 on real routes); the Python control layer measures from the
// same source when this is enabled (carcontroller_ext.InputTracker.get_current_curvature), so the layers always
// agree. ford_rx_hook skips its yaw block while this source is on.
static inline void ford_bp_rx_hook(const CANPacket_t *msg) {
  // addr and bus compared directly, not with upstream's msg_matches, which not every host has
  if ((msg->addr == FORD_SteeringPinion_Data) && (msg->bus == FORD_MAIN_BUS) && ford_bp_pinion_curvature) {
    // Signal: StePinComp_An_Est, 22|15@0+ (0.1,-1600) deg
    int angle_raw = ((msg->data[2] & 0x7FU) << 8) | msg->data[3];
    float pinion_angle_deg = ((float)angle_raw * 0.1f) - 1600.0f;
    float pinion_angle_rad = pinion_angle_deg * 0.017453292519943295f;  // DEG_TO_RAD
    // angle -> curvature via vehicle model (matches VehicleModel.curvature_factor);
    // sign: firmware angle_meas is Ford wire convention (the yaw block uses +yaw/v), and
    // pinion angle correlates +0.97 with wire desired curvature on real frames -> positive.
    float speed = SAFETY_MAX(vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR, 0.1);
    float curvature_factor = get_curvature_factor(speed, *ford_bp_pinion_params);
    float current_curvature = pinion_angle_rad * curvature_factor / ford_bp_pinion_params->steer_ratio;
    // convert current curvature into units on CAN for comparison with desired curvature
    update_sample(&BP_HOST_CURVATURE_STATE.meas, ROUND(current_curvature * FORD_STEERING_LIMITS.curvature_to_can));
  }
}

// BluePilot: read the SP safety param and reset the BP lateral state. param_sp is
// current_safety_param_sp, delivered via USB 0xdf before the safety model is set (a separate uint16
// from ford_init's param; pattern: subaru_common.h). Bits: 0 pinion-curvature enable, 1-4 pinion
// geometry index, 5 BP 4-signal lateral (clear = upstream's stock lateral safety, verbatim). A zero
// or out-of-range index disables the pinion source, so a half-configured param can never select the
// wrong geometry: the stock yaw path is kept. Returns whether the pinion source is on, i.e. whether
// ford_init applies the pinion RX check (ford_rx_checks_pinion). That check stays out of the stock
// config, so a pinion hiccup can't disable controls for users who never consume the message.
static inline bool ford_bp_init(uint16_t param_sp) {
  const uint16_t FORD_PARAM_SP_BP_LATERAL = 32;
  ford_bp_lateral = GET_FLAG(param_sp, FORD_PARAM_SP_BP_LATERAL);
  const uint16_t FORD_PARAM_SP_STEER_ANGLE_CURVATURE = 1;
  // Pinion measurement is only valid with the BP checks: the stock error band (100) was tuned for the
  // yaw source and the stock Python path clips against yaw-derived curvature.
  bool pinion_enabled = ford_bp_lateral && GET_FLAG(param_sp, FORD_PARAM_SP_STEER_ANGLE_CURVATURE);
  const uint16_t pinion_geometry_index = (param_sp >> 1) & 0xFU;
  if ((pinion_geometry_index == 0U) || (pinion_geometry_index > FORD_PINION_GEOMETRY_COUNT)) {
    pinion_enabled = false;
  }
  ford_bp_pinion_curvature = pinion_enabled;
  ford_bp_pinion_params = pinion_enabled ? &ford_pinion_geometry[pinion_geometry_index] : &ford_pinion_geometry[0];

  // start from a clean BP lateral state, as set_safety_hooks does for upstream's (desired_angle_last
  // etc.), so a re-init never checks against a previous session's commands. The angle-mode flag is
  // set again by the next Lane_Assist_Data1 frame.
  desired_path_angle_last = 0;
  desired_path_offset_last = 0;
  ford_bp_angle_mode_engaged = false;
  return ford_bp_pinion_curvature;
}
