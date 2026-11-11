#pragma once

#include "opendbc/safety/declarations.h"

// Safety-relevant CAN messages for Ford vehicles.
#define FORD_EngBrakeData          0x165U   // RX from PCM, for driver brake pedal and cruise state
#define FORD_EngVehicleSpThrottle  0x204U   // RX from PCM, for driver throttle input
#define FORD_DesiredTorqBrk        0x213U   // RX from ABS, for standstill state
#define FORD_BrakeSysFeatures      0x415U   // RX from ABS, for vehicle speed
#define FORD_EngVehicleSpThrottle2 0x202U   // RX from PCM, for second vehicle speed
#define FORD_Yaw_Data_FD1          0x91U    // RX from RCM, for yaw rate
#define FORD_SteeringPinion_Data   0x7EU    // RX from PSCM, optional angle_meas source (STEER_ANGLE_CURVATURE)
#define FORD_Steering_Data_FD1     0x083U   // TX by OP, various driver switches and LKAS/CC buttons
#define FORD_ACCDATA               0x186U   // TX by OP, ACC controls
#define FORD_ACCDATA_3             0x18AU   // TX by OP, ACC/TJA user interface
#define FORD_Lane_Assist_Data1     0x3CAU   // TX by OP, Lane Keep Assist
#define FORD_LateralMotionControl  0x3D3U   // TX by OP, Lateral Control message
#define FORD_LateralMotionControl2 0x3D6U   // TX by OP, alternate Lateral Control message
#define FORD_IPMA_Data             0x3D8U   // TX by OP, IPMA and LKAS user interface

// CAN bus numbers.
#define FORD_MAIN_BUS 0U
#define FORD_CAM_BUS  2U

static uint8_t ford_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cnt
    cnt = (msg->data[2] >> 2) & 0xFU;
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYaw_No_Cnt
    cnt = msg->data[5];
  } else if (msg->addr == FORD_SteeringPinion_Data) {
    // Signal: StePinAn_No_Cnt (47|4@0+)
    cnt = (msg->data[5] >> 4) & 0xFU;
  } else {
  }
  return cnt;
}

static uint32_t ford_get_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cs
    chksum = msg->data[3];
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYawW_No_Cs
    chksum = msg->data[4];
  } else {
  }
  return chksum;
}

static uint32_t ford_compute_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    chksum += msg->data[0] + msg->data[1];  // Veh_V_ActlBrk
    chksum += msg->data[2] >> 6;                    // VehVActlBrk_D_Qf
    chksum += (msg->data[2] >> 2) & 0xFU;           // VehVActlBrk_No_Cnt
    chksum = 0xFFU - chksum;
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    chksum += msg->data[0] + msg->data[1];  // VehRol_W_Actl
    chksum += msg->data[2] + msg->data[3];  // VehYaw_W_Actl
    chksum += msg->data[5];                         // VehRollYaw_No_Cnt
    chksum += msg->data[6] >> 6;                    // VehRolWActl_D_Qf
    chksum += (msg->data[6] >> 4) & 0x3U;           // VehYawWActl_D_Qf
    chksum = 0xFFU - chksum;
  } else {
  }
  return chksum;
}

static bool ford_get_quality_flag_valid(const CANPacket_t *msg) {
  bool valid = false;
  if (msg->addr == FORD_BrakeSysFeatures) {
    valid = (msg->data[2] >> 6) == 0x3U;           // VehVActlBrk_D_Qf
  } else if (msg->addr == FORD_EngVehicleSpThrottle2) {
    valid = ((msg->data[4] >> 5) & 0x3U) == 0x3U;  // VehVActlEng_D_Qf
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    valid = ((msg->data[6] >> 4) & 0x3U) == 0x3U;  // VehYawWActl_D_Qf
  } else if (msg->addr == FORD_SteeringPinion_Data) {
    valid = ((msg->data[5] >> 2) & 0x3U) == 0x3U;  // StePinCompAnEst_D_Qf (3=OK)
  } else {
  }
  return valid;
}

#define FORD_INACTIVE_CURVATURE 1000U
#define FORD_INACTIVE_CURVATURE_RATE 4096U
#define FORD_INACTIVE_PATH_OFFSET 512U
#define FORD_INACTIVE_PATH_ANGLE 1000U

#define FORD_CANFD_INACTIVE_CURVATURE_RATE 1024U

// Control signal limits — curvature magnitude must match MAX_CURVATURE; rate tables must match
// opendbc/sunnypilot/car/ford/values_ext.py BP_ANGLE_LIMITS (FORD_LIMITS macro below).
#define FORD_CURVATURE_MIN -0.02f
#define FORD_CURVATURE_MAX 0.02f
#define FORD_CURVATURE_RATE_MIN -0.001024f
#define FORD_CURVATURE_RATE_MAX 0.00102375f
#define FORD_PATH_OFFSET_MIN -1.0f
#define FORD_PATH_OFFSET_MAX 1.0f
#define FORD_PATH_ANGLE_MIN -0.25f
#define FORD_PATH_ANGLE_MAX 0.25f
// BluePilot: full DBC signal range for path_angle (LatCtlPath_An, 0.0005 scale). Used as the
// value limit in ANGLE mode, where path_angle is the steering actuator (see LMC/LMC2 tx checks).
// Matches lateral_angle_ext.py FORD_DBC_PATH_ANGLE_MIN/MAX. In curvature mode the tight ±0.25
// cap above applies instead (path_angle there only trims and amplifies wound-up curvature).
#define FORD_DBC_PATH_ANGLE_MIN -0.5f
#define FORD_DBC_PATH_ANGLE_MAX 0.5235f



// Curvature rate limits
// max_angle_err: 100 (0.002) on the stock yaw-sourced angle_meas path; 150 (0.003) on the
// BluePilot pinion-sourced path (STEER_ANGLE_CURVATURE), because the raw pinion angle has
// no roll/alignment-offset compensation in firmware (the Python layer compensates via
// liveParameters; firmware uses the raw pinion angle).
#define FORD_LIMITS(limit_lateral_acceleration, max_angle_err) {                                \
  .max_angle = 1000,          /* 0.02 curvature */                                              \
  .angle_deg_to_can = 50000,  /* 1 / (2e-5) rad to can */                                       \
  .max_angle_error = (max_angle_err),                                                           \
  /* Looser symmetric ROCs (former down table); Python control uses stricter up row in values_ext */ \
  .angle_rate_up_lookup = {                                                                     \
    {5., 16., 25.},                                                                             \
    {0.0025f, 0.0014f, 0.00018f}                                                                \
  },                                                                                            \
  .angle_rate_down_lookup = {                                                                   \
    {5., 16., 25.},                                                                             \
    {0.0025f, 0.0014f, 0.00018f}                                                                \
  },                                                                                            \
                                                                                                \
  /* no blending at low speed due to lack of torque wind-up and inaccurate current curvature */ \
  .angle_error_min_speed = 10.0,    /* m/s */                                                   \
  .frequency = 20U,                 /* LateralMotionControl / LateralMotionControl2 @ 20 Hz */   \
                                                                                                \
  .angle_is_curvature = (limit_lateral_acceleration),                                           \
  .enforce_angle_error = true,                                                                  \
  .inactive_angle_is_zero = true,                                                               \
}

// PathAngle rate limits
static const AngleSteeringLimits FORD_PATH_ANGLE_LIMITS = {
  .max_angle = 1000,
  // 0.0005
  .angle_deg_to_can = 2000,        // 1 / (2e-5) rad to can
  .max_angle_error = 4,           // 0.002 * FORD_STEERING_LIMITS.angle_deg_to_can
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
  .angle_error_min_speed = 9.9,   // m/s
  .frequency = 20U,               // Hz -- LateralMotionControl/LateralMotionControl2 @ 20Hz (matches
                                  // actual STEER_STEP=5 cadence; was 100U, a stale leftover from an
                                  // abandoned 100Hz-cadence experiment. Currently unread by
                                  // path_angle_cmd_checks (only angle_rate_up/down_lookup matter),
                                  // but corrected for consistency/documentation and in case a future
                                  // rt_angle_rate_limit_check() wiring starts consuming it.

  .enforce_angle_error = true,
  .inactive_angle_is_zero = true,
};

// PathOffset rate limits
static const AngleSteeringLimits FORD_PATH_OFFSET_LIMITS = {
  .max_angle = 100,               // 1.0 meter in CAN units (100 * 0.01)
  .angle_deg_to_can = 100,        // 1 / (0.01) meter to can
  .max_angle_error = 2,           // 0.02 * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can
  .angle_rate_up_lookup = {
    .x = {5., 15., 25.},
    .y = {0.05, 0.025, 0.01}     // Slower rate limits for path offset
  },
  .angle_rate_down_lookup = {
    .x = {5., 15., 25.},
    .y = {0.05, 0.025, 0.01}     // Slower rate limits for path offset
  },
  .angle_error_min_speed = 5.0,   // m/s - lower speed threshold for path offset
  .frequency = 20U,               // Hz - 20Hz message rate

  .enforce_angle_error = true,
  .inactive_angle_is_zero = true,
};

// PathOffset rate limits
static const AngleSteeringLimits FORD_CURVATURE_RATE_LIMITS_CAN = {
  .max_angle = 100,               // 1.0 meter in CAN units (100 * 0.01)
  .angle_deg_to_can = 4000000,    // 1 / (1E-6) meter to can
  .max_angle_error = 2,           // 0.02 * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can
  .angle_rate_up_lookup = {
    .x = {5., 15., 25.},
    .y = {0.05, 0.025, 0.01}     // Slower rate limits for path offset
  },
  .angle_rate_down_lookup = {
    .x = {5., 15., 25.},
    .y = {0.05, 0.025, 0.01}     // Slower rate limits for path offset
  },
  .angle_error_min_speed = 5.0,   // m/s - lower speed threshold for path offset
  .frequency = 20U,               // Hz - 20Hz message rate

  .enforce_angle_error = true,
  .inactive_angle_is_zero = true,
};

static const AngleSteeringLimits FORD_CURVATURE_RATE_LIMITS_CANFD = {
  .max_angle = 100,               // 1.0 meter in CAN units (100 * 0.01)
  .angle_deg_to_can = 1000000,    // 1 / (1E-6) meter to can
  .max_angle_error = 2,           // 0.02 * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can
  .angle_rate_up_lookup = {
    .x = {5., 15., 25.},
    .y = {0.05, 0.025, 0.01}     // Slower rate limits for path offset
  },
  .angle_rate_down_lookup = {
    .x = {5., 15., 25.},
    .y = {0.05, 0.025, 0.01}     // Slower rate limits for path offset
  },
  .angle_error_min_speed = 5.0,   // m/s - lower speed threshold for path offset
  .frequency = 20U,               // Hz - 20Hz message rate

  .enforce_angle_error = true,
  .inactive_angle_is_zero = true,
};

static const AngleSteeringLimits FORD_STEERING_LIMITS = FORD_LIMITS(false, 100);
static const AngleSteeringLimits FORD_STEERING_LIMITS_PINION = FORD_LIMITS(false, 150);

// BluePilot: per-platform geometry for pinion-angle -> curvature conversion (the optional
// angle_meas source), selected by the 4-bit geometry index in current_safety_param_sp
// bits 1-4. Row order and values must match FORD_PINION_GEOMETRY_INDEX in
// opendbc/sunnypilot/car/ford/values_ext.py (enforced by test_ford.py's
// geometry-consistency test against CarSpecs + calc_slip_factor(VehicleModel(CP))).
// Index 0 is reserved as invalid; ford_init disables the feature outright on a zero or
// out-of-range index so a half-configured param can never select the wrong geometry.
// FORD_EDGE_MK2 (ALT_STEER_ANGLE: relative pinion angle + learned offset) is unsupported
// and deliberately absent.
#define FORD_PINION_GEOMETRY_COUNT 12U
static const AngleSteeringParams ford_pinion_geometry[FORD_PINION_GEOMETRY_COUNT + 1U] = {
  {.slip_factor = 0.0f, .steer_ratio = 1.0f, .wheelbase = 1.0f},                  // 0: invalid
  {.slip_factor = -0.00062819555f, .steer_ratio = 17.7f, .wheelbase = 2.670f},    // 1: FORD_BRONCO_SPORT_MK1
  {.slip_factor = -0.00061892325f, .steer_ratio = 16.7f, .wheelbase = 2.710f},    // 2: FORD_ESCAPE_MK4
  {.slip_factor = -0.00061892325f, .steer_ratio = 16.7f, .wheelbase = 2.710f},    // 3: FORD_ESCAPE_MK4_5
  {.slip_factor = -0.00045454798f, .steer_ratio = 17.0f, .wheelbase = 3.690f},    // 4: FORD_EXPEDITION_MK4
  {.slip_factor = -0.00055447339f, .steer_ratio = 16.8f, .wheelbase = 3.025f},    // 5: FORD_EXPLORER_MK6
  {.slip_factor = -0.00062121569f, .steer_ratio = 15.0f, .wheelbase = 2.700f},    // 6: FORD_FOCUS_MK4
  {.slip_factor = -0.00045331952f, .steer_ratio = 16.9f, .wheelbase = 3.700f},    // 7: FORD_F_150_LIGHTNING_MK1
  {.slip_factor = -0.00042037149f, .steer_ratio = 17.0f, .wheelbase = 3.990f},    // 8: FORD_F_150_MK14
  {.slip_factor = -0.00054528036f, .steer_ratio = 17.0f, .wheelbase = 3.076f},    // 9: FORD_MAVERICK_MK1
  {.slip_factor = -0.00058852001f, .steer_ratio = 14.8f, .wheelbase = 2.850f},    // 10: FORD_MONDEO_MK5
  {.slip_factor = -0.00056209187f, .steer_ratio = 17.0f, .wheelbase = 2.984f},    // 11: FORD_MUSTANG_MACH_E_MK1
  {.slip_factor = -0.00051293030f, .steer_ratio = 17.0f, .wheelbase = 3.270f},    // 12: FORD_RANGER_MK2
};

// BluePilot: steering-angle curvature measurement state (STEER_ANGLE_CURVATURE), set once
// in ford_init from current_safety_param_sp. Default off = stock yaw-sourced angle_meas.
static bool ford_bp_pinion_curvature = false;
static const AngleSteeringParams *ford_bp_pinion_params = &ford_pinion_geometry[0];



static int desired_path_angle_last = 0;

// Reset latch: allows bypass for a short period after reset (both curvature and path_angle = 0)
// This enables smooth ramp-up after human turn detection without blocked messages
// Latch activates when reset detected, stays active for ~3 seconds (60 frames at 20Hz)
// Prevents exploitation by requiring reset state first and having a timeout
// BluePilot: openpilot must send curvature_rate ~= 0 during reset and keep apply_curvature_last
// aligned with the prior TX (see carcontroller BP path); else curvature_rate_cmd_checks can trip.
static uint8_t reset_bypass_latch_counter = 0;
static const uint8_t RESET_BYPASS_LATCH_DURATION = 60;  // ~3.0 seconds at 20Hz
static bool test = false;

// BluePilot: angle_mode_engaged + shadow_curvature, read synchronously out of Lane_Assist_Data1's
// unused bits inside ford_tx_hook below (no separate CAN message, no RX -- see fordcan_ext.py's
// create_lka_msg for the wire layout and why: panda does not self-receive its own TX, confirmed
// 2026-07-09). shadow_curvature is the curvature (kappa) that angle mode's path_angle was derived
// from (see lateral_angle_ext.py's bp_kappa_cmd) -- angle mode holds the real curvature signal at
// the inactive sentinel (0) on the wire, so without this there is no commanded-vs-measured
// deviation check for angle mode at all (steer_angle_cmd_checks below is only enforced when
// desired_curvature != 0). Feeding shadow_curvature into that same check when angle mode is
// confirmed engaged restores that protection.
static bool ford_bp_angle_mode_engaged = false;
static int16_t ford_bp_shadow_curvature_raw = 0;  // wire units, scale 1e-6 1/m (see fordcan_ext.py)

// shadow_curvature is packed at scale 1e-6 1/m; convert to the CAN units steer_angle_cmd_checks
// expects, matching FORD_STEERING_LIMITS/FORD_CANFD_STEERING_LIMITS.angle_deg_to_can (50000, i.e.
// physical scale 2e-5): raw * 1e-6 * 50000 = raw * 0.05.
#define FORD_BP_SHADOW_CURVATURE_TO_CAN(raw) ((int)((float)(raw) * 0.05f))

static bool path_angle_cmd_checks(int desired_path_angle, bool steer_control_enabled, const AngleSteeringLimits limits) {
  bool violation = false;

  if(steer_control_enabled){
    float speed = ((float)vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;

    int delta_path_angle_roc = (safety_interpolate(limits.angle_rate_up_lookup, speed) * limits.angle_deg_to_can) + 1.;

    int highest_desired_path_angle = desired_path_angle_last + delta_path_angle_roc;
    int lowest_desired_path_angle = desired_path_angle_last - delta_path_angle_roc;

    violation |= safety_max_limit_check(desired_path_angle, highest_desired_path_angle, lowest_desired_path_angle);
    if (test) {
      FORD_SAFETY_DBG("path_angle_cmd_checks 1: desired_path_angle: %d desired_path_angle_last: %d highest_desired_path_angle: %d lowest_desired_path_angle: %d violation: %d \n",
                      desired_path_angle, desired_path_angle_last, highest_desired_path_angle, lowest_desired_path_angle, (int)violation);
    }
  }
  desired_path_angle_last = desired_path_angle;

  if (!steer_control_enabled) {
    violation |= (desired_path_angle != 0);
  }
  if (test) {
    FORD_SAFETY_DBG("path_angle_cmd_checks 2: violation: %d \n", (int)violation);
  }

  return violation;
}

static int desired_path_offset_last = 0;

static bool path_offset_cmd_checks(int desired_path_offset, bool steer_control_enabled, const AngleSteeringLimits limits) {
  bool violation = false;

  if(steer_control_enabled){
    float speed = ((float)vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;

    int delta_path_offset_roc = (safety_interpolate(limits.angle_rate_up_lookup, speed) * limits.angle_deg_to_can) + 1.;

    int highest_desired_path_offset = desired_path_offset_last + delta_path_offset_roc;
    int lowest_desired_path_offset = desired_path_offset_last - delta_path_offset_roc;

    violation |= safety_max_limit_check(desired_path_offset, highest_desired_path_offset, lowest_desired_path_offset);
    if (test) {
      FORD_SAFETY_DBG("path_offset_cmd_checks 1: desired_path_offset: %d desired_path_offset_last: %d highest_desired_path_offset: %d lowest_desired_path_offset: %d violation: %d \n",
                      desired_path_offset, desired_path_offset_last, highest_desired_path_offset, lowest_desired_path_offset, (int)violation);
    }

  }
  desired_path_offset_last = desired_path_offset;

  if (!steer_control_enabled) {
    violation |= (desired_path_offset != 0);
  }
  if (test) {
    FORD_SAFETY_DBG("path_offset_cmd_checks 2: violation: %d \n", (int)violation);
  }

  return violation;
}

static int desired_curvature_rate_last = 0;

static bool curvature_rate_cmd_checks(int desired_curvature_rate, bool steer_control_enabled, const AngleSteeringLimits limits) {
  bool violation = false;

  if(steer_control_enabled){
    float speed = ((float)vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;

    int desired_curvature_rate_roc = (safety_interpolate(limits.angle_rate_up_lookup, speed) * limits.angle_deg_to_can) + 1.;

    int highest_desired_curvature_rate = desired_curvature_rate_last + desired_curvature_rate_roc;
    int lowest_desired_curvature_rate = desired_curvature_rate_last - desired_curvature_rate_roc;

    violation |= safety_max_limit_check(desired_curvature_rate, highest_desired_curvature_rate, lowest_desired_curvature_rate);
    if (test) {
      FORD_SAFETY_DBG("curvature_rate_cmd_checks 1: desired_curvature_rate: %d desired_curvature_rate_last: %d highest_desired_curvature_rate: %d lowest_desired_curvature_rate: %d violation: %d \n",
                      desired_curvature_rate, desired_curvature_rate_last, highest_desired_curvature_rate, lowest_desired_curvature_rate, (int)violation);
    }
  }
  desired_curvature_rate_last = desired_curvature_rate;


  if (!steer_control_enabled) {
    violation |= (desired_curvature_rate != 0);
  }
  if (test) {
    FORD_SAFETY_DBG("curvature_rate_cmd_checks 2: violation: %d \n", (int)violation);
  }

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
static bool ford_shadow_curvature_error_check(int desired_curvature, bool steer_control_enabled,
                                               const AngleSteeringLimits limits) {
  bool violation = false;
  if (steer_control_enabled && limits.enforce_angle_error &&
      ((vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR) > limits.angle_error_min_speed)) {
    int lowest_allowed = angle_meas.min - limits.max_angle_error - 1;
    int highest_allowed = angle_meas.max + limits.max_angle_error + 1;
    violation = safety_max_limit_check(desired_curvature, highest_allowed, lowest_allowed);
  }
  return violation;
}


static void ford_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == FORD_MAIN_BUS) {
    // Update in motion state from standstill signal
    if (msg->addr == FORD_DesiredTorqBrk) {
      // Signal: VehStop_D_Stat
      vehicle_moving = ((msg->data[3] >> 3) & 0x3U) != 1U;
    }

    // Update vehicle speed
    if (msg->addr == FORD_BrakeSysFeatures) {
      // Signal: Veh_V_ActlBrk
      UPDATE_VEHICLE_SPEED(((msg->data[0] << 8) | msg->data[1]) * 0.01 * KPH_TO_MS);
    }

    // Check vehicle speed against a second source
    if (msg->addr == FORD_EngVehicleSpThrottle2) {
      // Disable controls if speeds from ABS and PCM ECUs are too far apart.
      // Signal: Veh_V_ActlEng
      float filtered_pcm_speed = ((msg->data[6] << 8) | msg->data[7]) * 0.01 * KPH_TO_MS;
      speed_mismatch_check(filtered_pcm_speed);
    }

    // Update vehicle yaw rate (stock angle_meas source; skipped when the pinion source is enabled)
    if ((msg->addr == FORD_Yaw_Data_FD1) && !ford_bp_pinion_curvature) {
      // Signal: VehYaw_W_Actl
      // TODO: we should use the speed which results in the closest angle measurement to the desired angle
      float ford_yaw_rate = (((msg->data[2] << 8U) | msg->data[3]) * 0.0002) - 6.5;
      float current_curvature = ford_yaw_rate / SAFETY_MAX(vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR, 0.1);
      // convert current curvature into units on CAN for comparison with desired curvature
      update_sample(&angle_meas, ROUND(current_curvature * FORD_STEERING_LIMITS.angle_deg_to_can));
    }

    // BluePilot: optional angle_meas source -- measured curvature from the steering pinion
    // angle (PSCM) via the vehicle model, for vehicles whose RCM broadcasts implausible yaw
    // (sign-inverted vs IMU/steering geometry) while its quality flag still reads OK. The
    // pinion angle was validated against the comma IMU (corr +0.99 on real routes); the
    // Python control layer measures from the same source when this is enabled
    // (lateral_curv_ext.get_current_curvature), so the layers always agree.
    if ((msg->addr == FORD_SteeringPinion_Data) && ford_bp_pinion_curvature) {
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
      update_sample(&angle_meas, ROUND(current_curvature * FORD_STEERING_LIMITS.angle_deg_to_can));
    }

    // Update gas pedal
    if (msg->addr == FORD_EngVehicleSpThrottle) {
      // Pedal position: (0.1 * val) in percent
      // Signal: ApedPos_Pc_ActlArb
      gas_pressed = (((msg->data[0] & 0x03U) << 8) | msg->data[1]) > 0U;
    }

    // Update brake pedal and cruise state
    if (msg->addr == FORD_EngBrakeData) {
      // Signal: BpedDrvAppl_D_Actl
      brake_pressed = ((msg->data[0] >> 4) & 0x3U) == 2U;

      // Signal: CcStat_D_Actl
      unsigned int cruise_state = msg->data[1] & 0x07U;
      bool cruise_engaged = (cruise_state == 4U) || (cruise_state == 5U);
      pcm_cruise_check(cruise_engaged);
      acc_main_on = (cruise_state == 3U) || cruise_engaged;
    }

    if (msg->addr == FORD_Steering_Data_FD1) {
      mads_button_press = GET_BIT(msg, 40U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }
  }
}

static bool ford_tx_hook(const CANPacket_t *msg) {
  const LongitudinalLimits FORD_LONG_LIMITS = {
    // acceleration cmd limits (used for brakes)
    // Signal: AccBrkTot_A_Rq
    .max_accel = 5641,       //  1.9999 m/s^s
    .min_accel = 4231,       // -3.4991 m/s^2
    .inactive_accel = 5128,  // -0.0008 m/s^2

    // gas cmd limits
    // Signal: AccPrpl_A_Rq & AccPrpl_A_Pred
    .max_gas = 700,          //  2.0 m/s^2
    .min_gas = 450,          // -0.5 m/s^2
    .inactive_gas = 0,       // -5.0 m/s^2
  };

  bool tx = true;

  // Safety check for ACCDATA accel and brake requests
  if (msg->addr == FORD_ACCDATA) {
    // Signal: AccPrpl_A_Rq
    int gas = ((msg->data[6] & 0x3U) << 8) | msg->data[7];
    // Signal: AccPrpl_A_Pred
    int gas_pred = ((msg->data[2] & 0x3U) << 8) | msg->data[3];
    // Signal: AccBrkTot_A_Rq
    int accel = ((msg->data[0] & 0x1FU) << 8) | msg->data[1];
    // Signal: CmbbDeny_B_Actl
    bool cmbb_deny = (msg->data[4] >> 5) & 1U;

    // Signal: AccBrkPrchg_B_Rq & AccBrkDecel_B_Rq
    bool brake_actuation = ((msg->data[6] >> 6) & 1U) || ((msg->data[6] >> 7) & 1U);

    bool violation = false;
    violation |= longitudinal_accel_checks(accel, FORD_LONG_LIMITS);
    violation |= longitudinal_gas_checks(gas, FORD_LONG_LIMITS);
    violation |= longitudinal_gas_checks(gas_pred, FORD_LONG_LIMITS);

    // Safety check for stock AEB
    violation |= cmbb_deny; // do not prevent stock AEB actuation

    violation |= !get_longitudinal_allowed() && brake_actuation;

    if (violation) {
      tx = false;
    }
  }

  // Safety check for Steering_Data_FD1 button signals
  // Note: Many other signals in this message are not relevant to safety (e.g. blinkers, wiper switches, high beam)
  // which we passthru in OP.
  if (msg->addr == FORD_Steering_Data_FD1) {
    // Violation if resume button is pressed while controls not allowed, or
    // if cancel button is pressed when cruise isn't engaged.
    bool violation = false;
    violation |= ((msg->data[1] >> 0) & 1U) && !cruise_engaged_prev;   // Signal: CcAslButtnCnclPress (cancel)
    violation |= ((msg->data[3] >> 1) & 1U) && !controls_allowed;     // Signal: CcAsllButtnResPress (resume)
    // BluePilot: ICBM's set-speed presses only while controls are allowed
    violation |= ((msg->data[3] >> 3) & 1U) && !controls_allowed;     // Signal: CcAslButtnSetIncPress (set +)
    violation |= ((msg->data[3] >> 4) & 1U) && !controls_allowed;     // Signal: CcAslButtnSetDecPress (set -)

    if (violation) {
      tx = false;
    }
  }

  // Safety check for Lane_Assist_Data1 action
  if (msg->addr == FORD_Lane_Assist_Data1) {
    // Do not allow steering using Lane_Assist_Data1 (Lane-Departure Aid).
    // This message must be sent for Lane Centering to work, and can include
    // values such as the steering angle or lane curvature for debugging,
    // but the action (LkaActvStats_D2_Req) must be set to zero.
    unsigned int action = msg->data[0] >> 5;
    if (action != 0U) {
      tx = false;
    }

    // BluePilot: angle_mode_engaged + shadow_curvature packed into bits with no DBC signal mapped
    // to them (byte4 bit0, bytes 5-6 -- confirmed unused on real F-150 dashcam routes; see
    // fordcan_ext.py's create_lka_msg for the full layout and rationale). Read directly out of the
    // message being transmitted right now, same as curvature/path_angle elsewhere in this file --
    // no separate CAN ID, no RX round-trip.
    ford_bp_angle_mode_engaged = (msg->data[4] & 0x1U) != 0U;
    ford_bp_shadow_curvature_raw = (int16_t)((msg->data[5] << 8) | msg->data[6]);
  }

  // Safety check for LateralMotionControl action
  if (msg->addr == FORD_LateralMotionControl) {
    // Signal: LatCtl_D_Rq
    bool steer_control_enabled = ((msg->data[4] >> 2) & 0x7U) != 0U;
    unsigned int raw_curvature = (msg->data[0] << 3) | (msg->data[1] >> 5);
    unsigned int raw_curvature_rate = ((msg->data[1] & 0x1FU) << 8) | msg->data[2];
    unsigned int raw_path_angle = (msg->data[3] << 3) | (msg->data[4] >> 5);
    unsigned int raw_path_offset = (msg->data[5] << 2) | (msg->data[6] >> 6);
    // unsigned int raw_ramp_type = (msg->data[6] >> 4) & 0x3U;

    bool violation = false;

    // Check curvature value limits (convert to signed values first)
    int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.00002) - 0.02
    // So: raw = (physical + 0.02) / 0.00002 = (physical + 0.02) * 50000
    int curvature_min_can = (int)(FORD_CURVATURE_MIN * FORD_STEERING_LIMITS.angle_deg_to_can);
    int curvature_max_can = (int)(FORD_CURVATURE_MAX * FORD_STEERING_LIMITS.angle_deg_to_can);
    violation |= (desired_curvature < curvature_min_can) || (desired_curvature > curvature_max_can);
    if (test) {
      FORD_SAFETY_DBG("CAN Out: `desired_curvature:%d, curvature_min_can:%d, curvature_max_can:%d, violation: %d\n",
                      desired_curvature, curvature_min_can, curvature_max_can, (int)violation);
    }

    // Check curvature rate value limits (convert to signed values first)
    int desired_curvature_rate = raw_curvature_rate - FORD_INACTIVE_CURVATURE_RATE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 2.5E-007) - 0.001024
    // So: raw = (physical + 0.001024) / 2.5E-007 = (physical + 0.001024) * 4000000
    int curvature_rate_min_can = (int)(FORD_CURVATURE_RATE_MIN * FORD_CURVATURE_RATE_LIMITS_CAN.angle_deg_to_can);
    int curvature_rate_max_can = (int)(FORD_CURVATURE_RATE_MAX * FORD_CURVATURE_RATE_LIMITS_CAN.angle_deg_to_can);
    violation |= (desired_curvature_rate < curvature_rate_min_can) || (desired_curvature_rate > curvature_rate_max_can);
    if (test) {
      FORD_SAFETY_DBG("CAN Out: `desired_curvature_rate:%d, curvature_rate_min_can:%d, curvature_rate_max_can:%d, violation: %d\n",
                      desired_curvature_rate, curvature_rate_min_can, curvature_rate_max_can, (int)violation);
    }

    // Check path offset value limits (convert to signed values first)
    int desired_path_offset = raw_path_offset - FORD_INACTIVE_PATH_OFFSET;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.01) - 5.12
    // So: raw = (physical + 5.12) / 0.01 = (physical + 5.12) * 100
    int path_offset_min_can = (int)(FORD_PATH_OFFSET_MIN * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
    int path_offset_max_can = (int)(FORD_PATH_OFFSET_MAX * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
    violation |= (desired_path_offset < path_offset_min_can) || (desired_path_offset > path_offset_max_can);
    if (test) {
      FORD_SAFETY_DBG("CAN Out: `desired_path_offset:%d, path_offset_min_can:%d, path_offset_max_can:%d, violation: %d\n",
                      desired_path_offset, path_offset_min_can, path_offset_max_can, (int)violation);
    }

    // Check path angle value limits (convert to signed values first)
    int desired_path_angle = raw_path_angle - FORD_INACTIVE_PATH_ANGLE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.0005) - 0.5
    // So: raw = (physical + 0.5) / 0.0005 = (physical + 0.5) * 2000
    // BluePilot: angle mode uses path_angle as the actuator and may swing to the full DBC range,
    // corroborated by ford_bp_angle_mode_engaged (read from Lane_Assist_Data1, see above) so a
    // frame can't unlock this wider range by merely setting curvature to 0. Curvature mode (the
    // default; matches pre-angle-mode bp-6.0 behavior byte for byte) always keeps the tight 0.25
    // cap, including at curvature == 0 (straight driving, or the reset/human-turn frame below) --
    // path_angle there only trims and amplifies wound-up curvature, never needs the wide range.
    float path_angle_min_phys = ford_bp_angle_mode_engaged ? FORD_DBC_PATH_ANGLE_MIN : FORD_PATH_ANGLE_MIN;
    float path_angle_max_phys = ford_bp_angle_mode_engaged ? FORD_DBC_PATH_ANGLE_MAX : FORD_PATH_ANGLE_MAX;
    int path_angle_min_can = (int)(path_angle_min_phys * FORD_PATH_ANGLE_LIMITS.angle_deg_to_can);
    int path_angle_max_can = (int)(path_angle_max_phys * FORD_PATH_ANGLE_LIMITS.angle_deg_to_can);
    violation |= (desired_path_angle < path_angle_min_can) || (desired_path_angle > path_angle_max_can);
    // End BluePilot
    if (test) {
      FORD_SAFETY_DBG("CAN Out: `desired_path_angle:%d, path_angle_min_can:%d, path_angle_max_can:%d, violation: %d\n",
                      desired_path_angle, path_angle_min_can, path_angle_max_can, (int)violation);
    }

    // Check angle error and steer_control_enabled for curvature
    // BluePilot: bp-6.0 (pre-angle-mode) applied steer_angle_cmd_checks's result unconditionally.
    // Angle mode holds curvature pinned at 0 while path_angle does the real steering, so the
    // deviation-vs-measured portion of that check would eventually trip as the car actually turns
    // (measured curvature moves, commanded curvature doesn't) -- skip applying it when
    // desired_curvature == 0. Still call it to keep desired_angle_last in sync, and path_angle
    // keeps its own checks regardless. But steer_angle_cmd_checks also carries the
    // controls_allowed gate bp-6.0 relied on for every frame; restore that piece explicitly so a
    // steer_control_enabled frame at curvature == 0 can't bypass it (see LMC2 block below).
    // BluePilot: the pinion-sourced angle_meas variant carries a wider error band (150 vs 100) --
    // see the FORD_LIMITS macro comment. Everything else in the two limit sets is identical.
    const AngleSteeringLimits *ford_lmc_limits = ford_bp_pinion_curvature ? &FORD_STEERING_LIMITS_PINION : &FORD_STEERING_LIMITS;
    bool curvature_violation = steer_angle_cmd_checks(desired_curvature, steer_control_enabled, *ford_lmc_limits);
    if (desired_curvature != 0) {
      violation |= curvature_violation;
    } else {
      violation |= steer_control_enabled && !(controls_allowed || controls_allowed_lateral);
    }
    // End BluePilot
    if (test) {
      FORD_SAFETY_DBG("CAN Out: 1. desired_curvature violation: %d\n", (int)violation);
    }

    // BluePilot: angle mode's own deviation-only check (no ROC -- path_angle_cmd_checks below
    // already rate-limits the real actuator) against shadow_curvature, once angle mode is confirmed
    // engaged via Lane_Assist_Data1 (see ford_bp_angle_mode_engaged above). If desired_curvature == 0
    // but angle mode is NOT confirmed, this is skipped -- that's ordinary curvature mode at zero
    // (straight driving or the reset/human-turn frame below), which needs no shadow-curvature check;
    // it's still bounded by the tight path_angle range above and steer_control_enabled's own checks.
    if ((desired_curvature == 0) && ford_bp_angle_mode_engaged) {
      int shadow_curvature_can = FORD_BP_SHADOW_CURVATURE_TO_CAN(ford_bp_shadow_curvature_raw);
      violation |= ford_shadow_curvature_error_check(shadow_curvature_can, steer_control_enabled, *ford_lmc_limits);
    }

    // Check path angle rate of change limits
    violation |= path_angle_cmd_checks(desired_path_angle, steer_control_enabled, FORD_PATH_ANGLE_LIMITS);
    if (test) {
      FORD_SAFETY_DBG("CAN Out: 2. desired_path_angle violation: %d\n", (int)violation);
    }

    // Check path offset rate of change limits
    violation |= path_offset_cmd_checks(desired_path_offset, steer_control_enabled, FORD_PATH_OFFSET_LIMITS);
    if (test) {
      FORD_SAFETY_DBG("CAN Out: 3. desired_path_offset violation: %d\n", (int)violation);
    }

    // Check curvature rate rate of change limits
    violation |= curvature_rate_cmd_checks(desired_curvature_rate, steer_control_enabled, FORD_CURVATURE_RATE_LIMITS_CAN);
    if (test) {
      FORD_SAFETY_DBG("CAN Out: 4. desired_curvature_rate violation: %d\n", (int)violation);
    }

    // Reset latch: activate when both curvature and path_angle are zero (reset/neutral state)
    // This allows smooth ramp-up after human turn detection without blocked messages
    if ((desired_curvature == 0) && (desired_path_angle == 0)) {
      // Reset detected, activate latch for ramp period
      reset_bypass_latch_counter = RESET_BYPASS_LATCH_DURATION;
      violation = false;  // Immediate bypass for reset state
    } else if (reset_bypass_latch_counter > 0) {
      // Latch active, allow bypass during ramp-up period
      reset_bypass_latch_counter--;
      violation = false;
    }

    if (violation) {
      tx = false;
    }
  }

  // Safety check for LateralMotionControl2 action
  if (msg->addr == FORD_LateralMotionControl2) {
    static const AngleSteeringLimits FORD_CANFD_STEERING_LIMITS = FORD_LIMITS(true, 100);
    static const AngleSteeringLimits FORD_CANFD_STEERING_LIMITS_PINION = FORD_LIMITS(true, 150);
    // BluePilot: see the CAN handler's ford_lmc_limits comment.
    const AngleSteeringLimits *ford_lmc2_limits = ford_bp_pinion_curvature ? &FORD_CANFD_STEERING_LIMITS_PINION : &FORD_CANFD_STEERING_LIMITS;

    // Signal: LatCtl_D2_Rq
    bool steer_control_enabled = ((msg->data[0] >> 4) & 0x7U) != 0U;
    unsigned int raw_curvature = (msg->data[2] << 3) | (msg->data[3] >> 5);
    unsigned int raw_curvature_rate = (msg->data[6] << 3) | (msg->data[7] >> 5);
    unsigned int raw_path_angle = ((msg->data[3] & 0x1FU) << 6) | (msg->data[4] >> 2);
    unsigned int raw_path_offset = ((msg->data[4] & 0x3U) << 8) | msg->data[5];
    // unsigned int raw_ramp_type = (msg->data[0] >> 1) & 0x3U;  // Extract bits 1-2 from byte 0

    bool violation = false;

    // Check curvature value limits (convert to signed values first)
    int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.00002) - 0.02
    // So: raw = (physical + 0.02) / 0.00002 = (physical + 0.02) * 50000
    int curvature_min_can = (int)(FORD_CURVATURE_MIN * FORD_STEERING_LIMITS.angle_deg_to_can);
    int curvature_max_can = (int)(FORD_CURVATURE_MAX * FORD_STEERING_LIMITS.angle_deg_to_can);
    violation |= (desired_curvature < curvature_min_can) || (desired_curvature > curvature_max_can);
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: `desired_curvature: %d, curvature_min_can: %d, curvature_max_can: %d, violation: %d\n",
                      desired_curvature, curvature_min_can, curvature_max_can, (int)violation);
    }

    // Check curvature rate value limits (convert to signed values first)
    int desired_curvature_rate = raw_curvature_rate - FORD_CANFD_INACTIVE_CURVATURE_RATE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 1E-006) - 0.001024
    // So: raw = (physical + 0.001024) / 1E-006 = (physical + 0.001024) * 1000000
    int curvature_rate_min_can = (int)(FORD_CURVATURE_RATE_MIN * FORD_CURVATURE_RATE_LIMITS_CANFD.angle_deg_to_can);
    int curvature_rate_max_can = (int)(FORD_CURVATURE_RATE_MAX * FORD_CURVATURE_RATE_LIMITS_CANFD.angle_deg_to_can);
    violation |= (desired_curvature_rate < curvature_rate_min_can) || (desired_curvature_rate > curvature_rate_max_can);
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: `desired_curvature_rate: %d, curvature_rate_min_can: %d, curvature_rate_max_can: %d, violation: %d\n",
                      desired_curvature_rate, curvature_rate_min_can, curvature_rate_max_can, (int)violation);
    }

    // Check path offset value limits (convert to signed values first)
    int desired_path_offset = raw_path_offset - FORD_INACTIVE_PATH_OFFSET;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.01) - 5.12
    // So: raw = (physical + 5.12) / 0.01 = (physical + 5.12) * 100
    int path_offset_min_can = (int)(FORD_PATH_OFFSET_MIN * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
    int path_offset_max_can = (int)(FORD_PATH_OFFSET_MAX * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
    violation |= (desired_path_offset < path_offset_min_can) || (desired_path_offset > path_offset_max_can);
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: `desired_path_offset: %d, path_offset_min_can: %d, path_offset_max_can: %d, violation: %d\n",
                      desired_path_offset, path_offset_min_can, path_offset_max_can, (int)violation);
    }

    // Check path angle value limits (convert to signed values first)
    int desired_path_angle = raw_path_angle - FORD_INACTIVE_PATH_ANGLE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.0005) - 0.5
    // So: raw = (physical + 0.5) / 0.0005 = (physical + 0.5) * 2000
    // BluePilot: angle mode uses path_angle as the actuator and may swing to the full DBC range,
    // corroborated by ford_bp_angle_mode_engaged (read from Lane_Assist_Data1, see above) so a
    // frame can't unlock this wider range by merely setting curvature to 0. Curvature mode (the
    // default; matches pre-angle-mode bp-6.0 behavior byte for byte) always keeps the tight 0.25
    // cap, including at curvature == 0 (straight driving, or the reset/human-turn frame below) --
    // path_angle there only trims and amplifies wound-up curvature, never needs the wide range.
    float path_angle_min_phys = ford_bp_angle_mode_engaged ? FORD_DBC_PATH_ANGLE_MIN : FORD_PATH_ANGLE_MIN;
    float path_angle_max_phys = ford_bp_angle_mode_engaged ? FORD_DBC_PATH_ANGLE_MAX : FORD_PATH_ANGLE_MAX;
    int path_angle_min_can = (int)(path_angle_min_phys * FORD_PATH_ANGLE_LIMITS.angle_deg_to_can);
    int path_angle_max_can = (int)(path_angle_max_phys * FORD_PATH_ANGLE_LIMITS.angle_deg_to_can);
    violation |= (desired_path_angle < path_angle_min_can) || (desired_path_angle > path_angle_max_can);
    // End BluePilot
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: `desired_path_angle: %d, path_angle_min_can: %d, path_angle_max_can: %d, violation: %d\n",
                      desired_path_angle, path_angle_min_can, path_angle_max_can, (int)violation);
    }

    // Check angle error and steer_control_enabled for curvature
    // BluePilot: bp-6.0 (pre-angle-mode) applied steer_angle_cmd_checks's result unconditionally.
    // Angle mode holds curvature pinned at 0 while path_angle does the real steering, so the
    // deviation-vs-measured portion of that check would eventually trip as the car actually turns
    // (measured curvature moves, commanded curvature doesn't) -- skip applying it when
    // desired_curvature == 0. Still call it to keep desired_angle_last in sync, and path_angle
    // keeps its own checks regardless. But steer_angle_cmd_checks also carries the
    // controls_allowed gate bp-6.0 relied on for every frame; restore that piece explicitly so a
    // steer_control_enabled frame at curvature == 0 can't bypass it.
    bool curvature_violation = steer_angle_cmd_checks(desired_curvature, steer_control_enabled, *ford_lmc2_limits);
    if (desired_curvature != 0) {
      violation |= curvature_violation;
    } else {
      violation |= steer_control_enabled && !(controls_allowed || controls_allowed_lateral);
    }
    // End BluePilot
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: 1. desired_curvature violation: %d\n", (int)violation);
    }

    // BluePilot: angle mode's own deviation-only check (no ROC -- path_angle_cmd_checks below
    // already rate-limits the real actuator) against shadow_curvature, once angle mode is confirmed
    // engaged via Lane_Assist_Data1 (see ford_bp_angle_mode_engaged above). If desired_curvature == 0
    // but angle mode is NOT confirmed, this is skipped -- that's ordinary curvature mode at zero
    // (straight driving or the reset/human-turn frame below), which needs no shadow-curvature check;
    // it's still bounded by the tight path_angle range above and steer_control_enabled's own checks.
    if ((desired_curvature == 0) && ford_bp_angle_mode_engaged) {
      int shadow_curvature_can = FORD_BP_SHADOW_CURVATURE_TO_CAN(ford_bp_shadow_curvature_raw);
      violation |= ford_shadow_curvature_error_check(shadow_curvature_can, steer_control_enabled, *ford_lmc2_limits);
    }

    // Check path angle rate of change limits
    violation |= path_angle_cmd_checks(desired_path_angle, steer_control_enabled, FORD_PATH_ANGLE_LIMITS);
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: 2. desired_path_angle violation: %d\n", (int)violation);
    }

    // Check path offset rate of change limits
    violation |= path_offset_cmd_checks(desired_path_offset, steer_control_enabled, FORD_PATH_OFFSET_LIMITS);
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: 3. desired_path_offset violation: %d\n", (int)violation);
    }

    // Check curvature rate rate of change limits
    violation |= curvature_rate_cmd_checks(desired_curvature_rate, steer_control_enabled, FORD_CURVATURE_RATE_LIMITS_CANFD);
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: 4. desired_curvature_rate violation: %d\n", (int)violation);
    }

    // Reset latch: activate when both curvature and path_angle are zero (reset/neutral state)
    // This allows smooth ramp-up after human turn detection without blocked messages
    if ((desired_curvature == 0) && (desired_path_angle == 0)) {
      // Reset detected, activate latch for ramp period
      reset_bypass_latch_counter = RESET_BYPASS_LATCH_DURATION;
      violation = false;  // Immediate bypass for reset state
    } else if (reset_bypass_latch_counter > 0) {
      // Latch active, allow bypass during ramp-up period
      reset_bypass_latch_counter--;
      violation = false;
    }

    if (violation) {
      tx = false;
    }
    if(test) {
      FORD_SAFETY_DBG("CANFD Out - final: violation: %d\n", (int)violation);
    }
  }

  return tx;
}

static safety_config ford_init(uint16_t param) {
  // warning: quality flags are not yet checked in openpilot's CAN parser,
  // this may be the cause of blocked messages
  #define FORD_COMMON_RX_CHECKS \
    {.msg = {{FORD_BrakeSysFeatures, 0, 8, 50U, .max_counter = 15U}, { 0 }, { 0 }}}, \
    /* FORD_EngVehicleSpThrottle2 has a counter that either randomly skips or by 2, likely ECU bug */ \
    /* Some hybrid models also experience a bug where this checksum mismatches for one or two frames under heavy acceleration with ACC */ \
    /* It has been confirmed that the Bronco Sport's camera only disallows ACC for bad quality flags, not counters or checksums, so we match that */ \
    {.msg = {{FORD_EngVehicleSpThrottle2, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}}, \
    {.msg = {{FORD_Yaw_Data_FD1, 0, 8, 100U, .max_counter = 255U}, { 0 }, { 0 }}}, \
    /* These messages have no counter or checksum */ \
    {.msg = {{FORD_EngBrakeData, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{FORD_EngVehicleSpThrottle, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{FORD_DesiredTorqBrk, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{FORD_Steering_Data_FD1, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \

  static RxCheck ford_rx_checks[] = {
    FORD_COMMON_RX_CHECKS
  };

  // BluePilot: only enforced when the pinion angle_meas source is enabled -- keeping this
  // entry in the stock config would make a pinion hiccup disable controls for users who
  // never consume the message.
  static RxCheck ford_rx_checks_pinion[] = {
    FORD_COMMON_RX_CHECKS
    // Pinion angle (angle_meas source). Counter verified 0-15 on real frames.
    // StePinAn_No_Cs checksum algorithm is unknown (Ford sum-invert patterns don't match
    // real frames) -> ignore_checksum; integrity via counter + quality flag + 100Hz check.
    {.msg = {{FORD_SteeringPinion_Data, 0, 8, 100U, .max_counter = 15U, .ignore_checksum = true}, { 0 }, { 0 }}},
  };

  // BluePilot: an earlier design tried a dedicated CAN message (0x5F0) for python->ford.h state,
  // relying on panda receiving back its own transmitted frame. Confirmed on real hardware
  // (2026-07-09) that panda does not self-receive its own TX (0x5F0 only ever showed up as a
  // bus+128 TX-echo in the `can` stream, never real RX) -- and registering it in ford_rx_checks
  // made safety_tick()'s 1Hz lagging check (safety.h) trip almost immediately after boot (no
  // per-entry way to exempt a message from that check), forcing safetyRxChecksInvalid=true and
  // controls_allowed=false car-wide, i.e. EventName.controlsMismatch. Replaced with reading
  // angle_mode_engaged/shadow_curvature directly out of Lane_Assist_Data1's unused bits inside its
  // own tx_hook check below -- synchronous, no RX involved. See ford_bp_angle_mode_engaged above.
  #define FORD_COMMON_TX_MSGS \
    {FORD_Steering_Data_FD1, 0, 8, .check_relay = false}, \
    {FORD_Steering_Data_FD1, 2, 8, .check_relay = false}, \
    {FORD_ACCDATA_3, 0, 8, .check_relay = true},          \
    {FORD_Lane_Assist_Data1, 0, 8, .check_relay = true},  \
    {FORD_IPMA_Data, 0, 8, .check_relay = true},          \

  static const CanMsg FORD_CANFD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };

  static const CanMsg FORD_CANFD_STOCK_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };

  static const CanMsg FORD_STOCK_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_LateralMotionControl, 0, 8, .check_relay = true},
  };

  static const CanMsg FORD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl, 0, 8, .check_relay = true},
  };

  const uint16_t FORD_PARAM_CANFD = 2;
  const bool ford_canfd = GET_FLAG(param, FORD_PARAM_CANFD);

  bool ford_longitudinal = false;

#ifdef ALLOW_DEBUG
  const uint16_t FORD_PARAM_LONGITUDINAL = 1;
  ford_longitudinal = GET_FLAG(param, FORD_PARAM_LONGITUDINAL);
#endif

  // Longitudinal is the default for CAN, and optional for CAN FD w/ ALLOW_DEBUG
  // ford_longitudinal = !ford_canfd || ford_longitudinal;

  // BluePilot: steering-angle curvature measurement (bad-yaw-sensor workaround), read from
  // the sunnypilot SP safety param (current_safety_param_sp, delivered via USB 0xdf before
  // the safety model is set -- a separate uint16 from this function's param; pattern:
  // subaru_common.h). Bit 0 enables; bits 1-4 carry the platform geometry-table index.
  // A zero or out-of-range index disables the feature outright, so a half-configured param
  // can never select the wrong geometry: the stock yaw path is kept in that case.
  const uint16_t FORD_PARAM_SP_STEER_ANGLE_CURVATURE = 1;
  bool pinion_enabled = GET_FLAG(current_safety_param_sp, FORD_PARAM_SP_STEER_ANGLE_CURVATURE);
  const uint16_t pinion_geometry_index = (current_safety_param_sp >> 1) & 0xFU;
  if ((pinion_geometry_index == 0U) || (pinion_geometry_index > FORD_PINION_GEOMETRY_COUNT)) {
    pinion_enabled = false;
  }
  ford_bp_pinion_curvature = pinion_enabled;
  ford_bp_pinion_params = pinion_enabled ? &ford_pinion_geometry[pinion_geometry_index] : &ford_pinion_geometry[0];

  safety_config ret;
  if (ford_canfd) {
    ret = ford_longitudinal ? BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_LONG_TX_MSGS) : \
                              BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_STOCK_TX_MSGS);
  } else {
    ret = ford_longitudinal ? BUILD_SAFETY_CFG(ford_rx_checks, FORD_LONG_TX_MSGS) : \
                              BUILD_SAFETY_CFG(ford_rx_checks, FORD_STOCK_TX_MSGS);
  }
  if (ford_bp_pinion_curvature) {
    // Enforce 100Hz/counter/QF on the pinion message only when it is actually consumed.
    SET_RX_CHECKS(ford_rx_checks_pinion, ret);
  }
  return ret;
}

const safety_hooks ford_hooks = {
  .init = ford_init,
  .rx = ford_rx_hook,
  .tx = ford_tx_hook,
  .get_counter = ford_get_counter,
  .get_checksum = ford_get_checksum,
  .compute_checksum = ford_compute_checksum,
  .get_quality_flag_valid = ford_get_quality_flag_valid,
};