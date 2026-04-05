#pragma once

#include "opendbc/safety/declarations.h"

// Safety-relevant CAN messages for Ford vehicles.
#define FORD_EngBrakeData          0x165U   // RX from PCM, for driver brake pedal and cruise state
#define FORD_EngVehicleSpThrottle  0x204U   // RX from PCM, for driver throttle input
#define FORD_DesiredTorqBrk        0x213U   // RX from ABS, for standstill state
#define FORD_BrakeSysFeatures      0x415U   // RX from ABS, for vehicle speed
#define FORD_EngVehicleSpThrottle2 0x202U   // RX from PCM, for second vehicle speed
#define FORD_Yaw_Data_FD1          0x91U    // RX from RCM, for yaw rate
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
  } else {
  }
  return valid;
}

#define FORD_INACTIVE_CURVATURE 1000U
#define FORD_INACTIVE_CURVATURE_RATE 4096U
#define FORD_INACTIVE_PATH_OFFSET 512U
#define FORD_INACTIVE_PATH_ANGLE 1000U

#define FORD_CANFD_INACTIVE_CURVATURE_RATE 1024U

// Control signal limits
#define FORD_CURVATURE_MIN -0.012f
#define FORD_CURVATURE_MAX 0.012f
#define FORD_CURVATURE_RATE_MIN -0.001024f
#define FORD_CURVATURE_RATE_MAX 0.00102375f
#define FORD_PATH_OFFSET_MIN -1.0f
#define FORD_PATH_OFFSET_MAX 1.0f
#define FORD_PATH_ANGLE_MIN -0.25f
#define FORD_PATH_ANGLE_MAX 0.25f



// Curvature rate limits
#define FORD_LIMITS(limit_lateral_acceleration) {                                               \
  .max_angle = 1000,          /* 0.02 curvature */                                              \
  .angle_deg_to_can = 50000,  /* 1 / (2e-5) rad to can */                                       \
  .max_angle_error = 100,     /* 0.002 * FORD_STEERING_LIMITS.angle_deg_to_can */               \
  .angle_rate_up_lookup = {                                                                     \
    {5., 16., 25.},                                                                             \
    {0.0026, 0.0013, 0.0001}                                                                   \
  },                                                                                            \
  .angle_rate_down_lookup = {                                                                   \
    {5., 16., 25.},                                                                             \
    {0.0026, 0.0015, 0.0002}                                                                 \
  },                                                                                            \
                                                                                                \
  /* no blending at low speed due to lack of torque wind-up and inaccurate current curvature */ \
  .angle_error_min_speed = 10.0,    /* m/s */                                                   \
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
  .angle_rate_up_lookup = {
    .x = {5., 15., 25.},
    .y = {0.003, 0.0015, 0.002}
  },
  .angle_rate_down_lookup = {
    .x = {5., 15., 25.},
    .y = {0.003, 0.0015, 0.002}
  },
  .angle_error_min_speed = 9.9,   // m/s
  .frequency = 100U,              // Hz

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

static const AngleSteeringLimits FORD_STEERING_LIMITS = FORD_LIMITS(false);



static int desired_path_angle_last = 0;

// Reset latch: allows bypass for a short period after reset (both curvature and path_angle = 0)
// This enables smooth ramp-up after human turn detection without blocked messages
// Latch activates when reset detected, stays active for ~3 seconds (60 frames at 20Hz)
// Prevents exploitation by requiring reset state first and having a timeout
static uint8_t reset_bypass_latch_counter = 0;
static const uint8_t RESET_BYPASS_LATCH_DURATION = 60;  // ~3.0 seconds at 20Hz

static bool path_angle_cmd_checks(int desired_path_angle, bool steer_control_enabled, const AngleSteeringLimits limits) {
  bool violation = false;

  if(steer_control_enabled){
    float speed = ((float)vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;

    int delta_path_angle_roc = (safety_interpolate(limits.angle_rate_up_lookup, speed) * limits.angle_deg_to_can) + 1.;

    int highest_desired_path_angle = desired_path_angle_last + delta_path_angle_roc;
    int lowest_desired_path_angle = desired_path_angle_last - delta_path_angle_roc;

    violation |= safety_max_limit_check(desired_path_angle, highest_desired_path_angle, lowest_desired_path_angle);
    // print("path_angle_cmd_checks 1: ");
    // print("desired_path_angle: "); puti(desired_path_angle); print(" ");
    // print("desired_path_angle_last: "); puti(desired_path_angle_last); print(" ");
    // print("highest_desired_path_angle: "); puti(highest_desired_path_angle); print(" ");
    // print("lowest_desired_path_angle: "); puti(lowest_desired_path_angle); print(" ");
    // print("violation: "); puti(violation); print(" ");
    // print("`\n");
  }
  desired_path_angle_last = desired_path_angle;

  if (!steer_control_enabled) {
    violation |= (desired_path_angle != 0);
  }
  // print("path_angle_cmd_checks 2: ");
  // print("violation: "); puti(violation); print(" ");
  // print("`\n");

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
    // print("path_offset_cmd_checks 1: ");
    // print("desired_path_offset: "); puti(desired_path_offset); print(" ");
    // print("desired_path_offset_last: "); puti(desired_path_offset_last); print(" ");
    // print("highest_desired_path_offset: "); puti(highest_desired_path_offset); print(" ");
    // print("lowest_desired_path_offset: "); puti(lowest_desired_path_offset); print(" ");
    // print("violation: "); puti(violation); print(" ");
    // print("`\n");

  }
  desired_path_offset_last = desired_path_offset;

  if (!steer_control_enabled) {
    violation |= (desired_path_offset != 0);
  }
  // print("path_offset_cmd_checks 2: ");
  // print("violation: "); puti(violation); print(" ");
  // print("`\n");

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
    // print("curvature_rate_cmd_checks 1: ");
    // print("desired_curvature_rate: "); puti(desired_curvature_rate); print(" ");
    // print("desired_curvature_rate_last: "); puti(desired_curvature_rate_last); print(" ");
    // print("highest_desired_curvature_rate: "); puti(highest_desired_curvature_rate); print(" ");
    // print("lowest_desired_curvature_rate: "); puti(lowest_desired_curvature_rate); print(" ");
    // print("violation: "); puti(violation); print(" ");
    // print("`\n");
  }
  desired_curvature_rate_last = desired_curvature_rate;


  if (!steer_control_enabled) {
    violation |= (desired_curvature_rate != 0);
  }
  // print("curvature_rate_cmd_checks 2: ");
  // print("violation: "); puti(violation); print(" ");
  // print("`\n");

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

    // Update vehicle yaw rate
    if (msg->addr == FORD_Yaw_Data_FD1) {
      // Signal: VehYaw_W_Actl
      // TODO: we should use the speed which results in the closest angle measurement to the desired angle
      float ford_yaw_rate = (((msg->data[2] << 8U) | msg->data[3]) * 0.0002) - 6.5;
      float current_curvature = ford_yaw_rate / SAFETY_MAX(vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR, 0.1);
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
  // bool test = true;

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
    // if(test){
    //   print("CAN Out: `desired_curvature:"); puti(desired_curvature); print(", curvature_min_can:"); puti(curvature_min_can); print(", curvature_max_can:"); puti(curvature_max_can); print(", violation:"); puti(violation); print("`\n");
    // }

    // Check curvature rate value limits (convert to signed values first)
    int desired_curvature_rate = raw_curvature_rate - FORD_INACTIVE_CURVATURE_RATE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 2.5E-007) - 0.001024
    // So: raw = (physical + 0.001024) / 2.5E-007 = (physical + 0.001024) * 4000000
    int curvature_rate_min_can = (int)(FORD_CURVATURE_RATE_MIN * FORD_CURVATURE_RATE_LIMITS_CAN.angle_deg_to_can);
    int curvature_rate_max_can = (int)(FORD_CURVATURE_RATE_MAX * FORD_CURVATURE_RATE_LIMITS_CAN.angle_deg_to_can);
    violation |= (desired_curvature_rate < curvature_rate_min_can) || (desired_curvature_rate > curvature_rate_max_can);
    // if(test){
    //   print("CAN Out: `desired_curvature_rate:"); puti(desired_curvature_rate); print(", curvature_rate_min_can:"); puti(curvature_rate_min_can); print(", curvature_rate_max_can:"); puti(curvature_rate_max_can); print(", violation:"); puti(violation); print("`\n");
    // }

    // Check path offset value limits (convert to signed values first)
    int desired_path_offset = raw_path_offset - FORD_INACTIVE_PATH_OFFSET;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.01) - 5.12
    // So: raw = (physical + 5.12) / 0.01 = (physical + 5.12) * 100
    int path_offset_min_can = (int)(FORD_PATH_OFFSET_MIN * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
    int path_offset_max_can = (int)(FORD_PATH_OFFSET_MAX * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
    violation |= (desired_path_offset < path_offset_min_can) || (desired_path_offset > path_offset_max_can);
    // if(test){
    //   print("CAN Out: `desired_path_offset:"); puti(desired_path_offset); print(", path_offset_min_can:"); puti(path_offset_min_can); print(", path_offset_max_can:"); puti(path_offset_max_can); print(", violation:"); puti(violation); print("`\n");
    // }

    // Check path angle value limits (convert to signed values first)
    int desired_path_angle = raw_path_angle - FORD_INACTIVE_PATH_ANGLE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.0005) - 0.5
    // So: raw = (physical + 0.5) / 0.0005 = (physical + 0.5) * 2000
    int path_angle_min_can = (int)(FORD_PATH_ANGLE_MIN * FORD_PATH_ANGLE_LIMITS.angle_deg_to_can);
    int path_angle_max_can = (int)(FORD_PATH_ANGLE_MAX * FORD_PATH_ANGLE_LIMITS.angle_deg_to_can);
    violation |= (desired_path_angle < path_angle_min_can) || (desired_path_angle > path_angle_max_can);
    // if(test){
    //   print("CAN Out: `desired_path_angle:"); puti(desired_path_angle); print(", path_angle_min_can:"); puti(path_angle_min_can); print(", path_angle_max_can:"); puti(path_angle_max_can); print(", violation:"); puti(violation); print("`\n");
    // }

    // Check angle error and steer_control_enabled for curvature
    violation |= steer_angle_cmd_checks(desired_curvature, steer_control_enabled, FORD_STEERING_LIMITS);
    // if(test){
    //  print("CAN Out: 1. violation:"); puti(violation); print("`\n");
    // }

    // Check path angle rate of change limits
    violation |= path_angle_cmd_checks(desired_path_angle, steer_control_enabled, FORD_PATH_ANGLE_LIMITS);
    // if(test){
    //   print("CAN Out: 2. violation:"); puti(violation); print("`\n");
    // }

    // Check path offset rate of change limits
    violation |= path_offset_cmd_checks(desired_path_offset, steer_control_enabled, FORD_PATH_OFFSET_LIMITS);
    // if(test){
    //   print("CAN Out: 3. violation:"); puti(violation); print("`\n");
    // }

    // Check curvature rate rate of change limits
    violation |= curvature_rate_cmd_checks(desired_curvature_rate, steer_control_enabled, FORD_CURVATURE_RATE_LIMITS_CAN);
    // if(test){
    //   print("CAN Out: 4. violation:"); puti(violation); print("`\n");
    // }

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
    static const AngleSteeringLimits FORD_CANFD_STEERING_LIMITS = FORD_LIMITS(true);

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
    // if(test){
    //   print("CANFD Out: `desired_curvature:"); puti(desired_curvature); print(", curvature_min_can:"); puti(curvature_min_can); print(", curvature_max_can:"); puti(curvature_max_can); print(", violation:"); puti(violation); print("`\n");
    // }

    // Check curvature rate value limits (convert to signed values first)
    int desired_curvature_rate = raw_curvature_rate - FORD_CANFD_INACTIVE_CURVATURE_RATE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 1E-006) - 0.001024
    // So: raw = (physical + 0.001024) / 1E-006 = (physical + 0.001024) * 1000000
    int curvature_rate_min_can = (int)(FORD_CURVATURE_RATE_MIN * FORD_CURVATURE_RATE_LIMITS_CANFD.angle_deg_to_can);
    int curvature_rate_max_can = (int)(FORD_CURVATURE_RATE_MAX * FORD_CURVATURE_RATE_LIMITS_CANFD.angle_deg_to_can);
    violation |= (desired_curvature_rate < curvature_rate_min_can) || (desired_curvature_rate > curvature_rate_max_can);
    // if(test){
    //   print("CANFD Out: `desired_curvature_rate:"); puti(desired_curvature_rate); print(", curvature_rate_min_can:"); puti(curvature_rate_min_can); print(", curvature_rate_max_can:"); puti(curvature_rate_max_can); print(", violation:"); puti(violation); print("`\n");
    // }

    // Check path offset value limits (convert to signed values first)
    int desired_path_offset = raw_path_offset - FORD_INACTIVE_PATH_OFFSET;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.01) - 5.12
    // So: raw = (physical + 5.12) / 0.01 = (physical + 5.12) * 100
    int path_offset_min_can = (int)(FORD_PATH_OFFSET_MIN * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
    int path_offset_max_can = (int)(FORD_PATH_OFFSET_MAX * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
    violation |= (desired_path_offset < path_offset_min_can) || (desired_path_offset > path_offset_max_can);
    // if(test){
    //   print("CANFD Out: `desired_path_offset:"); puti(desired_path_offset); print(", path_offset_min_can:"); puti(path_offset_min_can); print(", path_offset_max_can:"); puti(path_offset_max_can); print(", violation:"); puti(violation); print("`\n");
    // }

    // Check path angle value limits (convert to signed values first)
    int desired_path_angle = raw_path_angle - FORD_INACTIVE_PATH_ANGLE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.0005) - 0.5
    // So: raw = (physical + 0.5) / 0.0005 = (physical + 0.5) * 2000
    int path_angle_min_can = (int)(FORD_PATH_ANGLE_MIN * FORD_PATH_ANGLE_LIMITS.angle_deg_to_can);
    int path_angle_max_can = (int)(FORD_PATH_ANGLE_MAX * FORD_PATH_ANGLE_LIMITS.angle_deg_to_can);
    violation |= (desired_path_angle < path_angle_min_can) || (desired_path_angle > path_angle_max_can);
    // if(test){
    //   print("CANFD Out: `desired_path_angle:"); puti(desired_path_angle); print(", path_angle_min_can:"); puti(path_angle_min_can); print(", path_angle_max_can:"); puti(path_angle_max_can); print(", violation:"); puti(violation); print("`\n");
    // }

    // Check angle error and steer_control_enabled for curvature
    violation |= steer_angle_cmd_checks(desired_curvature, steer_control_enabled, FORD_CANFD_STEERING_LIMITS);
    // if(test){
    //   print("CANFD Out: 1. violation:"); puti(violation); print("`\n");
    // }

    // Check path angle rate of change limits
    violation |= path_angle_cmd_checks(desired_path_angle, steer_control_enabled, FORD_PATH_ANGLE_LIMITS);
    // if(test){
    //   print("CANFD Out: 2. violation:"); puti(violation); print("`\n");
    // }

    // Check path offset rate of change limits
    violation |= path_offset_cmd_checks(desired_path_offset, steer_control_enabled, FORD_PATH_OFFSET_LIMITS);
    // if(test){
    //   print("CANFD Out: 3. violation:"); puti(violation); print("`\n");
    // }

    // Check curvature rate rate of change limits
    violation |= curvature_rate_cmd_checks(desired_curvature_rate, steer_control_enabled, FORD_CURVATURE_RATE_LIMITS_CANFD);
    // if(test){
    //   print("CANFD Out: 4. violation:"); puti(violation); print("`\n");
    // }

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

  return tx;
}

static safety_config ford_init(uint16_t param) {
  // warning: quality flags are not yet checked in openpilot's CAN parser,
  // this may be the cause of blocked messages
  static RxCheck ford_rx_checks[] = {
    {.msg = {{FORD_BrakeSysFeatures, 0, 8, 50U, .max_counter = 15U}, { 0 }, { 0 }}},
    // FORD_EngVehicleSpThrottle2 has a counter that either randomly skips or by 2, likely ECU bug
    // Some hybrid models also experience a bug where this checksum mismatches for one or two frames under heavy acceleration with ACC
    // It has been confirmed that the Bronco Sport's camera only disallows ACC for bad quality flags, not counters or checksums, so we match that
    {.msg = {{FORD_EngVehicleSpThrottle2, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_Yaw_Data_FD1, 0, 8, 100U, .max_counter = 255U}, { 0 }, { 0 }}},
    // These messages have no counter or checksum
    {.msg = {{FORD_EngBrakeData, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_EngVehicleSpThrottle, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_DesiredTorqBrk, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_Steering_Data_FD1, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

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

  safety_config ret;
  if (ford_canfd) {
    ret = ford_longitudinal ? BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_LONG_TX_MSGS) : \
                              BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_STOCK_TX_MSGS);
  } else {
    ret = ford_longitudinal ? BUILD_SAFETY_CFG(ford_rx_checks, FORD_LONG_TX_MSGS) : \
                              BUILD_SAFETY_CFG(ford_rx_checks, FORD_STOCK_TX_MSGS);
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
