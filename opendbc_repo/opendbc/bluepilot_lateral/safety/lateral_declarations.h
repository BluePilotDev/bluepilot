/**
 * BluePilot lateral-control extensions shared across safety modes: declarations.
 * See lateral.h in this directory for the implementation.
 */

#pragma once

#include "../../safety/declarations.h"

// BluePilot: the host bindings. The library is written against comma's opendbc safety names; a host
// whose names differ defines these before including safety/ford.h (HOSTS.md, "Host bindings"). The
// defaults are upstream opendbc's (sunnypilot/opendbc), so a host on those names defines nothing.
// Type of FordBpCurvatureLimits.base: the host's curvature limit struct. The library reads only its
// max_curvature, curvature_to_can, frequency, max_curvature_error and curvature_error_min_speed.
#ifndef BP_HOST_CURVATURE_LIMITS
#define BP_HOST_CURVATURE_LIMITS CurvatureSteeringLimits
#endif
// The host's curvature state (desired_last, meas), the one its curvature RX hook fills.
#ifndef BP_HOST_CURVATURE_STATE
#define BP_HOST_CURVATURE_STATE curvature_state
#endif
// The host's real-time message-rate check, taking a BP_HOST_CURVATURE_LIMITS by value.
#ifndef BP_HOST_RT_CURVATURE_RATE_CHECK
#define BP_HOST_RT_CURVATURE_RATE_CHECK rt_curvature_rate_limit_check
#endif
// Whether lateral control is allowed: controls_allowed or the host's lateral-only engagement (MADS).
#ifndef BP_HOST_LAT_ALLOWED
#define BP_HOST_LAT_ALLOWED() (controls_allowed || controls_allowed_lateral)
#endif
// The second-speed cross-check upstream runs inside its curvature check. A host that runs it in its
// RX hook instead defines this as ((void)0).
#ifndef BP_HOST_SPEED_MISMATCH_CHECK
#define BP_HOST_SPEED_MISMATCH_CHECK() speed_mismatch_check((float)vehicle_speed_2.values[0] / VEHICLE_SPEED_FACTOR)
#endif

// BluePilot: curvature limits for the measured-table check. base holds upstream's fields
// (max_curvature, curvature_to_can, frequency, error band, ...); the rest are BluePilot's.
typedef struct {
  BP_HOST_CURVATURE_LIMITS base;
  // Explicit rate-of-change tables, mirrored from values_ext.py BP_ANGLE_LIMITS. Upstream derives
  // the per-tick delta from the ISO lateral jerk limit alone, which is looser than Ford's EPS
  // tolerates at low speed and tighter than it tolerates at high speed.
  struct lookup_t curvature_rate_up_lookup;
  struct lookup_t curvature_rate_down_lookup;
  // Ford CAN (Q3) is NOT lateral-accel limited in safety -- the EPS already caps it, and applying
  // the ISO cap here would clip commands the car accepts today. Ford CAN FD (Q4) has more torque
  // available and IS limited.
  bool limit_lateral_acceleration;
} FordBpCurvatureLimits;

// BluePilot: measured-table curvature rate-of-change + error-band check, used by
// ford_bp_curvature_cmd_checks below.
static inline bool ford_bp_curvature_rate_lookup_check(int desired_curvature, const FordBpCurvatureLimits *limits);

// BluePilot: curvature command checks for the BluePilot lateral (Ford only today, see the
// FORD_LIMITS macro in bluepilot_lateral/safety/ford.h). Upstream's steer_curvature_cmd_checks with
// the measured-table check in place of its ISO accel and jerk checks, and no value checks at
// curvature 0, the no-curvature-command sentinel (only the message-rate check and the gates).
static inline bool ford_bp_curvature_cmd_checks(int desired_curvature, bool steer_control_enabled,
                                                const FordBpCurvatureLimits *limits);
