/**
 * BluePilot Ford lateral-control extensions: type declarations, state, and prototypes.
 * See ford.h in this directory for the implementations.
 */

#pragma once

#include "../../safety/declarations.h"
#include "../safety/lateral_declarations.h"

// BluePilot: Ford safety debug prints. Compile-time switch: build with -DFORD_BP_DEBUG (bench /
// libsafety only) to turn every FORD_BP_DBG into a printf; otherwise it expands to nothing, so CI
// coverage and mutation see no code and the panda build carries no runtime flag. The printf
// branch is bench-only and is not part of the MISRA-checked configuration (test_misra.sh runs
// without FORD_BP_DEBUG). To use it on the bench, compile libsafety with
//   cc ... -DALLOW_DEBUG -DFORD_BP_DEBUG -I . -c <pkg>/safety/tests/libsafety/safety.c
// (the same flags <pkg>/safety/tests/libsafety/libsafety_py.py uses, plus FORD_BP_DEBUG; <pkg> is the
// host's opendbc package: opendbc, iqdbc).
#ifdef FORD_BP_DEBUG
#include <stdio.h>
#define FORD_BP_DBG(...) printf(__VA_ARGS__)
#else
#define FORD_BP_DBG(...) ((void)0)
#endif

// ===============================
// Constants and Defines
// ===============================

// BluePilot: steering-angle curvature measurement (bad-yaw-sensor workaround). Index 0 of
// ford_pinion_geometry is reserved as invalid; see ford_bp_init in ford.h.
#define FORD_PINION_GEOMETRY_COUNT 12U
// Row count including the reserved index-0 entry. Kept as a plain literal: the mutation
// harness rewrites arithmetic in array sizes into runtime expressions, which cannot compile.
#define FORD_PINION_GEOMETRY_ROWS 13U

// Control signal limits -- curvature magnitude must match MAX_CURVATURE; rate tables must match
// bluepilot_lateral/core/values_ext.py BP_ANGLE_LIMITS.
#define FORD_CURVATURE_MIN -0.02f
#define FORD_CURVATURE_MAX 0.02f
#define FORD_PATH_OFFSET_MIN -1.0f
#define FORD_PATH_OFFSET_MAX 1.0f
#define FORD_PATH_ANGLE_MIN -0.25f
#define FORD_PATH_ANGLE_MAX 0.25f
// BluePilot: full DBC signal range for path_angle (LatCtlPath_An, 0.0005 scale). Used as the
// value limit in ANGLE mode, where path_angle is the steering actuator (see ford_lmc_checks).
// Matches lateral_angle_ext.py FORD_DBC_PATH_ANGLE_MIN/MAX. In curvature mode the tight ±0.25
// cap above applies instead (path_angle there only trims and amplifies wound-up curvature).
#define FORD_DBC_PATH_ANGLE_MIN -0.5f
#define FORD_DBC_PATH_ANGLE_MAX 0.5235f

// ===============================
// Global Variables
// ===============================

// BluePilot: optional angle_meas source, measured curvature from the steering pinion angle
// (PSCM) via the vehicle model, selected by the geometry index in current_safety_param_sp
// bits 1-4. Default off = stock yaw-sourced angle_meas. See ford_bp_init in ford.h.
extern bool ford_bp_lateral;
extern bool ford_bp_pinion_curvature;
extern const AngleSteeringParams *ford_bp_pinion_params;
extern const AngleSteeringParams ford_pinion_geometry[FORD_PINION_GEOMETRY_ROWS];

// BluePilot: angle_mode_engaged + shadow_curvature, read out of Lane_Assist_Data1's unused
// bits by ford_bp_lka_tx (no separate CAN message, no RX). The layout is in
// bluepilot_lateral/can/LANE_ASSIST_DATA1.md; fordcan_ext.py packs it with the same values
// (tests/test_lane_assist_data1.py).
#define FORD_BP_LKA_ANGLE_MODE_BYTE 4U
#define FORD_BP_LKA_ANGLE_MODE_MASK 0x1U
#define FORD_BP_LKA_SHADOW_MSB_BYTE 5U  // shadow curvature: int16, big-endian, bytes 5-6
#define FORD_BP_LKA_SHADOW_LSB_BYTE 6U
// shadow curvature raw units (1e-6 1/m) to curvature CAN units (2e-5 1/m)
#define FORD_BP_LKA_SHADOW_RAW_TO_CAN 0.05f
extern bool ford_bp_angle_mode_engaged;
extern int16_t ford_bp_shadow_curvature_raw;  // wire units, scale 1e-6 1/m (LANE_ASSIST_DATA1.md)

// last commanded values for the path_angle / path_offset rate-of-change checks
extern int desired_path_angle_last;
extern int desired_path_offset_last;

// ===============================
// Function Declarations
// ===============================

static inline bool path_angle_cmd_checks(int desired_path_angle, bool steer_control_enabled, AngleSteeringLimits limits);
static inline bool path_offset_cmd_checks(int desired_path_offset, bool steer_control_enabled, AngleSteeringLimits limits);
static inline bool ford_shadow_curvature_error_check(int desired_curvature, bool steer_control_enabled, BP_HOST_CURVATURE_LIMITS limits);

// Shared curvature/curvature_rate/path_offset/path_angle command checks for LateralMotionControl
// (CAN) and LateralMotionControl2 (CAN FD) -- see ford_lmc_checks in ford.h.
static inline bool ford_lmc_checks(int desired_curvature, int desired_curvature_rate, int desired_path_offset, int desired_path_angle,
                            bool steer_control_enabled, const FordBpCurvatureLimits *limits, const char *dbg_prefix);

// Called from modes/ford.h's hooks and ford_init -- see bluepilot_lateral/safety/ford.h.
static inline bool ford_bp_lmc_tx(bool canfd, unsigned int raw_curvature, unsigned int raw_curvature_rate,
                                  unsigned int raw_path_offset, unsigned int raw_path_angle, bool steer_control_enabled);
static inline void ford_bp_lka_tx(const CANPacket_t *msg);
static inline void ford_bp_rx_hook(const CANPacket_t *msg);
static inline bool ford_bp_init(uint16_t param_sp);
