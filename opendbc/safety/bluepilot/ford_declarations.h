/**
 * BluePilot Ford lateral-control extensions: type declarations, state, and prototypes.
 * See ford.h in this directory for the implementations.
 */

#pragma once

#include "opendbc/safety/declarations.h"

// BluePilot: Ford safety debug -- printf in libsafety/safety.c only; no-op on panda (no libc printf)
#ifndef FORD_SAFETY_DBG
#define FORD_SAFETY_DBG(...) ((void)0)
#endif

// ===============================
// Constants and Defines
// ===============================

// BluePilot: steering-angle curvature measurement (bad-yaw-sensor workaround). Index 0 of
// ford_pinion_geometry is reserved as invalid; see ford_init in modes/ford.h.
#define FORD_PINION_GEOMETRY_COUNT 12U
// Row count including the reserved index-0 entry. Kept as a plain literal: the mutation
// harness rewrites arithmetic in array sizes into runtime expressions, which cannot compile.
#define FORD_PINION_GEOMETRY_ROWS 13U

// shadow_curvature is packed at scale 1e-6 1/m; convert to the CAN units steer_angle_cmd_checks
// expects, matching FORD_BP_STEERING_LIMITS/FORD_CANFD_STEERING_LIMITS.curvature_to_can (50000, i.e.
// physical scale 2e-5): raw * 1e-6 * 50000 = raw * 0.05.

// Reset latch duration: ~3.0 seconds at 20Hz
#define FORD_RESET_BYPASS_LATCH_DURATION 60U

// Control signal limits -- curvature magnitude must match MAX_CURVATURE; rate tables must match
// opendbc/sunnypilot/car/ford/values_ext.py BP_ANGLE_LIMITS.
#define FORD_CURVATURE_MIN -0.02f
#define FORD_CURVATURE_MAX 0.02f
#define FORD_CURVATURE_RATE_MIN -0.001024f
#define FORD_CURVATURE_RATE_MAX 0.00102375f
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
// bits 1-4. Default off = stock yaw-sourced angle_meas. See ford_init in modes/ford.h.
extern bool ford_bp_lateral;
extern bool ford_bp_pinion_curvature;
extern const AngleSteeringParams *ford_bp_pinion_params;
extern const AngleSteeringParams ford_pinion_geometry[FORD_PINION_GEOMETRY_ROWS];

// BluePilot: angle_mode_engaged + shadow_curvature, read out of Lane_Assist_Data1's unused
// bits inside ford_tx_hook (no separate CAN message, no RX -- see fordcan_ext.py's
// create_lka_msg for the wire layout).
extern bool ford_bp_angle_mode_engaged;
extern int16_t ford_bp_shadow_curvature_raw;  // wire units, scale 1e-6 1/m (see fordcan_ext.py)

// last commanded values for the path_angle / path_offset / curvature_rate rate-of-change checks
extern int desired_path_angle_last;
extern int desired_path_offset_last;
extern int desired_curvature_rate_last;

// BluePilot: reset latch state -- allows a bypass window after both curvature and path_angle
// reset to 0, so ramp-up after human turn detection isn't blocked. See
// ford_reset_bypass_latch_check. openpilot must send curvature_rate ~= 0 during reset and keep
// apply_curvature_last aligned with the prior TX (see carcontroller BP path); else
// curvature_rate_cmd_checks can trip.
extern uint8_t reset_bypass_latch_counter;

// BluePilot: debug flag -- gates FORD_SAFETY_DBG printf calls (libsafety/safety.c only; no-op on
// panda).
extern bool ford_bp_debug;

// ===============================
// Function Declarations
// ===============================

static inline bool path_angle_cmd_checks(int desired_path_angle, bool steer_control_enabled, AngleSteeringLimits limits);
static inline bool path_offset_cmd_checks(int desired_path_offset, bool steer_control_enabled, AngleSteeringLimits limits);
static inline bool curvature_rate_cmd_checks(int desired_curvature_rate, bool steer_control_enabled, AngleSteeringLimits limits);
static inline bool ford_shadow_curvature_error_check(int desired_curvature, bool steer_control_enabled, CurvatureSteeringLimits limits);
static inline bool ford_reset_bypass_latch_check(int desired_curvature, int desired_path_angle);

// Shared curvature/curvature_rate/path_offset/path_angle command checks for LateralMotionControl
// (CAN) and LateralMotionControl2 (CAN FD) -- see ford_lmc_checks in ford.h.
static inline bool ford_lmc_checks(int desired_curvature, int desired_curvature_rate, int desired_path_offset, int desired_path_angle,
                            bool steer_control_enabled, const CurvatureSteeringLimits *curvature_limits,
                            const CurvatureSteeringLimits *curvature_limits_pinion, const AngleSteeringLimits *curvature_rate_limits,
                            const char *dbg_prefix);
