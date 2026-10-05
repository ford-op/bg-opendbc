/**
 * BluePilot lateral-control extensions shared across safety modes: declarations.
 * See lateral.h in this directory for the implementation.
 */

#pragma once

#include "opendbc/safety/declarations.h"

// BluePilot: curvature limits for the measured-table check. base holds upstream's fields
// (max_curvature, curvature_to_can, frequency, error band, ...); the rest are BluePilot's.
typedef struct {
  CurvatureSteeringLimits base;
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
static inline bool bp_curvature_rate_lookup_check(int desired_curvature, const FordBpCurvatureLimits *limits);

// BluePilot: curvature command checks for the BluePilot lateral (Ford only today, see the
// FORD_LIMITS macro in opendbc/safety/modes/ford.h). Upstream's steer_curvature_cmd_checks with
// the measured-table check in place of its ISO accel and jerk checks.
static inline bool ford_bp_curvature_cmd_checks(int desired_curvature, bool steer_control_enabled,
                                                const FordBpCurvatureLimits *limits);
