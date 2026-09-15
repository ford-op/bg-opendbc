/**
 * BluePilot lateral-control extensions shared across safety modes: declarations.
 * See lateral.h in this directory for the implementation.
 */

#pragma once

#include "opendbc/safety/declarations.h"

// BluePilot: measured-table curvature rate-of-change + error-band check, used by
// steer_curvature_cmd_checks (opendbc/safety/lateral.h) when
// CurvatureSteeringLimits.use_rate_lookup is set -- currently only Ford, see the FORD_LIMITS
// macro in opendbc/safety/modes/ford.h.
extern bool bp_curvature_rate_lookup_check(int desired_curvature, const CurvatureSteeringLimits *limits);
