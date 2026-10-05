/**
 * BluePilot lateral-control extensions shared across safety modes: implementation.
 * See lateral_declarations.h in this directory for the declarations.
 */

#pragma once

#include "opendbc/safety/bluepilot/lateral_declarations.h"

// BluePilot: measured-table rate limiting. This is the pre-sync steer_angle_cmd_checks logic,
// carried over verbatim in the curvature domain so Ford lateral behavior is unchanged by the
// move off AngleSteeringLimits. Differences from upstream's ISO-only path (steer_curvature_cmd_checks
// in opendbc/safety/lateral.h) that matter:
//   - asymmetric up/down deltas selected by the sign of the last command
//   - the error boundary uses relaxed deltas and CONVERGES toward measured, rather than
//     hard-clamping, so a wound-up command unwinds instead of instantly faulting
//   - the ISO lateral accel cap is applied only on buses that need it (Q4/CAN FD)
static inline bool bp_curvature_rate_lookup_check(int desired_curvature, const FordBpCurvatureLimits *limits) {
  bool violation = false;

  const float fudged_speed_lookup = (vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;
  const int delta_up = (safety_interpolate(limits->curvature_rate_up_lookup, fudged_speed_lookup) * limits->base.curvature_to_can) + 1.;
  const int delta_down = (safety_interpolate(limits->curvature_rate_down_lookup, fudged_speed_lookup) * limits->base.curvature_to_can) + 1.;

  // allow down limits at zero since small floats from openpilot will be rounded to 0
  int highest_desired_curvature = curvature_state.desired_last + ((curvature_state.desired_last > 0) ? delta_up : delta_down);
  int lowest_desired_curvature = curvature_state.desired_last - ((curvature_state.desired_last >= 0) ? delta_down : delta_up);

  if (limits->base.max_curvature_error && ((vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR) > limits->base.curvature_error_min_speed)) {
    // flipped fudge to avoid false positives
    const float fudged_speed_error = (vehicle_speed.max / VEHICLE_SPEED_FACTOR) + 1.;
    const int delta_up_relaxed = (safety_interpolate(limits->curvature_rate_up_lookup, fudged_speed_error) * limits->base.curvature_to_can) - 1.;
    const int delta_down_relaxed = (safety_interpolate(limits->curvature_rate_down_lookup, fudged_speed_error) * limits->base.curvature_to_can) - 1.;

    const int lowest_desired_curvature_error = curvature_state.meas.min - limits->base.max_curvature_error - 1;
    const int highest_desired_curvature_error = curvature_state.meas.max + limits->base.max_curvature_error + 1;

    // the MAX/MIN allow the desired value to reach the edge of the bounds without going under
    if (curvature_state.desired_last > highest_desired_curvature_error) {
      const int delta = (curvature_state.desired_last >= 0) ? delta_down_relaxed : delta_up_relaxed;
      highest_desired_curvature = SAFETY_MAX(curvature_state.desired_last - delta, highest_desired_curvature_error);

    } else if (curvature_state.desired_last < lowest_desired_curvature_error) {
      const int delta = (curvature_state.desired_last <= 0) ? delta_down_relaxed : delta_up_relaxed;
      lowest_desired_curvature = SAFETY_MIN(curvature_state.desired_last + delta, lowest_desired_curvature_error);

    } else {
      // already inside error boundary, don't allow commanding outside it
      highest_desired_curvature = SAFETY_MIN(highest_desired_curvature, highest_desired_curvature_error);
      lowest_desired_curvature = SAFETY_MAX(lowest_desired_curvature, lowest_desired_curvature_error);
    }

    // don't enforce above the max curvature
    lowest_desired_curvature = SAFETY_CLAMP(lowest_desired_curvature, -limits->base.max_curvature, limits->base.max_curvature);
    highest_desired_curvature = SAFETY_CLAMP(highest_desired_curvature, -limits->base.max_curvature, limits->base.max_curvature);
  }

  violation |= safety_max_limit_check(desired_curvature, highest_desired_curvature, lowest_desired_curvature);

  if (limits->limit_lateral_acceleration) {
    // Limit to average banked road since safety doesn't have the roll. NOTE: this is the
    // pre-sync constant (ISO_LATERAL_ACCEL MINUS the roll term, ~2.4 m/s^2), not the
    // MAX_LATERAL_ACCEL used by the ISO path in steer_curvature_cmd_checks (~3.6 m/s^2). Do not
    // merge the two.
    static const float BP_MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL - (EARTH_G * AVERAGE_ROAD_ROLL);
    const float fudged_speed = SAFETY_MAX((vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.0, 1.0);
    const float max_curvature_accel = BP_MAX_LATERAL_ACCEL / (fudged_speed * fudged_speed);
    const int max_curvature_accel_can = (max_curvature_accel * limits->base.curvature_to_can) + 1.;
    violation |= safety_max_limit_check(desired_curvature, max_curvature_accel_can, -max_curvature_accel_can);
  }

  return violation;
}

// BluePilot: upstream's steer_curvature_cmd_checks (opendbc/safety/lateral.h) with
// bp_curvature_rate_lookup_check in place of its ISO lateral accel and jerk checks. Ford has no
// steer power signal (max_steer_power 0) and resets to zero on a violation
// (inactive_curvature_is_zero), so only those sides of upstream's branches are kept.
static inline bool ford_bp_curvature_cmd_checks(int desired_curvature, bool steer_control_enabled,
                                                const FordBpCurvatureLimits *limits) {
  bool violation = false;

  speed_mismatch_check((float)vehicle_speed_2.values[0] / VEHICLE_SPEED_FACTOR);

  if ((controls_allowed || controls_allowed_lateral) && steer_control_enabled) {
    // *** absolute curvature cap ***
    violation |= safety_max_limit_check(desired_curvature, limits->base.max_curvature, -limits->base.max_curvature);

    // *** measured-table rate limits, error band and (CAN FD) lateral accel cap ***
    violation |= bp_curvature_rate_lookup_check(desired_curvature, limits);

    // *** real time rate limit check ***
    violation |= rt_curvature_rate_limit_check(limits->base);
  }
  curvature_state.desired_last = desired_curvature;

  // Curvature must be 0 while not steering
  if (!steer_control_enabled) {
    violation |= desired_curvature != 0;
  }

  // No curvature control allowed when controls are not allowed
  violation |= !(controls_allowed || controls_allowed_lateral) && steer_control_enabled;

  // reset to zero (inactive_curvature_is_zero)
  if (violation || !(controls_allowed || controls_allowed_lateral)) {
    curvature_state.desired_last = 0;
  }

  return violation;
}
