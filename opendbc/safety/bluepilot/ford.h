/**
 * BluePilot Ford lateral-control extensions: state and implementations.
 * See ford_declarations.h in this directory for the declarations.
 */

#pragma once

#include "opendbc/safety/bluepilot/ford_declarations.h"

// ===============================
// Global Variables
// ===============================

bool ford_bp_pinion_curvature = false;
const AngleSteeringParams *ford_bp_pinion_params = &ford_pinion_geometry[0];

// BluePilot: per-platform geometry for pinion-angle -> curvature conversion (the optional
// angle_meas source), selected by the 4-bit geometry index in current_safety_param_sp
// bits 1-4. Row order and values must match FORD_PINION_GEOMETRY_INDEX in
// opendbc/sunnypilot/car/ford/values_ext.py (enforced by test_ford.py's
// geometry-consistency test against CarSpecs + calc_slip_factor(VehicleModel(CP))).
// Index 0 is reserved as invalid; ford_init disables the feature outright on a zero or
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
  {.slip_factor = -0.00042037149f, .steer_ratio = 17.0f, .wheelbase = 3.990f},    // 8: FORD_F_150_MK14
  {.slip_factor = -0.00054528036f, .steer_ratio = 17.0f, .wheelbase = 3.076f},    // 9: FORD_MAVERICK_MK1
  {.slip_factor = -0.00058852001f, .steer_ratio = 14.8f, .wheelbase = 2.850f},    // 10: FORD_MONDEO_MK5
  {.slip_factor = -0.00056209187f, .steer_ratio = 17.0f, .wheelbase = 2.984f},    // 11: FORD_MUSTANG_MACH_E_MK1
  {.slip_factor = -0.00051293030f, .steer_ratio = 17.0f, .wheelbase = 3.270f},    // 12: FORD_RANGER_MK2
};

bool ford_bp_angle_mode_engaged = false;
int16_t ford_bp_shadow_curvature_raw = 0;  // wire units, scale 1e-6 1/m (see fordcan_ext.py)

int desired_path_angle_last = 0;
int desired_path_offset_last = 0;
int desired_curvature_rate_last = 0;

uint8_t reset_bypass_latch_counter = 0;

bool ford_bp_debug = false;

// Curvature-rate value-check scale, CAN vs CAN FD
// cppcheck-suppress misra-c2012-8.9; read only by ford_tx_hook in modes/ford.h, kept beside its CAN/CAN FD sibling
static const AngleSteeringLimits FORD_CURVATURE_RATE_LIMITS_CAN = {
  .max_angle = 100,               // 1.0 meter in CAN units (100 * 0.01)
  .angle_deg_to_can = 4000000,    // 1 / (1E-6) meter to can
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

// cppcheck-suppress misra-c2012-8.9; read only by ford_tx_hook in modes/ford.h, kept beside its CAN/CAN FD sibling
static const AngleSteeringLimits FORD_CURVATURE_RATE_LIMITS_CANFD = {
  .max_angle = 100,               // 1.0 meter in CAN units (100 * 0.01)
  .angle_deg_to_can = 1000000,    // 1 / (1E-6) meter to can
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
    if (ford_bp_debug) {
      FORD_SAFETY_DBG("path_angle_cmd_checks 1: desired_path_angle: %d desired_path_angle_last: %d highest_desired_path_angle: %d lowest_desired_path_angle: %d violation: %d \n",
                      desired_path_angle, desired_path_angle_last, highest_desired_path_angle, lowest_desired_path_angle, (int)violation);
    }
  }
  desired_path_angle_last = desired_path_angle;

  if (!steer_control_enabled) {
    violation |= (desired_path_angle != 0);
  }
  if (ford_bp_debug) {
    FORD_SAFETY_DBG("path_angle_cmd_checks 2: violation: %d \n", (int)violation);
  }

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
    if (ford_bp_debug) {
      FORD_SAFETY_DBG("path_offset_cmd_checks 1: desired_path_offset: %d desired_path_offset_last: %d highest_desired_path_offset: %d lowest_desired_path_offset: %d violation: %d \n",
                      desired_path_offset, desired_path_offset_last, highest_desired_path_offset, lowest_desired_path_offset, (int)violation);
    }

  }
  desired_path_offset_last = desired_path_offset;

  if (!steer_control_enabled) {
    violation |= (desired_path_offset != 0);
  }
  if (ford_bp_debug) {
    FORD_SAFETY_DBG("path_offset_cmd_checks 2: violation: %d \n", (int)violation);
  }

  return violation;
}

static inline bool curvature_rate_cmd_checks(int desired_curvature_rate, bool steer_control_enabled, const AngleSteeringLimits limits) {
  bool violation = false;

  if (steer_control_enabled) {
    float speed = ((float)vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;

    int desired_curvature_rate_roc = (safety_interpolate(limits.angle_rate_up_lookup, speed) * limits.angle_deg_to_can) + 1.;

    int highest_desired_curvature_rate = desired_curvature_rate_last + desired_curvature_rate_roc;
    int lowest_desired_curvature_rate = desired_curvature_rate_last - desired_curvature_rate_roc;

    violation |= safety_max_limit_check(desired_curvature_rate, highest_desired_curvature_rate, lowest_desired_curvature_rate);
    if (ford_bp_debug) {
      FORD_SAFETY_DBG("curvature_rate_cmd_checks 1: desired_curvature_rate: %d desired_curvature_rate_last: %d highest_desired_curvature_rate: %d lowest_desired_curvature_rate: %d violation: %d \n",
                      desired_curvature_rate, desired_curvature_rate_last, highest_desired_curvature_rate, lowest_desired_curvature_rate, (int)violation);
    }
  }
  desired_curvature_rate_last = desired_curvature_rate;

  if (!steer_control_enabled) {
    violation |= (desired_curvature_rate != 0);
  }
  if (ford_bp_debug) {
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
// BluePilot: enforce_angle_error is gone from the struct; the check is unconditional now because
// the only caller passes FORD_STEERING_LIMITS(_PINION), both of which set it true pre-sync.
static inline bool ford_shadow_curvature_error_check(int desired_curvature, bool steer_control_enabled,
                                               const CurvatureSteeringLimits limits) {
  bool violation = false;
  if (steer_control_enabled &&
      ((vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR) > limits.curvature_error_min_speed)) {
    int lowest_allowed = curvature_state.meas.min - limits.max_curvature_error - 1;
    int highest_allowed = curvature_state.meas.max + limits.max_curvature_error + 1;
    violation = safety_max_limit_check(desired_curvature, highest_allowed, lowest_allowed);
  }
  return violation;
}

// BluePilot: shared reset latch for LateralMotionControl and LateralMotionControl2. Activates
// when both curvature and path_angle are zero (reset/neutral state), returning true so the
// caller can bypass its violation for this frame and for a short ramp period afterward -- this
// allows smooth ramp-up after human turn detection without blocked messages.
static inline bool ford_reset_bypass_latch_check(int desired_curvature, int desired_path_angle) {
  bool bypass = false;
  if ((desired_curvature == 0) && (desired_path_angle == 0)) {
    // Reset detected, activate latch for ramp period
    reset_bypass_latch_counter = FORD_RESET_BYPASS_LATCH_DURATION;
    bypass = true;
  } else if (reset_bypass_latch_counter > 0U) {
    // Latch active, allow bypass during ramp-up period
    reset_bypass_latch_counter--;
    bypass = true;
  } else {
  }
  return bypass;
}

// BluePilot: shared curvature/curvature_rate/path_offset/path_angle command checks for
// LateralMotionControl (CAN) and LateralMotionControl2 (CAN FD) -- the two messages carry
// identical signals at different bit offsets and CAN-unit scales, so the caller decodes the raw
// signals and picks the right limit tables (curvature_rate_limits, and curvature_limits /
// curvature_limits_pinion for the steer_curvature_cmd_checks + shadow-curvature call), and this
// function does the rest. dbg_prefix labels the FORD_SAFETY_DBG output ("CAN Out" / "CANFD Out").
static inline bool ford_lmc_checks(int desired_curvature, int desired_curvature_rate, int desired_path_offset, int desired_path_angle,
                            bool steer_control_enabled, const CurvatureSteeringLimits *curvature_limits,
                            const CurvatureSteeringLimits *curvature_limits_pinion, const AngleSteeringLimits *curvature_rate_limits,
                            const char *dbg_prefix) {
  // dbg_prefix is only read inside FORD_SAFETY_DBG, which expands to a no-op outside libsafety's
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

  // BluePilot: the pinion-sourced angle_meas variant carries a wider curvature error band -- see
  // the FORD_LIMITS macro comment in modes/ford.h. Everything else in the two limit sets is identical.
  const CurvatureSteeringLimits *limits = ford_bp_pinion_curvature ? curvature_limits_pinion : curvature_limits;

  bool violation = false;

  // Check curvature value limits (already converted to signed CAN units by the caller). Note:
  // curvature's wire scale (curvature_to_can) is the same for CAN and CAN FD, so this is safe to
  // read off the (possibly pinion-widened) limits struct regardless of bus.
  int curvature_min_can = (int)(FORD_CURVATURE_MIN * limits->curvature_to_can);
  int curvature_max_can = (int)(FORD_CURVATURE_MAX * limits->curvature_to_can);
  violation |= (desired_curvature < curvature_min_can) || (desired_curvature > curvature_max_can);
  if (ford_bp_debug) {
    FORD_SAFETY_DBG("%s: desired_curvature: %d, curvature_min_can: %d, curvature_max_can: %d, violation: %d\n",
                    dbg_prefix, desired_curvature, curvature_min_can, curvature_max_can, (int)violation);
  }

  // Check curvature rate value limits (CAN and CAN FD use different wire scales)
  int curvature_rate_min_can = (int)(FORD_CURVATURE_RATE_MIN * curvature_rate_limits->angle_deg_to_can);
  int curvature_rate_max_can = (int)(FORD_CURVATURE_RATE_MAX * curvature_rate_limits->angle_deg_to_can);
  violation |= (desired_curvature_rate < curvature_rate_min_can) || (desired_curvature_rate > curvature_rate_max_can);
  if (ford_bp_debug) {
    FORD_SAFETY_DBG("%s: desired_curvature_rate: %d, curvature_rate_min_can: %d, curvature_rate_max_can: %d, violation: %d\n",
                    dbg_prefix, desired_curvature_rate, curvature_rate_min_can, curvature_rate_max_can, (int)violation);
  }

  // Check path offset value limits
  int path_offset_min_can = (int)(FORD_PATH_OFFSET_MIN * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
  int path_offset_max_can = (int)(FORD_PATH_OFFSET_MAX * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
  violation |= (desired_path_offset < path_offset_min_can) || (desired_path_offset > path_offset_max_can);
  if (ford_bp_debug) {
    FORD_SAFETY_DBG("%s: desired_path_offset: %d, path_offset_min_can: %d, path_offset_max_can: %d, violation: %d\n",
                    dbg_prefix, desired_path_offset, path_offset_min_can, path_offset_max_can, (int)violation);
  }

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
  if (ford_bp_debug) {
    FORD_SAFETY_DBG("%s: desired_path_angle: %d, path_angle_min_can: %d, path_angle_max_can: %d, violation: %d\n",
                    dbg_prefix, desired_path_angle, path_angle_min_can, path_angle_max_can, (int)violation);
  }

  // Check angle error and steer_control_enabled for curvature. Angle mode holds curvature pinned
  // at 0 while path_angle does the real steering, so the deviation-vs-measured portion of
  // steer_curvature_cmd_checks would eventually trip as the car actually turns (measured curvature
  // moves, commanded curvature doesn't) -- skip applying it when desired_curvature == 0. Still call
  // it to keep desired_angle_last in sync, and path_angle keeps its own checks regardless. But
  // steer_curvature_cmd_checks also carries the controls_allowed gate; restore that piece
  // explicitly so a steer_control_enabled frame at curvature == 0 can't bypass it.
  bool curvature_violation = steer_curvature_cmd_checks(desired_curvature, 0, steer_control_enabled, *limits);
  if (desired_curvature != 0) {
    violation |= curvature_violation;
  } else {
    violation |= steer_control_enabled && !(controls_allowed || controls_allowed_lateral);
  }
  if (ford_bp_debug) {
    FORD_SAFETY_DBG("%s: 1. desired_curvature violation: %d\n", dbg_prefix, (int)violation);
  }

  // Angle mode's own deviation-only check against shadow_curvature, once angle mode is confirmed
  // engaged. If desired_curvature == 0 but angle mode is NOT confirmed, this is skipped -- that's
  // ordinary curvature mode at zero (straight driving or the reset/human-turn frame), which needs
  // no shadow-curvature check; it's still bounded by the tight path_angle range above and
  // steer_control_enabled's own checks.
  if ((desired_curvature == 0) && ford_bp_angle_mode_engaged) {
    // shadow_curvature raw scale 1e-6 -> CAN units (2e-5): * 0.05
    const float shadow_curvature_scaled = (float)ford_bp_shadow_curvature_raw * 0.05f;
    int shadow_curvature_can = (int)shadow_curvature_scaled;
    violation |= ford_shadow_curvature_error_check(shadow_curvature_can, steer_control_enabled, *limits);
  }

  // Check path angle rate of change limits
  violation |= path_angle_cmd_checks(desired_path_angle, steer_control_enabled, FORD_PATH_ANGLE_LIMITS);
  if (ford_bp_debug) {
    FORD_SAFETY_DBG("%s: 2. desired_path_angle violation: %d\n", dbg_prefix, (int)violation);
  }

  // Check path offset rate of change limits
  violation |= path_offset_cmd_checks(desired_path_offset, steer_control_enabled, FORD_PATH_OFFSET_LIMITS);
  if (ford_bp_debug) {
    FORD_SAFETY_DBG("%s: 3. desired_path_offset violation: %d\n", dbg_prefix, (int)violation);
  }

  // Check curvature rate rate of change limits
  violation |= curvature_rate_cmd_checks(desired_curvature_rate, steer_control_enabled, *curvature_rate_limits);
  if (ford_bp_debug) {
    FORD_SAFETY_DBG("%s: 4. desired_curvature_rate violation: %d\n", dbg_prefix, (int)violation);
  }

  // Reset latch: see ford_reset_bypass_latch_check above
  if (ford_reset_bypass_latch_check(desired_curvature, desired_path_angle)) {
    violation = false;
  }

  return violation;
}
