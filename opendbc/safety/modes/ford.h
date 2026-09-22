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

// BluePilot: the desired_curvature/desired_curvature_rate/desired_path_offset/desired_path_angle
// value-range macros (FORD_CURVATURE_MIN, FORD_PATH_ANGLE_MIN, FORD_DBC_PATH_ANGLE_MIN, etc.) now
// live in opendbc/safety/bluepilot/ford_declarations.h, next to ford_lmc_checks, their only user.

// Curvature rate limits
// max_angle_err: 100 (0.002) on the stock yaw-sourced angle_meas path; 150 (0.003) on the
// BluePilot pinion-sourced path (STEER_ANGLE_CURVATURE), because the raw pinion angle has
// no roll/alignment-offset compensation in firmware (the Python layer compensates via
// liveParameters; firmware uses the raw pinion angle).
// BluePilot: now a CurvatureSteeringLimits. Upstream moved Ford off AngleSteeringLimits when it
// made curvature a first-class SteerControlType, and in doing so removed max_angle_error,
// angle_error_min_speed, angle_is_curvature, enforce_angle_error and inactive_angle_is_zero from
// AngleSteeringLimits entirely. The VALUES below are unchanged from the pre-sync macro -- only the
// field names and the struct differ. use_rate_lookup keeps the measured tables in play instead of
// upstream's ISO-jerk-derived delta; see steer_curvature_cmd_checks in lateral.h.
#define FORD_LIMITS(limit_lateral_accel, max_curv_err) {                                         \
  .max_curvature = 1000,          /* 0.02 curvature */                                           \
  .curvature_to_can = 50000,      /* 1 / (2e-5) rad to can */                                    \
  .frequency = 20U,               /* LateralMotionControl / LateralMotionControl2 @ 20 Hz */     \
  .max_curvature_error = (max_curv_err),                                                         \
  /* no blending at low speed due to lack of torque wind-up and inaccurate current curvature */  \
  .curvature_error_min_speed = 10.0,  /* m/s */                                                  \
  .max_steer_power = 0,           /* Ford has no steer power signal */                           \
  .inactive_curvature_is_zero = true,                                                            \
                                                                                                 \
  .use_rate_lookup = true,                                                                       \
  /* Looser symmetric ROCs (former down table); Python control uses stricter up row in values_ext */ \
  .curvature_rate_up_lookup = {                                                                  \
    {5., 16., 25.},                                                                              \
    {0.0025f, 0.0014f, 0.00018f}                                                                 \
  },                                                                                             \
  .curvature_rate_down_lookup = {                                                                \
    {5., 16., 25.},                                                                              \
    {0.0025f, 0.0014f, 0.00018f}                                                                 \
  },                                                                                             \
  .limit_lateral_acceleration = (limit_lateral_accel),                                           \
}

static const CurvatureSteeringLimits FORD_STEERING_LIMITS = FORD_LIMITS(false, 100);
static const CurvatureSteeringLimits FORD_STEERING_LIMITS_PINION = FORD_LIMITS(false, 150);
static const CurvatureSteeringLimits FORD_CANFD_STEERING_LIMITS = FORD_LIMITS(true, 100);
static const CurvatureSteeringLimits FORD_CANFD_STEERING_LIMITS_PINION = FORD_LIMITS(true, 150);


// BluePilot: pinion-geometry table, reset latch, the PathAngle/PathOffset/curvature-rate limit
// tables, the path_angle/path_offset/curvature_rate ROC checks, the shadow-curvature deviation
// check, and ford_lmc_checks (the shared LateralMotionControl/LateralMotionControl2 command
// checks) now live in opendbc/safety/bluepilot/ford.h, alongside the state they operate on
// (ford_bp_pinion_curvature, ford_bp_pinion_params, ford_bp_angle_mode_engaged,
// ford_bp_shadow_curvature_raw, desired_path_angle_last, desired_path_offset_last,
// desired_curvature_rate_last, reset_bypass_latch_counter), mirroring the mads.h /
// mads_declarations.h split used by opendbc/safety/sunnypilot/.
#include "opendbc/safety/bluepilot/ford.h"

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
      UPDATE_VEHICLE_SPEED_2(filtered_pcm_speed);
    }

    // Update vehicle yaw rate (stock angle_meas source; skipped when the pinion source is enabled)
    if ((msg->addr == FORD_Yaw_Data_FD1) && !ford_bp_pinion_curvature) {
      // Signal: VehYaw_W_Actl
      // TODO: we should use the speed which results in the closest angle measurement to the desired angle
      float ford_yaw_rate = (((msg->data[2] << 8U) | msg->data[3]) * 0.0002) - 6.5;
      float current_curvature = ford_yaw_rate / SAFETY_MAX(vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR, 0.1);
      // convert current curvature into units on CAN for comparison with desired curvature
      update_sample(&curvature_state.meas, ROUND(current_curvature * FORD_STEERING_LIMITS.curvature_to_can));
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
      update_sample(&curvature_state.meas, ROUND(current_curvature * FORD_STEERING_LIMITS.curvature_to_can));
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

    // Convert raw signals to signed values (physical = (raw * scale) - offset; see ford_lmc_checks
    // in opendbc/safety/bluepilot/ford.h for the value-range and rate-of-change checks)
    int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;
    int desired_curvature_rate = raw_curvature_rate - FORD_INACTIVE_CURVATURE_RATE;
    int desired_path_offset = raw_path_offset - FORD_INACTIVE_PATH_OFFSET;
    int desired_path_angle = raw_path_angle - FORD_INACTIVE_PATH_ANGLE;

    bool violation = ford_lmc_checks(desired_curvature, desired_curvature_rate, desired_path_offset, desired_path_angle,
                                     steer_control_enabled, &FORD_STEERING_LIMITS, &FORD_STEERING_LIMITS_PINION,
                                     &FORD_CURVATURE_RATE_LIMITS_CAN, "CAN Out");

    if (violation) {
      tx = false;
    }
  }

  // Safety check for LateralMotionControl2 action
  if (msg->addr == FORD_LateralMotionControl2) {
    // Signal: LatCtl_D2_Rq
    bool steer_control_enabled = ((msg->data[0] >> 4) & 0x7U) != 0U;
    unsigned int raw_curvature = (msg->data[2] << 3) | (msg->data[3] >> 5);
    unsigned int raw_curvature_rate = (msg->data[6] << 3) | (msg->data[7] >> 5);
    unsigned int raw_path_angle = ((msg->data[3] & 0x1FU) << 6) | (msg->data[4] >> 2);
    unsigned int raw_path_offset = ((msg->data[4] & 0x3U) << 8) | msg->data[5];
    // unsigned int raw_ramp_type = (msg->data[0] >> 1) & 0x3U;  // Extract bits 1-2 from byte 0

    // Convert raw signals to signed values (physical = (raw * scale) - offset; see ford_lmc_checks
    // in opendbc/safety/bluepilot/ford.h for the value-range and rate-of-change checks)
    int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;
    int desired_curvature_rate = raw_curvature_rate - FORD_CANFD_INACTIVE_CURVATURE_RATE;
    int desired_path_offset = raw_path_offset - FORD_INACTIVE_PATH_OFFSET;
    int desired_path_angle = raw_path_angle - FORD_INACTIVE_PATH_ANGLE;

    bool violation = ford_lmc_checks(desired_curvature, desired_curvature_rate, desired_path_offset, desired_path_angle,
                                     steer_control_enabled, &FORD_CANFD_STEERING_LIMITS, &FORD_CANFD_STEERING_LIMITS_PINION,
                                     &FORD_CURVATURE_RATE_LIMITS_CANFD, "CANFD Out");

    if (violation) {
      tx = false;
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

#ifdef ALLOW_DEBUG
  static const CanMsg FORD_CANFD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };
#endif

  static const CanMsg FORD_CANFD_STOCK_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };

  static const CanMsg FORD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl, 0, 8, .check_relay = true},
  };

  const uint16_t FORD_PARAM_CANFD = 2;
  const bool ford_canfd = GET_FLAG(param, FORD_PARAM_CANFD);

  safety_config ret;
  if (ford_canfd) {
    ret = BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_STOCK_TX_MSGS);
#ifdef ALLOW_DEBUG
    const uint16_t FORD_PARAM_LONGITUDINAL = 1;
    if (GET_FLAG(param, FORD_PARAM_LONGITUDINAL)) {
      ret = BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_LONG_TX_MSGS);
    }
#endif
  } else {
    ret = BUILD_SAFETY_CFG(ford_rx_checks, FORD_LONG_TX_MSGS);
  }

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
