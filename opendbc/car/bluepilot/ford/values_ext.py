"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car.ford.values import CAR
from opendbc.car.lateral import AngleSteeringLimits


class FordSafetyFlagsSP:
  """Sunnypilot-level safety flags for Ford.

  Carried in CP_SP.safetyParam and delivered to the safety firmware as
  current_safety_param_sp (the separate SP uint16, USB control 0xdf) -- NOT the main
  safetyConfigs[].safetyParam. ford_init reads it with GET_FLAG(current_safety_param_sp,
  ...), same pattern as Subaru STOP_AND_GO (subaru_common.h). Plain int constants, not
  IntFlag: CP_SP.safetyParam must stay a plain int through capnp serialization in card.
  """
  # Bit layout of the SP safety param (mirrored in ford_init, modes/ford.h; pinned by
  # tests/test_safety_param_bits.py):
  #   bit 0    STEER_ANGLE_CURVATURE (pinion-sourced curvature measurement)
  #   bits 1-4 pinion geometry index (FORD_PINION_GEOMETRY_SHIFT / FORD_PINION_GEOMETRY_INDEX)
  #   bit 5    BP_LATERAL: the panda runs the 4-signal checks (ford_lmc_checks) instead of
  #            upstream's stock curvature-only checks. Clear = upstream safety, verbatim.
  STEER_ANGLE_CURVATURE = 1
  BP_LATERAL = 1 << 5


# Geometry-table index for the steering-angle curvature measurement, packed into
# CP_SP.safetyParam bits 1-4 when STEER_ANGLE_CURVATURE is set. Must match the
# ford_pinion_geometry table in safety/bluepilot/ford.h row for row.
# Index 0 is reserved as invalid: the firmware treats flag-set-but-no-index as feature
# off, so a half-configured param can never select the wrong geometry silently.
# FORD_EDGE_MK2 is deliberately absent: ALT_STEER_ANGLE platforms read a RELATIVE pinion
# angle (SteeringPinion_Data_Alt + learned offset) and lack the absolute measurement
# this feature needs -- the toggle no-ops there and yaw behavior is kept.
# Index 10 (FORD_MONDEO_MK5) is reserved but unmapped here: that platform exists in
# BluePilot but not in this opendbc baseline. Indices stay aligned with the C table.
FORD_PINION_GEOMETRY_SHIFT = 1
FORD_PINION_GEOMETRY_INDEX = {
  CAR.FORD_BRONCO_SPORT_MK1: 1,
  CAR.FORD_ESCAPE_MK4: 2,
  CAR.FORD_ESCAPE_MK4_5: 3,
  CAR.FORD_EXPEDITION_MK4: 4,
  CAR.FORD_EXPLORER_MK6: 5,
  CAR.FORD_FOCUS_MK4: 6,
  CAR.FORD_F_150_LIGHTNING_MK1: 7,
  CAR.FORD_F_150_MK14: 8,
  CAR.FORD_MAVERICK_MK1: 9,
  CAR.FORD_MUSTANG_MACH_E_MK1: 11,
  CAR.FORD_RANGER_MK2: 12,
}

# Pinion geometry wheelbase (m) where BluePilot deliberately differs from CarSpecs. Used by both the
# C table (ford_pinion_geometry in safety/bluepilot/ford.h) and the Python pinion path
# (lateral_curv_ext.pinion_vehicle_model), so the two layers convert pinion angle the same way.
# FORD_F_150_MK14: the trucks with lane centering come in 3.68 and 3.99 m wheelbases and nothing on
# the car tells them apart; the average keeps either within about 4% (#21).
FORD_PINION_WHEELBASE = {
  CAR.FORD_F_150_MK14: 3.84,
}


# BluePilot: Max curvature for steering command (m^-1), from DBC file limits
CURVATURE_MAX = 0.02

# BluePilot: Curvature rate limits — 3-point breakpoints for smoother lateral control.
# Upstream opendbc uses 2-point ([5, 25]) with more conservative values.
# These allow higher rates at low speed for responsiveness, lower rates at mid-speed
# for comfort, and very low rates at highway speed for stability.
#
# Control (Python) uses stricter windup than unwind so OP stays inside panda when apply_std
# picks the wrong table vs steer_curvature_cmd_checks. Safety firmware uses looser symmetric
# ROCs (former “down” table for both up/down) — see safety/bluepilot/ford.h FORD_LIMITS.
_BP_ANGLE_RATE_UP = ([5, 16, 25], [0.0025, 0.0012, 0.00008])
_BP_ANGLE_RATE_DOWN = ([5, 16, 25], [0.0025, 0.0014, 0.00018])
BP_ANGLE_LIMITS = AngleSteeringLimits(
  0.02,  # Max curvature for steering command, m^-1
  _BP_ANGLE_RATE_UP,
  _BP_ANGLE_RATE_DOWN,
)


# Mirror of openpilot's ModelConstants.T_IDXS (selfdrive/modeld/constants.py): the 33 prediction
# times, t = 10 * (i / 32)^2. Kept here so the lateral code does not import openpilot.
MODEL_T_IDXS = [10.0 * (i / 32) ** 2 for i in range(33)]


# UI-tunable lateral params, published by the fork in CarControlSP.params and read here through
# ParamStore (param_store.py). Keys must match the fork's params_keys.h; values arrive as the
# same "1"/"0" / str(number) text Params stores, and each caller casts them itself.
BP_LATERAL_PARAMS = (
  "disable_BP_lat_UI",
  "enable_human_turn_detection_curv",
  "enable_lane_positioning_curv",
  "enable_lane_full_mode_curv",
  "enable_lane_positioning_ang",
  "FordPrefLateralControl",
  "custom_profile_curv",
  "lane_change_factor_high_curv",
  "pc_blend_ratio_high_C_UI_curv",
  "pc_blend_ratio_low_C_UI_curv",
  "custom_path_offset_curv",
  "LC_PID_gain_UI_curv",
  "FordLowSpeedFactor_ang",
  "FordHighSpeedFactor_ang",
  "FordHighSpeedDampening_ang",
  "lane_change_factor_high_ang",
  "custom_path_offset_ang",
  "lane_centering_strength_ang",
)
