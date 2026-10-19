from opendbc.car import structs
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.bluepilot_lateral.params.manifest import PER_FRAME_KEYS, startup_value


class FordSafetyFlagsSP:
  """Sunnypilot-level safety flags for Ford.

  Carried in CP_SP.safetyParam and delivered to the safety firmware as
  current_safety_param_sp (the separate SP uint16, USB control 0xdf) -- NOT the main
  safetyConfigs[].safetyParam. ford_bp_init reads it with GET_FLAG(current_safety_param_sp,
  ...), same pattern as Subaru STOP_AND_GO (subaru_common.h). Plain int constants, not
  IntFlag: CP_SP.safetyParam must stay a plain int through capnp serialization in card.
  """
  # Bit layout of the SP safety param (mirrored in ford_bp_init, bluepilot_lateral/safety/ford.h; pinned by
  # tests/test_safety_param_bits.py):
  #   bit 0    STEER_ANGLE_CURVATURE (pinion-sourced curvature measurement)
  #   bits 1-4 pinion geometry index (FORD_PINION_GEOMETRY_SHIFT / FORD_PINION_GEOMETRY_INDEX)
  #   bit 5    BP_LATERAL: the panda runs the 4-signal checks (ford_lmc_checks) instead of
  #            upstream's stock curvature-only checks. Clear = upstream safety, verbatim.
  STEER_ANGLE_CURVATURE = 1
  BP_LATERAL = 1 << 5


# Geometry-table index for the steering-angle curvature measurement, packed into
# CP_SP.safetyParam bits 1-4 when STEER_ANGLE_CURVATURE is set. Must match the
# ford_pinion_geometry table in bluepilot_lateral/safety/ford.h row for row.
# Index 0 is reserved as invalid: the firmware treats flag-set-but-no-index as feature
# off, so a half-configured param can never select the wrong geometry silently.
# FORD_EDGE_MK2 is deliberately absent: ALT_STEER_ANGLE platforms read a RELATIVE pinion
# angle (SteeringPinion_Data_Alt + learned offset) and lack the absolute measurement
# this feature needs -- the toggle no-ops there and yaw behavior is kept.
# Index 10 (FORD_MONDEO_MK5) is reserved but unmapped here: that platform exists in
# BluePilot but not in this opendbc baseline. Indices stay aligned with the C table.
# BluePilot: the per-platform tables key on the platform name (CP.carFingerprint), not the CAR enum,
# so a row for a platform some host lacks never breaks import there; it just never matches.
# FORD_HOST_ONLY_PLATFORMS lists the names with rows here that this repo's CAR doesn't have (e.g.
# Mondeo MK5, which BP7 and StarPilot have; note the hosts with each); any name a host's CAR lacks
# must be on it (tests/test_platform_tables.py).
FORD_HOST_ONLY_PLATFORMS: frozenset[str] = frozenset()

FORD_PINION_GEOMETRY_SHIFT = 1
FORD_PINION_GEOMETRY_INDEX = {
  "FORD_BRONCO_SPORT_MK1": 1,
  "FORD_ESCAPE_MK4": 2,
  "FORD_ESCAPE_MK4_5": 3,
  "FORD_EXPEDITION_MK4": 4,
  "FORD_EXPLORER_MK6": 5,
  "FORD_FOCUS_MK4": 6,
  "FORD_F_150_LIGHTNING_MK1": 7,
  "FORD_F_150_MK14": 8,
  "FORD_MAVERICK_MK1": 9,
  "FORD_MUSTANG_MACH_E_MK1": 11,
  "FORD_RANGER_MK2": 12,
}


def init_ford_safety_param_sp(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params_dict: dict[str, str]) -> None:
  # Called from sunnypilot's setup_interfaces at car init.
  # Steering-angle curvature measurement (bad-yaw-sensor workaround): sets the
  # STEER_ANGLE_CURVATURE flag + the platform geometry-table index on CP_SP.safetyParam,
  # which reaches the safety firmware as current_safety_param_sp (USB 0xdf); the control
  # side mirrors the same flag (carcontroller_ext.LateralInputs.get_current_curvature). Platforms without
  # a geometry row silently keep stock yaw behavior -- the toggle no-ops rather than
  # half-configuring.
  if CP.brand == 'ford':
    # BluePilot 4-signal lateral: tell the panda to run the BP checks. Read once here, at init;
    # the carcontroller sends 4-signal messages only when this bit is set. Changing the setting
    # while driving does not change the panda: stock messages are then judged by the BP
    # envelope (which is not identical to the stock one), so the setting takes full effect at
    # the next init. Same property as openpilot longitudinal.
    bp_lateral = not startup_value(params_dict, "disable_BP_lat_UI")
    if bp_lateral:
      CP_SP.safetyParam |= FordSafetyFlagsSP.BP_LATERAL

    # The pinion-sourced measurement is a BP lateral feature: its wider error band and the
    # Python side's yaw-vs-pinion handling only line up on the BP path, so it is off whenever
    # BP lateral is off (ford_bp_init enforces the same on the panda).
    steer_angle_curvature = bp_lateral and startup_value(params_dict, "FordPrefSteerAngleCurvature")
    if steer_angle_curvature:
      geometry_index = FORD_PINION_GEOMETRY_INDEX.get(CP.carFingerprint)
      if geometry_index is not None:
        CP_SP.safetyParam |= FordSafetyFlagsSP.STEER_ANGLE_CURVATURE | (geometry_index << FORD_PINION_GEOMETRY_SHIFT)


# Pinion geometry wheelbase (m) where BluePilot deliberately differs from CarSpecs. Used by both the
# C table (ford_pinion_geometry in bluepilot_lateral/safety/ford.h) and the Python pinion path
# (carcontroller_ext.pinion_vehicle_model), so the two layers convert pinion angle the same way.
# FORD_F_150_MK14: the trucks with lane centering come in 3.68 and 3.99 m wheelbases and nothing on
# the car tells them apart; the average keeps either within about 4% (#21).
FORD_PINION_WHEELBASE = {
  "FORD_F_150_MK14": 3.84,
}


# BluePilot: Max curvature for steering command (m^-1), from DBC file limits
CURVATURE_MAX = 0.02

# BluePilot: Curvature rate limits — 3-point breakpoints for smoother lateral control.
# Upstream opendbc uses 2-point ([5, 25]) with more conservative values.
# These allow higher rates at low speed for responsiveness, lower rates at mid-speed
# for comfort, and very low rates at highway speed for stability.
#
# Control (Python) uses stricter windup than unwind so OP stays inside panda when apply_std
# picks the wrong table vs ford_bp_curvature_cmd_checks. Safety firmware is looser than both: its
# up table is the control's down table, and its down table is 1% looser again — see
# bluepilot_lateral/safety/ford.h FORD_LIMITS.
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


# UI-tunable lateral params the fork publishes in CarControlSP.params every frame (its controlsd
# imports this name). The settings, their defaults and how they are read are in params/manifest.py.
BP_LATERAL_PARAMS = PER_FRAME_KEYS

# Lane change smoothing, both modes: speed breakpoints (m/s) and the low-speed factor
LANE_CHANGE_FACTOR_BP = (4.4, 40.23)
LANE_CHANGE_FACTOR_LOW = 0.95


# Angle mode's per-platform high-speed gain defaults (low curvature, high curvature); any other
# platform gets ANGLE_GAIN_CAN. Read by lateral_angle_ext.update_angle_params.
# CAN vehicles (Escape MK4, Bronco Sport, Explorer, Maverick, Edge)
ANGLE_GAIN_CAN = (1.00, 1.15)
# CAN-FD body-on-frame trucks (F-150, Lightning, Expedition, Ranger)
ANGLE_GAIN_CANFD_BOF = (0.95, 0.95)
# CAN-FD unibody SUVs (Mustang Mach-E, Escape MK4.5)
ANGLE_GAIN_CANFD_SUV = (1.00, 1.05)

ANGLE_GAIN_CANFD_BOF_CARS = frozenset({
  "FORD_F_150_MK14",
  "FORD_F_150_LIGHTNING_MK1",
  "FORD_EXPEDITION_MK4",
  "FORD_RANGER_MK2",
})
ANGLE_GAIN_CANFD_SUV_CARS = frozenset({
  "FORD_MUSTANG_MACH_E_MK1",
  "FORD_ESCAPE_MK4_5",
})
