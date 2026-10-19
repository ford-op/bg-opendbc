"""
BluePilot Ford lateral: the one object the Ford CarController calls.

Holds the two strategies, LateralCurvExt and LateralAngleExt, with the inputs and state they
share, and does the strategy dispatch and message assembly. Lives above both strategy modules
(rather than inside either) so it can call into both without the circular import that would
result from one depending on the other.

The CarController owns apply_curvature_last, which the stock and BluePilot paths share: it is
passed into step_lateral and the result returned, so switching BluePilot lateral off mid-drive
continues from the last command, as before.
"""

import math
from types import SimpleNamespace

from opendbc.car import scale_rot_inertia, scale_tire_stiffness
from opendbc.car.ford.values import CarControllerParams, FordFlags
from opendbc.car.vehicle_model import VehicleModel
from opendbc.bluepilot_lateral.can import fordcan_ext
from opendbc.bluepilot_lateral.core.lateral_angle_ext import LateralAngleExt
from opendbc.bluepilot_lateral.core.lateral_curv_ext import LateralCurvExt, PrimaryLateralControl
from opendbc.bluepilot_lateral.params.manifest import value
from opendbc.bluepilot_lateral.params.param_store import ParamStore
from opendbc.bluepilot_lateral.core.values_ext import FORD_PINION_WHEELBASE, FordSafetyFlagsSP
from opendbc.bluepilot_lateral.core.human_turn import HumanTurnDetector
from opendbc.bluepilot_lateral.hosts.sunnypilot.lateral_inputs import ModelView, VehicleParamsView, lateral_inputs_complete


def pinion_vehicle_model(CP):
  """Vehicle model for the pinion-angle curvature measurement: CarSpecs, except where BluePilot's
  pinion geometry uses another wheelbase (FORD_PINION_WHEELBASE, the same value as the C table),
  keeping the platform's weight split and tire model."""
  wheelbase = FORD_PINION_WHEELBASE.get(CP.carFingerprint)
  if wheelbase is None:
    return VehicleModel(CP)
  center_to_front = wheelbase * CP.centerToFront / CP.wheelbase
  front, rear = scale_tire_stiffness(CP.mass, wheelbase, center_to_front, CP.tireStiffnessFactor)
  return VehicleModel(SimpleNamespace(
    mass=CP.mass, rotationalInertia=scale_rot_inertia(CP.mass, wheelbase), wheelbase=wheelbase,
    centerToFront=center_to_front, steerRatioRear=CP.steerRatioRear, steerRatio=CP.steerRatio,
    tireStiffnessFront=front, tireStiffnessRear=rear))


class LateralInputs:
  """The model / vehicle-parameter inputs both strategies read, and the car's measured curvature.
  Owned by BluePilotLateral and updated once per frame, before either strategy runs."""

  def __init__(self, CP, CP_SP):
    # Model / vehicle-parameter inputs arrive per frame in CC_SP.lateralInputs (see update)
    self.VM = pinion_vehicle_model(CP)  # used only for the pinion-angle curvature measurement
    self.model = None
    self.lp = None
    self.lateral_delay = 0.0

    # BluePilot: steering-angle curvature measurement (bad-yaw-sensor workaround).
    # Mirrors the STEER_ANGLE_CURVATURE flag the safety firmware reads from
    # current_safety_param_sp -- both layers must always agree, so this is init-time
    # state from CP_SP (set by values_ext.init_ford_safety_param_sp at car init), never a live Params read:
    # a live flip against stale firmware would fight the panda.
    self.bp_pinion_curvature_enabled = bool(
      CP_SP is not None and (CP_SP.safetyParam & FordSafetyFlagsSP.STEER_ANGLE_CURVATURE))

  def get_current_curvature(self, CS):
    """Measured curvature of the car right now (OP sign convention).

    The single measurement source for every BluePilot lateral consumer: the deviation
    clip, the stall detector, and the shadow curvature published to ford.h's angle-mode
    deviation check. The default source is the RCM yaw rate -- the same family stock
    ford.h derives its angle_meas from. The shadow value judged against that check must
    always come from the same measurement as the check's own reference, so route all
    reads through here.

    With the steering-angle curvature measurement enabled (FordPrefSteerAngleCurvature
    toggle -> FordSafetyFlagsSP.STEER_ANGLE_CURVATURE), the pinion angle via the vehicle
    model is used instead: some vehicles (e.g. a 2021 Explorer with a faulty RCM)
    broadcast implausible VehYaw_W_Actl (sign-inverted vs IMU and steering geometry)
    while its CAN quality flag still reads OK. The pinion angle (SteeringPinion_Data,
    PSCM) is an equivalent measurement, independently validated against the comma IMU
    (corr +0.99), and the panda safety angle_meas switches to the same source (see
    bluepilot_lateral/safety/ford.h) -- the layers always agree. angleOffsetDeg/roll come from
    liveParameters (paramsd, IMU-derived, not the car yaw sensor).
    """
    if self.bp_pinion_curvature_enabled:
      angle_offset_deg = self.lp.angleOffsetDeg if self.lp is not None else 0.0
      roll = self.lp.roll if self.lp is not None else 0.0
      return -self.VM.calc_curvature(math.radians(CS.out.steeringAngleDeg - angle_offset_deg),
                                     CS.out.vEgoRaw, roll)
    return -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)

  def update(self, CC_SP):
    """Take this frame's model / vehicle-parameter inputs from CC_SP and update the vehicle model.

    A frame is used only if it has the full modelV2 shape this code indexes into (33 path points,
    both inner lane lines, 3+ probs/stds); anything else is ignored and the previous inputs are
    kept, exactly as a stale SubMaster message was. That also makes fuzzed or partial CC_SP
    safe: the producer's valid flag is necessary but not trusted on its own."""
    li = CC_SP.lateralInputs
    if li.valid and lateral_inputs_complete(li):
      self.model = ModelView(li)
      self.lp = VehicleParamsView(li)
      self.lateral_delay = li.lateralDelay

    if self.lp is not None:
      x = max(self.lp.stiffnessFactor, 0.1)
      sr = max(self.lp.steerRatio, 0.1)
      self.VM.update_params(x, sr)


class SharedLateralState:
  """The values both strategies read and write, so either mode sees what the other left, as when
  the strategies were mixins on one object. bp_curvature_deviation_limited is the BluePilot UI's
  deviation flag, from whichever mode ran."""

  def __init__(self):
    self.path_angle_last = 0.0
    self.precision_type = 1  # 1=Precise, 0=Comfortable
    self.bp_curvature_deviation_limited = False
    self.human_turn_detector = HumanTurnDetector()


class BluePilotLateral:
  def __init__(self, CP, CP_SP, packer, CAN):
    self.CP = CP
    self.frame = 0
    self.inputs = LateralInputs(CP, CP_SP)
    self.shared = SharedLateralState()
    self.curv = LateralCurvExt(self.inputs, self.shared)
    self.angle = LateralAngleExt(CP, self.inputs, self.shared)
    self.packer = packer
    self.CAN = CAN

    self.disable_BP_lat_UI = False
    # the panda only accepts 4-signal messages when this bit was set at init
    self.bp_lateral_allowed = bool(CP_SP.safetyParam & FordSafetyFlagsSP.BP_LATERAL)

  def update(self, CC_SP, frame):
    """Every frame: take this frame's model / vehicle-parameter inputs and UI params from CC_SP
    (the fork fills both; opendbc reads nothing from openpilot directly). Returns whether
    BluePilot lateral is active."""
    self.frame = frame
    self.inputs.update(CC_SP)
    params = ParamStore.from_cc_sp(CC_SP)
    self.curv.update_lateral_params(params)
    self.angle.update_angle_params(params)
    # One flag for every BP consumer (LatCtl dispatch, LKA angle-mode bits, mode reporting): the UI
    # toggle, and the panda bit that was set at init. No 4-signal traffic to a stock-configured panda.
    self.disable_BP_lat_UI = value(params, "disable_BP_lat_UI") or not self.bp_lateral_allowed
    return not self.disable_BP_lat_UI

  def step_lateral(self, CC, CS, actuators, apply_curvature_last):
    """At STEER_STEP: the steer message(s). Returns (can_sends, apply_curvature): the CarController
    keeps it and passes it back next time as apply_curvature_last."""
    # BluePilot: select the BP lateral strategy by primary control variable.
    #   angle     -> LateralAngleExt: kappa -> path_angle (c1), apply_curvature held at 0.
    #   curvature -> LateralCurvExt: full 4-signal curvature-primary (default).
    # Both return a LateralResult packed identically below.
    # Do not run the stock curvature limits here or overwrite apply_curvature_last before the
    # strategy runs. Panda rate-checks desired_curvature vs the last TX on the bus; that must match
    # the prior frame's lat.apply_curvature only (not an intermediate stock-limited value).
    angle_mode = self.curv.primary_lateral_control == PrimaryLateralControl.angle
    if angle_mode:
      lat = self.angle.update_angle_strategy(CC, CS, actuators)
    else:
      lat = self.curv.update(CC, CS, actuators, apply_curvature_last, self.CP)
    apply_curvature = float(lat.apply_curvature)

    # BluePilot: angle-mode human-turn override -- send lateral inactive (mode 0) while the
    # driver manually turns, so the PSCM releases cleanly instead of stalling 2-3 s on
    # re-engage (observed on Mach-E). Panda-clean: every ford.h check has a legitimate
    # !steer_control_enabled branch for the zeroed frames; on release, path_angle ramps back
    # from 0 through the soft ROC. Curvature mode keeps its own reset_steering path (zeroed
    # signals with mode still active) in LateralCurvExt.
    # The stall blip (lateral_angle_ext.py) rides the same mode-0 path: a short pulse that
    # resets the PSCM's post-override attenuation when the deviation clip deadlocks hands-free.
    lat_active = CC.latActive and not (angle_mode and (self.angle.angle_human_turn_active or self.angle.angle_stall_blip_active))

    can_sends = []
    if self.CP.flags & FordFlags.CANFD:
      mode = 1 if lat_active else 0
      counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
      can_sends.append(fordcan_ext.create_lat_ctl2_msg(
        self.packer, self.CAN, mode, lat.ramp_type, lat.precision_type,
        -lat.path_offset, -lat.path_angle, -lat.apply_curvature, -lat.curvature_rate, counter
      ))
    else:
      can_sends.append(fordcan_ext.create_lat_ctl_msg(
        self.packer, self.CAN, lat_active, lat.ramp_type, lat.precision_type,
        -lat.path_offset, -lat.path_angle, -lat.apply_curvature, -lat.curvature_rate
      ))
    return can_sends, apply_curvature

  def step_lka(self, CC, hud_control):
    """At LKA_STEP: the Lane_Assist_Data1 message."""
    # BluePilot: tell bluepilot_lateral/safety/ford.h whether angle mode is engaged, packed into
    # Lane_Assist_Data1's unused bits. shadow_curvature is negated to match the
    # path_angle/apply_curvature wire convention that ford.h's angle_meas is calibrated against.
    # Only called while BluePilot lateral is active (update returned True this frame), so the mode
    # alone decides.
    angle_mode_engaged = self.curv.primary_lateral_control == PrimaryLateralControl.angle
    shadow_curvature = -self.angle.bp_kappa_cmd if angle_mode_engaged else 0.0
    return fordcan_ext.create_lka_msg(
      self.packer, self.CAN, CC.latActive, hud_control, angle_mode_engaged, shadow_curvature
    )
