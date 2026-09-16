import math
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.ford import fordcan
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX
from openpilot.common.params import Params

# BluePilot: 4-signal lateral control extensions (curvature-primary and angle-primary)
from opendbc.sunnypilot.car.ford import fordcan_ext
from opendbc.sunnypilot.car.ford.lateral_angle_ext import LateralAngleExt
from opendbc.sunnypilot.car.ford.lateral_curv_ext import LateralCurvExt, PrimaryLateralControl

LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert

def anti_overshoot(apply_curvature, apply_curvature_last, v_ego):
  diff = 0.1
  tau = 5  # 5s smooths over the overshoot
  dt = DT_CTRL * CarControllerParams.STEER_STEP
  alpha = 1 - np.exp(-dt / tau)

  lataccel = apply_curvature * (v_ego ** 2)
  last_lataccel = apply_curvature_last * (v_ego ** 2)
  last_lataccel = apply_hysteresis(lataccel, last_lataccel, diff)
  last_lataccel = alpha * lataccel + (1 - alpha) * last_lataccel

  output_curvature = last_lataccel / (max(v_ego, 1) ** 2)

  return float(np.interp(v_ego, [5, 10], [apply_curvature, output_curvature]))


def apply_creep_compensation(accel: float, v_ego: float) -> float:
  creep_accel = np.interp(v_ego, [1., 3.], [0.6, 0.])
  creep_accel = np.interp(accel, [0., 0.2], [creep_accel, 0.])
  accel -= creep_accel
  return float(accel)


# BluePilot: CarController inherits from LateralCurvExt and LateralAngleExt for 4-signal
# lateral control (curvature- or angle-primary). Init order: CarControllerBase first
# (sets self.CP, self.frame), then the extension classes.
class CarController(CarControllerBase, LateralCurvExt, LateralAngleExt):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    # BluePilot: initialize lateral extension classes
    LateralCurvExt.__init__(self, CP, CP_SP)
    LateralAngleExt.__init__(self, CP, CP_SP)

    self.params = Params()
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.CAN = fordcan.CanBus(CP)

    self.disable_BP_lat_UI = False
    self.apply_curvature_last = 0
    self.anti_overshoot_curvature_last = 0
    self.accel = 0.0
    self.gas = 0.0
    self.brake_request = False
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    # BluePilot: update SubMaster (modelV2, vehicleParameters, selfdriveState, lateralDelay)
    # and the vehicle model, then read the lateral params from the UI
    LateralCurvExt.update_sm(self)
    LateralCurvExt.update_lateral_params(self, self.params)
    LateralAngleExt.update_angle_params(self, self.params)
    self.disable_BP_lat_UI = self.params.get_bool("disable_BP_lat_UI")

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    ### acc buttons ###
    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
    # if stock lane centering isn't off, send a button press to toggle it off
    # the stock system checks for steering pressed, and eventually disengages cruise control
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, tja_toggle=True))

    ### lateral control ###
    # send steer msg at 20Hz
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      if self.disable_BP_lat_UI:
        # Stock curvature-only path, unchanged from upstream. Used as the "BluePilot lateral off"
        # fallback; anti-overshoot is not used when BP lateral is active.
        # Bronco and some other cars consistently overshoot curv requests
        # Apply some deadzone + smoothing convergence to avoid oscillations
        if self.CP.carFingerprint in (CAR.FORD_BRONCO_SPORT_MK1, CAR.FORD_F_150_MK14):
          self.anti_overshoot_curvature_last = anti_overshoot(actuators.curvature, self.anti_overshoot_curvature_last, CS.out.vEgoRaw)
          apply_curvature = self.anti_overshoot_curvature_last
        else:
          apply_curvature = actuators.curvature

        # apply rate limits, curvature error limit, and clip to signal range
        current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
        # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
        if CS.out.vEgoRaw > 9:
          apply_curvature = float(np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                                          current_curvature + CarControllerParams.CURVATURE_ERROR))
        apply_curvature = CarControllerParams.CURVATURE_LIMITS.apply_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw,
                                                                            0., CC.latActive, CarControllerParams.STEER_STEP)
        self.apply_curvature_last = apply_curvature

        if self.CP.flags & FordFlags.CANFD:
          # TODO: extended mode
          # Ford uses four individual signals to dictate how to drive to the car. Curvature alone (limited to 0.02 m^-1)
          # can actuate the steering for a large portion of any lateral movements. However, in order to get further control on
          # steer actuation, the other three signals are necessary. Ford controls vehicles differently than most other makes.
          # A detailed explanation on ford control can be found here:
          # https://www.f150gen14.com/forum/threads/introducing-bluepilot-a-ford-specific-fork-for-comma3x-openpilot.24241/#post-457706
          mode = 1 if CC.latActive else 0
          counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
          can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 0., 0., -self.apply_curvature_last, 0., counter))
        else:
          can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive, 0., 0., -self.apply_curvature_last, 0.))
      else:
        # BluePilot: select the BP lateral strategy by primary control variable.
        #   1 (angle)     -> LateralAngleExt: κ → path_angle (c1), apply_curvature held at 0.
        #   0 (curvature) -> LateralCurvExt: full 4-signal curvature-primary (default).
        # Both return a LateralResult packed identically below.
        # Do not run the stock curvature limits here or overwrite apply_curvature_last before the
        # strategy runs. Panda rate-checks desired_curvature vs the last TX on the bus; that must match
        # the prior frame's lat.apply_curvature only (not an intermediate stock-limited value).
        if self.primary_lateral_control == PrimaryLateralControl.angle:
          lat = LateralAngleExt.update_angle_strategy(self, CC, CS, actuators, self.CP)
        else:
          lat = LateralCurvExt.update(self, CC, CS, actuators, self.apply_curvature_last, self.CP)
        self.apply_curvature_last = lat.apply_curvature
        _angle_mode = self.primary_lateral_control == PrimaryLateralControl.angle

        # BluePilot: angle-mode human-turn override -- send lateral inactive (mode 0) while the
        # driver manually turns, so the PSCM releases cleanly instead of stalling 2-3 s on
        # re-engage (observed on Mach-E). Panda-clean: every ford.h check has a legitimate
        # !steer_control_enabled branch for the zeroed frames; on release, path_angle ramps back
        # from 0 through the soft ROC (no reset-bypass latch involvement). Curvature mode keeps
        # its own reset_steering path (zeroed signals with mode still active) in LateralCurvExt.
        # The stall blip (lateral_angle_ext.py) rides the same mode-0 path: a short pulse that
        # resets the PSCM's post-override attenuation when the deviation clip deadlocks hands-free.
        lat_active = CC.latActive and not (_angle_mode and (self.angle_human_turn_active or self.angle_stall_blip_active))
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

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      # BluePilot: tell safety/bluepilot/ford.h whether angle mode is engaged, out-of-band from
      # LMC/LMC2, packed into Lane_Assist_Data1's unused bits (read synchronously in ford_tx_hook,
      # no separate CAN ID/RX needed). disable_BP_lat_UI means BP lateral is off entirely, so angle
      # mode can't be engaged then. shadow_curvature is only meaningful in angle mode
      # (self.bp_kappa_cmd is stale/unused CurvExt state otherwise, so force it to 0 there).
      # Negated to match the sign convention path_angle/apply_curvature use on the wire -- ford.h's
      # angle_meas (measured curvature, from raw yaw rate with no negation) is calibrated against
      # that wire convention, not bp_kappa_cmd's internal one.
      angle_mode_engaged = (not self.disable_BP_lat_UI) and (self.primary_lateral_control == PrimaryLateralControl.angle)
      shadow_curvature = -self.bp_kappa_cmd if angle_mode_engaged else 0.0
      can_sends.append(fordcan_ext.create_lka_msg(
        self.packer, self.CAN, CC.latActive, hud_control, angle_mode_engaged, shadow_curvature
      ))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      accel = actuators.accel
      gas = accel

      if CC.longActive:
        # Compensate for engine creep at low speed.
        # Either the ABS does not account for engine creep, or the correction is very slow
        # TODO: verify this applies to EV/hybrid
        accel = apply_creep_compensation(accel, CS.out.vEgo)

        # The stock system has been seen rate limiting the brake accel to 5 m/s^3,
        # however even 3.5 m/s^3 causes some overshoot with a step response.
        accel = max(accel, self.accel - (3.5 * CarControllerParams.ACC_CONTROL_STEP * DT_CTRL))

      accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      gas = float(np.clip(gas, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

      # Both gas and accel are in m/s^2, accel is used solely for braking
      if not CC.longActive or gas < CarControllerParams.MIN_GAS:
        gas = CarControllerParams.INACTIVE_GAS

      # PCM applies pitch compensation to gas/accel, but we need to compensate for the brake/pre-charge bits
      accel_due_to_pitch = 0.0
      if len(CC.orientationNED) == 3:
        accel_due_to_pitch = math.sin(CC.orientationNED[1]) * ACCELERATION_DUE_TO_GRAVITY

      accel_pitch_compensated = accel + accel_due_to_pitch
      if accel_pitch_compensated > 0.3 or not CC.longActive:
        self.brake_request = False
      elif accel_pitch_compensated < 0.0:
        self.brake_request = True

      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      # TODO: look into using the actuators packet to send the desired speed
      can_sends.append(fordcan.create_acc_msg(self.packer, self.CAN, CC.longActive, gas, accel, stopping, self.brake_request, v_ego_kph=V_CRUISE_MAX))

      self.accel = accel
      self.gas = gas

    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)
    # send lkas ui msg at 1Hz or if ui state changes
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_msg(self.packer, self.CAN, main_on, CC.latActive, steer_alert, hud_control, CS.lkas_status_stock_values))

    # send acc ui msg at 5Hz or if ui state changes
    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      send_ui = True
      self.distance_bar_frame = self.frame

    if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      show_distance_bars = self.frame - self.distance_bar_frame < 400
      can_sends.append(fordcan.create_acc_ui_msg(self.packer, self.CAN, self.CP, main_on, CC.latActive,
                                                 fcw_alert, CS.out.cruiseState.standstill, show_distance_bars,
                                                 hud_control, CS.acc_tja_status_stock_values))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    new_actuators = actuators.as_builder()
    # float(): capnp rejects np.float64, and the BP lateral strategies compute with numpy
    new_actuators.curvature = float(self.apply_curvature_last)
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends
