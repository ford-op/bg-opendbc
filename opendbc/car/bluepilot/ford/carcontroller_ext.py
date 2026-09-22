"""
BluePilot Ford CarController extension: strategy dispatch and message assembly.

Mixed into CarController alongside LateralCurvExt and LateralAngleExt. Lives above both
(rather than inside either) so it can call into both strategies without the circular import
that would result from one strategy module depending on the other.
"""

from opendbc.car.ford.values import CarControllerParams, FordFlags
from opendbc.car.bluepilot.ford import fordcan_ext
from opendbc.car.bluepilot.ford.lateral_angle_ext import LateralAngleExt
from opendbc.car.bluepilot.ford.lateral_curv_ext import LateralCurvExt, PrimaryLateralControl


class CarControllerExt:
  def build_steer_can_sends(self, CC, CS, actuators):
    # BluePilot: select the BP lateral strategy by primary control variable.
    #   angle     -> LateralAngleExt: kappa -> path_angle (c1), apply_curvature held at 0.
    #   curvature -> LateralCurvExt: full 4-signal curvature-primary (default).
    # Both return a LateralResult packed identically below.
    # Do not run the stock curvature limits here or overwrite apply_curvature_last before the
    # strategy runs. Panda rate-checks desired_curvature vs the last TX on the bus; that must match
    # the prior frame's lat.apply_curvature only (not an intermediate stock-limited value).
    angle_mode = self.primary_lateral_control == PrimaryLateralControl.angle
    if angle_mode:
      lat = LateralAngleExt.update_angle_strategy(self, CC, CS, actuators, self.CP)
    else:
      lat = LateralCurvExt.update(self, CC, CS, actuators, self.apply_curvature_last, self.CP)
    self.apply_curvature_last = float(lat.apply_curvature)

    # BluePilot: angle-mode human-turn override -- send lateral inactive (mode 0) while the
    # driver manually turns, so the PSCM releases cleanly instead of stalling 2-3 s on
    # re-engage (observed on Mach-E). Panda-clean: every ford.h check has a legitimate
    # !steer_control_enabled branch for the zeroed frames; on release, path_angle ramps back
    # from 0 through the soft ROC (no reset-bypass latch involvement). Curvature mode keeps
    # its own reset_steering path (zeroed signals with mode still active) in LateralCurvExt.
    # The stall blip (lateral_angle_ext.py) rides the same mode-0 path: a short pulse that
    # resets the PSCM's post-override attenuation when the deviation clip deadlocks hands-free.
    lat_active = CC.latActive and not (angle_mode and (self.angle_human_turn_active or self.angle_stall_blip_active))

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
    return can_sends

  def build_lka_can_send(self, CC, hud_control):
    # BluePilot: tell safety/bluepilot/ford.h whether angle mode is engaged, packed into
    # Lane_Assist_Data1's unused bits. shadow_curvature is negated to match the
    # path_angle/apply_curvature wire convention that ford.h's angle_meas is calibrated against.
    angle_mode_engaged = (not self.disable_BP_lat_UI) and (self.primary_lateral_control == PrimaryLateralControl.angle)
    shadow_curvature = -self.bp_kappa_cmd if angle_mode_engaged else 0.0
    return fordcan_ext.create_lka_msg(
      self.packer, self.CAN, CC.latActive, hud_control, angle_mode_engaged, shadow_curvature
    )
