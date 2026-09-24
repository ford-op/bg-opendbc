#!/usr/bin/env python3
"""
BluePilot Ford lateral: upstream's whole Ford safety matrix with the BP_LATERAL bit set.

Subclasses the three stock test classes so relay-malfunction, forwarding, button, longitudinal,
rx-hook and every other upstream test runs against the BluePilot tx hook. Only the tests whose
stock assertions the 4-signal lateral changes on purpose are replaced:

  * test_steer_allowed, test_curvature_rate_limits: assert upstream's single-signal limits; the
    BluePilot equivalents are in test_ford_bluepilot_limits.py (every signal, exact boundaries).
  * test_max_lateral_acceleration: BluePilot caps at ISO_LATERAL_ACCEL - g*roll (~2.41 m/s^2)
    with a speed fudge, not upstream's ~3.6 m/s^2; CAN FD only.
  * test_curvature_violation, test_rt_limits: the stock versions send curvature-0 frames, which
    arm the BluePilot reset latch (#9) and bypass the assertion they make; the versions here use
    a non-zero curvature so the check under test is the one deciding.
"""
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.bluepilot.ford.values_ext import FordSafetyFlagsSP
from opendbc.safety.tests import test_ford
from opendbc.safety.tests.ford_bluepilot_common import (
  BPFordLateralModel, drain_reset_latch, CURVATURE_TO_CAN, MAX_CURVATURE_ERROR_CAN, CURVATURE_SIGNAL_MAX_CAN,
)

SKIP_REASON = "BluePilot 4-signal limits are asserted in test_ford_bluepilot_limits.py"


class BPLateralMatrixMixin:
  """Set the bit before ford_init runs, clear it after, and swap the five lateral tests.

  Class names must start with 'TestFord': common.py's cross-mode TX sweep only skips
  Ford-vs-Ford comparisons by that prefix."""

  CANFD = True
  PARAM_SP = int(FordSafetyFlagsSP.BP_LATERAL)
  MAX_CURVATURE_ERROR_CAN_BP = MAX_CURVATURE_ERROR_CAN  # FORD_LIMITS(_, 100); the pinion classes use MAX_CURVATURE_ERROR_CAN_PINION

  def setUp(self):
    self.safety = test_ford.libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.PARAM_SP)
    super().setUp()  # stock setUp: packer, set_safety_hooks, init_tests
    # the angle-mode flag outlives ford_init; a plain Lane_Assist_Data1 frame clears it (byte 4 bit 0)
    self._tx(self.packer.make_can_msg_safety("Lane_Assist_Data1", 0, {}))
    self.model = BPFordLateralModel(self.CANFD, self.MAX_CURVATURE_ERROR_CAN_BP)

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def _drain_latch(self):
    # only the three BP tests below call this: it leaves controls_allowed set and speed samples
    # populated, which upstream's default-state tests must not see
    self.safety.set_controls_allowed(True)
    self._reset_curvature_measurement(0, 15.0)
    drain_reset_latch(self)

  @unittest.skip(SKIP_REASON)
  def test_steer_allowed(self):
    pass

  @unittest.skip(SKIP_REASON)
  def test_curvature_rate_limits(self):
    pass

  def test_max_lateral_acceleration(self):
    """BluePilot's cap (CAN FD only) at ISO_LATERAL_ACCEL - g*roll over the fudged speed squared."""
    self._drain_latch()
    for speed in (12.0, 15.0, 20.0, 25.0, 30.0, 35.0):
      self.safety.set_controls_allowed(True)
      self._reset_curvature_measurement(0, speed)
      v_min = float(self.safety.get_vehicle_speed_min())
      cap = self.model.cap(v_min)
      for sign in (1, -1):
        for offset in (-5, -1, 0, 1, 5):
          curvature_can = sign * (cap + offset)
          if abs(curvature_can) > CURVATURE_SIGNAL_MAX_CAN:
            continue
          self._reset_curvature_measurement(curvature_can / CURVATURE_TO_CAN, speed)
          self._set_prev_desired_angle(curvature_can / CURVATURE_TO_CAN)
          should_tx = abs(curvature_can) <= cap
          with self.subTest(speed=speed, curvature_can=curvature_can):
            self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(True, 0, 0, curvature_can / CURVATURE_TO_CAN, 0)))

  def test_curvature_violation(self):
    """After a violation the last command resets to 0, so an over-rate command keeps failing until
    a frame within one delta of 0 is sent. Uses a non-zero recovery frame so the latch stays out of it."""
    self._drain_latch()
    speed = 25.0
    self._reset_curvature_measurement(0, speed)
    v_min = float(self.safety.get_vehicle_speed_min())
    delta = self.model.rate_delta(v_min)
    self._set_prev_desired_angle(0)
    over = (delta + 5) / CURVATURE_TO_CAN
    for _ in range(20):
      self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, over, 0)))
    # within one delta of the reset value: accepted, and the accepted command becomes the new last
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, delta / CURVATURE_TO_CAN, 0)))
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, 2 * delta / CURVATURE_TO_CAN, 0)))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, (3 * delta + 1) / CURVATURE_TO_CAN, 0)))

  def test_rt_limits(self):
    """Upstream's rolling-window message-rate check, driven with a non-zero curvature so the
    result is not masked by the reset latch."""
    self._drain_latch()
    self._reset_curvature_measurement(0, 0)
    curvature = 1 / CURVATURE_TO_CAN
    max_rt_msgs = int(self.LATERAL_FREQUENCY * common.RT_INTERVAL / 1e6 * 1.2 + 1)
    half = common.RT_INTERVAL // 2

    def send():
      self._set_prev_desired_angle(curvature)
      return self._tx(self._lat_ctl_msg(True, 0, 0, curvature, 0, increment_timer=False))

    self.safety.set_timer(0)
    for i in range(max_rt_msgs * 2):
      self.assertEqual(i <= max_rt_msgs, send())
    self.safety.set_timer(half)
    self.assertFalse(send())
    self.safety.set_timer(half + 2 * common.RT_INTERVAL // 5)
    self.assertFalse(send())
    self.assertFalse(send())
    self.safety.set_timer(half + common.RT_INTERVAL)
    self.assertFalse(send())
    self.safety.set_timer(half + 2 * common.RT_INTERVAL)
    for _ in range(max_rt_msgs):
      self.assertTrue(send())


class TestFordBPCANFDStockSafety(BPLateralMatrixMixin, test_ford.TestFordCANFDStockSafety):
  CANFD = True


class TestFordBPCANFDLongitudinalSafety(BPLateralMatrixMixin, test_ford.TestFordCANFDLongitudinalSafety):
  CANFD = True


class TestFordBPLongitudinalSafety(BPLateralMatrixMixin, test_ford.TestFordLongitudinalSafety):
  CANFD = False


if __name__ == "__main__":
  unittest.main()
