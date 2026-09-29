#!/usr/bin/env python3
"""
BluePilot Ford lateral: the 4-signal limits with the BP_LATERAL bit set.

Every expectation comes from ford_bluepilot_common.BPFordLateralModel, a float32 mirror of the
C, so boundaries are asserted at the exact CAN unit on both x86 and ARM. Frames never combine
curvature == 0 with path_angle == 0 unless a test is about that, because that frame arms the
reset-bypass latch (#9) and disables every check for the next 60 frames.
"""
import unittest

import opendbc.safety.tests.common as common

from opendbc.safety.tests.ford_bluepilot_common import (
  BPFordTestCase, angle_roc_delta, SMALL_ANGLE, CURVATURE_SIGNAL_MAX_CAN,
  CURVATURE_TO_CAN, MAX_CURVATURE_CAN, MAX_CURVATURE_ERROR_CAN, CURVATURE_ERROR_MIN_SPEED,
  PATH_ANGLE_TO_CAN, PATH_ANGLE_MAX_CAN, PATH_ANGLE_DBC_MIN_CAN, PATH_ANGLE_DBC_MAX_CAN,
  PATH_ANGLE_LOOKUP_X, PATH_ANGLE_LOOKUP_Y,
  PATH_OFFSET_TO_CAN, PATH_OFFSET_MAX_CAN, PATH_OFFSET_LOOKUP_X, PATH_OFFSET_LOOKUP_Y,
  CURVATURE_RATE_MAX, CURVATURE_RATE_MIN, CURVATURE_RATE_TO_CAN_CANFD, CURVATURE_RATE_TO_CAN_CAN,
  CURVATURE_RATE_LOOKUP_X, CURVATURE_RATE_LOOKUP_Y,
)

# speeds chosen to hit both sides of the error-band gate (10 m/s) and every segment of the
# {5, 16, 25} rate table, plus its clamped ends
SPEEDS = (3.0, 5.0, 8.0, 10.5, 12.0, 16.0, 20.0, 25.0, 30.0)


class TestFordBPLimitsCANFD(BPFordTestCase):
  CANFD = True

  def _probe(self, desired_can: int, last_can: int) -> bool:
    self.set_prev_curvature_can(last_can)
    return self.tx(self.lat(True, 0, SMALL_ANGLE, desired_can / CURVATURE_TO_CAN, 0))

  def _assert_matches_model(self, last_can: int, probes):
    v_min, v_max = self.speeds()
    m_min, m_max = self.meas()
    for desired in probes:
      if desired == 0 or abs(desired) > CURVATURE_SIGNAL_MAX_CAN:  # 0 would arm the latch
        continue
      expected = self.model.allowed(desired, last_can, m_min, m_max, v_min, v_max, v_max)
      with self.subTest(speed=v_max, meas=(m_min, m_max), last=last_can, desired=desired):
        self.assertEqual(expected, self._probe(desired, last_can))

  def test_curvature_rate_lookup_boundaries(self):
    """Per-frame delta from the {5,16,25} m/s table (bp_curvature_rate_lookup_check), both
    directions, both signs, exact boundary: the speed fudge and the delta arithmetic."""
    for speed in SPEEDS:
      self.set_meas(0, speed)
      v_min, v_max = self.speeds()
      delta = self.model.rate_delta(v_min)
      for last in (0, 40, -40, 250, -250):
        lowest, highest = self.model.bounds(last, *self.meas(), v_min, v_max, v_max)
        self._assert_matches_model(last, (highest - 1, highest, highest + 1, lowest - 1, lowest, lowest + 1,
                                          last + delta, last + delta + 1, last - delta, last - delta - 1))

  def test_curvature_error_band_convergence(self):
    """Above 10 m/s the command must stay within max_curvature_error of measured; when the last
    command is already outside the band it must move back toward it by at least the relaxed
    delta, and may not move further away. Below 10 m/s the band is off (the relaxed branches of
    bp_curvature_rate_lookup_check). Seen in the car: a 4-frame block at 26 mph with the driver
    steering harder than the command, sitting exactly on this window's edge (#12)."""
    # measured both signs, and last commands on both sides of zero, so both relaxed branches run
    # with last commands of either sign. The C picks the up or down table by that sign, and the
    # down table is 1% looser (#20), so each relaxed probe below uses the table the C picks.
    for meas_can in (200, -200, 600, -600):
      for speed in (8.0, 12.0, 16.0, 20.0):
        self.set_meas(meas_can / CURVATURE_TO_CAN, speed)
        v_min, v_max = self.speeds()
        m_min, m_max = self.meas()
        band_on = v_max > CURVATURE_ERROR_MIN_SPEED
        delta = self.model.rate_delta(v_min)
        lowest_err = m_min - MAX_CURVATURE_ERROR_CAN - 1
        highest_err = m_max + MAX_CURVATURE_ERROR_CAN + 1
        cap = self.model.cap(v_min)  # the explicit asserts below stay under it

        # last command below the band (on either side of zero): must climb toward it
        for last in (lowest_err - 150, 0, -1, 1):
          if last >= lowest_err:
            continue
          relaxed = self.model.rate_delta_relaxed(v_max, up=last > 0)  # climbing: down table while last <= 0
          self._assert_matches_model(last, (last + relaxed - 1, last + relaxed, last + delta, last + delta + 1,
                                            last - 1, lowest_err))
          if band_on:
            ctx = f"meas={meas_can} speed={speed} last={last} relaxed={relaxed} band=({lowest_err},{highest_err})"
            self.assertFalse(self._probe(last + relaxed - 1, last), "must move toward the band " + ctx)
            if abs(min(last + relaxed, lowest_err)) <= cap:
              self.assertTrue(self._probe(min(last + relaxed, lowest_err), last), ctx)
            if last - 1 != 0:  # curvature 0 is angle mode's sentinel and skips the curvature checks
              self.assertFalse(self._probe(last - 1, last), "may not move away from the band " + ctx)

        # last command above the band (on either side of zero): must descend toward it
        for last in (highest_err + 150, 0, -1, 1):
          if last <= highest_err:
            continue
          relaxed = self.model.rate_delta_relaxed(v_max, up=last < 0)  # descending: down table while last >= 0
          self._assert_matches_model(last, (last - relaxed + 1, last - relaxed, last - delta, last - delta - 1,
                                            last + 1, highest_err))
          if band_on:
            ctx = f"meas={meas_can} speed={speed} last={last} relaxed={relaxed} band=({lowest_err},{highest_err})"
            self.assertFalse(self._probe(last - relaxed + 1, last), "must move toward the band " + ctx)
            if abs(max(last - relaxed, highest_err)) <= cap:
              self.assertTrue(self._probe(max(last - relaxed, highest_err), last), ctx)
            if last + 1 != 0:
              self.assertFalse(self._probe(last + 1, last), "may not move away from the band " + ctx)

        # last command inside the band: may not leave it
        last = highest_err - 1
        self._assert_matches_model(last, (highest_err, highest_err + 1, lowest_err, lowest_err - 1))
        if band_on and delta > 1 and abs(highest_err) <= cap:
          self.assertTrue(self._probe(highest_err, last))
          self.assertFalse(self._probe(highest_err + 1, last), "inside the band, may not command outside it")

  def test_lateral_accel_cap(self):
    """CAN FD only: |curvature| <= (ISO_LATERAL_ACCEL - g*roll) / v^2, with the speed fudged down 1 m/s."""
    for speed in (15.0, 20.0, 25.0, 30.0):
      self.set_meas(0, speed)
      v_min, _ = self.speeds()
      cap = self.model.accel_cap(v_min)
      self.assertLess(cap, MAX_CURVATURE_CAN, "cap only means something below the 0.02 hard limit")
      for sign in (1, -1):
        # measured curvature at the cap so the error band and rate limits allow it; last at the cap too
        self.set_meas(sign * cap / CURVATURE_TO_CAN, speed)
        with self.subTest(speed=speed, sign=sign):
          self.assertTrue(self._probe(sign * cap, sign * cap))
          self.assertFalse(self._probe(sign * (cap + 1), sign * cap))

  def test_max_curvature_hard_limit(self):
    """+-0.02 rad/m (1000 CAN) regardless of speed. At 3 m/s the accel cap and the band are off."""
    self.set_meas(0.02, 3.0)
    for sign in (1, -1):
      self.assertTrue(self._probe(sign * MAX_CURVATURE_CAN, sign * MAX_CURVATURE_CAN))
      self.assertFalse(self._probe(sign * (MAX_CURVATURE_CAN + 1), sign * MAX_CURVATURE_CAN))

  def test_signals_zero_when_steer_disabled(self):
    """With LatCtl_D2_Rq = 0 every signal must be zero (path_angle/path_offset/curvature_rate_cmd_checks
    and steer_curvature_cmd_checks).

    A disabled frame with curvature 0 and path_angle 0 arms the reset latch and is bypassed
    whatever its offset or rate carry, so those two can only be asserted alongside a non-zero
    path_angle or curvature until #9 is decided; the lines still execute (coverage)."""
    self.set_meas(0, 15.0)
    self.assertFalse(self.tx(self.lat(False, 0, 0.01, 0, 0)))
    self.assertFalse(self.tx(self.lat(False, 0.1, 0.01, 0, 0)))
    self.assertFalse(self.tx(self.lat(False, 0, 0.01, 0, 0.0001)))
    self.assertFalse(self.tx(self.lat(False, 0, 0, 0.001, 0)))
    self.assertFalse(self.tx(self.lat(False, 0.1, 0, 0.001, 0)))
    self.assertFalse(self.tx(self.lat(False, 0, 0, 0.001, 0.0001)))
    # the all-zero disabled frame is the one thing allowed (it also arms the latch, hence last)
    self.assertTrue(self.tx(self.lat(False, 0, 0, 0, 0)))

  def test_controls_not_allowed_blocks_steering(self):
    """steer enabled with controls not allowed is blocked whatever the signals carry: a real
    curvature command (steer_curvature_cmd_checks), in curvature mode and with the angle-mode
    flag set. Upstream's test_steer_allowed asserts this for the stock path; this is the BP path."""
    self.set_meas(0, 15.0)
    for angle_mode in (False, True):
      self.tx(self.lka_bp_status_msg(angle_mode, 0.0))
      self.safety.set_controls_allowed(True)
      self.assertTrue(self._probe(20, 10))
      self.safety.set_controls_allowed(False)
      self.safety.set_controls_allowed_lateral(False)
      with self.subTest(angle_mode=angle_mode):
        self.assertFalse(self._probe(20, 10))
        self.assertFalse(self.tx(self.lat(True, 0, SMALL_ANGLE, 0, 0)))
      self.safety.set_controls_allowed(True)
    self.tx(self.lka_bp_status_msg(False, 0.0))

  def test_newest_speed_sample_gates_the_band(self):
    """The error band switches on vehicle_speed.values[0], the newest sample, not the window's
    min or max, and only strictly above 10 m/s. Six samples at 9 m/s then one at 11: band on;
    the reverse: band off; a newest sample of exactly 10.0: band off."""
    meas_can = 200
    last = meas_can + MAX_CURVATURE_ERROR_CAN  # inside the band, so only the band decides
    below, above, gate = CURVATURE_ERROR_MIN_SPEED - 1, CURVATURE_ERROR_MIN_SPEED + 1, CURVATURE_ERROR_MIN_SPEED
    # outside the band (its edge is meas + band + 1), inside the rate delta from 'last' at the slowest window
    out_of_band = last + self.model.rate_delta(below) // 2
    self.assertGreater(out_of_band, meas_can + MAX_CURVATURE_ERROR_CAN + 1)
    for window, newest, expect_blocked in ((below, above, True), (above, below, False), (above, gate, False), (below, gate, False)):
      self.set_meas(meas_can / CURVATURE_TO_CAN, window)
      self.rx(self.stock._speed_msg(newest))
      v_min, v_max = self.speeds()
      m_min, m_max = self.meas()
      with self.subTest(window=window, newest=newest):
        self.assertEqual(not expect_blocked, self.model.allowed(out_of_band, last, m_min, m_max, v_min, v_max, newest))
        self.assertEqual(not expect_blocked, self._probe(out_of_band, last))

  def test_zero_curvature_skips_curvature_checks(self):
    """Curvature == 0 (angle mode's sentinel) discards the curvature checks' result; only the
    controls-allowed gate remains (the curvature == 0 branch of ford_lmc_checks). Pins current
    behavior, see #10."""
    self.set_meas(0.01, 15.0)  # measured 500 CAN: a real curvature command of 0 would be far outside the band
    self.assertTrue(self.tx(self.lat(True, 0, SMALL_ANGLE, 0, 0)))
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(False)
    self.assertFalse(self.tx(self.lat(True, 0, SMALL_ANGLE, 0, 0)))
    self.safety.set_controls_allowed(True)

  # --- the other three signals ---

  # the three non-curvature probes carry 1 CAN unit of curvature (measured 0, last 1) so a
  # path_angle of 0 never makes a zero/zero frame that would arm the latch
  TINY_CURVATURE_CAN = 1

  def _angle_probe(self, angle_can: int) -> bool:
    self.set_prev_curvature_can(self.TINY_CURVATURE_CAN)
    return self.tx(self.lat(True, 0, angle_can / PATH_ANGLE_TO_CAN, self.TINY_CURVATURE_CAN / CURVATURE_TO_CAN, 0))

  def test_path_angle_range_and_roc(self):
    """Per-frame ROC from the {10,15,25} table (path_angle_cmd_checks) and the +-0.25 rad range in curvature
    mode. A frame always becomes the new 'last' even when blocked, so each probe re-baselines."""
    for speed in (8.0, 12.0, 20.0, 25.0):
      self.set_meas(0, speed)
      v_min, _ = self.speeds()
      delta = angle_roc_delta(PATH_ANGLE_LOOKUP_X, PATH_ANGLE_LOOKUP_Y, PATH_ANGLE_TO_CAN, v_min)
      for base in (0, 200, -200):
        for sign in (1, -1):
          with self.subTest(speed=speed, base=base, sign=sign):
            self._angle_probe(base)
            self.assertTrue(self._angle_probe(base + sign * delta))
            self._angle_probe(base)
            self.assertFalse(self._angle_probe(base + sign * (delta + 1)))
      # range: step from just inside so the ROC allows it
      if delta > 2:
        for sign in (1, -1):
          self._angle_probe(sign * (PATH_ANGLE_MAX_CAN - 1))
          self.assertTrue(self._angle_probe(sign * PATH_ANGLE_MAX_CAN))
          self.assertFalse(self._angle_probe(sign * (PATH_ANGLE_MAX_CAN + 1)))

  def test_path_angle_dbc_range_in_angle_mode(self):
    """With angle mode engaged (Lane_Assist_Data1 bit) the range widens to the DBC's [-0.5, 0.5235]."""
    self.set_meas(0, 8.0)  # below the shadow-check speed gate so only the range matters
    self.tx(self.lka_bp_status_msg(True, 0.0))
    v_min, _ = self.speeds()
    delta = angle_roc_delta(PATH_ANGLE_LOOKUP_X, PATH_ANGLE_LOOKUP_Y, PATH_ANGLE_TO_CAN, v_min)
    self.assertGreater(delta, 60)
    for target, over in ((PATH_ANGLE_DBC_MAX_CAN, PATH_ANGLE_DBC_MAX_CAN + 1), (PATH_ANGLE_DBC_MIN_CAN, PATH_ANGLE_DBC_MIN_CAN - 1)):
      # walk up in ROC-sized steps (the signal can pack these values)
      pos = 0
      while abs(target - pos) > delta:
        pos += delta if target > pos else -delta
        self._angle_probe(pos)
      self.assertTrue(self._angle_probe(target))
      self.assertFalse(self._angle_probe(over))
      self._angle_probe(pos)
    self.tx(self.lka_bp_status_msg(False, 0.0))

  def _offset_probe(self, offset_can: int) -> bool:
    self.set_prev_curvature_can(self.TINY_CURVATURE_CAN)
    return self.tx(self.lat(True, offset_can / PATH_OFFSET_TO_CAN, SMALL_ANGLE, self.TINY_CURVATURE_CAN / CURVATURE_TO_CAN, 0))

  def test_path_offset_range_and_roc(self):
    """Per-frame ROC from the {5,15,25} table (path_offset_cmd_checks) and the +-1.0 m range."""
    for speed in (3.0, 8.0, 12.0, 20.0, 25.0):
      self.set_meas(0, speed)
      v_min, _ = self.speeds()
      delta = angle_roc_delta(PATH_OFFSET_LOOKUP_X, PATH_OFFSET_LOOKUP_Y, PATH_OFFSET_TO_CAN, v_min)
      for base in (0, 50, -50):
        for sign in (1, -1):
          with self.subTest(speed=speed, base=base, sign=sign):
            self._offset_probe(base)
            self.assertTrue(self._offset_probe(base + sign * delta))
            self._offset_probe(base)
            self.assertFalse(self._offset_probe(base + sign * (delta + 1)))
      for sign in (1, -1):
        self._offset_probe(sign * (PATH_OFFSET_MAX_CAN - 1))
        self.assertTrue(self._offset_probe(sign * PATH_OFFSET_MAX_CAN))
        self.assertFalse(self._offset_probe(sign * (PATH_OFFSET_MAX_CAN + 1)))

  RATE_TO_CAN = CURVATURE_RATE_TO_CAN_CANFD

  def _rate_probe(self, rate_can: int) -> bool:
    self.set_prev_curvature_can(self.TINY_CURVATURE_CAN)
    return self.tx(self.lat(True, 0, SMALL_ANGLE, self.TINY_CURVATURE_CAN / CURVATURE_TO_CAN, rate_can / self.RATE_TO_CAN))

  def test_curvature_rate_range_and_roc(self):
    """Curvature rate: the ROC table (curvature_rate_cmd_checks) allows tens of thousands of CAN
    units per frame, far more than the signal can carry, so the whole DBC range is reachable in one
    step; the value limits in ford_lmc_checks coincide with the signal's own range."""
    self.set_meas(0, 12.0)
    v_min, _ = self.speeds()
    delta = angle_roc_delta(CURVATURE_RATE_LOOKUP_X, CURVATURE_RATE_LOOKUP_Y, self.RATE_TO_CAN, v_min)
    top = int(CURVATURE_RATE_MAX * self.RATE_TO_CAN)
    bottom = int(CURVATURE_RATE_MIN * self.RATE_TO_CAN)
    self.assertGreater(delta, top - bottom)
    self._rate_probe(0)
    self.assertTrue(self._rate_probe(top))
    self.assertTrue(self._rate_probe(bottom))
    self.assertTrue(self._rate_probe(top // 2))
    self._rate_probe(0)

  def test_curvature_zero_frames_are_rate_limited(self):
    """#24: angle mode sends curvature 0 on every frame. Those frames are held to the same message-rate
    window as the rest (upstream's rolling 250 ms window, split in two halves): sent faster than 20 Hz,
    every frame past the limit is blocked until the window rolls over. Path angle carries the command,
    so these frames never arm the reset latch."""
    self.set_meas(0, 15.0)
    max_rt_msgs = int(20 * common.RT_INTERVAL / 1e6 * 1.2 + 1)
    half = common.RT_INTERVAL // 2

    def send():
      return self.tx(self.lat(True, 0, SMALL_ANGLE, 0, 0, increment_timer=False))

    self.safety.set_timer(0)
    for i in range(max_rt_msgs * 2):
      self.assertEqual(i <= max_rt_msgs, send(), i)
    self.safety.set_timer(half)
    self.assertFalse(send())  # the overflow moves into the previous half
    self.safety.set_timer(half + common.RT_INTERVAL)
    self.assertFalse(send())
    self.safety.set_timer(half + 2 * common.RT_INTERVAL)
    for _ in range(max_rt_msgs):
      self.assertTrue(send())

class TestFordBPLimitsCAN(TestFordBPLimitsCANFD):
  """Same checks on the CAN (non-FD) LateralMotionControl message: no lateral-accel cap, and the
  curvature-rate signal has 4x the wire resolution."""
  CANFD = False
  RATE_TO_CAN = CURVATURE_RATE_TO_CAN_CAN

  def test_lateral_accel_cap(self):
    """CAN: no cap. A curvature well above the CAN FD cap is accepted up to the 0.02 hard limit."""
    self.set_meas(MAX_CURVATURE_CAN / CURVATURE_TO_CAN, 25.0)
    v_min, _ = self.speeds()
    self.assertLess(self.model.accel_cap(v_min), MAX_CURVATURE_CAN)  # the FD cap would bite here
    self.assertTrue(self._probe(MAX_CURVATURE_CAN, MAX_CURVATURE_CAN))


if __name__ == "__main__":
  unittest.main()
