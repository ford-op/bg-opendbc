import unittest

from opendbc.car import structs
from opendbc.car.bluepilot.ford.lateral_curv_ext import LateralCurvExt
from opendbc.car.bluepilot.ford.values_ext import MODEL_T_IDXS


def _cc_sp(valid=True):
  cc_sp = structs.CarControlSP()
  li = cc_sp.lateralInputs
  li.valid = valid
  li.pathYawRate = [0.01 * i for i in range(33)]
  li.pathX = [float(i) for i in range(33)]
  li.pathY = [0.1 * i for i in range(33)]
  li.laneLineLeftX = li.laneLineRightX = [float(i) for i in range(33)]
  li.laneLineLeftY = [-1.8] * 33
  li.laneLineRightY = [1.8] * 33
  li.laneLineProbs = [0.1, 0.9, 0.8, 0.1]
  li.laneLineStds = [1.0, 0.1, 0.1, 1.0]
  li.laneChangeState = 2
  li.laneChangeDirection = 1
  li.steerRatio = 15.5
  li.stiffnessFactor = 0.9
  li.roll = 0.02
  li.angleOffsetDeg = 0.3
  li.lateralDelay = 0.12
  return cc_sp


class _Ext:
  """Bare object carrying only the attributes update_inputs touches."""
  def __init__(self):
    self.model = None
    self.lp = None
    self.lateral_delay = 0.0
    self.VM = self
    self.updated = None

  def update_params(self, x, sr):
    self.updated = (x, sr)


class TestLateralInputs(unittest.TestCase):
  def test_model_view_has_modelv2_shape(self):
    ext = _Ext()
    LateralCurvExt.update_inputs(ext, _cc_sp())
    m = ext.model
    self.assertEqual(len(m.orientationRate.z), 33)
    self.assertEqual(m.position.y[10], 1.0)
    self.assertEqual(m.laneLines[1].y[0], -1.8)
    self.assertEqual(m.laneLines[2].y[0], 1.8)
    self.assertEqual(m.laneLineProbs[2], 0.8)
    self.assertEqual(m.laneLineStds[1], 0.1)
    self.assertEqual(m.meta.laneChangeState, 2)
    self.assertEqual(m.meta.laneChangeDirection, 1)

  def test_vehicle_params_and_delay(self):
    ext = _Ext()
    LateralCurvExt.update_inputs(ext, _cc_sp())
    self.assertEqual(ext.lp.angleOffsetDeg, 0.3)
    self.assertEqual(ext.lp.roll, 0.02)
    self.assertEqual(ext.lateral_delay, 0.12)
    self.assertEqual(ext.updated, (0.9, 15.5))

  def test_invalid_inputs_leave_previous_values(self):
    ext = _Ext()
    LateralCurvExt.update_inputs(ext, _cc_sp())
    LateralCurvExt.update_inputs(ext, _cc_sp(valid=False))
    self.assertEqual(ext.lateral_delay, 0.12)
    self.assertIsNotNone(ext.model)

  def test_default_cc_sp_is_not_valid(self):
    self.assertFalse(structs.CarControlSP().lateralInputs.valid)

  def test_t_idxs_matches_modeld(self):
    # openpilot ModelConstants: index_function(i, max_val=10.0) = 10 * (i/32)^2, 33 points
    self.assertEqual(len(MODEL_T_IDXS), 33)
    self.assertEqual(MODEL_T_IDXS[0], 0.0)
    self.assertAlmostEqual(MODEL_T_IDXS[16], 2.5)
    self.assertEqual(MODEL_T_IDXS[32], 10.0)


if __name__ == "__main__":
  unittest.main()
