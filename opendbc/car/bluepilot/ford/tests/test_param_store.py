import unittest

from opendbc.car import structs
from opendbc.car.bluepilot.ford.param_store import ParamStore
from opendbc.car.bluepilot.ford.lateral_curv_ext import _read_param
from opendbc.car.bluepilot.ford.values_ext import BP_LATERAL_PARAMS, BP_LATERAL_BOOL_PARAMS


def _cc_sp(**kv):
  cc_sp = structs.CarControlSP()
  for k, v in kv.items():
    p = structs.CarControlSP.Param()
    p.key, p.value = k, v
    cc_sp.params.append(p)
  return cc_sp


class TestParamStore(unittest.TestCase):
  def test_reads_bool_int_float_like_params(self):
    ps = ParamStore.from_cc_sp(_cc_sp(disable_BP_lat_UI=b"1", custom_profile_curv=b"2", LC_PID_gain_UI_curv=b"3.5"))
    self.assertTrue(ps.get_bool("disable_BP_lat_UI"))
    self.assertEqual(_read_param(ps, "custom_profile_curv", int, 0), 2)
    self.assertEqual(_read_param(ps, "LC_PID_gain_UI_curv", float, 1.0), 3.5)

  def test_missing_key_falls_back_to_caller_default(self):
    ps = ParamStore.from_cc_sp(_cc_sp())
    self.assertTrue(_read_param(ps, "enable_human_turn_detection_curv", bool, True))
    self.assertEqual(_read_param(ps, "LC_PID_gain_UI_curv", float, 1.0), 1.0)
    self.assertIsNone(ps.get("FordLowSpeedFactor_ang", return_default=True))
    with self.assertRaises(KeyError):
      ps.get_bool("disable_BP_lat_UI")

  def test_angle_side_read_shape(self):
    # lateral_angle_ext decodes bytes itself and accepts str; either must parse
    ps = ParamStore.from_cc_sp(_cc_sp(FordLowSpeedFactor_ang=b"1.2"))
    raw = ps.get("FordLowSpeedFactor_ang", return_default=True)
    self.assertEqual(float(raw.decode() if isinstance(raw, bytes) else raw), 1.2)

  def test_key_list_is_consistent(self):
    self.assertEqual(len(BP_LATERAL_PARAMS), len(set(BP_LATERAL_PARAMS)))
    self.assertIn("disable_BP_lat_UI", BP_LATERAL_BOOL_PARAMS)


if __name__ == "__main__":
  unittest.main()
