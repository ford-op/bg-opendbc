import unittest

from opendbc.car import structs
from opendbc.car.bluepilot.ford.param_store import ParamStore
from opendbc.car.bluepilot.ford.lateral_curv_ext import LateralCurvExt, _read_param
from opendbc.car.bluepilot.ford.values_ext import BP_LATERAL_PARAMS


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

  def test_unset_keys_behave_like_params(self):
    ps = ParamStore.from_cc_sp(_cc_sp())
    # ParamStore mirrors Params: an unset bool reads as False, get() raises, get(return_default=True) is None
    self.assertFalse(ps.get_bool("enable_human_turn_detection_curv"))
    self.assertIsNone(ps.get("FordLowSpeedFactor_ang", return_default=True))
    # _read_param gives the caller default for any key the fork did not publish, bools included
    self.assertTrue(_read_param(ps, "enable_human_turn_detection_curv", bool, True))
    self.assertEqual(_read_param(ps, "LC_PID_gain_UI_curv", float, 1.0), 1.0)

  def test_empty_params_give_registry_defaults(self):
    """#6: with nothing published, the lateral runs the defaults the settings UI shows
    (params_keys.h in the fork): human-turn detection on, low-curvature PID gain 1.0."""
    ext = type("Ext", (), {})()
    LateralCurvExt.update_lateral_params(ext, ParamStore.from_cc_sp(_cc_sp()))
    self.assertTrue(ext.enable_human_turn_detection_curv)
    self.assertEqual(ext.LC_PID_gain_UI_curv, 1.0)
    # a published value still wins
    LateralCurvExt.update_lateral_params(ext, ParamStore.from_cc_sp(_cc_sp(enable_human_turn_detection_curv=b"0")))
    self.assertFalse(ext.enable_human_turn_detection_curv)

  def test_bool_wire_format(self):
    ps = ParamStore.from_cc_sp(_cc_sp(a=b"1", b=b"0", c=b"True"))
    self.assertTrue(ps.get_bool("a"))
    self.assertFalse(ps.get_bool("b"))
    self.assertFalse(ps.get_bool("c"))  # Params stores "1"/"0" only

  def test_angle_side_read_shape(self):
    # lateral_angle_ext decodes bytes itself and accepts str; either must parse
    ps = ParamStore.from_cc_sp(_cc_sp(FordLowSpeedFactor_ang=b"1.2"))
    raw = ps.get("FordLowSpeedFactor_ang", return_default=True)
    self.assertEqual(float(raw.decode() if isinstance(raw, bytes) else raw), 1.2)

  def test_key_list_is_consistent(self):
    self.assertEqual(len(BP_LATERAL_PARAMS), len(set(BP_LATERAL_PARAMS)))
    self.assertEqual(len(BP_LATERAL_PARAMS), 18)


if __name__ == "__main__":
  unittest.main()
