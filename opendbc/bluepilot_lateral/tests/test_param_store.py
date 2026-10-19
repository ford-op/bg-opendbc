import unittest

from opendbc.car import structs
from opendbc.bluepilot_lateral.params.param_store import ParamStore
from opendbc.bluepilot_lateral.core.lateral_curv_ext import LateralCurvExt
from opendbc.bluepilot_lateral.params.manifest import value
from opendbc.bluepilot_lateral.core.values_ext import BP_LATERAL_PARAMS


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
    self.assertTrue(value(ps, "disable_BP_lat_UI"))
    self.assertEqual(value(ps, "custom_profile_curv"), 2)
    self.assertEqual(value(ps, "LC_PID_gain_UI_curv"), 3.5)

  def test_unset_keys_behave_like_params(self):
    ps = ParamStore.from_cc_sp(_cc_sp())
    self.assertIsNone(ps.get("FordLowSpeedFactor_ang"))
    # the manifest default applies to any key the fork did not publish, bools included
    self.assertTrue(value(ps, "enable_human_turn_detection_curv"))
    self.assertEqual(value(ps, "LC_PID_gain_UI_curv"), 1.0)

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
    # the fork publishes bools as "1"/"0"
    ps = ParamStore.from_cc_sp(_cc_sp(enable_lane_positioning_curv=b"1", enable_human_turn_detection_curv=b"0"))
    self.assertIs(value(ps, "enable_lane_positioning_curv"), True)
    self.assertIs(value(ps, "enable_human_turn_detection_curv"), False)

  def test_angle_side_values(self):
    ps = ParamStore.from_cc_sp(_cc_sp(FordLowSpeedFactor_ang=b"1.2"))
    self.assertEqual(value(ps, "FordLowSpeedFactor_ang"), 1.2)

  def test_key_list_is_consistent(self):
    # the count is pinned against the manifest in test_settings_manifest.py
    self.assertEqual(len(BP_LATERAL_PARAMS), len(set(BP_LATERAL_PARAMS)))


if __name__ == "__main__":
  unittest.main()
