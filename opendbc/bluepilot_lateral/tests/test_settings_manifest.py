import unittest

from opendbc.car import structs
from opendbc.bluepilot_lateral.params import manifest
from opendbc.bluepilot_lateral.params.manifest import SETTINGS, PER_FRAME_KEYS, startup_value, value
from opendbc.bluepilot_lateral.params.param_store import ParamStore
from opendbc.bluepilot_lateral.core.values_ext import BP_LATERAL_PARAMS


def _store(**kv):
  cc_sp = structs.CarControlSP()
  for k, v in kv.items():
    p = structs.CarControlSP.Param()
    p.key, p.value = k, v
    cc_sp.params.append(p)
  return ParamStore.from_cc_sp(cc_sp)


class TestSettingsManifest(unittest.TestCase):
  def test_entries_are_consistent(self):
    keys = [s.key for s in SETTINGS]
    self.assertEqual(len(keys), len(set(keys)))
    for s in SETTINGS:
      with self.subTest(key=s.key):
        self.assertTrue(s.type in (bool, int, float))
        self.assertIs(type(s.default), s.type)
        self.assertTrue(s.read in (manifest.STARTUP, manifest.PER_FRAME, manifest.BOTH))
        self.assertTrue(s.label and s.description)
        if s.range is not None:
          self.assertLessEqual(s.range[0], s.default)
          self.assertLessEqual(s.default, s.range[1])
        self.assertTrue(not s.clamp or s.range is not None)

  def test_per_frame_keys_are_what_hosts_publish(self):
    # hosts (bp-sunnypilot's controlsd) publish BP_LATERAL_PARAMS in CarControlSP.params
    self.assertEqual(BP_LATERAL_PARAMS, PER_FRAME_KEYS)
    self.assertEqual((len(SETTINGS), len(PER_FRAME_KEYS)), (19, 18))

  def test_one_read_rule(self):
    """A published, parseable value is used; missing, empty or unparsable gives the default."""
    self.assertEqual(value(_store(lane_change_factor_high_curv=b"0.7"), "lane_change_factor_high_curv"), 0.7)
    self.assertEqual(value(_store(), "lane_change_factor_high_curv"), 0.85)
    self.assertEqual(value(_store(lane_change_factor_high_curv=b""), "lane_change_factor_high_curv"), 0.85)
    self.assertEqual(value(_store(lane_change_factor_high_curv=b"x"), "lane_change_factor_high_curv"), 0.85)
    self.assertEqual(value(_store(custom_profile_curv=b"1"), "custom_profile_curv"), 1)
    self.assertIs(value(_store(enable_human_turn_detection_curv=b"0"), "enable_human_turn_detection_curv"), False)
    self.assertIs(value(_store(), "enable_human_turn_detection_curv"), True)
    self.assertIs(value(_store(enable_human_turn_detection_curv=b""), "enable_human_turn_detection_curv"), True)

  def test_only_the_angle_factors_are_clamped(self):
    self.assertEqual(value(_store(FordLowSpeedFactor_ang=b"9"), "FordLowSpeedFactor_ang"), 1.5)
    self.assertEqual(value(_store(lane_centering_strength_ang=b"-1"), "lane_centering_strength_ang"), 0.0)
    self.assertEqual(value(_store(lane_change_factor_high_curv=b"9"), "lane_change_factor_high_curv"), 9.0)

  def test_startup_values(self):
    # get_car's params dict holds whatever Params returned; a restored backup can hold text
    for on in (True, 1, "1", "true", " True "):
      self.assertIs(startup_value({"disable_BP_lat_UI": on}, "disable_BP_lat_UI"), True, on)
    for off in (False, 0, None, "0", "false", "", "junk"):
      self.assertIs(startup_value({"disable_BP_lat_UI": off}, "disable_BP_lat_UI"), False, off)
    self.assertIs(startup_value({}, "FordPrefSteerAngleCurvature"), False)

  def test_params_keys_lines(self):
    lines = manifest.params_keys_lines()
    self.assertEqual(len(lines), len(SETTINGS))
    self.assertTrue('{"FordPrefLateralControl", {PERSISTENT | BACKUP, INT, "0"}},' in lines)
    self.assertTrue('{"enable_human_turn_detection_curv", {PERSISTENT | BACKUP, BOOL, "1"}},' in lines)
    self.assertTrue('{"lane_change_factor_high_curv", {PERSISTENT | BACKUP, FLOAT, "0.85"}},' in lines)


if __name__ == "__main__":
  unittest.main()
