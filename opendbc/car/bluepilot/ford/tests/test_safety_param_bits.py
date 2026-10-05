import re
import unittest
from pathlib import Path

from opendbc.car import structs
from opendbc.car.bluepilot.ford.values_ext import FordSafetyFlagsSP, FORD_PINION_GEOMETRY_INDEX, FORD_PINION_GEOMETRY_SHIFT, \
  init_ford_safety_param_sp
from opendbc.car.ford.values import CAR

SAFETY = Path(__file__).resolve().parents[4] / "safety"


def _c_const(text, name):
  m = re.search(rf"{name}\s*=?\s*(\d+)U?\s*;?", text)
  return int(m.group(1))


class TestSafetyParamBits(unittest.TestCase):
  """The SP safety-param bit layout is written on both sides (Python here, C in ford_init).
  Pin them to each other and to non-overlap."""

  def test_python_and_c_agree(self):
    ford_h = (SAFETY / "modes" / "ford.h").read_text()
    decl_h = (SAFETY / "bluepilot" / "ford_declarations.h").read_text()
    self.assertEqual(_c_const(ford_h, "FORD_PARAM_SP_BP_LATERAL"), FordSafetyFlagsSP.BP_LATERAL)
    self.assertEqual(_c_const(ford_h, "FORD_PARAM_SP_STEER_ANGLE_CURVATURE"), FordSafetyFlagsSP.STEER_ANGLE_CURVATURE)
    self.assertGreaterEqual(_c_const(decl_h, "#define FORD_PINION_GEOMETRY_COUNT"), max(FORD_PINION_GEOMETRY_INDEX.values()))

  def test_geometry_index_never_touches_the_lateral_bit(self):
    # ford_init masks the index with 0xF after a 1-bit shift: bits 1-4
    for index in FORD_PINION_GEOMETRY_INDEX.values():
      self.assertLessEqual(index, 0xF)
      self.assertEqual((index << FORD_PINION_GEOMETRY_SHIFT) & FordSafetyFlagsSP.BP_LATERAL, 0)
      self.assertEqual((index << FORD_PINION_GEOMETRY_SHIFT) & FordSafetyFlagsSP.STEER_ANGLE_CURVATURE, 0)
    self.assertEqual(FordSafetyFlagsSP.BP_LATERAL, 0x20)


class TestInitFordSafetyParamSp(unittest.TestCase):
  """What init_ford_safety_param_sp puts in CP_SP.safetyParam at car init."""

  def _param_sp(self, params_dict, brand="ford", fingerprint=CAR.FORD_F_150_MK14):
    CP = structs.CarParams()
    CP.brand, CP.carFingerprint = brand, fingerprint
    CP_SP = structs.CarParamsSP()
    init_ford_safety_param_sp(CP, CP_SP, params_dict)
    return CP_SP.safetyParam

  def test_bp_lateral_on_unless_disabled(self):
    self.assertEqual(self._param_sp({}), FordSafetyFlagsSP.BP_LATERAL)
    for off in (True, 1, "1", "true", " True "):
      self.assertEqual(self._param_sp({"disable_BP_lat_UI": off}), 0)
    for on in (False, 0, None, "0", "false", ""):
      self.assertEqual(self._param_sp({"disable_BP_lat_UI": on}), FordSafetyFlagsSP.BP_LATERAL)

  def test_pinion_sets_flag_and_geometry_index(self):
    self.assertEqual(self._param_sp({"FordPrefSteerAngleCurvature": "1"}),
                     FordSafetyFlagsSP.BP_LATERAL | FordSafetyFlagsSP.STEER_ANGLE_CURVATURE |
                     (FORD_PINION_GEOMETRY_INDEX[CAR.FORD_F_150_MK14] << FORD_PINION_GEOMETRY_SHIFT))

  def test_pinion_needs_bp_lateral_and_a_geometry_row(self):
    self.assertEqual(self._param_sp({"FordPrefSteerAngleCurvature": "1", "disable_BP_lat_UI": "1"}), 0)
    # the Edge MK2 (#30) has no geometry row and isn't a platform in this baseline
    self.assertEqual(self._param_sp({"FordPrefSteerAngleCurvature": "1"}, fingerprint="FORD_EDGE_MK2"),
                     FordSafetyFlagsSP.BP_LATERAL)

  def test_other_brands_untouched(self):
    self.assertEqual(self._param_sp({"FordPrefSteerAngleCurvature": "1"}, brand="toyota"), 0)


if __name__ == "__main__":
  unittest.main()
