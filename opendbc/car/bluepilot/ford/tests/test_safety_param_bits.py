import re
import unittest
from pathlib import Path

from opendbc.car.bluepilot.ford.values_ext import FordSafetyFlagsSP, FORD_PINION_GEOMETRY_INDEX, FORD_PINION_GEOMETRY_SHIFT

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


if __name__ == "__main__":
  unittest.main()
