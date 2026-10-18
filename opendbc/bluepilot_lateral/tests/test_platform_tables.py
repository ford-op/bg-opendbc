import unittest

from opendbc.car.ford.values import CAR
from opendbc.bluepilot_lateral.core.values_ext import (
  ANGLE_GAIN_CANFD_BOF_CARS, ANGLE_GAIN_CANFD_SUV_CARS, FORD_HOST_ONLY_PLATFORMS, FORD_PINION_GEOMETRY_INDEX,
  FORD_PINION_WHEELBASE,
)

TABLES = {
  "FORD_PINION_GEOMETRY_INDEX": FORD_PINION_GEOMETRY_INDEX,
  "FORD_PINION_WHEELBASE": FORD_PINION_WHEELBASE,
  "ANGLE_GAIN_CANFD_BOF_CARS": ANGLE_GAIN_CANFD_BOF_CARS,
  "ANGLE_GAIN_CANFD_SUV_CARS": ANGLE_GAIN_CANFD_SUV_CARS,
}


class TestPlatformTables(unittest.TestCase):
  def test_names_are_known_platforms(self):
    """Every per-platform row is keyed by a plain platform name, so a host without that platform
    imports fine, and the name is a Ford platform here or a declared host-only one: a typo would
    otherwise silently drop the platform's row."""
    known = {str(c) for c in CAR} | FORD_HOST_ONLY_PLATFORMS
    for table, names in TABLES.items():
      for name in names:
        with self.subTest(table=table, name=name):
          self.assertIs(type(name), str)
          self.assertIn(name, known)

  def test_host_only_platforms_are_not_here(self):
    self.assertFalse(FORD_HOST_ONLY_PLATFORMS & {str(c) for c in CAR})


if __name__ == "__main__":
  unittest.main()
