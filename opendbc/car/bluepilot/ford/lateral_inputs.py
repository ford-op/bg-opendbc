"""
BluePilot: adapt CarControlSP.lateralInputs to the modelV2 / vehicleParameters shape the lateral
code reads. The fork fills lateralInputs once per frame; nothing here touches messaging.
"""
from types import SimpleNamespace

MODEL_PATH_POINTS = 33  # modelV2 orientationRate / position / laneLines length (ModelConstants.IDX_N)


def lateral_inputs_complete(li) -> bool:
  """True when every array the lateral indexes into has the shape modelV2 always provides."""
  return (len(li.pathYawRate) == MODEL_PATH_POINTS and len(li.pathX) == MODEL_PATH_POINTS and len(li.pathY) == MODEL_PATH_POINTS
          and len(li.laneLineLeftX) == MODEL_PATH_POINTS and len(li.laneLineLeftY) == MODEL_PATH_POINTS
          and len(li.laneLineRightX) == MODEL_PATH_POINTS and len(li.laneLineRightY) == MODEL_PATH_POINTS
          and len(li.laneLineProbs) >= 3 and len(li.laneLineStds) >= 3)


class _LaneLine:
  def __init__(self, x, y):
    self.x = x
    self.y = y


class ModelView:
  """The subset of modelV2 the lateral code uses, with the same attribute paths."""

  def __init__(self, li):
    self.orientationRate = SimpleNamespace(z=li.pathYawRate)
    self.position = SimpleNamespace(x=li.pathX, y=li.pathY)
    # indices match modelV2.laneLines: 1 = left, 2 = right; 0 and 3 are not used
    self.laneLines = [_LaneLine([], []),
                      _LaneLine(li.laneLineLeftX, li.laneLineLeftY),
                      _LaneLine(li.laneLineRightX, li.laneLineRightY),
                      _LaneLine([], [])]
    self.laneLineProbs = li.laneLineProbs
    self.laneLineStds = li.laneLineStds
    self.meta = SimpleNamespace(laneChangeState=li.laneChangeState, laneChangeDirection=li.laneChangeDirection)


class VehicleParamsView:
  """The subset of vehicleParameters the lateral code uses."""

  def __init__(self, li):
    self.steerRatio = li.steerRatio
    self.stiffnessFactor = li.stiffnessFactor
    self.roll = li.roll
    self.angleOffsetDeg = li.angleOffsetDeg
