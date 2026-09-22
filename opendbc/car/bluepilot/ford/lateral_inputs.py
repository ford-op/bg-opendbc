"""
BluePilot: adapt CarControlSP.lateralInputs to the modelV2 / vehicleParameters shape the lateral
code reads. The fork fills lateralInputs once per frame; nothing here touches messaging.
"""
from types import SimpleNamespace


class _LaneLine:
  def __init__(self, x, y):
    self.x = list(x)
    self.y = list(y)


class ModelView:
  """The subset of modelV2 the lateral code uses, with the same attribute paths."""

  def __init__(self, li):
    self.orientationRate = SimpleNamespace(z=list(li.pathYawRate))
    self.position = SimpleNamespace(x=list(li.pathX), y=list(li.pathY))
    # indices match modelV2.laneLines: 1 = left, 2 = right; 0 and 3 are not used
    self.laneLines = [_LaneLine([], []),
                      _LaneLine(li.laneLineLeftX, li.laneLineLeftY),
                      _LaneLine(li.laneLineRightX, li.laneLineRightY),
                      _LaneLine([], [])]
    self.laneLineProbs = list(li.laneLineProbs)
    self.laneLineStds = list(li.laneLineStds)
    self.meta = SimpleNamespace(laneChangeState=li.laneChangeState, laneChangeDirection=li.laneChangeDirection)


class VehicleParamsView:
  """The subset of vehicleParameters the lateral code uses."""

  def __init__(self, li):
    self.steerRatio = li.steerRatio
    self.stiffnessFactor = li.stiffnessFactor
    self.roll = li.roll
    self.angleOffsetDeg = li.angleOffsetDeg
