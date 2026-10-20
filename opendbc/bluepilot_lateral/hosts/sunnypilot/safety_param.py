"""
BluePilot: the startup safety bits onto sunnypilot's CarParamsSP (HOSTS.md). Its own module, so car
init imports only values_ext, not the whole lateral.
"""
from opendbc.car import structs
from opendbc.bluepilot_lateral.core.values_ext import ford_safety_param


def init_ford_safety_param_sp(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params_dict: dict[str, str]) -> None:
  """Called from sunnypilot's setup_interfaces at car init. CP_SP.safetyParam reaches the panda as
  current_safety_param_sp (USB 0xdf)."""
  CP_SP.safetyParam |= ford_safety_param(CP, params_dict)
