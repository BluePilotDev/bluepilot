"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from functools import cache

from openpilot.common.params import Params


def get_minimum_set_speed(is_metric: bool) -> int:
  return 30 if is_metric else 20


def icbm_with_op_long(CP) -> bool:
  """BluePilot: brands where ICBM may also run under openpilot longitudinal.

  On Ford the PCM keeps its own cruise setpoint under openpilot longitudinal and holds the car to
  it, and it still takes Set+/Set- presses. Letting ICBM manage that setpoint there gives the same
  set-speed behavior as ICBM on stock ACC: openpilot keeps the driver's set speed, Speed Limit
  Assist sets it to the limit directly, and the cluster is walked to match. Without it, Speed
  Limit Assist needs the cluster parked at 70/80 mph (120/130 km/h) as a ceiling.
  """
  return CP.brand == "ford"


@cache
def icbm_manages_set_speed_with_op_long(brand: str, op_long: bool) -> bool:
  """True when ICBM is on under openpilot longitudinal (CP_SP.pcmCruiseSpeed is False there).

  For callers that only have CarParams. The ICBM toggle is offroad-only, so one read per process is enough.
  """
  return op_long and brand == "ford" and Params().get_bool("IntelligentCruiseButtonManagement")
