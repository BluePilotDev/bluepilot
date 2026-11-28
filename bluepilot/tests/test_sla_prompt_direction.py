"""Speed Limit Assist's comma four prompt names the button that moves the set speed toward the limit (#272).

vCruiseCluster and controlsState.deprecated.vCruise are km/h; speedLimitFinalLast is m/s."""
import unittest
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

from opendbc.car.structs import car
from openpilot.common.parameterized import parameterized
from openpilot.sunnypilot.selfdrive.selfdrived import events

MPH_TO_KPH = 1.609344
MPH_TO_MS = 0.44704
KPH_TO_MS = 1 / 3.6
PLUS = "Press + to confirm speed limit"
MINUS = "Press - to confirm speed limit"


def prompt(set_kph, limit_ms, metric, cluster=True, mici=True):
  CP = car.CarParams(openpilotLongitudinalControl=False, pcmCruise=False)
  CS = car.CarState(vCruiseCluster=set_kph if cluster else 0.0)
  sm: Any = {  # stands in for the SubMaster
    'controlsState': SimpleNamespace(deprecated=SimpleNamespace(vCruise=set_kph)),
    'longitudinalPlanSP': SimpleNamespace(speedLimit=SimpleNamespace(resolver=SimpleNamespace(speedLimitFinalLast=limit_ms))),
  }
  with patch.object(events, 'IS_MICI', mici):
    return events.speed_limit_pre_active_alert(CP, CS, sm, metric, 0, None)


class TestSpeedLimitPromptDirection(unittest.TestCase):
  @parameterized.expand([
    ("mph_below", 35 * MPH_TO_KPH, 45 * MPH_TO_MS, False, PLUS),
    ("mph_above", 45 * MPH_TO_KPH, 35 * MPH_TO_MS, False, MINUS),
    ("mph_equal", 45 * MPH_TO_KPH, 45 * MPH_TO_MS, False, ""),
    ("kph_below", 60, 70 * KPH_TO_MS, True, PLUS),
    ("kph_above", 70, 60 * KPH_TO_MS, True, MINUS),
    ("kph_equal", 70, 70 * KPH_TO_MS, True, ""),
  ], ids=lambda name, *_: name)
  def test_prompt(self, _name, set_kph, limit_ms, metric, expected):
    for cluster in (True, False):  # False: vCruiseCluster is 0.0, the set speed comes from controlsState
      with self.subTest(cluster=cluster):
        self.assertEqual(prompt(set_kph, limit_ms, metric, cluster=cluster).alert_text_1, expected)

  def test_no_prompt_off_the_comma_four(self):
    alert = prompt(35 * MPH_TO_KPH, 45 * MPH_TO_MS, False, mici=False)
    self.assertEqual(alert.alert_size, events.AlertSize.none)
    self.assertEqual(alert.alert_text_1, "")


if __name__ == "__main__":
  unittest.main()
