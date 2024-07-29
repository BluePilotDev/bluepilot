#!/usr/bin/env python3
import os
import numpy as np
from cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.lib.lateral_planner import LateralPlanner
from openpilot.selfdrive.modeld.custom_model_metadata import CustomModelMetadata, ModelCapabilities
import cereal.messaging as messaging


def plannerd_thread():
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("plannerd got CarParams: %s", CP.carName)

  debug_mode = bool(int(os.getenv("DEBUG", "0")))

  longitudinal_planner = LongitudinalPlanner(CP)

  custom_model_metadata = CustomModelMetadata(params=params, init_only=True)
  model_use_lateral_planner = custom_model_metadata.valid and custom_model_metadata.capabilities & ModelCapabilities.LateralPlannerSolution
  lateral_planner = LateralPlanner(CP, debug=debug_mode, model_use_lateral_planner=model_use_lateral_planner)
  lateral_planner_svs = ['lateralPlanDEPRECATED', 'lateralPlanSPDEPRECATED']

  pm = messaging.PubMaster(['longitudinalPlan', 'longitudinalPlanSP'] + lateral_planner_svs)
  sm = messaging.SubMaster(['carControl', 'carState', 'controlsState', 'radarState', 'modelV2',
                            'longitudinalPlan', 'navInstruction', 'longitudinalPlanSP',
                            'liveMapDataSP', 'e2eLongStateSP', 'controlsStateSP'] + lateral_planner_svs,
                           poll='modelV2', ignore_avg_freq=['radarState'])

  while True:
    sm.update()
    if sm.updated['modelV2']:
      lateral_planner.update(sm)
      lateral_planner.publish(sm, pm)
      longitudinal_planner.update(sm)
      longitudinal_planner.publish(sm, pm)


def main():
  plannerd_thread()


if __name__ == "__main__":
  main()
