import time
import numpy as np
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.numpy_fast import interp
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import N as LAT_MPC_N
from openpilot.selfdrive.controls.lib.lane_planner import LanePlanner, TRAJECTORY_SIZE
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED, get_speed_error, get_road_edge
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper

import cereal.messaging as messaging
from cereal import log

LaneChangeState = log.LaneChangeState


PATH_COST = 1.0
LATERAL_MOTION_COST = 0.11
LATERAL_ACCEL_COST = 0.0
LATERAL_JERK_COST = 0.04
# Extreme steering rate is unpleasant, even
# when it does not cause bad jerk.
# TODO this cost should be lowered when low
# speed lateral control is stable on all cars
STEERING_RATE_COST = 700.0


class LateralPlanner:
  def __init__(self, CP, debug=False, model_use_lateral_planner=False):
    self.LP = LanePlanner()
    self.DH = DesireHelper()

    # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.velocity_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.plan_yaw_rate = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.y_pts = np.zeros((TRAJECTORY_SIZE,))
    self.v_plan = np.zeros((TRAJECTORY_SIZE,))
    self.x_sol = np.zeros((TRAJECTORY_SIZE, 4), dtype=np.float32)
    self.v_ego = MIN_SPEED
    self.l_lane_change_prob = 0.0
    self.r_lane_change_prob = 0.0
    self.d_path_w_lines_xyz = np.zeros((TRAJECTORY_SIZE, 3))

    self.debug_mode = debug

    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))

    self.param_s = Params()
    self.dynamic_lane_profile = int(self.param_s.get("DynamicLaneProfile", encoding="utf8"))
    self.dynamic_lane_profile_status = True
    self.dynamic_lane_profile_status_buffer = False

    self.standstill_elapsed = 0.0
    self.standstill = False

    self.vision_curve_laneless = self.param_s.get_bool("VisionCurveLaneless")

    self.road_edge = False
    self.edge_toggle = self.param_s.get_bool("RoadEdge")

    self.param_read_counter = 0
    self.read_param()

    self.model_use_lateral_planner = model_use_lateral_planner

  def read_param(self):
    self.dynamic_lane_profile = int(self.param_s.get("DynamicLaneProfile", encoding='utf8'))
    if self.param_read_counter % 50 == 0:
      self.vision_curve_laneless = self.param_s.get_bool("VisionCurveLaneless")
      self.edge_toggle = self.param_s.get_bool("RoadEdge")
    self.param_read_counter += 1

  def reset_mpc(self, x0=None):
    if x0 is None:
      x0 = np.zeros(4)
    self.x0 = x0
    self.lat_mpc.reset(x0=self.x0)

  def update(self, sm):
    self.read_param()
    self.standstill = sm['carState'].standstill
    # clip speed , lateral planning is not possible at 0 speed
    measured_curvature = sm['controlsState'].curvature
    v_ego_car = sm['carState'].vEgo

    # Parse model predictions
    md = sm['modelV2']

    if self.model_use_lateral_planner:
      self.LP.parse_model(md)
      if len(md.position.x) == TRAJECTORY_SIZE and (len(md.orientation.x) == TRAJECTORY_SIZE or
                                                    (len(md.velocity.x) == TRAJECTORY_SIZE and len(md.lateralPlannerSolutionDEPRECATED.x) == TRAJECTORY_SIZE)):
        if len(md.orientation.x) == TRAJECTORY_SIZE:
          self.t_idxs = np.array(md.position.t)
          self.plan_yaw = np.array(md.orientation.z)
          self.plan_yaw_rate = np.array(md.orientationRate.z)
        if len(md.velocity.x) == TRAJECTORY_SIZE and len(md.lateralPlannerSolutionDEPRECATED.x) == TRAJECTORY_SIZE:
          self.x_sol = np.column_stack([md.lateralPlannerSolutionDEPRECATED.x, md.lateralPlannerSolutionDEPRECATED.y, md.lateralPlannerSolutionDEPRECATED.yaw, md.lateralPlannerSolutionDEPRECATED.yawRate])
        self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
        self.velocity_xyz = np.column_stack([md.velocity.x, md.velocity.y, md.velocity.z])
        car_speed = np.linalg.norm(self.velocity_xyz, axis=1) - get_speed_error(md, v_ego_car)
        self.v_plan = np.clip(car_speed, MIN_SPEED, np.inf)
        self.v_ego = self.v_plan[0]

      # Lane change logic
      lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
      self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, model_data=md)

      # Turn off lanes during lane change
      if self.DH.desire == log.Desire.laneChangeRight or self.DH.desire == log.Desire.laneChangeLeft:
        self.LP.lll_prob *= self.DH.lane_change_ll_prob
        self.LP.rll_prob *= self.DH.lane_change_ll_prob
      self.d_path_w_lines_xyz = self.LP.get_d_path(self.v_ego, self.t_idxs, self.path_xyz)

      low_speed = v_ego_car < 10 * CV.MPH_TO_MS

      if not self.get_dynamic_lane_profile(sm['longitudinalPlanSP']) and not low_speed:
        self.path_xyz = self.d_path_w_lines_xyz
        self.dynamic_lane_profile_status = False
      else:
        self.path_xyz[:, 1] += self.LP.path_offset
        self.dynamic_lane_profile_status = True

      if not self.dynamic_lane_profile_status:
        self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                                 LATERAL_ACCEL_COST, LATERAL_JERK_COST,
                                 STEERING_RATE_COST)

        y_pts = self.path_xyz[:LAT_MPC_N+1, 1]
        heading_pts = self.plan_yaw[:LAT_MPC_N+1]
        yaw_rate_pts = self.plan_yaw_rate[:LAT_MPC_N+1]
        self.y_pts = y_pts

        assert len(y_pts) == LAT_MPC_N + 1
        assert len(heading_pts) == LAT_MPC_N + 1
        assert len(yaw_rate_pts) == LAT_MPC_N + 1
        lateral_factor = np.clip(self.factor1 - (self.factor2 * self.v_plan**2), 0.0, np.inf)
        p = np.column_stack([self.v_plan, lateral_factor])
        self.lat_mpc.run(self.x0,
                         p,
                         y_pts,
                         heading_pts,
                         yaw_rate_pts)
        # init state for next iteration
        # mpc.u_sol is the desired second derivative of psi given x0 curv state.
        # with x0[3] = measured_yaw_rate, this would be the actual desired yaw rate.
        # instead, interpolate x_sol so that x0[3] is the desired yaw rate for lat_control.
        self.x0[3] = interp(DT_MDL, self.t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])

        #  Check for infeasible MPC solution
        mpc_nans = np.isnan(self.lat_mpc.x_sol[:, 3]).any()
        t = time.monotonic()
        if mpc_nans or self.lat_mpc.solution_status != 0:
          self.reset_mpc()
          self.x0[3] = measured_curvature * self.v_ego
          if t > self.last_cloudlog_t + 5.0:
            self.last_cloudlog_t = t
            cloudlog.warning("Lateral mpc - nan: True")

        if self.lat_mpc.cost > 1e6 or mpc_nans:
          self.solution_invalid_cnt += 1
        else:
          self.solution_invalid_cnt = 0

    if not self.model_use_lateral_planner:
      self.road_edge = get_road_edge(sm['carState'], md, self.edge_toggle)

  def get_dynamic_lane_profile(self, longitudinal_plan_sp):
    if self.dynamic_lane_profile == 1:
      return True
    elif self.dynamic_lane_profile == 0:
      return False
    elif self.dynamic_lane_profile == 2:
      # laneless while lane change in progress
      if self.DH.lane_change_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing):
        return True
      # only while lane change is off
      elif self.DH.lane_change_state == LaneChangeState.off:
        # laneline probability too low, we switch to laneless mode
        if (self.LP.lll_prob + self.LP.rll_prob) / 2 < 0.3 \
          or ((longitudinal_plan_sp.visionCurrentLatAcc > 1.0 or longitudinal_plan_sp.visionMaxPredLatAcc > 1.4)
           and self.vision_curve_laneless):
          self.dynamic_lane_profile_status_buffer = True
        if (self.LP.lll_prob + self.LP.rll_prob) / 2 > 0.5 \
          and ((longitudinal_plan_sp.visionCurrentLatAcc < 0.6 and longitudinal_plan_sp.visionMaxPredLatAcc < 0.7)
           or not self.vision_curve_laneless):
          self.dynamic_lane_profile_status_buffer = False
        if self.dynamic_lane_profile_status_buffer:  # in buffer mode, always laneless
          return True
    return False

  def publish(self, sm, pm):
    plan_solution_valid = self.solution_invalid_cnt < 2
    plan_send = messaging.new_message('lateralPlanDEPRECATED')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlanDEPRECATED = plan_send.lateralPlanDEPRECATED
    lateralPlanDEPRECATED.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlanDEPRECATED.dPathPoints = self.path_xyz[:,1].tolist() if self.dynamic_lane_profile_status else self.y_pts.tolist()
    lateralPlanDEPRECATED.psis = self.x_sol[0:CONTROL_N, 2].tolist() if self.dynamic_lane_profile_status else self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()

    lateralPlanDEPRECATED.curvatures = (self.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist() if self.dynamic_lane_profile_status else (self.lat_mpc.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist()
    lateralPlanDEPRECATED.curvatureRates = [float(0) for _ in range(CONTROL_N-1)] if self.dynamic_lane_profile_status else [float(x.item() / self.v_ego) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0] # TODO: unused

    lateralPlanDEPRECATED.mpcSolutionValid = bool(1) if self.dynamic_lane_profile_status else bool(plan_solution_valid)
    lateralPlanDEPRECATED.solverExecutionTime = 0.0 if self.dynamic_lane_profile_status else self.lat_mpc.solve_time
    if self.debug_mode:
      lateralPlanDEPRECATED.solverState.x = self.x_sol.tolist() if self.dynamic_lane_profile_status else self.lat_mpc.x_sol.tolist()
      if not self.dynamic_lane_profile_status:
        lateralPlanDEPRECATED.solverCost = self.lat_mpc.cost
        lateralPlanDEPRECATED.solverState = log.LateralPlan.SolverState.new_message()
        lateralPlanDEPRECATED.solverState.u = self.lat_mpc.u_sol.flatten().tolist()

    lateralPlanDEPRECATED.desire = self.DH.desire
    lateralPlanDEPRECATED.useLaneLines = not self.dynamic_lane_profile_status
    lateralPlanDEPRECATED.laneChangeState = self.DH.lane_change_state
    lateralPlanDEPRECATED.laneChangeDirection = self.DH.lane_change_direction

    pm.send('lateralPlanDEPRECATED', plan_send)

    plan_sp_send = messaging.new_message('lateralPlanSPDEPRECATED')
    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlanSPDEPRECATED = plan_sp_send.lateralPlanSPDEPRECATED

    lateralPlanSPDEPRECATED.laneWidth = float(self.LP.lane_width)
    lateralPlanSPDEPRECATED.lProb = float(self.LP.lll_prob)
    lateralPlanSPDEPRECATED.rProb = float(self.LP.rll_prob)
    lateralPlanSPDEPRECATED.dProb = float(self.LP.d_prob)

    lateralPlanSPDEPRECATED.dynamicLaneProfile = int(self.dynamic_lane_profile)
    lateralPlanSPDEPRECATED.dynamicLaneProfileStatus = bool(self.dynamic_lane_profile_status)

    lateralPlanSPDEPRECATED.laneChangeEdgeBlockDEPRECATED = self.road_edge

    if self.standstill:
      self.standstill_elapsed += DT_MDL
    else:
      self.standstill_elapsed = 0.0
    lateralPlanSPDEPRECATED.standstillElapsed = int(self.standstill_elapsed)

    pm.send('lateralPlanSPDEPRECATED', plan_sp_send)
