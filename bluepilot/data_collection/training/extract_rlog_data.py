#!/usr/bin/env python3
import os
import sys
import json
import logging
import argparse
import numpy as np
import multiprocessing
from tqdm import tqdm
from typing import Optional

# Add openpilot to path
OPENPILOT_DIR = os.getenv("OPENPILOT_DIR", "/data/openpilot")
sys.path.append(OPENPILOT_DIR)

# Import openpilot modules
from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.lib.route import Route, SegmentRange
from openpilot.selfdrive.modeld.constants import ModelConstants
# from openpilot.common.transformations.orientation import rot_from_euler

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("ford_data_extractor")


class FordDataExtractor:
  """Extract Ford lateral control data from route logs"""

  def __init__(self, output_dir: str, min_speed_ms: float = 8.0):
    """
    Initialize the data extractor

    Args:
        output_dir: Directory to save extracted data
        min_speed_ms: Minimum vehicle speed in m/s to include data (default 8.0 m/s ~ 18 mph)
    """
    self.output_dir = output_dir
    self.min_speed_ms = min_speed_ms

    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

  def process_route(self, route_name: str) -> None:
    """
    Process all segments of a route

    Args:
        route_name: Route identifier in format 'dongleId|timestamp'
    """
    try:
      route = Route(route_name)
      segments = route.segments()

      logger.info(f"Processing route {route_name} with {len(segments)} segments")

      # Process each segment in the route
      for i, segment in enumerate(segments):
        logger.info(f"Processing segment {i + 1}/{len(segments)}")
        segment_path = segment[0]

        # Process the segment
        data = self._process_segment(route_name, segment_path, i)

        # Save data for this segment
        if data:
          output_file = os.path.join(self.output_dir, f"{route_name.replace('|', '_')}_{i}.jsonl")
          with open(output_file, 'w') as f:
            for item in data:
              f.write(json.dumps(item) + '\n')
          logger.info(f"Saved {len(data)} samples to {output_file}")
        else:
          logger.warning(f"No valid data found in segment {i}")

    except Exception as e:
      logger.error(f"Error processing route {route_name}: {e}")

  def _process_segment(self, route_name: str, segment_path: str, segment_idx: int) -> list[dict[str, any]]:
    """
    Process a single segment to extract lateral control data

    Args:
        route_name: Route identifier
        segment_path: Path to the segment data
        segment_idx: Segment index

    Returns:
        List of data points with input features and target signals
    """
    # Initialize data storage
    car_states = {}
    control_signals = {}
    model_data = {}

    # Load the log
    try:
      sr = SegmentRange(route_name, [segment_idx])
      log_path = sr.rlog_paths[0]
      logger.info(f"Reading log from {log_path}")

      lr = LogReader(log_path)

      # Process all messages
      for msg in tqdm(lr):
        try:
          # Extract car state data
          if msg.which() == 'carState':
            cs = msg.carState
            time_ns = msg.logMonoTime

            # Basic vehicle state
            car_states[time_ns] = {
              'timestamp': time_ns,
              'v_ego_raw': float(cs.vEgoRaw),
              'steering_angle_deg': float(cs.steeringAngleDeg),
              'yaw_rate': float(cs.yawRate),
              'steering_pressed': bool(cs.steeringPressed),
              'left_blinker': bool(cs.leftBlinker),
              'right_blinker': bool(cs.rightBlinker),
            }

          # Extract lateral control data - check both message types
          elif msg.which() in ('lateralMotionControl', 'lateralMotionControl2'):
            time_ns = msg.logMonoTime
            if msg.which() == 'lateralMotionControl':
              lmc = msg.lateralMotionControl
              mode = 1 if lmc.active else 0  # Simple mode for old message
            else:
              lmc = msg.lateralMotionControl2
              mode = int(lmc.mode)  # 0=None, 1=Limited, 2=Extended

            # These are our target signals
            control_signals[time_ns] = {
              'timestamp': time_ns,
              'apply_curvature': float(lmc.curvature),
              'desired_curvature_rate': float(lmc.curvatureRate),
              'path_offset': float(lmc.pathOffset),
              'path_angle': float(lmc.pathAngle),
              'ramp_type': int(lmc.rampType),
              'precision_type': int(lmc.precisionType),
              'mode': mode,
              'lat_active': mode > 0,
            }

          # Extract model prediction data
          elif msg.which() == 'modelV2':
            time_ns = msg.logMonoTime
            model = msg.modelV2

            # Extract relevant model data
            position_y = [float(y) for y in model.position.y]
            orientation_x = [float(x) for x in model.orientation.x]
            orientation_y = [float(y) for y in model.orientation.y]
            orientation_z = [float(z) for z in model.orientation.z]

            # Check if we have lane lines
            lane_line_probs = []
            lane_line_stds = []

            if len(model.laneLines) > 0:
              for i in range(len(model.laneLines)):
                lane_line_probs.append(float(model.laneLineProbs[i]))
                lane_line_stds.append(float(model.laneLineStds[i]))

            # Check for lane change data
            lane_change_state = 0
            lane_change_direction = 0

            if hasattr(model, 'meta'):
              if hasattr(model.meta, 'laneChangeState'):
                lane_change_state = int(model.meta.laneChangeState)
              if hasattr(model.meta, 'laneChangeDirection'):
                lane_change_direction = int(model.meta.laneChangeDirection)

            model_data[time_ns] = {
              'timestamp': time_ns,
              'position_y': position_y,
              'orientation_x': orientation_x,
              'orientation_y': orientation_y,
              'orientation_z': orientation_z,
              'lane_line_probs': lane_line_probs,
              'lane_line_stds': lane_line_stds,
              'lane_change_state': lane_change_state,
              'lane_change_direction': lane_change_direction,
              'model_path_offset': float(position_y[0]) if len(position_y) > 0 else 0.0,
              'desired_curvature': float(model.lateralPlan.curvature) if hasattr(model, 'lateralPlan') else 0.0,
            }

        except Exception as e:
          logger.warning(f"Error processing message: {e}")
          continue

      # Match and process all data
      return self._match_and_process_data(car_states, control_signals, model_data, route_name)

    except Exception as e:
      logger.error(f"Error reading log: {e}")
      return []

  def _match_and_process_data(
    self, car_states: dict[int, dict], control_signals: dict[int, dict], model_data: dict[int, dict], fingerprint: str
  ) -> list[dict[str, any]]:
    """
    Match timestamps and create complete data points

    Args:
        car_states: Dictionary of car state data by timestamp
        control_signals: Dictionary of control signal data by timestamp
        model_data: Dictionary of model data by timestamp
        fingerprint: Vehicle fingerprint

    Returns:
        List of complete data points
    """
    # Create matched data points
    complete_data = []

    # Get all unique timestamps
    all_timestamps = sorted(set(list(car_states.keys()) + list(control_signals.keys()) + list(model_data.keys())))

    # Find the closest timestamps for each data type
    for ts in all_timestamps:
      # Skip if we don't have control signals (our targets)
      if not self._find_closest_ts(control_signals, ts):
        continue

      # Find closest car state and model data
      cs_ts = self._find_closest_ts(car_states, ts)
      model_ts = self._find_closest_ts(model_data, ts)

      # Skip if we don't have all required data
      if not cs_ts or not model_ts:
        continue

      # Get the actual data
      cs_data = car_states[cs_ts]
      control_data = control_signals[ts]
      model_item = model_data[model_ts]

      # Skip if speed is too low
      if cs_data['v_ego_raw'] < self.min_speed_ms:
        continue

      # Calculate current curvature from yaw rate and speed
      current_curvature = -cs_data['yaw_rate'] / max(cs_data['v_ego_raw'], 0.1)

      # Determine if this is a human turn
      human_turn = cs_data['steering_pressed'] and abs(cs_data['steering_angle_deg']) > 45

      # Extract model path offset at 0.2s ahead
      model_path_offset = 0.0
      if len(model_item['position_y']) > 0:
        model_path_offset = np.interp(0.2, ModelConstants.T_IDXS, model_item['position_y'])

      # Create a complete data point
      data_point = {
        # Metadata
        'timestamp': ts,
        'fingerprint': fingerprint,
        'human_turn': human_turn,
        # Input features
        'v_ego_raw': cs_data['v_ego_raw'],
        'steering_angle_deg': cs_data['steering_angle_deg'],
        'yaw_rate': cs_data['yaw_rate'],
        'current_curvature': current_curvature,
        'desired_curvature': model_item['desired_curvature'],
        'model_path_offset': model_path_offset,
        'left_blinker': cs_data['left_blinker'],
        'right_blinker': cs_data['right_blinker'],
        'lane_change_state': model_item['lane_change_state'],
        'lane_change_direction': model_item['lane_change_direction'],
        # Target outputs (from actual signals)
        'apply_curvature': control_data['apply_curvature'],
        'desired_curvature_rate': control_data['desired_curvature_rate'],
        'path_offset': control_data['path_offset'],
        'path_angle': control_data['path_angle'],
        'ramp_type': control_data['ramp_type'],
        'precision_type': control_data['precision_type'],
        'mode': control_data['mode'],
      }

      # Add lane line probabilities if available
      if model_item['lane_line_probs']:
        for i, prob in enumerate(model_item['lane_line_probs']):
          data_point[f'lane_line_{i}_prob'] = prob
          data_point[f'lane_line_{i}_std'] = model_item['lane_line_stds'][i]

      complete_data.append(data_point)

    logger.info(f"Created {len(complete_data)} complete data points")
    return complete_data

  def _find_closest_ts(self, data_dict: dict[int, dict], target_ts: int, max_diff_ns: int = 100000000) -> Optional[int]:
    """
    Find the closest timestamp in a dictionary

    Args:
        data_dict: Dictionary with timestamps as keys
        target_ts: Target timestamp to find
        max_diff_ns: Maximum difference in nanoseconds (default 100ms)

    Returns:
        Closest timestamp or None if none found within max_diff
    """
    if target_ts in data_dict:
      return target_ts

    closest_ts = None
    min_diff = float('inf')

    for ts in data_dict.keys():
      diff = abs(ts - target_ts)
      if diff < min_diff:
        min_diff = diff
        closest_ts = ts

    # Only return if within max difference
    if min_diff <= max_diff_ns:
      return closest_ts
    return None

  def process_routes_batch(self, route_names: list[str], num_workers: int = None) -> None:
    """
    Process multiple routes in parallel

    Args:
        route_names: List of route names to process
        num_workers: Number of worker processes (default: CPU count)
    """
    if num_workers is None:
      num_workers = max(1, multiprocessing.cpu_count() - 1)

    logger.info(f"Processing {len(route_names)} routes with {num_workers} workers")

    with multiprocessing.Pool(num_workers) as pool:
      pool.map(self.process_route, route_names)


def main():
  parser = argparse.ArgumentParser(description='Extract Ford NNLC training data from rlogs')
  parser.add_argument('--output_dir', type=str, default='/data/bluepilot_data', help='Directory to save extracted data')
  parser.add_argument('--routes_file', type=str, default=None, help='File containing list of routes to process (one per line)')
  parser.add_argument('--route', type=str, default=None, help='Single route to process')
  parser.add_argument('--min_speed', type=float, default=8.0, help='Minimum vehicle speed in m/s to include data')
  parser.add_argument('--workers', type=int, default=None, help='Number of worker processes')

  args = parser.parse_args()

  extractor = FordDataExtractor(args.output_dir, args.min_speed)

  # Process routes
  if args.route:
    # Process a single route
    extractor.process_route(args.route)
  elif args.routes_file:
    # Process multiple routes from a file
    with open(args.routes_file, 'r') as f:
      routes = [line.strip() for line in f if line.strip()]
    extractor.process_routes_batch(routes, args.workers)
  else:
    logger.error("Either --route or --routes_file must be specified")
    sys.exit(1)

  logger.info("Completed data extraction")


if __name__ == "__main__":
  main()
