# Credits

The BluePilot Ford lateral library: Ford's four-signal lateral control in curvature and angle
modes, the CAN messages that carry it, and the panda safety that checks it. Everything here is
under the MIT terms in [LICENSE](LICENSE).

## Contributors

- **Alan Polk (`alan-polk`)**: principal author of the curvature and angle controllers, the
  human-turn behavior, the angle-mode side channel and its panda safety, and the lane centering
  trim.
- **tonesto7**: the CAN message builders and BluePilot's early releases.
- **John Christman**: lateral integration, platform data, and the angle-mode anti-stall work.
- **Jacob Neulight**: the steering-pinion curvature measurement and its panda safety, and the
  shadow-curvature measurement.
- **Blue Genie (`blue-genie`)**: the split of BluePilot's lateral control into opendbc that this
  library grew from.
- **ghbarker**: the lane centering trim's confidence blend, fallback, filter and rate limit.
- **Nathan Ingraham**: the angle-mode high-speed damping adjustment.
- **Praeuner**: the angle-mode damping range.
- **OneFast440**: the early Ford safety declarations.
- **nitrobass24**: the library's structure, its inputs and settings plumbing, and the safety tests
  and CI.

## Third-party code

### comma.ai's opendbc

MIT, Copyright (c) 2020, Comma.ai, Inc.

- `can/fordcan_ext.py`: `create_lat_ctl_msg` and `create_lat_ctl2_msg`, comma's Ford lateral
  message builders from `opendbc/car/ford/fordcan.py`, with ramp-type and precision parameters
  added.
- `safety/lateral.h`: the measured-table rate check, curvature-error band and lateral-accel cap,
  from comma's earlier checks in `opendbc/safety/lateral.h`, and `ford_bp_curvature_cmd_checks`,
  adapted from comma's `steer_curvature_cmd_checks`.
- `safety/ford.h`: the rate-of-change shape of the path angle and path offset checks, the
  curvature-error band in the shadow-curvature check, and the message-rate formula.
- `core/lateral_curv_ext.py`: `apply_ford_curvature_limits_ext`, adapted from comma's
  `apply_ford_curvature_limits`, with its road-roll and lateral-accel constants.
- Small pieces: the message packing and the yaw-rate curvature measurement in
  `core/carcontroller_ext.py`, and the model time-index formula in `core/values_ext.py`.
- The BluePilot safety tests start from comma's Ford safety test skeletons.

### StarPilot

MIT, Copyright (c) 2026, firestar5683 and StarPilot contributors

- `core/lane_center_trim.py`: the lane-centering core (the correction formula, lookahead clip,
  lane-line probability, std and width gates, the 0.004 correction limit and the lane-line
  handling), ported from StarPilot's `selfdrive/controls/lib/lane_centering.py`
  (`LaneCenteringController`, originally by jc01rho). The confidence blend, fallback, filter and
  rate limit are BluePilot's.
