# Lane_Assist_Data1: BluePilot's angle-mode side channel

In angle mode the curvature signal in `LateralMotionControl2` is held at 0 and path angle does the
steering, so the panda has no commanded curvature to check against the car's measured curvature.
BluePilot sends that curvature, the *shadow curvature*, plus an "angle mode engaged" flag, in bits of
`Lane_Assist_Data1` (0x3CA) that carry no DBC signal. openpilot sends this message itself at 33 Hz,
and the panda reads it in the TX hook as it goes out. A separate message was tried first: the panda
never receives its own TX, and an RX check for it blocked controls car-wide (2026-07-09).

Packed by `create_lka_msg` in `fordcan_ext.py`; read by `ford_bp_lka_tx` in `../safety/ford.h`. The
byte positions, mask and scale are named once on each side (`BP_LKA_*` in `fordcan_ext.py`,
`FORD_BP_LKA_*` in `../safety/ford_declarations.h`), and `../tests/test_lane_assist_data1.py` checks
that they agree with each other and with the DBC.

## Layout

Bit numbers are within the byte, 7 = most significant. Stock signals are from `ford_lincoln_base_pt.dbc`.

| byte | bits | stock Ford signal | BluePilot |
|---|---|---|---|
| 0 | 7-5 | `LkaActvStats_D2_Req` | |
| 0 | 4-2 | `LdwActvStats_D_Req` | |
| 0 | 1-0 | `LdwActvIntns_D_Req` | |
| 1-2 | 12 bits | `LaCurvature_No_Calc` | |
| 2-3 | 12 bits | `LaRefAng_No_Req` | |
| 4 | 7 | `LaRampType_B_Req` | |
| 4 | 6-5 | `LkaDrvOvrrd_D_Rq` | |
| 4 | 4-1 | none | free |
| 4 | 0 | none | **angle mode engaged** |
| 5-6 | all | none | **shadow curvature** |
| 7 | all | none | free |

## Fields

- **Angle mode engaged** (byte 4 bit 0): 1 while angle mode is the selected lateral mode and
  BluePilot lateral is on.
- **Shadow curvature** (bytes 5-6): int16, big-endian, 1e-6 1/m per unit, clamped to
  -32768..32767 (about ±0.0328 1/m). It is the curvature angle mode's path angle was derived from,
  after the same error clip curvature mode applies; while the driver is pressing, inactive, or
  during a human-turn or stall-blip pulse, it is the car's measured curvature instead. It is sent
  negated, in Ford's sign convention, like the curvature and path angle in `LateralMotionControl2`.

## What the panda does

- It latches both fields from every `Lane_Assist_Data1` frame that goes through its TX hook,
  including one it blocks for another reason.
- **Angle mode engaged widens the path-angle range.** While the flag is set, the lateral command's
  path angle may use the DBC's full range (-0.5 to 0.5235 rad); otherwise it is held to ±0.25 rad
  (`ford_lmc_checks`). The flag alone does it: curvature 0 without the flag keeps the tight range.
- **The shadow curvature check.** For a lateral command (`LateralMotionControl` on CAN or
  `LateralMotionControl2` on CAN FD) with curvature 0, steering enabled and the flag set, the
  panda scales the shadow curvature by 0.05 to curvature CAN units (2e-5 1/m, truncated toward
  zero) and, above 10 m/s, blocks the frame if it is outside the measured curvature ± the curvature
  error band + 1 (`ford_shadow_curvature_error_check`). It has no rate limit: path angle has its own.
- `ford_bp_init` clears the flag.

## Not here yet

A protocol-version byte and a shared bit allocation for other forks (StarPilot) are Phase 3 (the StarPilot host issue, #16):
they change bytes on the wire.
