# The clean Ford lateral build: developer guide

For people who work on this code and need to know how the Ford steering works and why it was built
this way. You don't need to know Ford steering already: section 1.1 defines the terms the rest uses.

- **Section 1, the high level:** the terms, the problem, the goal and the shape of the solution.
- **Section 2, the decisions:** one at a time, each with its problem, options, evidence, choice and cost.
- **Section 3, the low level:** one steering frame end to end, the panda's checks, how a table row is
  made, logs and tests.
- **Section 4:** known gaps, how to extend it, and the path upstream.

The owner's version is [USER_GUIDE.md](USER_GUIDE.md). Installing is in [INSTALL.md](INSTALL.md).

**Sources.** Code claims are as of the submodule commits `clean/ford-lateral` pins: opendbc `3a3df995`
and the library `369ead8`. The code lives in one repository, bluepilot-clean, as three branches (2.12;
INSTALL.md has the address). Research notes are cited by file name, for example `approach-comparison.md`. They are the
project's research notes (to be published), and the scripts and twin they name are in the same set.
Where a claim wasn't checked, it says so.

**Paths.** `L/` is `opendbc_repo/opendbc/bluepilot_lateral/` (the library), `O/` is `opendbc_repo/`,
and `B/` is this repo's root.

## 1. The high level

### 1.1 Terms

- **PSCM, the steering module.** The power steering control module. It owns the rack motor and runs
  Ford's lateral controller. openpilot never commands torque on a Ford: it sends the module a path,
  and the module turns that into a steering-angle target with its own controller, limits and driver
  override.
- **The lane message.** The message Ford's camera sends the module for lane centering:
  LateralMotionControl on CAN, LateralMotionControl2 on CAN FD (the faster bus on newer Fords).
  openpilot sends it in the camera's place. It carries a path of four signals, plus a request mode, a
  ramp type (on some strategies it selects how fast the module ramps its torque out) and a precision bit
  (on some strategies it shortens the preview):

  | Signal | Name here | Meaning |
  |---|---|---|
  | c0 | path offset | lateral distance to the path, m |
  | c1 | path angle | heading of the path relative to the car, rad |
  | c2 | curvature | 1/radius, 1/m |
  | c3 | curvature rate | change of curvature along the path |

  Stock openpilot sends c2 only.
- **Strategy (F188), data file (F10A), calibration (F124).** The module's program, the file of vehicle
  numbers the program uses, and its calibration. Each has a Ford part number such as ML3V-14D003-BD.
  F188, F10A and F124 are the diagnostic identifiers they are read by.
- **Preview distance d and K2.** The module steers by pure pursuit: it aims at a point d metres ahead
  on the path. So a path angle c1 adds `K2·c1` of curvature (K2 in 1/m per rad). d comes from
  per-module tables by speed and curvature.
- **Ford's newer and older lateral firmware.** Ford's modules run one of two generations of lateral
  code. On the newer one `K2 ≈ 2/d` (for example the 2021-23 F-150 and the Lightning); on the older
  one `K2 ≈ 4/d` (the 2020-24 Explorer, the Ranger, the 2022-24 Maverick, LX6C). "Library" on its own
  in this guide means bluepilot-lateral, not these.
- **The twin.** A Python copy of a module's lateral controller, built block for block from the
  decompiled firmware (`twin.md`). Most evidence below that isn't from a drive is from the twin.
- **The panda.** The comma device's safety microcontroller. It checks every message openpilot sends to
  the car and refuses those outside its rules. Its Ford rules are in `O/opendbc/safety/modes/ford.h`.
  At car init it is given a **safety param**, a number whose bits select which rules run; BluePilot's
  bits are in the second one, `safetyParam` of `CarParamsSP` (2.4).
- **Host.** The openpilot fork the library plugs into through an adapter in `L/hosts/`: sunnypilot (which
  BluePilot is built on), IQ.Pilot and StarPilot.
- **bp-dev.** The earlier BluePilot development branch, with its own Ford controller and panda code.
  **7.1** is BluePilot 7.1, whose steering code was moved into the bluepilot-lateral library this build
  starts from (2.12).
- **lagd.** openpilot's delay learner. It estimates the lateral delay from driving, and the driving
  model plans ahead by that delay.

### 1.2 The problem

The same openpilot command gives different results on different Fords. The research found that most of
the variation is **between vehicles, not within one truck**:

- **Each module runs its own strategy with its own data file.** One program serves several vehicle
  lines, one openpilot platform covers different modules, and Ford changes both over the air and at the
  dealer (`strategy-table.md`, `cfg-diff.md`).
- **Below about 80 km/h the binding limit is the module's own rate limit on its copy of c2:**
  0.003 1/m per second on the decoded F-150-family strategies. That is 0.2 m/s³ of lateral jerk at
  30 km/h. No sender that uses curvature alone can follow a town curve that builds faster than that,
  whatever its lag or outer loop (`approach-comparison.md` §1-2, `firmware-core.md` §4.2).
- **The module's path law is pure pursuit at its own preview distance.** It steers for
  `c2 + K2(d)·c1 + K1(d)·(c0 + I)`, where I is its integrator on the path offset. K1 and K2 are pure-pursuit gains on offset
  and angle (K2 as in 1.1), and d comes from per-module tables (`firmware-core.md` §4.3,
  `explorer-firmware.md`, `rl14-firmware.md` §5). bp-dev's angle mode put the planned curvature κ times speed v
  into c1, scaled by hand-tuned per-speed gains. Those gains turned out to be, in effect, one truck's
  1/K2 (`twin.md` §1 item 5, §7), so they fit that truck only.
- **The module steers to its yaw-rate signal.** A sensor offset of 0.1 °/s is a standing curvature
  error the module never removes (`twin.md` §1 item 2). comma's stock controller, and bp-dev after it,
  clip the command to measured curvature ±0.002 1/m, matching the panda's **band** (comma's check that
  refuses a command further than that from measured curvature). That ties the command to the biased
  measurement.
- **bp-dev's panda had a latch.** It switched every check off for 3 s after any zero frame, and it let
  curvature move at up to 7.3 m/s³, twice comma's limit (`change-analysis.md` §0, §1.2).

### 1.3 The goal

Model the module's own steering closely enough that every supported Ford behaves the same, in one
clean branch of the control code with the workarounds removed, and with one setup for every Ford that
can be supported. The logic is meant to be fit for upstream; the packaging isn't yet (4.3).

### 1.4 The shape of the solution

1. **One controller, on curvature,** fed by `CarControl.actuators.curvature` alone. No per-car tuning
   settings.
2. **A table keyed on the module's own answers:** exact (F188, F10A) pairs, read fresh at every start.
   A row carries that module's constants. A module without a row gets the **fallback**: comma's stock
   command without its clip to measured curvature.
   With path angle off, a row can change up to three things against the fallback, where it has the
   value: the lateral-acceleration limit (2.9), lagd's starting delay (2.8), and the ramp type (2 on
   most rows, 0 on the fallback and BC). The health check also delays the command by the row's lag (2.7).
3. **Path angle (approach A′)** on rows whose preview is known. Curvature moves at the module's rate,
   and path angle carries what that rate still holds back, in the module's own K2. It is a per-row
   switch, **off on every row** in this build.
4. **The panda bounds the command by lateral acceleration and jerk** (comma's numbers), on the
   curvature signal alone and on the implied curvature at the highest K2 any module has. A startup bit
   allows path angle only for a row that has it on.
5. **A read-only health check** and a **Steering Support row** in the settings that say how the car is
   steered and why.

## 2. The decisions

### 2.1 Curvature base, path angle as a per-row switch (A′)

**Problem.** Curvature alone is slow in town because of the module's c2 rate (1.2). bp-dev's answer,
angle mode, behaved differently on every vehicle line.

**Background.** comma moved Ford to curvature in 2023 as the more universal interface. The BluePilot
dev group's 50,000+ miles of path-angle-only control show that angle control without the module's
curvature filter tracks better. The research explains both: c2 goes through the module's rate limit,
and c1 doesn't, but c1's effect depends on each module's K2.

**Options.** `approach-comparison.md` §3 scores 17 approaches (A to Q) on native feel, authority across
speeds, sameness across Fords, fit with openpilot, robustness and cost. The main ones:

- **A, the earlier plan:** curvature plus a lag value and c0/c1 lane positioning per row. c0 feeds the
  module's integrator, which never forgets while engaged (`twin.md` §1 item 6), and its gains differ
  3 to 10 times between trucks.
- **B, the full path as Ford's camera sends it,** relative to the lane. That replaces openpilot's plan
  with Ford's soft highway lane follower, which runs 0.6 to 2 m wide in town curves in the twin.
- **D, bp-dev's modes today.** **N, path angle only, scaled by the table:** a table error becomes a
  steady-state gain error.
- **F (an interceptor), H (the park-assist angle request at speed) and K (changing the module's
  tables)** all leave Ford's lane-centering interface and fail comma's rules (comma, "Safer control of
  steering", 2020-10-27).
- **I, curve speed** (used in 2.9, alongside A′).
- **A′:** c2 carries the request. c1 carries only `(want − c2) / K2`, the part c2's rate still holds
  back. c0 = c3 = 0.

**Evidence.** In the twin, A′ makes a town-speed step 2 to 3.5 times faster than curvature alone
(`clean-build.md` §13). The steady state is on c2, so the gain stays 1.0 without a table. A K2 that is
wrong by 0.5 to 2.8 times costs at most 0.7 m in a transient, and never the steady state
(`approach-comparison.md` §3, §4). Path angle fades by itself with speed: at 130 km/h comma's jerk limit
already allows less than the module's c2 rate, so path angle adds nothing (`clean-build.md` §13).
Ford's own camera sends c1 all the time, so the module isn't asked for anything new.

**Choice.** A′, because it is the only option that gets past the module's c2 rate in town while keeping
the steady state on curvature, so a wrong K2 costs a transient and never the gain. The lane offset (c0)
is removed for now, because it feeds the module's integrator and its gains differ 3 to 10 times between
trucks. Path angle is a constant in each row (`StrategyRow.path_angle`), off everywhere
(`test_path_angle_is_a_switch_per_row_and_off`).

**Costs.**

- Recorded requests through the F-150 twin make the wheel 1.2 to 2.6 times as busy (more steering
  movement) below 12 m/s as the fallback. Path angle's sign flips 1.7 to 4.4 times a second where it is
  active, even with the dead band that holds small path angles at zero (3.1 step 5; `clean-build.md`
  §14.3). Whether that feels responsive or twitchy is for the first drive to
  show. It is one reason path angle ships off.
- The controller keeps its own copy of the module's preview time. Where the copy differs from the
  module's, it errs to a higher K2: less path angle and a little less tracking, never overshoot
  (`L/core/strategy.py` `Preview`).
- Older-firmware rows hold K2 as 4.17/d, the most the modules' own K2 table (ten points over distance)
  reaches over the rows' distances. Against the twin, the row's gain is 1.001 to 1.045 times the module's, never below
  (`clean-build.md` §5).

### 2.2 Exact F188 + F10A pairs, not families

**Problem.** Which module is this? openpilot's platform name doesn't say. R1MC sits under the 2020-24
Explorer platform, and one program serves several vehicle lines.

**Options.** Key on F188 alone, on an F188 family with a default, or on exact pairs.

**Evidence.** `cfg-diff.md` §11 compared data files from Ford's archive, and files that look alike
differ. TL34-14D007-BDA (the 157" F-150) has other state-machine thresholds. SL3V-14D007-BC (the 2024
Raptor) differs in 18 of 70 path values, and RL14-14D007-AE/-EE/-GE (Expedition and Navigator) in 19 of
70. The 2022-24 Maverick and the 2021-24 Mach-E run the same control law with their own numbers. The
2025 Maverick shares its program with the 2025 Mach-E, but its numbers may differ, so a shared program
is no reason to share a row.

**Choice.** Exact pairs, because a family key would apply wrong numbers to files like these. A row
lists its pairs (`StrategyRow.modules`). Several data files share a row only where the note shows equal
lane-centering numbers (`test_data_files_with_the_same_numbers_share_a_row`). A level across
generations never shares a row (`test_the_mach_e_generations_never_share_a_row`). No calibration (F124)
is pinned yet: F124 hasn't been read from a truck, and the 2022-24 Maverick's module has none.

**One exception, the BC row (2.11).** It uses `ANY_DATA_FILE`, which a test allows only on a curvature
row with stock constants, where the data file can't change what is sent. Even there the module must
answer F10A: `match_strategy` returns "didn't answer" before it looks at any row when either key is
empty.

**Cost: coverage.** Of 63 vehicles' module records from Ford's service archive, 26 match a row today. Of 67 logged openpilot vehicles, 42
are on the BC row once their module answers F10A, 19 run a strategy with a row but an unread data file,
and 6 have no row (`clean-build.md` §14.3, from `scripts/clean-build/table_coverage.py`).

### 2.3 The fallback and "not supported"

**Problem.** What to send to a module nobody has read.

**Options.** A conservative guess at the module's rate and lag, or comma's stock command.

**Evidence.** For an unread strategy, its rate is unknown. A 0.003 limiter fits the Maverick and
Explorer fleet data worse than no limiter (`verify-cross-vehicle.md` §6, via `change-analysis.md` §3).
comma's command is the one with fleet evidence: gain 0.97 to 1.05 on 102 vehicles of 11 platforms
(`cross-vehicle.md`).

**Choice.** comma's command, because it is the only one with fleet evidence. `FALLBACK_ROW` is curvature
only, ramp type 0, precision 1, with nothing of a module's own. It is a row like any other, so the
controller always reads exactly one row. The fallback has no measured-curvature band either (2.4).

- **`not_supported`** makes car init set dashcam only, for a module that takes no lane message built
  here. No row uses it today.
- **Rows kept on the fallback** (Maverick 2025, Super Duty) are in the table so Steering Support can say
  "known, and not yet supported" instead of "not in the table".
- **Replaced levels (`SUPERSEDED`)** carry a reason the owner can act on: the wording of Ford's service
  message SSM 53105 for RL34-14D003-AD with an RL34-14D004-E\* calibration (`ssm-53105.md`), and "older
  than Ford's current level" for RL34-14D003-AD otherwise and for ML3V-14D003-BA.

**Cost.** Every car we have logs from ran the fallback in replay, because those logs predate the
start-up read (`clean-build.md` §9 item 10).

### 2.4 The panda's bounds and the startup bits

**Problem.** comma's Ford check accepts curvature only, and refuses a command more than 0.002 1/m from
measured curvature above 10 m/s. A′ moves the curvature signal away from the measurement on purpose. And
the panda doesn't know a module's K2.

**Options.**

- Keep the band. That is incompatible with A′.
- Bound only `c2 + K2max·c1`, as `approach-comparison.md` §6 proposed. **This is unsound:** a large
  curvature can hide behind an opposite path angle. The sum stays small, while a module whose real gain
  is below K2max steers for most of the curvature. The first simulation run produced such a frame
  (`clean-build.md` §1 item 3).
- Bound both the curvature signal alone (K2 = 0) and the implied curvature at K2max, each by the
  lateral-acceleration cap and comma's jerk limit. What a module steers for is linear in K2, so the two
  bounds cover every gain from 0 to K2max.

**Evidence.** comma bounds its angle-steered cars the same way, by acceleration and jerk with no
measured band (`steer_angle_cmd_checks_vm`). K2max was first 0.155 at low speed. The twin then showed
the early 2024 Mustang (PR3C) at 0.25 and the Ranger at 0.167 (`scripts/clean-build/k2_envelope.py`), so
it is now `min(0.26, max(1.7/v, 0.03))` per rad. The floor of 0.03 is there because above 205 km/h,
`1.7/v` fell under the Explorer row.

**Choice.** The two bounds, because the band is incompatible with A′ and the single sum bound is unsound.
BluePilot's paths have no measured band, the fallback included. bp-dev's pinion yaw source (an
alternative measured curvature) stays removed, since nothing reads it without the band. Two startup bits
go in the second safety param (`current_safety_param_sp`):

- **bit 5, `BP_LATERAL`:** run BluePilot's check instead of the stock one. Cleared by "Disable BP
  Lateral Control".
- **bit 6, `BP_PATH_ANGLE`:** path angle may steer. Set only when the matched row's switch is on, so a
  bound that rests on the gains of read modules never applies to a module nobody has read
  (`clean-build.md` §14.1).

Path offset and curvature rate must be inactive on every module.

**Costs.**

- This relaxes comma's Ford check, and it applies to the fallback too, because the band is the tie to a
  biased yaw signal that 1.2 describes. A stricter choice would keep comma's check for unknown modules.
- One K2max curve for all modules. At 60 km/h the 2021 F-150's gain is 0.047 and the panda assumes
  0.102, so the panda allows about half the path angle that module could take. Per-row bounds would
  need more startup bits (`clean-build.md` §10 item 3).
- "Disable BP Lateral Control" is read at car init only, because that is when the panda is told which
  check to run.

### 2.5 Removing the old settings and the lane offset

**Problem.** bp-dev had two controllers and 19 settings.

**Evidence.** `change-analysis.md` §0-2. Most of them patched the effects of three root causes: the
panda latch, the measured-curvature clip pinned by a biased yaw signal, and angle mode's path-angle
mapping.

**Choice.** Remove them all, because each patched an effect of a cause this build removes (the latch,
the clip) or replaces (angle mode, by A′). Removed: angle mode, the stall and hand-off pulses, both
human-turn resets, the predicted-curvature blend, lane positioning, the lane-change factors, the
post-lane-change limiter, the clip, the pinion source, and 18 of the 19 settings. One setting is left,
`disable_BP_lat_UI`. The library is about 2,300 lines shorter. The lane offset goes for the reason in
2.1, and comes back later as an offset in openpilot's plan, not in the car controller.

**Cost.** Owners lose their tuning.

### 2.6 The start-up reads

**Problem.** openpilot reuses a cached fingerprint whenever the VIN matches, and Ford changes module
software over the air.

**Choice.** Read the module fresh at every start, because a row must never be applied to software it
wasn't read from. `L/core/pscm_read.py` reads these from the module at 0x730:

| Identifier | What | Use |
|---|---|---|
| F188, F10A | strategy, data file | the table's key |
| F124 | calibration | matching: superseded levels, and rows that pin one (none yet) |
| D02B | over-the-air update counter | logged; not every module answers |
| DE00-DE02 | As-Built configuration blocks | logged |
| EE80 | version of Ford's lane-keeping software inside the module | logged |

It uses diagnostic service 0x22 (read data by identifier) after a tester-present message, the same
request shape as
openpilot's own Ford query (`test_reads_are_read_only`). Nothing is written. The answers go into
`CarParams.carFw` under brand `ford_pscm_live`, so they are logged and never take part in fingerprint
matching. F188 and F10A are asked up to three times, because one lost frame would otherwise put the car
on the fallback for the whole drive (`test_a_lost_answer_to_the_key_is_asked_again`). A silent module
costs the start 0.1 s per identifier, plus 0.2 s more for each key identifier. Only the steering module
is read. Camera status and fault-code reads are future improvements, after a truck test
(`clean-build.md` §10).

**Not checked.** Whether F10A answers in the default session on a real truck; no truck has been read
with this code. If it doesn't, every car is on the fallback. Which diagnostic session EE80 needs isn't known
either. It is read anyway, because a module that refuses it simply gives no answer, and the first truck
log will show which. DE01 isn't decoded: where its fields sit is a hypothesis for most modules.

### 2.7 The health check and its gain windows

**Problem.** Tell the owner, and us, when a car doesn't steer as its row says, without changing the
command.

**Evidence.** Every Ford measured follows curvature at 0.97 to 1.05 (`verify-cross-vehicle.md`). The
review of the first version found that its gain check couldn't reach the "degraded" threshold and gave
noisy readings (`clean-build.md` §14.1). A yaw zero of
0.025 rad/s moved one-minute gains between 0.9 and 1.6 on a car that follows its command.

**Choice** (`L/core/health.py`). Read-only signals, because the check must never change the steering:

- **Curvature gain.** A window is at least a minute of hands-off driving above 9 m/s with enough steering
  in it (`MIN_EXCITATION`). The command is delayed by the row's lag, and the yaw zero measured at
  standstill is removed. No window counts until that zero is known. A window is out of range outside
  0.85 to 1.15, and three out-of-range windows in a row mean degraded. Windows carry across drives of
  the same car and software (`health.carry` and `restore`, stored in `FordLateralSupport`). On the
  seven recorded routes, the seven windows read 0.94 to 1.03
  (`logs-derived/clean-build/health_on_routes.json`).
- **The module's capability signal reads Faulty** while BluePilot is steering. The module reports its
  status on CAN FD only, so a CAN car has no such signal.
- **The panda blocked messages** while steering and allowed. One counts only after 30 frames of
  "allowed", so an ordinary disengage, which the controller hears up to 100 ms late, doesn't latch
  "degraded" (`clean-build.md` §14.1). A single blocked message is enough.

The panda and module signals are per drive. The gain signal clears when a window reads in range.

Response time is not estimated. `commandedCurvature` and `measuredCurvature` are in the status every
frame, for offline analysis.

**Costs.** The thresholds are first values. The panda's count is every blocked message, not only
steering ones, so a refused non-steering message can show "degraded".

### 2.8 Lag per row and the delay learner's start

**Problem.** openpilot plans ahead by the lateral delay. lagd starts at `steerActuatorDelay + 0.2 s`
(0.4 s on every Ford) and learns above 50 mph.
"Lag" here is the time from command to 63 % of the response.

**Options.** Put the row's lag in the driving model's delay directly, start lagd from it, or leave it.

**Evidence.** The rows' lags don't share one definition. The F-150's is from the command on the wire, with
the rate limit not acting. The Explorer's, Ranger's and Maverick's are measured behind the module's own slewed
command, 0.1 to 0.2 s short of what lagd measures (`clean-build.md` §6 item 4).

**Choice.** Start lagd from the row's lag at 100 km/h, because a learnt value still takes over and only
the starting point changes. 100 km/h is where lagd measures (above 50 mph). The mechanism is
`steerActuatorDelay = start − 0.2 s` (`StrategyRow.delay_learner_start`, set in
`L/hosts/sunnypilot/safety_param.py`), so lagd starts at the row's lag, and with the learner off the
fixed delay is the row's lag too. The rejected option would have bypassed `steerActuatorDelay`. That gives 0.39 s on the 2021-23 F-150 BD row, 0.28 s on BC, 0.25 s on the 2021-24
Mach-E, 0.44 s on the Escape 2025 and 0.26 s on LX6C. Rows with `lag_behind_slewed_command`, the
fallback and the stock path keep openpilot's start.

**Not run:** a model replay. The delay reaches the driving model as an input, and a replay needs a
route's video.

### 2.9 Curve speed, and the planner's clip from the module's limit

**Problem** (approach I in 2.1). Above about 105 km/h the module's own lateral-acceleration table is the
ceiling, and the only lever left is speed.

**Choice.** Tell the planner the limit, so a turn the module won't take shows as a warning instead of
being clipped silently. The status publishes `lateralAccelLimit`: the panda's cap, or the row's limit
where that is lower. controlsd's curvature clip uses it where it is below openpilot's 3 m/s², so a
request beyond it shows as "turn exceeds steering limit". Road roll can't widen it
(`test_road_roll_does_not_widen_the_steering_limit`). sunnypilot's vision curve speed (SCC-V) plans for
it where it is below its own 2.0 m/s² (`B/openpilot/sunnypilot/selfdrive/controls/controlsd_ext.py`,
`.../smart_cruise_control/vision_controller.py`).

**Cost.** The rows' limits drop under 2.0 only from about 130 to 160 km/h, so at legal speeds curve
speed changes nothing today. The limit is read at the car's speed, which errs low. Not checked on a
drive.

### 2.10 Two steps, one release

**Problem.** Path angle's numbers rest on firmware reads and the twin, and no drive has sent it. The
review found a busier wheel on real requests (`clean-build.md` §14.1).

**Choice.** The base ships with path angle off, and a family switches on only after a drive confirms its
row, because the numbers haven't been driven (`clean-build.md` §14.5).

1. **Step one, the base:** every row's switch off, bit 6 clear on every car.
2. **Step two, per family:** a row switches on after a drive on one of its modules where
   `drive_check.py` (3.4) finds the measured path-angle gain within ±30 % of the row's below 20 m/s,
   where path angle does its work, and a low-speed turn with hands on, released at the exit, shows no
   wind-up (path angle left over that keeps the car turning after the driver lets go). Switching on is one line
   in the row, and it also sets bit 6 for that row.
3. **The confirming drive needs path angle on,** and the switch is a constant, so the step-one build
   can't produce it. The first drive uses a test branch, **`clean/ford-lateral-drive1`**, identical
   except that the Explorer row's L1MC-14D003-AL / MB5C-14D007-BA pair has path angle on. The window
   test already runs that case with zero refused frames.
4. **One release:** after the first confirming drive, on a 2020-24 Explorer, the rows confirmed by then
   switch on and the build ships once. Unconfirmed rows ship off.

**Release-notes wording** (`clean-build.md` §14.5 item 5): engaging in a curve starts from straight and
builds in at the module's own rate, about 5 to 6 s at 30 km/h and about 2 s at 60 km/h. Only rows whose
path angle is confirmed build in within one to three seconds. Hold the wheel through the first seconds.
For BC: "steers by curvature only, with timing measured on BC trucks."

**Why engaging in a curve is slow.** comma's rule says the inactive frame must be all zero, so the
module's copy of c2 starts at zero, and the panda's check restarts from zero too. Any controller that
obeys the rule starts from straight. On newer-library rows, frames before the module reports control
move at the panda's step, not the module's rate (`follows_at_once_before_control`, `firmware-core.md`
§4.2), which shortens this once path angle is on.

### 2.11 The BC row, from fleet behaviour

**Problem.** 42 of 45 logged 2021-23 F-150s run ML3V-14D003-BC. Our row and twin are BD. No BC file was
found (`bc-sources.md`).

**Evidence** (`bc-fleet.md`). BC trucks answer curvature at gain 1.00 to 1.02 by speed band (33 trucks),
the same as BD, and engage the same way. They are **faster**: 0.27 s against 0.57 s from command to yaw
at 60 to 75 mph, and the difference sits in the module, not the vehicle. Path angle, path offset and
curvature rate can't be tested from public data: every engaged second on every truck is curvature only.

**Choice.** A curvature row, because the behaviour supports curvature and nothing else: comma's stock
constants, the BD row's lateral-acceleration limit, BC's own lag (0.35, 0.33, 0.28 and 0.27 s at 30, 60,
100 and 130 km/h), and `ANY_DATA_FILE` (2.2). BA, the level before BC, keeps "older than Ford's
current level".

**Cost.** It rests on behaviour, not files, so it is the one curvature row without the control law read
(3.3). It applies only once the module answers F10A.

### 2.12 The three-repo layout

**Problem.** Where the code lives.

**Options.** bp-dev's in-tree Ford code, or the 7.1 bluepilot-lateral library.

**Choice.** The 7.1 library, taken from `f267286`, because 7.1 already discards bp-dev's Ford safety and
its host already has the comma-compatible limits and the stock fallback. Building on bp-dev would mean
writing the safety change twice (`change-analysis.md` §4.1). Panda safety and controller are on the same
branch, in separate commits, with one test set, so one set of tests shows the controller never leaves
the panda's window (`change-analysis.md` §4.2). The three parts are branches of one repository,
bluepilot-clean:

| Part | Upstream base | Branch in bluepilot-clean |
|---|---|---|
| BluePilot (this repo) | BluePilotDev/bluepilot `bp-7.1-dev` `fc0b77e5e` | `clean/ford-lateral` |
| opendbc, at `opendbc_repo/` | ford-op/opendbc `e80aa6cb` | `clean/opendbc` |
| bluepilot-lateral, at `opendbc_repo/opendbc/bluepilot_lateral/` | BluePilotDev/bluepilot-lateral `f267286` | `clean/bluepilot-lateral` |

`B/.gitmodules` points `opendbc_repo` at that repository, branch `clean/opendbc`. `O/.gitmodules` points
`opendbc/bluepilot_lateral` at the same repository, branch `clean/bluepilot-lateral`. A recursive clone
gets all three. LFS objects stay at sunnypilot's store (`.lfsconfig`), so pushes use `--no-verify` to
skip the LFS hook. Each library commit is followed by an opendbc pointer commit and a bluepilot pointer
commit.

**Cost.** The packaging isn't the shape upstream would take: upstream would want plain files in
`opendbc/car/ford` and `opendbc/safety` (4.3). Not checked: bp-7.1-dev itself on a drive
(BluePilotDev/bluepilot#226).

## 3. The low level

### 3.1 One steering frame, end to end

1. **Planner.** openpilot's model and planner produce the desired curvature. controlsd clips it, also
   to `lateralAccelLimit` (2.9), and hands `CarControl.actuators.curvature` to card.
2. **Host hook.** `O/opendbc/car/ford/carcontroller.py` builds one `BluePilotLateral` through the
   sunnypilot adapter (`L/hosts/sunnypilot/lateral.py`). Each frame, `bp.update(CC_SP)` passes
   `LateralInputs`: `pandaLateralAllowed` and `safetyTxBlocked`, filled from `pandaStates` in
   `B/openpilot/sunnypilot/selfdrive/controls/controlsd_ext.py`. If it returns False, the stock path
   runs. At `STEER_STEP` (20 Hz) it calls `bp.step_lateral(CC, CS)`.
3. **Request rule** (`L/core/request.py`). Ask only while openpilot wants lateral control, the panda
   allows it, and the module says Available or Active. The module's state (`pscm_status`) is read from
   Lane_Assist_Data3_FD1 by `O/opendbc/car/ford/carstate.py`, on CAN FD only; Denied is its own state. On
   CAN, where there is no status, the rule follows openpilot and the panda. Otherwise the inactive frame
   is sent: all zero, ramp type 0, precision 1. The reasons: a module configured for Ford's hands-free
   driving stays Unavailable while it is asked, a refused frame is a missing frame to the module, and the inactive
   frame always passes (`test_never_asks_through_unavailable`).
4. **Limit** (`L/core/carcontroller_ext.py` `step_lateral`). The request is clipped to
   `lat_accel_limit / v²`.
5. **Path** (`_path`). Without path angle, `want` goes straight through. With it:
   - curvature moves toward `want` at the row's rate, so it always equals the module's own copy. The
     position is kept unrounded (`clean-build.md` §14.1);
   - the controller's copy of the module's preview time T drops at once when curvature rises and
     recovers at `t_rise` per second, as the module's does. `gain = K2(v, T)`;
   - path angle is `(want − c2)/gain`. It may cancel whatever the slow curvature still holds back,
     plus at most what the row's path-angle rate can take back in 1 s. While the driver steers it gets
     nothing beyond cancelling, so nothing is left over when they let go (the wind-up rule);
   - path angle stays at zero until it would be at least 1.5 wire steps, so it doesn't flip sign frame
     by frame (the dead band), and it moves at the row's path-angle rate.
6. **Panda window** (`L/core/limits.py` `window`, `fit_path`). Starting from the last command the panda
   saw (zero after any inactive frame), it allows one jerk step either way within the cap, for the
   curvature signal and for the implied curvature. The controller reads K2max 1 m/s below the car's
   speed, so its K2max is at or above the panda's, which reads it at its lowest recent speed sample. Curvature
   is fitted first, then path angle into what is left. So every frame is inside the panda's check by
   construction.
7. **Pack** (`L/core/lane_message.py`, `L/can/fordcan_ext.py`). Each path signal is clamped to its DBC
   range in the wire's sign (the wrap fix, `test_wire_clamp.py`). The library sets the
   LateralMotionControl2 checksum itself. `lane_message.replaces(address)` tells the car controller
   which camera messages it sends in the camera's place.
8. **Panda** (`O/opendbc/safety/modes/ford.h` `ford_tx_hook` → `L/safety/ford.h` `ford_bp_lmc_tx` →
   `L/safety/lateral.h` `ford_bp_path_cmd_checks`): section 3.2.
9. **Module.** Steers for `c2 + K2·c1` with its own limits and driver override.
10. **Status.** `BluePilotLateral.status()` returns a `LateralStatus`, published every frame as
    `controllerStateBP.lateral` by `B/bluepilot/selfdrive/car/bp_card_publisher.py`.
    `B/bluepilot/selfdrive/car/ford_lateral_support.py` keeps `FordLateralSupport`: the settings row,
    the comparison with the last start, and the offroad notice `Offroad_FordLateralSupport`. card calls
    `record_start` once per start (`B/openpilot/selfdrive/car/card.py`).

**At car init** (`L/hosts/sunnypilot/safety_param.py` `init_ford_lateral_sp`, called from sunnypilot's
`setup_interfaces`): the start-up read; `check_plan`, which compares the camera messages the row
replaces with the messages the camera was heard sending at this start, and logs a mismatch without
changing anything; dashcam only for `not_supported`; the safety
bits into `CP_SP.safetyParam`; `lateralInputsRequired`; and `steerActuatorDelay` from the row.

**How a car is matched** (`L/core/strategy.py` `match_strategy`), in order:

1. a superseded level gives its reason;
2. a missing F188 or F10A gives "didn't answer";
3. a strategy in no row gives "isn't in the table";
4. a row with the exact pair (or BC's `*`), and a listed calibration if the row pins any, matches. A
   fallback row gives "kept on the fallback";
5. otherwise, "another data file or calibration".

`match.plan` is the matched row, or `FALLBACK_ROW`.

### 3.2 The panda's checks

`ford_tx_hook` decodes each steering message. If `ford_bp_lateral` (bit 5) is set, `ford_bp_lmc_tx`
runs:

- curvature rate and path offset must be inactive. Path angle must be inactive unless
  `ford_bp_path_angle` (bit 6) is set;
- `ford_bp_path_cmd_checks` runs while lateral control is allowed and the request is on:
  - signal ranges: ±0.02 1/m and ±0.5 rad;
  - the lateral-acceleration cap on the curvature signal **and** on the implied curvature
    `c2 + K2max·c1`, with K2max read at the lowest recent speed sample and the cap at that speed minus
    1 m/s;
  - comma's lateral jerk limit (about 3.6 m/s³) on both, against the last command;
  - comma's real-time rate check and the speed-mismatch check;
- while not steering, both signals must be zero, and there is no steering while controls aren't
  allowed;
- after any violation, and while lateral control isn't allowed, the last command is zero, so the next
  ramp starts from zero. There is no latch; three tests pin that.

The caps are about 3.6 m/s² on CAN and about 2.4 m/s² on CAN FD (`FORD_BP_LIMITS`).

Without bit 5, the stock check runs: `ford_stock_curvature_checks`, a wrapper around comma's
`steer_curvature_cmd_checks` with its band. The wrapper restarts the check from zero while lateral
control isn't allowed, as the stock controller does, and from the measured curvature after a refused
frame. Setting comma's `inactive_curvature_is_zero` flag instead would restart from zero after every
refused frame too, and on replay that refused every frame to the end of a curve after one refusal
(`clean-build.md` §14.1 #6).

### 3.3 How a row is made from firmware

The pipeline lives in the project's research notes (to be published):

1. **Files.** The module's VBF files (Ford's flash file format: program, data file, calibration) for a strategy come from Ford's
   service-software archive, collected per VIN (`notes/sourcing/`; what's missing is in
   `notes/sourcing/wanted.md`). Raw files and decompiles are kept apart from the notes.
2. **Read the control law.** Decompile with Ghidra (RH850 on the 2021-23 F-150's module, PowerPC VLE on
   the 2020-24 Explorer's) and write a note per strategy: path law, preview time and distance tables,
   integrator, rate limits, lateral-acceleration limit, authority and latches (`firmware-core.md` for
   ML3V-BD; `rl14-firmware.md`, `explorer-firmware.md`, `rz1c-firmware.md`, `lj9c-firmware.md`,
   `lx6c-firmware.md` and the others). This gives a **curvature** row.
3. **Compare data files** (`cfg-diff.md`): which data files hold the same lane-centering numbers and can
   share a row.
4. **Check against the twin** (`twin.md`). `twin/` is the module's lateral controller in Python, block
   for block. It was checked against the firmware itself in an emulator: 595,165 block calls, no
   mismatches after fixes (`twin-vs-firmware.md`). Against drives, its angle target matches the measured
   pinion angle to 0.41° RMS on BD trucks; `twin.md` §10 covers the older library. A row checked this
   way, or by a drive, is **full**. Its preview and rates come from the twin's calibration tables
   (`scripts/clean-build/row_numbers.py`), and its K2 is checked over speed and curvature
   (`k2_rows_check.py`, and `k2_envelope.py`, which also has to stay under the panda's K2max).
5. **Row in `strategy-table.md`,** with every value marked proven, supported or hypothesis. Then a
   `StrategyRow` in `L/core/strategy.py` that cites its note.
6. **Tests:** a `test_strategy.py` case for its pairs and for what must not match it; the window test
   with the row, path angle on and off; and `table_coverage.py` re-run.
7. **Path angle on:** only after `drive_check.py` passes on a drive of that family (2.10).

**Evidence each level needs:**

| Level or field | Needs |
|---|---|
| curvature | the control law read from that strategy's software, and the exact pair. BC is the one exception (2.11) |
| full | the above, plus a twin run or a drive |
| a preview in the row | both distance clamps at all eight speed points. RZ1C and PZ11 have four, so their path angle stays zero |
| the path-angle switch | a confirming drive |

Unverified numbers err toward a high K2, so less path angle is sent, and a low c2 rate, so the
module's copy of the curvature never falls behind the controller's.

### 3.4 Logs and the drive check

- **Every start** logs the read (`carlog` "BluePilot steering module read"), the support record
  ("BluePilot steering support"), a part-number change ("BluePilot steering module changed", with old
  and new) and any plan mismatch ("BluePilot lane message plan").
- **Every frame:** `controllerStateBP.lateral`, with the request, the command, both signals, measured
  curvature, whether the command was limited, the module's state, the limit, the capability and the
  health check.
- **The drive check:** `scripts/clean-build/drive_check.py <rlog or route dir>... [--json out.json]`, or
  `--selftest`, in the tree's venv. Per speed band (5-12, 12-20, 20-45 m/s), hands off, it fits
  `measured(t + lag) = a·curvature + b·(commandedCurvature − curvature) + offset` at the best lag
  (0-0.8 s). Here `curvature` is the c2 sent and `commandedCurvature` is `c2 + K2·c1`, both status
  fields, so b is the path-angle gain against the row's. It passes where |b − 1| ≤ 0.30 with at least 400 frames of path angle at work. It also
  fits small and large path angles separately. The self-test recovers gains of 1.0, 0.8 and 0.55, and
  fails the 0.55 case. No route has these fields yet.
- **Replay:** `scripts/replay/cleanreplay.py` (with `run_clean.sh` and `analyze_clean.py`) runs these
  files with the compiled panda in the loop. `--pscm` gives a what-if row.

### 3.5 Tests

Python environment: `UV_PROJECT_ENVIRONMENT=~/tools/venvs/clean-build uv sync --frozen --extra testing
--extra tools`, then `pyyaml`, `pytest` and `cantools` installed into it. Then `scons -j9` at this
repo's root. The commands below call that environment as `.venv/bin/python`. Whole suites are
heavy: run them in CI or on a large machine, not on a shared small one. From `opendbc_repo/` with
`PYTHONPATH=$PWD`:

```
.venv/bin/python -m unittest discover -s opendbc/bluepilot_lateral/tests -t .          # library: 108 OK
(cd opendbc/safety/tests && ../../../.venv/bin/python -m unittest test_ford test_bluepilot_lateral)  # 650 OK, 107 skipped
.venv/bin/python -m unittest opendbc.car.tests.test_car_interfaces opendbc.car.ford.tests.test_ford  # 284 OK
bash opendbc/bluepilot_lateral/safety/tests/bluepilot_coverage.sh                       # 100 % of both headers and modes/ford.h
```

Also: `opendbc/sunnypilot/car/ford/tests` (75 OK), ruff and ty on the library, MISRA (`test_misra.sh`,
passes), mutation testing (`L/safety/tests/bluepilot_mutation.py`, 58 of 58 killed), and the bluepilot
tree on a built tree:

```
tools/test_runner.py bluepilot openpilot/sunnypilot/selfdrive/controls/tests openpilot/sunnypilot/sunnylink/tests \
  openpilot/selfdrive/ui/bp openpilot/selfdrive/controls openpilot/selfdrive/selfdrived openpilot/selfdrive/car
```

That gives 545 passed, 1 skipped and 1 error. The error is in `test_ui_frames`' harness: a `params.cc`
log line lands on the child's result line. All counts are from `clean-build.md` §14.4, on a build
machine, at library `ca0b134`. The library has changed only its credits file since.

What the main tests guard against:

| Test | Guards against |
|---|---|
| `L/safety/tests/test_ford_bluepilot_window.py` | the real car interface and controller leaving the compiled panda's window: speed grid, engaging in a curve, hard acceleration and braking, random requests, module Unavailable/RampOut/Denied, the panda withdrawing; CAN and CAN FD, rows, fallback and stock path. Zero refused frames |
| `L/safety/tests/test_ford_bluepilot.py` | breaking the panda rule: `test_curvature_cannot_hide_behind_an_opposite_path_angle`, no latch, restart from zero, bit 6 off refusing path angle, upstream's `test_steer_allowed` sweep |
| `L/tests/test_strategy.py` | wrong matches: exact pairs, generations never sharing, BC, superseded reasons, read-only reads, retries, the switch off on every row |
| `L/tests/test_controller.py` | breaking the request rule, the exact c2 rate, the first frame, asking through Unavailable |
| `L/tests/test_recorded_turns.py` | path-angle wind-up on recorded low-speed turns (`tests/data/low_speed_turns.csv`) |
| `L/tests/test_health.py` | a false "degraded" after a disengage; breaking the gain window's rules |
| `L/tests/test_wire_clamp.py` | signals wrapping past their range (failed first against `f267286`) |
| `L/tests/test_lane_message_plan.py` | the plan drifting from the panda's transmit table and the car controller |
| `L/tests/test_safety_param_bits.py` | Python's bits drifting from `ford_bp_init`'s |
| `L/tests/test_core_imports.py`, `test_bluepilot_upstream_copies.py` | absolute host imports; copies of upstream functions going stale |
| `B/bluepilot/selfdrive/car/tests/test_ford_lateral_support.py` | the support row, notices and `record_start` breaking with the real Ford interface |
| upstream `test_ford.py` | the stock check changing: it passes unchanged |

## 4. Gaps, extending, upstream

### 4.1 Known gaps (`clean-build.md` §6, §14.6)

- F10A and F124 haven't been read from a real truck. Without F10A every car is on the fallback.
- Coverage: 37 of 63 archive records and 25 of 67 logged vehicles have no exact row. Rows are wanted for
  PZ11-14D003-EA and the unread files. RZ1C and PZ11 need preview tables. Calibrations aren't pinned
  yet.
- No route exercises a row's own numbers yet. This build has never sent path angle on a drive.
- The stock path ("Disable BP Lateral Control") still has 143 refused frames over seven routes, inside
  comma's band. That is outside this build's goal.
- IQ.Pilot hasn't run its coverage step (it needs clang-18 and mull) and doesn't do the start-up read.
  StarPilot is untested. Neither host reads the module, so every car there is on the fallback.
- What to do when a `check_plan` mismatch is logged: today it is only logged.
- Not run: the model replay for the delay learner's start, and `tools/test_on_host.sh` end to end.
- If modules don't answer F10A in the default session, there is no plan B yet.
- Future improvements: camera and fault-code reads at start, the DE01 decode, K2max per row
  (`clean-build.md` §10).
- Not checked: installing on a comma four.

### 4.2 Extending

- **A row:** follow section 3.3. Keep exact pairs, mark the level by its evidence, and leave
  `path_angle` False.
- **A new way a module takes its lane message:** a new variant in `L/core/lane_message.py`, its own
  check in `L/safety/`, and rows that name it. The controller and the table don't change.
- **A new host:** an adapter against `BluePilotLateral(CP, safety_param, packer, CAN)`, `update`,
  `step_lateral` and `status` (`L/HOSTS.md`).
- **Every change to the safety code:** the window test, coverage at 100 %, MISRA and mutation testing.

### 4.3 The path upstream

The aim is a build that can go upstream, even if that means larger changes to comma's Ford code where
the current code rests on faulty logic. It needs:

- the library as plain files in `opendbc/car/ford` and `opendbc/safety`, without host bindings;
- the early commits squashed into safety, controller, tests and A′. A′'s panda change and controller
  change are already their own commits;
- an argument for the fallback half, which is nearest to upstream: comma's command without the clip.
  Removing the band is a relaxation that has to be argued with the acceleration-and-jerk bound;
- for the path-angle half, comma accepting a c1 rule (the two-bound check in 2.4, with a K2max curve
  derived from read modules), and confirming drives;
- upstream's `test_ford.py` already passes unchanged, and MISRA and mutation testing pass on the
  headers.
