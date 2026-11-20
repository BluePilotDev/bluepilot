# The clean Ford lateral build: developer guide

For people who work on this code and need to know how the Ford steering works and why it was built
this way. You don't need to know Ford steering already: section 1.1 defines the terms the rest uses.

- **Section 1, the high level:** the terms, the problem, the goal and the shape of the solution.
- **Section 2, the decisions:** one at a time, each with its problem, options, evidence, choice and cost.
- **Section 3, the low level:** one steering frame end to end, the panda's checks, how a table row is
  made, logs and tests.
- **Section 4:** known gaps, how to extend it, and the path upstream.

The owner's version is [USER_GUIDE.md](USER_GUIDE.md). Installing is in [INSTALL.md](INSTALL.md).

**Sources.** Code claims are as of the submodule commits `clean/ford-lateral-pathangle` pins. The code lives in one repository, bluepilot-clean, as three branches (2.12;
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
- **The twin.** A Python model of a module's lateral controller. For the 2021-23 F-150 (software BD)
  and the Ranger's controller chain it is held to the firmware's own code run in an emulator. For every
  other module it is a reading of the decompiled code that has not been run. Each claim below says
  which (3.6).
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
- **Learner start and delay.** A row carries one timing number, its **learner start**
  (`StrategyRow.learner_start`): where lagd starts on that software, measured on logged drives of it.
  It is a measurement of car and module together, not a firmware value, and lagd replaces it. The
  **delay** is what lagd estimates: the time shift that best lines up requested and measured lateral
  acceleration. The driving model plans with the delay. No row carries a lag any more
  (`StrategyRow.lag` is None, and the status's `lag` reads 0), 2.8.

### 1.2 The problem

The same openpilot command gives different results on different Fords. The research found that most of
the variation is **between vehicles, not within one truck**:

- **Each module runs its own strategy with its own data file.** One program serves several vehicle
  lines, one openpilot platform covers different modules, and Ford changes both over the air and at the
  dealer (`strategy-table.md`, `cfg-diff.md`).
- **Below about 80 km/h the binding limit is the module's own rate limit on its copy of c2:**
  0.003 1/m per second on the decoded F-150-family strategies. That is 0.2 m/s³ of lateral jerk at
  30 km/h. No sender that uses curvature alone can follow a town curve that builds faster than that,
  whatever its delay or outer loop (`approach-comparison.md` §1-2, `firmware-core.md` §4.2).
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
   command, including its `anti_overshoot` smoothing on the 2021-23 F-150 and Bronco Sport platforms,
   without its clip to measured curvature, and with BluePilot's lower cap on CAN FD (2.3).
   With path angle off, a row can change up to three things against the fallback, where it has the
   value: the lateral-acceleration limit (2.9), lagd's starting delay (2.8), and the ramp type (2 on
   most rows, 0 on the fallback and BC). Rows don't smooth the request. The health check delays the
   command by lagd's delay, else the row's learner start, else 0.4 s (2.7). Every number a row takes
   from its module is generated from the module's files (3.3).
3. **Path angle (approach A′)** on rows whose preview is known. Curvature moves at the module's rate,
   and path angle carries what that rate still holds back, in the module's own K2. It is a per-row
   switch, **on for every row whose module's preview, c2 rate and c1 rate were read, and for BC with
   BD's numbers** (seven rows), off on the others and the fallback.
4. **The panda bounds the command by lateral acceleration and jerk** (comma's numbers, except a lower
   cap of BluePilot's own on CAN FD, 3.2), on the
   curvature signal alone, on the implied curvature at the highest K2 the module can have at that
   speed (its own curve, from its firmware), and on what the module's own slewed copies of the two can
   steer for. Startup bits allow path angle only for a row that has it on, and name its gain curve.
5. **A read-only health check** and a **Steering Support row** in the settings that say how the car is
   steered and why.

## 2. The decisions

### 2.1 Curvature base, path angle as a per-row switch (A′)

**Problem.** Curvature alone is slow in town because of the module's c2 rate (1.2). bp-dev's answer,
angle mode, behaved differently on every vehicle line.

**Background.** comma moved Ford to curvature in 2023; its PR says "Curvature allows for better control
instead of path angle signal" (openpilot#27091). The BluePilot dev group's account is that the reason was
universality, and its experience over 50,000+ miles of path-angle-only control is that angle control
without the module's curvature filter tracks better. One such route is measured: steps below 60 km/h
reach 63 % in 0.14 s, against 0.28 s on curvature trucks, on 10 steps against 50 (research note
`pathangle-route`). The research explains both views: c2 goes through the module's rate limit, and c1
doesn't, but c1's effect depends on each module's K2.

**Options.** `approach-comparison.md` §3 scores 18 approaches (A, A′ and B to Q) on native feel, authority across
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
trucks. Path angle is a constant in each row (`StrategyRow.path_angle`). It is on for every row whose
module's preview, c2 rate and c1 rate were read from its files: F-150 2021-23 (BD), F-150 Lightning,
Explorer 2020-24, Ranger 2024-25, Maverick 2022-24, and the Escape 2020-22 and Bronco Sport 2021-22
(LX6C-14D003-AL). Of these, only BD's code (and through the same code the Lightning's) and the Ranger's
controller chain have been run in an emulator; the others are read, not run (3.6). Two pairs take another
module's numbers, by the project's choice: the F-150's level BC takes BD's (2.11), and the Explorer's
second pair (L1MC-14D003-AL with MB5C-14D007-BA, a data file not in the archive) takes the first pair's. It is off on every other row and on the fallback
(`test_path_angle_is_on_for_every_row_that_can_use_it`). The evidence for switching it on is the
BluePilot dev group's 50,000+ miles of path-angle-only control, and the two rows measured on a road
(2.10).

**Costs.**

- Recorded requests through the F-150 twin make the wheel 1.2 to 2.6 times as busy (more steering
  movement) below 12 m/s as the fallback. Path angle's sign flips 1.7 to 4.4 times a second where it is
  active, even with the dead band that holds small path angles at zero (3.1 step 5; `clean-build.md`
  §14.3). Whether that feels responsive or twitchy is for the first drive to
  show with this controller.
- The controller keeps its own copy of the module's preview time. Where the copy differs from the
  module's, it errs to a higher K2: less path angle and a little less tracking, not overshoot
  (`L/core/strategy.py` `Preview`). The copy starts where the module's start-up leaves T, when the build
  starts (at or after the module), and rises at the module's rate whether or not it steers: the 2020-24
  Explorer's T is 0 at power-up and takes about 25 s to reach 5 s, so until then it previews its lower
  distance at every speed (executed); the 2021-23 F-150's and the Ranger's start at T max (executed); the
  rows whose start-up was not run (BC, the Lightning, the 2022-24 Maverick, LX6C) start at 0. That holds only where the row's K2 itself isn't low. The F-150 and
  Lightning rows' K2 was 11 % low at a 15 m preview, the overshoot side, until they took the module's
  K2 table (below).
- Every row with path angle carries its module's own K2 table, generated from the module's files, and
  reads it straight between its points as the module does (`Preview.k2_table`). On the 2021-23 F-150 that
  is 1.96/d at its points and 2.205/d at the 15 m of town speeds, where a single 1.96/d held K2 11 % low.
  The older library's rows held 4.17/d until this round, 0.1 to 4.5 % above their tables.

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
is pinned yet: no F124 has been read with this code (the Explorer's, M1MC-14D004-AB, is known from
Ford's records and a read with another tool), and the 2022-24 Maverick's module has none.

**One exception, the BC row (2.11).** It uses `ANY_DATA_FILE`, which a test allows only on a curvature
row with comma's stock ramp type and precision. The row still carries BD's limit table and BC's learner start, which
change what is sent above about 109 km/h on CAN FD and where lagd starts, so it holds only while every BC
data file has those numbers; none has been read. Even there the module must
answer F10A: `match_strategy` returns "didn't answer" before it looks at any row when either key is
empty.

**Cost: coverage.** Of 63 vehicles' module records from Ford's service archive, 26 match a row today. Of 67 logged openpilot vehicles, 42
are on the BC row once their module answers F10A, 19 run a strategy with a row but an unread data file,
and 6 have no row (`clean-build.md` §14.3, from `scripts/clean-build/table_coverage.py`).

### 2.3 The fallback and "not supported"

**Problem.** What to send to a module nobody has read.

**Options.** A conservative guess at the module's rate and delay, or comma's stock command.

**Evidence.** For an unread strategy, its rate is unknown. A 0.003 limiter fits the Maverick and
Explorer fleet data worse than no limiter (`verify-cross-vehicle.md` §6, via `change-analysis.md` §3).
comma's command is the one with fleet evidence: on 102 vehicles of 11 platforms, platform medians 0.96 to
1.05 and per vehicle 0.83 to 1.18, on comma's clipped command (`cross-vehicle.md`,
`verify-mythos-limits-guides.md` §1.1).

**Choice.** comma's command, because it is the only one with fleet evidence. `FALLBACK_ROW` is curvature
only, ramp type 0, precision 1, with nothing of a module's own. It is a row like any other, so the
controller always reads exactly one row. On the platforms where comma's stock code smooths the request
(`CAR.FORD_F_150_MK14` and `CAR.FORD_BRONCO_SPORT_MK1`), the fallback smooths it the same way
(`anti_overshoot`, copied in `L/core/carcontroller_ext.py` and pinned to comma's body), because the
fallback is for modules nobody has read. Our fits put most logged 2021-23 F-150s (level BC) and the
one logged Bronco Sport at a damping of about 0.40, the least damped Fords logged
(`assumptions-audit.md` §3.7). A row on a read module takes the request as it is. Two things still
differ from stock: no clip to measured curvature (and no measured-curvature band in the panda, 2.4),
and BluePilot's lower cap on CAN FD (3.2).

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
it became `min(0.26, max(1.7/v, 0.03))` per rad, with the floor because above 205 km/h `1.7/v` fell under
the Explorer row. The modules' start then raised the floor. A module previews from its T at power-up: 0 on the
2020-24 Explorer (executed: its 85 m lower distance for about 25 s, 0.0475 per rad, above `1.7/v` from
129 km/h), and taken as 0 where the start-up was not run, so the 2022-24 Maverick and LX6C may preview their
30 m lower distance, 0.1333 at every speed. The curve is now `min(0.26, max(1.7/v, 0.147))`, 0.147 being
that 0.1333 with a 10 % margin. The single curve was already 1.2 to 5.3 times each switched-on row's own
gain, so it held path angle back. Each module's own curve replaces it (Choice). One read data file, on a
row with path angle off, is above the single curve at 15 km/h (the 2025-26 Corsair's, 0.340 against 0.26).
The single curve bounds the modules with a path-angle row, not every module read.

**The wire is not what the module steers for.** The module moves its own copies of c2 and c1 toward each
frame at its own rates: c2 at the row's rate (0.003 1/m/s, 0.01 on the Maverick), c1 at 0.1 rad/s. Below
about 125 km/h comma's jerk limit is faster than that c2 rate. So a sender could walk c2 to the opposite
cap at the jerk step while raising c1 so `c2 + K2max·c1` never moved: the panda accepted every frame, and
the module held the old c2 with the new c1, 1.4 to 2.4 times the cap. A per-module gain made that larger,
because it removed the single curve's slack (`verify-mythos-clean-build.md` finding 1).

**Choice.** The two bounds, because the band is incompatible with A′ and the single sum bound is unsound.
BluePilot's paths have no measured band, the fallback included. bp-dev's pinion yaw source (an
alternative measured curvature) stays removed, since nothing reads it without the band. Startup bits go
in the second safety param (`current_safety_param_sp`):

- **bit 5, `BP_LATERAL`:** run BluePilot's check instead of the stock one. Cleared by "Disable BP
  Lateral Control".
- **bit 6, `BP_PATH_ANGLE`:** path angle may steer. Set only when the matched row's switch is on, so a
  bound that rests on the gains of the modules with a path-angle row never applies to any other module
  (`clean-build.md` §14.1).
- **bits 0 to 2, `BP_PATH_ANGLE_GAIN`:** which module's own gain curve bounds the path angle
  (`StrategyRow.panda_gain_curve`). Set with bit 6.

**The gain curves.** The panda holds five, one per module family with path angle on. F-150 2021-23,
Lightning and BC share one, because the F-150's and Lightning's firmware hold the same numbers and BC
takes BD's. Each is generated from that module's flash (`safety/ford_gain_curves.h`, written by the same
script and from the same reading as `core/module_tables.py`; `core/limits.py` `GAIN_CURVES` is built from
the Python file, and `test_limits.py` pins the two): its c2 rate, its shortest preview time (taken no
higher than its T when the module starts, below), the two
preview-distance clamps by speed (past the upper the distance is the upper, the module's own order), and
its K2 table by distance, read straight between the table's points as the module reads it. The bound is
that module's largest K2 at the speed, times a margin of 1.10, and no higher than the single curve (which is
above every curve with its margin, so it caps none). A module may start or restart at any time, and it previews
from its T at power-up until T reaches its floor, so each curve takes that T as its floor where it is lower
(`FORD_BP_PREVIEW_TIME_AT_START` in `safety/ford.h`, `PREVIEW_TIME_AT_START` in `core/limits.py`; not generated,
because it is what the start-up code does, not a table in the flash): 0 on the Explorer's (executed), the
Maverick's and LX6C's (not run) and the F-150's (BD loads T max, executed, but the curve also serves BC and the
Lightning, whose start-up was not run); the Ranger's T max (executed), above its floor. Before, at 140 km/h within
25 s of the module starting, the Explorer's gain was up to 1.23 times the bound and the Maverick's and LX6C's up
to 4.1 times (`test_a_module_start_at_highway_speed`). The panda reads the gain at its lowest recent speed
sample, which is right only if the curve never rises with speed, so each distance point is held to the
least of the later ones. That moves one point, on the strict side: the F-150's lower distance at
100 km/h, from 50 m to 46.9 m. With no curve, or bits 6 or 7, the panda keeps the single curve. Bits 0
to 4 once carried a measurement source, in builds that never set bit 6. Bits 3 and 4 stay unused. Three
bits fit StarPilot, whose adapter carries seven.

**The module's copies.** The panda keeps the range each copy can be in (`FordBpModuleCopies`): moved
toward the last frame sent, for the time since, at the slowest rate any read module has (which only
widens it), and always including the frame itself, for a module that takes it at once. A frame is
refused when, once sent, the most the copies can steer for passes both the cap and what they could
already steer for: at the largest gain for a path angle that adds, none for one that takes away. So a
curvature-only frame is judged as before, a frame that makes nothing worse passes (the copies lagging
above a cap that fell as the car sped up included), and only frames that pass move the copies' target.
The controller keeps the same range twice, moved for 0.8 and for 1.25 of a frame, and fits each frame
inside it, so its frames pass whatever time the panda counts between them.

Path offset and curvature rate must be inactive on every module.

**Costs.**

- This relaxes comma's Ford check, and it applies to the fallback too, because the band is the tie to a
  biased yaw signal that 1.2 describes. A stricter choice would keep comma's check for unknown modules.
- The bound is only as right as the row's match and the firmware read behind its curve. A wrong curve
  index would loosen it to another module's gain. `test_limits.py` holds every curve at or above what its
  module steers for at that speed and every higher one, and at or under the single curve. For BC the
  curve is BD's by choice (2.11).
- The floor at power-up holds path angle back on three curves for the whole drive, not only its first
  25 s, because the panda can't tell when the module started: on the Explorer the bound is the 85 m gain
  (0.0475) at every speed, against 0.0293 at T max above 100 km/h; on the Maverick and LX6C the 30 m gain
  (0.1333), against 0.027 to 0.04 above 100 km/h. On the F-150's curve it is up to 5 % stricter above 120 km/h.
- The margin is for what the panda can't see (`core/limits.py`). The module reads its K2 table in fixed
  point, at most 0.45 % above the float lookup (BD's own lookups, executed; the older library's, run on the
  Explorer, Maverick and LX6C, are in float). It reads it at its own speed: on BD that is the larger of the
  faster wheel of one axle and the brake module's speed the panda reads, truncated to 1/64 km/h (executed), so
  the panda's over the module's is at most (v + a·age) / (v − 1/64 km/h): 1.068 at 5 km/h, 1.017 at 20,
  1.003 at 120. With those executed inputs BD's curve needs a margin of 1.027. Not executed: the 30 ms sample
  age (at 3 m/s²); that the module's cell holds that very signal (read from its receive code); below about
  5 km/h (BD's curve is flat there); a truck configured without the brake module's speed (no stability
  control); every program but BD. Where the chain was not run the allowance stays this project's choice, 6 %:
  BC and the Lightning share BD's curve, and the Ranger has its own. With it the least margin over every curve
  is 1.0949, so 1.10. Without the margin, a speed reading 1 % under the panda's would put the module's gain
  about 1 % above the bound. It costs a few % of
  path angle in sharp entries at the cap.
- What each module's own curve buys over the single one (controller only, open loop, with the copies'
  bound and the margin): in sharp curve entries (2.4 m/s² over 0.5 s) at 45 to 60 km/h on the F-150,
  Lightning and Ranger rows, the module gets 96 to 98 % of the path angle asked, against 94 to 96 %, and
  the tracking error at 60 km/h falls from 0.0025 to 0.0015 1/m on the F-150 and from 0.0021 to 0.0012
  on the Ranger. The cap and the jerk limit are unchanged.
- The copies' range assumes a module never moves its copies slower than the rates above. After a
  disengage the panda lets them settle at those rates, so an engagement soon after one in a sharper curve
  can send less path angle for a second or two.
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

**From the firmware.** Each program's own read-identifier check allows F10A in the default session on
every program with a row except the Super Duty's, which wasn't decoded and is on the fallback anyway.
EE80 is allowed in the default session on every program decoded (`assumptions-audit.md` §3.1, item 47).
BC's software was never read.

**Not checked.** Whether F10A answers on a real truck; no truck has been read with this code. If it
doesn't, that car is on the fallback. DE01 isn't decoded: where its fields sit is a hypothesis for most
modules.

### 2.7 The health check and its gain windows

**Problem.** Tell the owner, and us, when a car doesn't steer as its row says, without changing the
command.

**Evidence.** Logged Fords follow comma's clipped curvature command at 0.83 to 1.18 per vehicle, platform
medians 0.96 to 1.05 (`verify-cross-vehicle.md`, `verify-mythos-limits-guides.md` §1.1). Run over the
logged fleet with each car's lagd delay, the check puts 8 of 575 one-minute windows outside its range
and flags no vehicle (`assumptions-audit.md` §3.5). The
review of the first version found that its gain check couldn't reach the "degraded" threshold and gave
noisy readings (`clean-build.md` §14.1). A yaw zero of
0.025 rad/s moved one-minute gains between 0.9 and 1.6 on a car that follows its command.

**Choice** (`L/core/health.py`). Read-only signals, because the check must never change the steering:

- **Curvature gain.** A window is at least a minute of hands-off driving above 9 m/s with enough steering
  in it (`MIN_EXCITATION`). The command is delayed by lagd's delay (`LateralInputs.lateralDelay`), else
  the row's learner start, else 0.4 s, and the yaw zero measured at standstill is removed. With a delay of 0 s,
  as the fallback once had, one logged Lightning on the fallback read 0.59 to 0.90 and went degraded; with
  lagd's delay no logged vehicle of 84 does (`assumptions-audit.md` §3.5). No window counts until that zero is known. A window is out of range outside
  0.85 to 1.15, and three out-of-range windows in a row mean degraded. Windows carry across drives of
  the same car and software (`health.carry` and `restore`, stored in `FordLateralSupport`). On the
  seven recorded routes, the seven windows read 0.94 to 1.03
  (`logs-derived/clean-build/health_on_routes.json`).
- **The module's capability signal reads Faulty** while BluePilot is steering. The module reports that
  whenever its lane-centering inputs fail its own check: our steering frames, or another input such as
  a vehicle sensor, or a module fault (`assumptions-audit.md` §3.9). The module reports its status on
  CAN FD only, so a CAN car has no such signal.
- **The panda blocked messages** while steering and allowed. One counts only after 30 frames of
  "allowed", so an ordinary disengage, which the controller hears up to 100 ms late, doesn't latch
  "degraded" (`clean-build.md` §14.1). A single blocked message is enough.
- **The module answers Denied** while asked: lane centering isn't configured on that truck as built, or
  the module latched a fault for the drive. It holds for the drive, and the settings row reads "Not
  configured on this truck" instead of the level.

Each signal clears in its own way. The module's is per frame, not latched: it shows only while the
module reports Faulty (`L/core/health.py` `update`). The panda's count lasts the drive. The gain signal
carries across drives and clears when a window reads in range. The stored reason in `FordLateralSupport`
is cleared at every start and rewritten when the check changes its mind.

Response time is not estimated. `commandedCurvature` and `measuredCurvature` are in the status every
frame, for offline analysis.

**Costs.** The thresholds are first values. The panda's count is every blocked message, not only
steering ones, so a refused non-steering message can show "degraded".

### 2.8 The delay learner's start

**Problem.** openpilot plans ahead by the delay (1.1). lagd starts at `steerActuatorDelay + 0.2 s`
(0.4 s on every Ford) and learns above 50 mph.

**Options.** Put a lag per row in the driving model's delay directly, start lagd from it, start lagd
from the delay lagd learns on that module's drives, or leave it.

**Evidence.** Rows used to carry a lag, made four different ways, each fitted (a model step with a
fitted servo, fleet delay fits, dead time plus a time constant, a model with a fitted plant;
`assumptions-audit.md` item 25). None of them was lagd's delay: lagd's own estimator, run over logged
drives with its own gates, learns 0.50 s on BD, where the old row lag at 100 km/h was 0.39 s
(`assumptions-audit.md` §3.4). Nothing steered by the lag, so it is gone. A row carries one timing
number: where the delay learner starts, measured on logged drives of that software. It is a measurement
of car and module together, not a firmware value, and the learner replaces it.

**Choice.** Start lagd from what lagd itself learns on logged drives of the row's strategy (median over
vehicles), because a learnt value still takes over and only the starting point changes. The mechanism is
`steerActuatorDelay = start − 0.2 s` (`StrategyRow.learner_start`, returned by `delay_learner_start` and
set in `L/hosts/sunnypilot/safety_param.py`), and with the learner off that start is the fixed delay. The
rejected option would have bypassed `steerActuatorDelay`. Starts: 0.50 s on the 2021-23 F-150 BD row,
0.34 s on BC, 0.43 s on the Lightning, 0.27 s and 0.31 s on the 2021-22 and 2023-24 Mach-E, 0.30 s on
LX6C, 0.41 s on the Explorer and 0.32 s on the Maverick. Rows with no logged drive (the Bronco Sport and
Escape 2025 among them), the fallback and the stock path keep openpilot's start.

lagd keeps its learnt delay across drives while the car model is the same. After the steering module's
software changes on the same car (another F188 or F10A than the last start's), BluePilot has lagd start
again from the row's start (`steering_software_changed` in `bluepilot/selfdrive/car/ford_lateral_support.py`,
called from `lagd.retrieve_initial_lag`); otherwise a BC → BD update would run about 0.16 s short for
about 20 minutes of highway turning.

**Not run:** a model replay. The delay reaches the driving model as an input, and a replay needs a
route's video.

### 2.9 Curve speed, and the planner's clip from the module's limit

**Problem** (approach I in 2.1). Where the module's own lateral-acceleration table is below the panda's
cap, it is the ceiling, and the only lever left is speed. On the CAN FD F-150 rows that is above about
109 km/h, where the table falls under the 2.41 m/s² cap. On a CAN row with a table it is every speed:
the 2022-24 Maverick's 2.3 against 3.6.

**Choice.** Tell the planner the limit, so a turn the module won't take shows as a warning instead of
being clipped silently. The status publishes `lateralAccelLimit`: the panda's cap, or the row's limit
where that is lower. controlsd's curvature clip uses it where it is below openpilot's 3 m/s², so a
request beyond it shows as "turn exceeds steering limit". Road roll can't widen it
(`test_road_roll_does_not_widen_the_steering_limit`). sunnypilot's vision curve speed (SCC-V) plans for
it where it is below its own 2.0 m/s² (`B/openpilot/sunnypilot/selfdrive/controls/controlsd_ext.py`,
`.../smart_cruise_control/vision_controller.py`).

**Cost.** Most rows' limits drop under 2.0 only from about 130 to 160 km/h, so at legal speeds curve
speed changes nothing there. The exception is the 2022-24 Maverick row, under 2.0 from about 116 km/h
(72 mph). The limit is read at the car's speed, which errs low. Not checked on a
drive.

### 2.10 Two steps, one release

**Problem.** Path angle's numbers rest on firmware reads and the twin, and no drive has sent it with this
build's controller. Two rows' gains have been measured on the road with earlier BluePilot (table below).
The review found a busier wheel on real requests (`clean-build.md` §14.1).

**Choice.** This version switches path angle on for every row whose module's numbers were read from its
files, for BC with BD's, and for the Explorer's second pair with the first pair's (2.1, 2.11). It is ready to release once testing is done. The reason is the BluePilot dev
group's 50,000+ miles of path-angle-only control, and the two rows measured on a road (table below).

- **What testing checks, per family:** a drive where `drive_check.py` (3.4) finds the measured
  path-angle gain within ±30 % of the row's below 20 m/s, where path angle does its work, and a
  low-speed turn with hands on, released at the exit, that shows no wind-up (path angle left over that
  keeps the car turning after the driver lets go). The BD F-150 route in the table below already passes
  the first part.
- **A row that fails** goes back to curvature only: `path_angle=False` in its row. That also clears bit 6
  and the row's gain curve for that row.
- **The window test** runs every switched-on row with path angle on: zero refused frames.
- **The other two versions stay as they are.** The base, `clean/ford-lateral`, has every switch off. The
  test branch `clean/ford-lateral-drive1` has path angle on for the Explorer row only, both its pairs.

**What has been on a road.** For each row a car can reach on this branch. "Logged" drives were steered
by comma's stock command (curvature only); the path-angle measurements used earlier BluePilot builds.

| Row | Control law read | Code run in an emulator | Logged drives, curvature | Path-angle gain on a road |
|---|---|---|---|---|
| F-150 2021-23 BD | yes | yes: the twin is held to it | 1 truck | one truck, bp-7.0: 0.93, 1.01 and 1.00 of the row's at 5-12, 12-20 and 20-45 m/s (research note `pathangle-route`) |
| F-150 2021-23 BC | no (no file) | no | 33 trucks | none |
| F-150 Lightning | yes | its code is BD's instruction for instruction; its own image not run | 2 trucks | none |
| Explorer 2020-24 | yes | yes: M1MC-AC with MB5C-BD's start-up and whole lateral step, on two emulators that agree on every tick; L1MC-AL's start-up and K2 lookup. MB5C-BA's numbers are still BD's | 9 trucks | one truck: 0.97 of the model's on an angle-mode drive, 0.95 with Ford's own camera (`twin.md` §10.4) |
| Ranger 2024-25 | yes | its controller chain | 1 truck | none |
| Maverick 2022-24 | yes | its K2 and distance lookups only, not its start-up or lateral step | 23 trucks | none |
| Escape 2020-22, Bronco Sport 2021-22 (LX6C-AL) | yes | as the Maverick's | 3 cars | none |
| Mach-E 2021-22, 2023-24 | yes | no | 6 cars | none (curvature rows) |

The truck counts are those lagd's estimator found usable windows on (`assumptions-audit.md` §3.4), and
33 for BC (`bc-fleet.md`). Rows for the Transit, the 2025 Escape and the cars the owner's guide lists as
not steered can't be reached on this branch: openpilot here has no platform or fingerprint for them, or
the build doesn't support them.

**Release-notes wording** (`clean-build.md` §14.5 item 5): engaging in a curve starts from straight and
builds in at the module's own rate on a row without path angle, about 5 to 6 s at 30 km/h and about 2 s
at 60 km/h. On a row with path angle it is faster: the twin of the 2021-23 F-150's module puts it at about
2.7 s at 30 km/h and about 1 s at 60 km/h. The other rows haven't been modelled for this case. Hold the
wheel through the first seconds. These figures come from a model whose steering servo is read from the
module's code and not yet run, with a lag fitted to drives. For BC: "steers with BD's path-angle numbers, with timing measured on
BC trucks."

**Why engaging in a curve is slow.** comma's rule says the inactive frame must be all zero, so the
module's copy of c2 starts at zero, and the panda's check restarts from zero too. Any controller that
obeys the rule starts from straight. On newer-library rows, frames before the module reports control
move at the panda's step, not the module's rate (`follows_at_once_before_control`, `firmware-core.md`
§4.2), which shortens this on the rows with path angle on.

### 2.11 The BC row, from fleet behaviour

**Problem.** 42 of 45 logged 2021-23 F-150s run ML3V-14D003-BC. Our row and twin are BD. No BC file was
found (`bc-sources.md`).

**Evidence** (`bc-fleet.md`). BC trucks answer curvature at gain 1.00 to 1.02 by speed band (33 trucks),
the same as BD, and engage the same way. They are **faster**: 0.27 s against 0.57 s at 60 to 75 mph, a
delay fit from command to yaw on the fleet, and the difference sits in the module, not the vehicle. That
0.57 s is the fleet's BD trucks on the same fit. Path angle, path offset and
curvature rate can't be tested from public data: every engaged second on every truck is curvature only.

**Choice.** A full row with BD's preview, c2 rate, c1 rate and panda gain curve, so path angle is on.
That is the project's choice: BC's files were never found, its curvature behaviour is BD's, and it is the
level most of these trucks run. The rest is BC's own or comma's: comma's stock ramp type and precision,
the BD row's lateral-acceleration limit (also by that choice), lagd's start at 0.34 s (measured on logged BC
drives), and `ANY_DATA_FILE` (2.2). Like every row it doesn't smooth the request, so it drops comma's `anti_overshoot`, which stock
applies on these trucks (2.3). BA, the level before BC, keeps "older than Ford's current level".

**Cost.** It rests on behaviour, not files, so it is the one row without the control law read (3.3).
Whether BC's preview, rates, K2 table and limiter are BD's is not confirmed. BC trucks reach their limiter
at lower lateral acceleration than BD's table predicts (`bc-fleet.md` §3). A BC data file
(ML34-14D007-BDH or -BKH) or a BC drive with path angle through `drive_check.py` would settle it. It applies only once the module answers F10A.

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
   `LateralInputs`: `pandaLateralAllowed` and `safetyTxBlocked` from `pandaStates`, and `lateralDelay`
   from lagd, filled in
   `B/openpilot/sunnypilot/selfdrive/controls/controlsd_ext.py`. If it returns False, the stock path
   runs. At `STEER_STEP` (20 Hz) it calls `bp.step_lateral(CC, CS)`.
3. **Request rule** (`L/core/request.py`). Ask only while openpilot wants lateral control, the panda
   allows it, and the module says Available or Active. The module's state (`pscm_status`) is read from
   Lane_Assist_Data3_FD1 by `O/opendbc/car/ford/carstate.py`, on CAN FD only; Denied is its own state. On
   CAN, where there is no status, the rule follows openpilot and the panda. Otherwise the inactive frame
   is sent: all zero, ramp type 0, precision 1. The reasons: a module configured for Ford's hands-free
   driving stays Unavailable while it is asked, a refused frame is a missing frame to the module, and the inactive
   frame always passes (`test_never_asks_through_unavailable`).
4. **Limit** (`L/core/carcontroller_ext.py` `step_lateral`). On the fallback on the F-150 MK14 and
   Bronco Sport MK1 platforms, the request first goes through comma's `anti_overshoot`, every frame as
   stock does. Then it is clipped to `lat_accel_limit / v²`.
5. **Path** (`_path`). Without path angle, `want` goes straight through. With it:
   - curvature moves toward `want` at the row's rate, so it always equals the module's own copy. The
     position is kept unrounded (`clean-build.md` §14.1);
   - the controller's copy of the module's preview time T drops at once when curvature rises and
     recovers at `t_rise` per second, as the module's does. `gain = K2(v, T)`;
   - path angle is `(want − c2)/gain`. It may cancel whatever the slow curvature still holds back,
     plus at most what the row's path-angle rate can take back in 3 s. While the driver steers it gets
     nothing beyond cancelling, so nothing is left over when they let go (the wind-up rule);
   - path angle stays at zero until it would be at least 1.5 wire steps, so it doesn't flip sign frame
     by frame (the dead band), and it moves at the row's path-angle rate.
6. **Panda window** (`L/core/limits.py` `window`, `fit_path`). Starting from the last command the panda
   saw (zero after any inactive frame), it allows one jerk step either way within the cap, for the
   curvature signal and for the implied curvature. The controller reads K2max, from the same gain curve
   as the panda, 1 m/s below the car's speed, so its K2max is at or above the panda's, which reads it at
   its lowest recent speed sample. It also keeps the panda's range of the module's copies (2.4). Curvature
   is fitted first, then path angle into what is left. A car whose path angle is off never gets a
   path angle step from this fit, even when a jump in the speed reading leaves no whole step that fits.
   So every frame is inside the panda's check by construction, given that the panda saw the last one.
   **Restart:** if a frame is lost or refused, the panda restarts from zero and the controller doesn't.
   When the host reports a rise in `safetyTxBlocked` while the panda allows steering, the next frame is
   the inactive frame, so both restart from zero (`test_one_lost_frame_doesnt_stop_the_steering`).
   Before, every later frame was refused until the request came back near zero. The panda never took the
   refused frame, so its range of the module's copies holds only frames it accepted. On the restart the
   controller's narrow copy starts again from the inactive frame, and its wide copy keeps everything and
   takes back where it was over the last 0.4 s, without moving that frame: on a falling ramp it had followed
   frames the panda refused, while the panda's stays with the last frame it took. So they stay inside and
   around the panda's on a rising and on a falling ramp (`test_after_a_refused_frame_the_copies_match_the_pandas`,
   `test_after_a_frame_lost_on_a_falling_ramp_the_copies_still_match_the_pandas`).
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
    `c2 + K2max·c1`, with K2max the module's own gain curve (bits 0 to 2) or the single curve, read at
    the lowest recent speed sample, and the cap at that speed minus 1 m/s;
  - the cap on what the module's copies can steer for once the frame is sent (2.4);
  - comma's lateral jerk limit (about 3.6 m/s³) on both, against the last command;
  - comma's real-time rate check and the speed-mismatch check;
- while not steering, both signals must be zero, and there is no steering while controls aren't
  allowed;
- after any violation, and while lateral control isn't allowed, the last command is zero, so the next
  ramp starts from zero. There is no latch; three tests pin that.

The caps are about 3.6 m/s² on CAN and about 2.4 m/s² on CAN FD (`FORD_BP_LIMITS`). The CAN one is
comma's. The CAN FD one is BluePilot's own: comma's check in this branch has one cap, 3.6, for both
(`O/opendbc/safety/lateral.h`). Its reason, more steering force on CAN FD, is an old comment of comma's
that nothing of ours shows (`assumptions-audit.md` item 3). So "Disable BP Lateral Control" raises a CAN
FD car's cap to 3.6.

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
4. **Check against the firmware run in an emulator.** For the 2021-23 F-150's software BD the twin was
   checked against the firmware's own code in an emulator: 595,165 block calls, no mismatches. The
   Ranger's and the 2024 Mustang's controller chains were checked the same way. No other module's code
   has been run; the Lightning's is BD's instruction for instruction, but its own image has not been run. A row's
   evidence level says which: **executed**, **read** or **unread** (3.6). Every number in a row is
   generated from the module's files by one script, into `L/core/module_tables.py`, with the SHA-256 of
   each file block it was read from, and a check re-reads the files (`gain_tables.py --check`). The
   rows refer to the generated tables and type none of their numbers (`tests/test_module_tables.py`).
   A row with path angle on also needs its module's gain curve and c2 rate in the panda's table, generated
   from the same reading into `L/safety/ford_gain_curves.h` and pinned to the Python tables (2.4).
5. **Row in `strategy-table.md`,** with every value marked proven, supported or hypothesis. Then a
   `StrategyRow` in `L/core/strategy.py` that cites its note.
6. **Tests:** a `test_strategy.py` case for its pairs and for what must not match it; the window test
   with the row, path angle on and off; and `table_coverage.py` re-run.
7. **Path angle on:** when the row holds a preview with both distance clamps at all eight speed points,
   a c2 rate and a c1 rate, and its module's gain curve is in the panda's table. A family's drive check
   (2.10) can take it off again.
8. **The block above the row** in `L/core/strategy.py`, and its line in the table below: where the
   numbers come from, what is confirmed and how, what isn't, and what would confirm or change it.

**Evidence each level needs:**

| Level or field | Needs |
|---|---|
| curvature | the control law read from that strategy's software, and the exact pair. BC is the one exception (2.11) |
| full | the above, plus a comparison with the twin or a drive (the level's name; what the comparison rests on is the evidence level, 3.6) |
| executed (evidence) | the module's lateral code run in the emulator, with recorded traces the twin is tested against. A drive checks the result; it does not make a row |
| a preview in the row | both distance clamps at all eight speed points. RZ1C and PZ11 have four, so their path angle stays zero |
| the path-angle switch | a preview with both rates, and the module's gain curve in the panda |

Unverified numbers err toward a high K2, so less path angle is sent, and a low c2 rate, so the
module's copy of the curvature never falls behind the controller's.

**Evidence per row, and what would change it.** The same as the block above each row in
`L/core/strategy.py`. "Read" means from the module's own software files; "twin" is the model of 1.1;
"logged" drives were steered by comma's command, curvature only.

| Row | Numbers from | Confirmed | Not confirmed | What would confirm or change it |
|---|---|---|---|---|
| F-150 2021-23 BD (path angle) | ML3V-14D003-BD with ML34-14D007-EDL; -BDL holds the same lane-centering numbers | read; the twin held to the firmware's code run in an emulator; path-angle gain on a road, 0.93 to 1.01 of the row's up to 96 km/h | path angle above 96 km/h; a drive on -BDL | a drive above 100 km/h through `drive_check.py`; any drive on -BDL |
| F-150 2021-23 BC (path angle) | no file of BC read: BD's numbers, by choice; BC's earlier response measured on 33 trucks | curvature gain and response, from those trucks | that BC's preview, rates, K2 and limiter are BD's; BC trucks reach their limiter at lower lateral acceleration than BD's table | reading ML34-14D007-BDH or -BKH, or the BC program; a BC drive with path angle through `drive_check.py` |
| F-150 Lightning (path angle) | RL38-14D003-AA with RL38-14D007-DD or -AD | the F-150's code instruction for instruction and its values byte for byte, so the F-150's executed code and drive carry over | its own image run; no Lightning drive with path angle | a Lightning drive through `drive_check.py` |
| Explorer 2020-24 (path angle) | M1MC-14D003-AC with MB5C-14D007-BD; L1MC-14D003-AL has the same code | executed: M1MC-AC with MB5C-BD's start-up and whole lateral step (two emulators), L1MC-AL's start-up and K2 lookup; path-angle gain on a road, 0.95 to 0.97 of the model's | MB5C-14D007-BA's numbers (not in the archive; still -BD's); the supplier code after the library | reading MB5C-14D007-BA; a drive with this build through `drive_check.py` |
| Ranger 2024-25 (path angle) | RB3C-14D003-AA with RB3C-14D007-ECC | read; its controller chain run in the emulator | no drive. The row most sensitive to a gain error: 43 % off puts the car 0.5 to 0.7 m off in town curves | a Ranger drive through `drive_check.py` |
| Maverick 2022-24 (path angle) | NZ6C-14D003-AL with NZ6C-14D007-ABD | read; not run (the twin for it is a translation of the same reading) | its path terms on a road (its path angle stays near zero: a fast curvature rate) | a drive through `drive_check.py` |
| Escape 2020-22, Bronco Sport 2021-22 (path angle) | LX6C-14D003-AL with LX6C-14D007-CAH | read; not run (as the Maverick's); one public Escape on this level driven | that Escape's data file; no drive with this file known | a drive with LX6C-14D007-CAH known, through `drive_check.py` |
| Bronco Sport 2025 (and the 2026 Maverick on its row), Escape 2025 | RZ1C-14D003-AC, PZ11-14D003-FC with their data files; SZ1C-14D003-AE with -AAG | read; not run | preview tables not in the row, so no path angle; no drive | the full preview tables from the firmware, then the window test and a drive |
| Mach-E 2021-22, 2023-24 | LJ9C levels and data files in the rows | every lane-centering value read; the delay learner's start from logged drives | its path terms, in a model or on a drive | a model run of its path terms, and a drive |
| F-150 2024-26, Explorer 2025 and 2026, Mustang 2025, Mach-E 2025, Mustang 2024, Transit 2024 | their programs and data files | the control law, read; the 2024-26 F-150's library and limiter executed, the 2024 Mustang's controller chain executed | no drive: curvature only | a twin run or a drive; then preview tables for path angle |
| Maverick 2025, Super Duty 2023 and 2025 (fallback) | their programs | the program read | Maverick 2025: its data file; Super Duty: a model of its law | the SZ1C data files; a model of the Super Duty law |

### 3.4 Logs and the drive check

- **Every start** logs the read (`carlog` "BluePilot steering module read"), the support record
  ("BluePilot steering support"), a part-number change ("BluePilot steering module changed", with old
  and new) and any plan mismatch ("BluePilot lane message plan").
- **Every frame:** `controllerStateBP.lateral`, with the request, the command, both signals, measured
  curvature, whether the command was limited, the module's state, the limit, the capability and the
  health check.
- **The drive check:** `scripts/clean-build/drive_check.py <rlog or route dir>... [--json out.json]`, or
  `--selftest`, in the tree's venv. Per speed band (5-12, 12-20, 20-45 m/s), hands off, it fits
  `measured(t + delay) = a·curvature + b·(commandedCurvature − curvature) + offset` at the best delay
  (0-0.8 s). Here `curvature` is the c2 sent and `commandedCurvature` is `c2 + K2·c1`, both status
  fields, so b is the path-angle gain against the row's. It passes where |b − 1| ≤ 0.30 with at least 400 frames of path angle at work.
  The drive check confirms the car follows the module's own numbers. It is not what switches a row on.
  It does not show the numbers are
  the firmware's: that is the evidence level (3.6). It also
  fits small and large path angles separately. The self-test recovers gains of 1.0, 0.8 and 0.55, and
  fails the 0.55 case. No route of this build exists yet. The BD route in 2.10 was checked with a variant
  that builds the same arrays from the bus (research note `pathangle-route`).
- **Replay:** `scripts/replay/cleanreplay.py` (with `run_clean.sh` and `analyze_clean.py`) runs these
  files with the compiled panda in the loop. `--pscm` gives a what-if row.

### 3.5 Tests

(Counts as of the fix round that added `tests/test_module_tables.py`: library 122 OK.)

Python environment: `UV_PROJECT_ENVIRONMENT=~/tools/venvs/clean-build uv sync --frozen --extra testing
--extra tools`, then `pyyaml`, `pytest` and `cantools` installed into it. Then `scons -j9` at this
repo's root. The commands below call that environment as `.venv/bin/python`. Whole suites are
heavy: run them in CI or on a large machine, not on a shared small one. From `opendbc_repo/` with
`PYTHONPATH=$PWD`:

```
.venv/bin/python -m unittest discover -s opendbc/bluepilot_lateral/tests -t .          # library: 141 OK
(cd opendbc/safety/tests && ../../../.venv/bin/python -m unittest test_ford test_bluepilot_lateral)  # 662 OK, 107 skipped
.venv/bin/python -m unittest opendbc.car.tests.test_car_interfaces opendbc.car.ford.tests.test_ford  # 284 OK
bash opendbc/bluepilot_lateral/safety/tests/bluepilot_coverage.sh                       # 100 % of both headers and modes/ford.h
```

Also: `opendbc/sunnypilot/car/ford/tests` (75 OK), ruff and ty on the library, MISRA (`test_misra.sh`,
passes), mutation testing (`L/safety/tests/bluepilot_mutation.py`, 127 of 127 killed, 6 not buildable), and the bluepilot
tree on a built tree:

```
tools/test_runner.py bluepilot openpilot/sunnypilot/selfdrive/controls/tests openpilot/sunnypilot/sunnylink/tests \
  openpilot/selfdrive/ui/bp openpilot/selfdrive/controls openpilot/selfdrive/selfdrived openpilot/selfdrive/car
```

That gives 545 passed, 1 skipped and 1 error. The error is in `test_ui_frames`' harness: a `params.cc`
log line lands on the child's result line. The library, safety, coverage, MISRA and mutation counts
above are from this version's heads; the car counts from the second fix round; the bluepilot tree's and
the others are from `clean-build.md` §14.4, at library `ca0b134`, and
weren't re-run since.

What the main tests guard against:

| Test | Guards against |
|---|---|
| `L/safety/tests/test_ford_bluepilot_window.py` | the real car interface and controller leaving the compiled panda's window: speed grid, engaging in a curve, hard acceleration and braking, random requests, module Unavailable/RampOut/Denied, the panda withdrawing; CAN and CAN FD, every row with path angle on (a guard fails if one is missing), rows with it off, fallback and stock path. Zero refused frames |
| `L/safety/tests/test_ford_bluepilot.py` | breaking the panda rule: `test_curvature_cannot_hide_behind_an_opposite_path_angle`, no latch, restart from zero, bit 6 off refusing path angle, upstream's `test_steer_allowed` sweep; each module's gain curve to the wire step, no curve meaning the single one, the cap and jerk limit the same with every curve; a sender walking c2 to the opposite cap behind a rising c1 (what the module's copies steer for stays inside the cap) |
| `L/tests/test_strategy.py` | wrong matches: exact pairs, generations never sharing, BC, superseded reasons, read-only reads, retries, the switch on exactly for the rows that can use it, BC with BD's numbers |
| `L/tests/test_controller.py` | breaking the request rule, the exact c2 rate, the first frame, asking through Unavailable; every row as built: bit 6, its gain curve and path angle sent on the switched-on rows only; a sharp curve entry held back by the gain bound (97 % or more delivered) |
| `L/tests/test_limits.py` | a gain curve drifting from the panda's table, rising with speed, falling under what its module steers for, or above the single curve |
| `L/tests/test_recorded_turns.py` | path-angle wind-up on recorded low-speed turns (`tests/data/low_speed_turns.csv`) |
| `L/tests/test_health.py` | a false "degraded" after a disengage; breaking the gain window's rules |
| `L/tests/test_wire_clamp.py` | signals wrapping past their range (failed first against `f267286`) |
| `L/tests/test_lane_message_plan.py` | the plan drifting from the panda's transmit table and the car controller |
| `L/tests/test_safety_param_bits.py` | Python's bits drifting from `ford_bp_init`'s |
| `L/tests/test_core_imports.py`, `test_bluepilot_upstream_copies.py` | absolute host imports; copies of upstream functions going stale |
| `B/bluepilot/selfdrive/car/tests/test_ford_lateral_support.py` | the support row, notices and `record_start` breaking with the real Ford interface |
| upstream `test_ford.py` | the stock check changing: it passes unchanged |

### 3.6 Evidence levels

Each number in this build rests on one of five things. The project's rule is that the firmware is the
source of truth, running its code in an emulator proves a reading, and drives only check. So a number
read from code but never run, or fitted to drives, is labelled that way and counted as open.

| Level | Means | Numbers in this build |
|---|---|---|
| **Generated from firmware bytes** | a script reads it from the module's flash at its address; a check re-reads it | every row's preview (T, both distance clamps, the K2 table), its curvature and path-angle rates, and its lateral-acceleration limit on its own speed axis (on the newer library as the module applies it, in 1/4096 steps: 0.0998535 at 220 km/h where the table holds 0.0999756, executed on BD and RL14) (`L/core/module_tables.py`, with the SHA-256 of each source block); the panda's per-module gain curves and module rates (`L/safety/ford_gain_curves.h`) |
| **Executed** | the module's own code was run in an emulator, and the model or the build is held to that run | the 2021-23 F-150 BD library: its eight control blocks, the mode machine, the limiter, the input scalings and the wire's ranges; the order of the two distance clamps; the request rule (ask only while Available or Active); `K2 = f(d)` read straight between points, with the module's fixed point 0.45 % at most; the copies following at once before control. The Ranger's and the 2024 Mustang's (PR3C) controller chains. The 2020-24 Explorer's start-up and whole lateral step (M1MC-AC with MB5C-BD; L1MC-AL's start-up and K2 lookup): its K2 table, distance clamps, preview time from 0 at power-up, rates, and no lateral-acceleration limit. The preview time at start on BD and the Ranger (T max). The 2024-26 F-150's (RL14-AC) library and limiter, ECD, BCD, CCD and BCA alike (BAD, EFD and EAD not run). The Maverick's and LX6C's K2 and distance lookups. BD's speed chain behind its K2 lookup (wheel and brake-module speeds, slew, 1/64 km/h) |
| **Read, not run** | a person or agent read the decompiled code; nobody has run it | every other row's control law (V1MC, R1MC, SR3C, the Mach-Es, the C2 programs, the Maverick 2022-24 and LX6C beyond their lookups, the Transit, the Super Duty); the Lightning, whose code is BD's instruction for instruction and whose parameters are BD's byte for byte, but whose own image has not been run; the 2026 Maverick and Explorer rows; the steering servo behind the module's angle target; the panda's single gain curve, an envelope over the models' sweeps |
| **Measured on drives** | from logged drives: allowed for what no firmware holds (the vehicle), and as a check | the delay learner's start per row; the health check's thresholds; the BC row's whole basis (no BC file was read); per-vehicle gain spread; the path-angle gains on the two driven rows (2.10) |
| **Ours, or outside** | a design choice of this project, or a number from comma or a standard | comma's caps and jerk limit; the lower CAN FD cap; path angle's unwind time, dead band and restart hold-off; the controller's copy timing (0.8 and 1.25 frames); the 6 % speed allowance in the panda's margin, where the module's speed chain was not run (every program but BD), and the 30 ms sample age and 3 m/s² in BD's; openpilot's default delay |

Each row says its program's level as data (`StrategyRow.evidence`: executed, read or unread), as the
project's architecture audit has it. Nothing switches on it yet: a row's path-angle switch is a separate
decision (2.10).

## 4. Gaps, extending, upstream

### 4.1 Known gaps (`clean-build.md` §6, §14.6)

- F10A and F124 haven't been read from a real truck. Without F10A every car is on the fallback.
- Coverage: 37 of 63 archive records and 25 of 67 logged vehicles have no exact row. Rows are wanted for
  PZ11-14D003-EA and the unread files. RZ1C and PZ11 need preview tables. Calibrations aren't pinned
  yet.
- No route exercises a row's own numbers yet. This build's controller has never sent path angle on a
  drive. Two rows' path-angle gains are measured with earlier BluePilot (2.10). BC's numbers are BD's,
  unread (2.11).
- The stock path ("Disable BP Lateral Control") still has 143 refused frames over seven routes, inside
  comma's band. That is outside this build's goal.
- IQ.Pilot hasn't run its coverage step (it needs clang-18 and mull) and doesn't do the start-up read.
  StarPilot is untested. Neither host reads the module, so every car there is on the fallback.
- What to do when a `check_plan` mismatch is logged: today it is only logged.
- Not run: the model replay for the delay learner's start, and `tools/test_on_host.sh` end to end.
- If modules don't answer F10A in the default session, there is no plan B yet.
- **Tight slow turns on curvature alone.** On the fallback and on rows that steer by curvature alone,
  the car cannot make a turn tighter than about a 50 m radius, and it takes about 7 seconds to build a
  50 m curve at 30 km/h. That is the limit of the curvature signal (0.02 1/m at most, executed in the
  module's input stage) and of the module's own pace (0.003 1/m per second), the same as with comma's
  stock code. On three test turns (20, 30 and 50 m radius at 15, 20 and 30 km/h) a model of the F-150,
  with a servo fitted to drives, leaves the lane on all three (`assumptions-audit.md` §3.11). The driver
  steers those turns. What would cure it is path angle on that row,
  which needs its module's code run first.
- Future improvements: camera and fault-code reads at start, the DE01 decode (`clean-build.md` §10).
- Not checked: installing on a comma four.

### 4.2 Extending

- **A row:** follow section 3.3. Keep exact pairs and mark the level by its evidence. Set `path_angle`
  True only when the row holds a preview, a c2 rate and a c1 rate and its module's gain curve is in the
  panda's table (`test_strategy.py` checks it), and add the row to the window test's `SWITCHED_ON`.
  Write its evidence block (3.3).
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
- an argument for the fallback half, which is nearest to upstream: comma's command, its smoothing
  included, without the clip, and with BluePilot's lower CAN FD cap.
  Removing the band is a relaxation that has to be argued with the acceleration-and-jerk bound;
- for the path-angle half, comma accepting a c1 rule (the checks in 2.4, with each module's K2 curve
  read from its firmware and the bound on its slewed copies), and confirming drives;
- upstream's `test_ford.py` already passes unchanged, and MISRA and mutation testing pass on the
  headers.
