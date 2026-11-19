# The clean Ford lateral build: a developer's guide

For people who work on this project and want to know how the Ford steering works and why each decision
was made. Section 1 is the high level. Section 2 takes one decision at a time. Section 3 is the low
level: the code path, the panda's checks, how a table row is made, and the tests. Section 4 is gaps,
extending and the path upstream. The owner's version is [USER_GUIDE.md](USER_GUIDE.md); installing is in
[INSTALL.md](INSTALL.md).

Claims are as of bluepilot `7fb69ebfb`, opendbc `88701a67`, library `ca0b134`. Research notes are cited by
file name (for example `approach-comparison.md`, at `notes/approach-comparison.md`): they are the project's
research notes (to be published), and the scripts and twin they name are in the same set. Where a claim
wasn't checked it says so.

Paths: `L/` = `opendbc_repo/opendbc/bluepilot_lateral/` (the library), `O/` = `opendbc_repo/`, `B/` = this
repo's root.

## 1. The high level

### 1.1 The problem

openpilot steers a Ford through the steering module (PSCM), using the lane-centering message Ford's own
camera sends: LateralMotionControl on CAN, LateralMotionControl2 on CAN FD. The message carries a path:
curvature (c2), path angle (c1), path offset (c0) and curvature rate (c3), plus a mode, a ramp type and
a precision bit. The module turns the path into a steering-angle target with its own controller, limits
and driver override.

The question this build starts from is why the same command gives different results on different Fords.
The research's answer is that most of the variation is **across vehicles, not within one truck**:

- **Each module runs its own strategy (F188) with its own data file (F10A).** One program serves
  several vehicle lines, one platform name covers different modules, and Ford changes both over the air
  and at the dealer (`strategy-table.md`, `cfg-diff.md`).
- **The binding limit below about 80 km/h is the module's own rate limit on its copy of c2**, 0.003 1/m
  per second on the decoded F-150-family strategies. That is 0.2 m/s³ of lateral jerk at 30 km/h. No
  curvature-only sender can follow a town curve that builds faster than that, whatever its lag or outer
  loop (`approach-comparison.md` §1–2, `firmware-core.md` §4.2).
- **The module's path law is pure pursuit at its own preview distance d**: it steers for
  `c2 + K2(d)·c1 + K1(d)·(c0 + I)`, with K2 ≈ 2/d on the newer lateral library and 4/d on the older one,
  and d from per-module tables by speed and curvature (`firmware-core.md` §4.3, `explorer-firmware.md`,
  `rl14-firmware.md` §5). BluePilot's old angle mode sent κ·v in c1 and hand-tuned per-speed gains that
  turned out to be one truck's 1/K2 table inverted (`twin.md` §1 item 5, §7).
- **The module steers to its yaw-rate signal.** A sensor offset of 0.1 °/s is a standing curvature error
  the module never removes (`twin.md` §1 item 2). bp-dev's clip of the command to measured curvature
  ±0.002 tied the command to that biased measurement.
- **bp-dev's panda had a latch** that switched every check off for 3 s after any zero frame, and let
  curvature move at up to 7.3 m/s³, twice comma's limit (`change-analysis.md` §0, §1.2).

### 1.2 The goal

Understand and implement the module's own steering closely enough that every supported Ford behaves the
same. The build is one clean branch of the control code, with the workarounds, hacks and anything else
unnecessary removed. The code is meant to be upstream-ready: one setup for every Ford that can be
supported.

### 1.3 The shape of the solution

1. **One controller, on curvature**, from `CarControl.actuators.curvature` alone. No per-car tuning
   knobs.
2. **A table keyed on the module's own answers**: exact (F188, F10A) pairs, read fresh at every start.
   A row carries that module's constants. No row means the **fallback**: comma's stock command without
   its clip to measured curvature.
3. **Path angle (A′)** on rows whose preview is known: curvature moves at the module's rate, and path
   angle carries what that rate still holds back, in the module's own K2. A per-row switch, **off on
   every row** in this build.
4. **The panda bounds the command by lateral acceleration and jerk** (comma's numbers), on the curvature
   signal alone and on the implied curvature at the highest K2 any module has. A startup bit allows path
   angle only for a row that has it on.
5. **A read-only health check** and a **support row** in the settings that say how the car is steered
   and why.

## 2. The decisions

Each section gives the problem, the options, the evidence, the choice and its cost.

### 2.1 Curvature base, path angle as a per-row switch (A′)

**Problem.** Curvature alone is slow in town, because of the module's c2 rate (1.1). bp-dev's answer,
angle mode, behaved differently on every vehicle line.

**Options.** `approach-comparison.md` §3 scores 17 approaches (A–Q) on native feel, authority across
speeds, sameness across Fords, fit with openpilot, robustness and cost. The main ones:
- A, the earlier plan: curvature plus a lag value and c0/c1 lane positioning per row. c0 feeds the
  module's integrator, which never forgets while engaged (`twin.md` §1 item 6), and its gains differ
  3–10× between trucks.
- B, the full path like Ford's camera, lane-relative. That replaces openpilot's plan with Ford's soft
  highway lane follower: 0.6–2 m wide in town curves in the model.
- D, today's bp-dev modes. N, path angle only, scaled by the table: a table error becomes a
  steady-state gain error.
- F (interceptor), H (park-assist angle request at speed) and K (changing the module's tables) all
  leave Ford's lane-centering interface and fail comma's rules (comma, "Safer control of steering",
  2020-10-27).
- **A′**: c2 carries the request. c1 carries only `(want − c2) / K2`, the part c2's rate still holds
  back. c0 = c3 = 0.

**Evidence.** In the twin, A′ makes a town-speed step 2–3.5 times faster than curvature alone
(`clean-build.md` §13), with steady state on c2, so the gain stays 1.0 without a table. A wrong K2 by
0.5–2.8× costs at most 0.7 m in a transient and never the steady state (`approach-comparison.md` §3,
§4). Path angle fades by itself with speed: at 130 km/h comma's jerk limit already allows less than the
module's c2 rate, so path angle adds nothing (`clean-build.md` §13). The stock camera sends c1 all the
time, so the module isn't being asked for anything new.

**Choice.** The project chose A′ because it is the only option that gets past the module's c2 rate in
town while keeping the steady state on curvature, so a wrong K2 costs a transient and never the gain
(evidence above). The lane offset (c0) is removed for now, because it feeds the module's integrator and its
gains differ 3–10× between trucks (option A). Path angle is a constant in each row
(`StrategyRow.path_angle`), off everywhere (`test_path_angle_is_a_switch_per_row_and_off`).

**Trade-offs.**
- Recorded requests through the F-150 twin make the wheel 1.2–2.6 times as busy below 12 m/s as the
  fallback, and path angle's sign flips 1.7–4.4 times a second where it is active, even with the dead
  band (`clean-build.md` §14.3). Whether that reads as responsive or twitchy is for the first drive. It is
  one reason path angle ships off.
- The controller keeps a copy of the module's preview time. Where the copy differs, it errs to a higher
  K2: less path angle, a little less tracking, never overshoot (`L/core/strategy.py` `Preview`).
- Older-library rows hold K2 as 4.17/d, the most the modules' ten-point table reaches over the rows'
  distances. Against the twin the row's gain is 1.001–1.045 times the module's, never below
  (`clean-build.md` §5).

### 2.2 Exact F188 + F10A pairs, not families

**Problem.** Which module is this? The fingerprint's platform name doesn't say. R1MC sits under the
2020–24 Explorer platform, and one program serves several vehicle lines.

**Options.** Key on F188 alone; on an F188 family with a default; or on exact pairs.

**Evidence.** `cfg-diff.md` §11 compared archive data files. Files that look alike differ: TL34-14D007-BDA
(the 157" F-150) has other state-machine thresholds, SL3V-14D007-BC (the 2024 Raptor) differs in 18 of 70
path values, and RL14-14D007-AE/-EE/-GE (Expedition and Navigator) in 19 of 70. The 2022–24 Maverick
and the 2021–24 Mach-E run the same control law with their own numbers, and the 2025 Maverick shares
its program with the 2025 Mach-E: neither may use the other's row.

**Choice.** A row lists exact pairs (`StrategyRow.modules`). Several data files share a row only where
the note shows equal lane-centering numbers (`test_data_files_with_the_same_numbers_share_a_row`). A
level across generations never shares a row (`test_the_mach_e_generations_never_share_a_row`). No
calibration (F124) is pinned yet, because F124 on a truck is unconfirmed (no vehicle read yet), and the 2022–24
Maverick's module has none.

**One exception, the BC row** (2.11): `ANY_DATA_FILE`, allowed by test only on a curvature row with
stock constants, where the data file can't change what is sent. Even there the module must answer F10A:
`match_strategy` returns "didn't answer" before it looks at rows when either key is empty.

**Trade-off.** Coverage. Of 63 factory records, 26 match a row today. Of 67 logged openpilot vehicles,
42 are on the BC row once their module answers F10A, 19 run a strategy with a row but an unread data
file, and 6 have no row (`clean-build.md` §14.3, from `scripts/clean-build/table_coverage.py`).

### 2.3 The fallback and "not supported"

**Problem.** What to send to a module nobody has read.

**Options.** A conservative guess at the module's slew and lag, or comma's stock command.

**Evidence.** For an unread strategy we don't know its rate. A 0.003 limiter fits the Maverick and
Explorer fleet data worse than none (`verify-cross-vehicle.md` §6, via `change-analysis.md` §3). comma's
command is the one with fleet evidence: gain 0.97–1.05 on 102 vehicles of 11 platforms
(`cross-vehicle.md`).

**Choice.** `FALLBACK_ROW`: curvature only, ramp type 0, precision 1, nothing of a module's own. It is a
row like any other, so the controller always reads one row. The fallback has no band either
(2.4). A level `not_supported` makes car init set dashcam only, for a module that takes no
lane message built here. No row uses it today. Rows kept on the fallback (Maverick 2025, Super Duty) are
in the table so the support row can say "known, and not yet supported" instead of "not in the table".
Replaced levels (`SUPERSEDED`) carry a reason the owner can act on: SSM 53105's wording for RL34-14D003-AD
with an RL34-14D004-E\* calibration (`ssm-53105.md`), "older than Ford's current level" otherwise and for
ML3V-14D003-BA.

**Trade-off.** Every car we have logs from ran the fallback in replay, because those logs predate the
start-up read (`clean-build.md` §9 item 10).

### 2.4 The panda bounds and the startup bits

**Problem.** comma's Ford check accepts curvature only and refuses a command more than 0.002 1/m from
measured curvature above 10 m/s. A′ moves the curvature signal away from the measurement on purpose.
The panda doesn't know a module's K2.

**Options.**
- Keep the band: incompatible with A′.
- Bound only `c2 + K2max·c1`, the proposal in `approach-comparison.md` §6. **Proven unsound**: a large
  curvature hides behind an opposite path angle. The sum stays small, while a module whose real gain is
  below K2max steers for most of the curvature. The first simulation run produced such a frame
  (`clean-build.md` §1 item 3).
- Bound both the curvature signal alone (K2 = 0) and the implied curvature at K2max, each by the
  lateral-acceleration cap and comma's jerk limit. What a module steers for is linear in K2, so both
  bounds cover every gain from 0 to K2max.

**Evidence.** comma bounds its angle-steered cars the same way, by acceleration and jerk with no
measured band (`steer_angle_cmd_checks_vm`). K2max was first 0.155 at low speed. The twin showed the
early 2024 Mustang (PR3C) at 0.25 and the Ranger at 0.167 (`scripts/clean-build/k2_envelope.py`), so it
is now `min(0.26, max(1.7/v, 0.03))` per rad. The floor of 0.03 is there because above 205 km/h `1.7/v`
fell under the Explorer row.

**Choice.** The third option, because the band is incompatible with A′ and the single sum bound is
unsound. There is no measured band on BluePilot's paths, the fallback included. Pinion yaw (K20) stays
removed, since nothing reads it without the band. Two startup bits in the second safety param
(`current_safety_param_sp`):
- **bit 5 `BP_LATERAL`**: BluePilot's check instead of the stock one. Cleared by "Disable BP Lateral
  Control".
- **bit 6 `BP_PATH_ANGLE`**: path angle may steer. Set only when the matched row's switch is on, so a
  bound that rests on the gains of read modules never applies to a module nobody has read (review
  finding 5).

Path offset and curvature rate must be inactive on every module.

**Trade-offs.**
- This is a relaxation against comma's Ford check, and it applies to the fallback too. A stricter
  choice would keep comma's check for unknown modules.
- One K2max curve for all: at 60 km/h the 2021 F-150's gain is 0.047 and the panda assumes 0.102, so it
  allows about half the path angle that module could take. Per-row bounds would need more startup bits
  (`clean-build.md` §10 item 3).
- The setting is read at car init only, because the panda is told then which check to run.

### 2.5 Removing the old settings and the lane offset

**Problem.** bp-dev had two controllers and 19 settings. **Evidence:** `change-analysis.md` §0–2. Most
patched effects of three root causes: the panda latch, the measured-curvature clip pinned by a biased yaw
signal, and angle mode's path-angle mapping. **Choice:** remove them all, since each patched an effect of
a cause this build removes (the latch, the clip) or replaces (angle mode, by A′, 2.1); the lane offset as
in 2.1. Removed: angle mode, the stall and hand-off pulses,
both human-turn resets, the predicted-curvature blend, lane positioning, the lane-change factors, the
post-lane-change limiter, the clip, the pinion source, and 18 of 19 settings. One setting is left:
`disable_BP_lat_UI`. The library is about 2,300 lines shorter. **Trade-off:** owners lose their tuning.
The lane offset comes back later as an offset in openpilot's plan, not in the car controller.

### 2.6 The start-up reads

**Problem.** openpilot reuses a cached fingerprint whenever the VIN matches. Ford changes module
software over the air.

**Choice.** `L/core/pscm_read.py` reads F188, F10A, F124, D02B (OTA counter), DE00–DE02 (As-Built,
logged) and EE80 (Ford's lane-keeping software version, logged) from the module at 0x730 at every
start. It uses service 0x22 after tester-present, the same shape as openpilot's own Ford query
(`test_reads_are_read_only`). The answers go into `CarParams.carFw` under brand `ford_pscm_live`, so they
are logged and never take part in matching. F188 and F10A are asked up to three times, because one lost
frame would otherwise put the car on the fallback for the whole drive
(`test_a_lost_answer_to_the_key_is_asked_again`). A silent module costs the start about 0.1 s per
identifier, plus 0.2 s more per key identifier. Only the steering module is read; camera status and
fault-code reads are future improvements, after a truck test (`clean-build.md` §10).

**Trade-offs, not checked:** whether F10A answers in the default session on a real truck (no vehicle read yet). If
it doesn't, every car is on the fallback. EE80's session isn't known either. It was added because a
refusal answers nothing and the first truck log is the check. DE01 isn't decoded: the byte offsets are a
hypothesis on most rows.

### 2.7 The health check and its gain windows

**Problem.** Tell an owner, and us, when a car doesn't steer as its row says, without changing the
command.

**Evidence.** Every Ford measured follows curvature at 0.97–1.05 (`verify-cross-vehicle.md`). The first
gain check could not fire and was noisy (review finding 4b). A yaw zero of 0.025 rad/s moved one-minute
gains between 0.9 and 1.6 on a car that follows its command.

**Choice** (`L/core/health.py`). A window is a minute of hands-off driving above 9 m/s with enough
steering in it (MIN_EXCITATION). The command is delayed by the row's lag, and the standstill yaw zero is
removed; no window counts until the zero is known. A window is out of range outside 0.85–1.15, and three
out-of-range windows in a row mean degraded. Windows carry across drives of the same car and software
(`health.carry/restore`, stored in `FordLateralSupport`). On the seven recorded routes the seven windows
read 0.94–1.03 (`logs-derived/clean-build/health_on_routes.json`). The other signals: the module's
capability Faulty, and panda-blocked messages while steering and allowed. Those count only after 30
frames of "allowed", so an ordinary disengage, where the controller hears up to 100 ms late, doesn't
latch "degraded" (finding 4a). Response time is not estimated: `commandedCurvature` and
`measuredCurvature` are in the status every frame, for offline analysis.

**Trade-off.** Thresholds are first values. The panda's count is every blocked message, not only
steering ones.

### 2.8 Lag per row and the delay learner's start

**Problem.** openpilot leads the plan by the lateral delay. lagd starts at `steerActuatorDelay + 0.2 s`
(0.4 s on every Ford) and learns above 50 mph.

**Options.** Put the row's lag in modeld's delay directly; start lagd from it; or leave it.

**Evidence.** The rows' lags don't share one definition. The F-150's is command to 63 % with the rate
limit not acting. The Explorer's, Ranger's and Maverick's are measured behind the module's own slewed
command, 0.1–0.2 s short of what lagd measures (`clean-build.md` §6 item 4).

**Choice.** Start lagd from the row's lag at 100 km/h (`StrategyRow.delay_learner_start`, set in
`L/hosts/sunnypilot/safety_param.py`). That gives 0.39 s on the 2021–23 F-150 BD row, 0.28 s on BC,
0.25 s on the 2021–24 Mach-E, 0.44 s on the Escape 2025 and 0.26 s on LX6C. Rows with
`lag_behind_slewed_command`, the fallback and the stock path keep openpilot's start. A learnt value
still takes over.

**Not run:** a model replay. The delay reaches the driving model as an input, and a replay needs a
route's video.

### 2.9 Curve speed and the planner's clip from the module's limit

**Problem** (approach I). Above about 105 km/h the module's own lateral-acceleration table is the
ceiling. The only lever left is speed.

**Choice.** The status publishes `lateralAccelLimit`: the panda's cap, or the row's limit where lower.
controlsd's curvature clip uses it where it is below openpilot's 3 m/s², so a request beyond it shows as
"turn exceeds steering limit" instead of being clipped silently in the car controller. Road roll can't
widen it (`test_road_roll_does_not_widen_the_steering_limit`). sunnypilot's vision curve speed (SCC-V)
plans for it where it is below its own 2.0 m/s² (`B/openpilot/sunnypilot/selfdrive/controls/controlsd_ext.py`,
`.../smart_cruise_control/vision_controller.py`).

**Trade-off.** The rows' limits drop under 2.0 only from about 130–160 km/h, so at legal speeds curve
speed changes nothing today. The limit is read at the car's speed, which errs low. Not checked on a drive.

### 2.10 Two steps, one release

**Problem.** Path angle's numbers rest on firmware reads and the twin, and no drive has sent it. Review
finding 8 showed a busier wheel on real requests.

**Choice.** Two steps, documented, and one release after the first confirming drive (`clean-build.md`
§14.5), because path angle's numbers haven't been driven: the base ships with it off, and a family
switches on only after a drive confirms its row.
1. **Step one, the base:** every row's switch off, bit 6 clear on every car.
2. **Step two, per family:** a row switches on after a drive on one of its modules where
   `drive_check.py` (3.6) finds the measured path-angle gain within ±30 % of the row's below 20 m/s, and a
   low-speed turn with hands on, released at the exit, shows no wind-up. Switching on is one line in the
   row, and it also sets bit 6 for that row.
3. **The confirming drive needs path angle on**, and the switch is a constant, so the step-one build
   can't produce it. The first drive uses a test branch, **`clean/ford-lateral-drive1`**, identical
   except that the Explorer row's L1MC-14D003-AL / MB5C-14D007-BA pair has path angle on. The window test
   already runs that case with zero refused frames.
4. **One release:** after the first confirming drive, on a 2020–24 Explorer, the rows confirmed by then
   switch on and the build ships once. Unconfirmed rows ship off.

**Release-notes wording** (§14.5 item 5): engaging in a curve starts from straight and builds in at the
module's own rate, about 5–6 s at 30 km/h and about 2 s at 60 km/h. Only rows whose path angle is
confirmed build in one to three seconds. Hold the wheel through the first seconds. For BC: "steers by
curvature only, with timing measured on BC trucks."

**Why it's slow.** comma's rule says the inactive frame must be all zero, so the module's copy starts at
zero, and the panda's check restarts from zero too. Any controller that obeys the rule straightens
first. On newer-library rows, frames before the module reports control move at the panda's step, not
the module's rate (`follows_at_once_before_control`, `firmware-core.md` §4.2), which shortens this once
path angle is on.

### 2.11 The BC row from fleet behaviour

**Problem.** 42 of 45 logged 2021–23 F-150s run ML3V-14D003-BC. Our row and twin are BD. No BC file was
found (`bc-sources.md`).

**Evidence** (`bc-fleet.md`). BC trucks answer curvature at gain 1.00–1.02 by band (33 trucks), the
same as BD, and engage the same way. They are **faster**: 0.27 s against 0.57 s from command to yaw at
60–75 mph, and the difference sits in the module, not the vehicle. Path angle, path offset and curvature
rate can't be tested from public data: every engaged second on every truck is curvature only.

**Choice.** A curvature row with comma's stock constants, the BD row's lateral-acceleration limit, BC's
own lag (0.35, 0.33, 0.28, 0.27 s at 30/60/100/130 km/h), and `ANY_DATA_FILE` (2.2). BC no longer
carries the "older level" reason it had at first. BA keeps "older than Ford's current level". **Trade-off:** it rests on behaviour, not files, and it applies only once the module answers F10A.

### 2.12 The three-repo layout

**Problem.** Where the code lives.

**Options.** bp-dev's in-tree Ford code, or the 7.1 bluepilot-lateral library. **Choice:** the 7.1
library, taken from `f267286`, because 7.1 already discards bp-dev's Ford safety and its host already has
the comma-compatible limits and the stock fallback; building on bp-dev would mean writing the safety change
twice (`change-analysis.md` §4.1). Panda safety and controller are on the same branch, in separate
commits, with one test set, so one set of tests shows the controller never leaves the panda's window
(`change-analysis.md` §4.2). The three parts are branches of one repository, bluepilot-clean.

| Part | Upstream base | In bluepilot-clean |
|---|---|---|
| BluePilot (this repo) | BluePilotDev/bluepilot `bp-7.1-dev` `fc0b77e5e` | `clean/ford-lateral` `7fb69ebfb` (local branch `clean/ford-lateral-fix`) |
| opendbc at `opendbc_repo/` | ford-op/opendbc `e80aa6cb` | `clean/opendbc` `88701a67` |
| bluepilot-lateral at `opendbc_repo/opendbc/bluepilot_lateral/` | BluePilotDev/bluepilot-lateral `f267286` | `clean/bluepilot-lateral` `ca0b134` |

`B/.gitmodules` points `opendbc_repo` at that repository, branch
`clean/opendbc`. `O/.gitmodules` points `opendbc/bluepilot_lateral` at the same repo, branch
`clean/bluepilot-lateral`. A recursive clone gets all three. LFS objects stay at sunnypilot's store
(`.lfsconfig`), so pushes use `--no-verify` to skip the LFS hook. Each library commit is followed by an
opendbc pointer commit and a bluepilot pointer commit.

**Trade-off.** Not upstream-shaped: upstream would take plain files in `opendbc/car/ford` and
`opendbc/safety` (4.3). Not checked: bp-7.1-dev itself on a drive (BluePilotDev/bluepilot#226).

## 3. The low level

### 3.1 One steering frame, end to end

1. **Planner.** openpilot's model and planner produce the desired curvature. controlsd clips it, also
   to `lateralAccelLimit` (2.9), and hands `CarControl.actuators.curvature` to card.
2. **Host hook.** `O/opendbc/car/ford/carcontroller.py` builds one `BluePilotLateral` through the
   sunnypilot adapter (`L/hosts/sunnypilot/lateral.py`). Each frame `bp.update(CC_SP)` passes
   `LateralInputs`: `pandaLateralAllowed` and `safetyTxBlocked`, filled from `pandaStates` in
   `B/openpilot/sunnypilot/selfdrive/controls/controlsd_ext.py`. If it returns False, the stock path
   runs. At `STEER_STEP` (20 Hz) it calls `bp.step_lateral(CC, CS)`.
3. **Request rule** (`L/core/request.py`). Ask only while openpilot wants lateral, the panda allows it,
   and the module (`pscm_status`, read from Lane_Assist_Data3_FD1 by `O/opendbc/car/ford/carstate.py`, CAN
   FD only) says Available or Active. Denied is its own state. On CAN, where there is no status, the
   rule follows openpilot and the panda. Otherwise the inactive frame is sent: all zero, ramp type 0,
   precision 1. Why: a hands-free-configured module stays Unavailable while asked, a refused frame is a
   missing frame to the module, and the inactive frame always passes (`test_never_asks_through_unavailable`).
4. **Limit** (`L/core/carcontroller_ext.py` `step_lateral`). The request is clipped to
   `lat_accel_limit / v²`.
5. **Path** (`_path`). Without path angle, `want` goes straight through. With it: curvature moves at the
   row's rate from the module's copy, kept unrounded (finding 9). The preview time copy falls at once and
   recovers at `t_rise`. `gain = K2(v, T)`. Path angle is `(want − c2)/gain`, clipped to cancelling what
   the slow curvature still holds plus `path_angle_rate × 1 s` more, and to nothing extra while the
   driver steers (the wind-up rule, finding 2). Then a 1.5-step dead band from zero and the row's
   path-angle rate.
6. **Panda window** (`L/core/limits.py` `window`, `fit_path`). From the last command the panda saw (zero
   after any inactive frame), one jerk step either way, within the cap, for the curvature signal and for
   the implied curvature at K2max read 1 m/s below the car's speed. Curvature is fitted first, then path
   angle into what is left. So every frame is inside the panda's check by construction.
7. **Pack** (`L/core/lane_message.py`, `L/can/fordcan_ext.py`). Each path signal is clamped to its DBC
   range in the wire's sign (the wrap fix, `test_wire_clamp.py`). The LateralMotionControl2 checksum is
   set by the library itself. `lane_message.replaces(address)` tells the car controller which camera
   messages it sends in the camera's place.
8. **Panda** (`O/opendbc/safety/modes/ford.h` `ford_tx_hook` → `L/safety/ford.h` `ford_bp_lmc_tx` →
   `L/safety/lateral.h` `ford_bp_path_cmd_checks`), section 3.2.
9. **Module.** Steers for `c2 + K2·c1` with its own limits and driver override.
10. **Status** (`BluePilotLateral.status()` → `LateralStatus`, 26 fields) is published every frame as
    `controllerStateBP.lateral` by `B/bluepilot/selfdrive/car/bp_card_publisher.py`.
    `B/bluepilot/selfdrive/car/ford_lateral_support.py` keeps `FordLateralSupport` (the settings row, the
    comparison with the last start, the offroad notice `Offroad_FordLateralSupport`). card calls
    `record_start` once per start (`B/openpilot/selfdrive/car/card.py`).

**At car init** (`L/hosts/sunnypilot/safety_param.py` `init_ford_lateral_sp`, called from sunnypilot's
`setup_interfaces`): the start-up read, `check_plan` against the start's CAN fingerprint (logs a camera
message not heard as planned, changes nothing), dashcam only for `not_supported`, the safety bits into
`CP_SP.safetyParam`, `lateralInputsRequired`, and `steerActuatorDelay` from the row.

**How a car is matched** (`L/core/strategy.py` `match_strategy`), in order: a superseded level gives its
reason; a missing F188 or F10A gives "didn't answer"; a strategy in no row gives "isn't in the table";
a row with the exact pair (or BC's `*`) and, if the row pins any, a listed calibration matches (or "kept
on the fallback" for a fallback row); otherwise "another data file or calibration". `match.plan` is the
row, or `FALLBACK_ROW`.

### 3.2 The panda's checks

`ford_tx_hook` decodes each steering message. If `ford_bp_lateral` (bit 5) is set, `ford_bp_lmc_tx` runs:

- curvature rate and path offset must be inactive. Path angle must be inactive unless
  `ford_bp_path_angle` (bit 6) is set;
- `ford_bp_path_cmd_checks`, while lateral is allowed and the request is on:
  - signal ranges: ±0.02 1/m and ±0.5 rad;
  - the lateral-acceleration cap on the curvature signal **and** on the implied curvature
    `c2 + K2max·c1`, with K2max read at the lowest recent speed sample and the cap at that speed minus
    1 m/s;
  - comma's lateral jerk limit (about 3.6 m/s³) on both, against the last command;
  - comma's real-time rate check, and the speed-mismatch check;
- while not steering, both signals must be zero. No steering while controls aren't allowed;
- after any violation, and while lateral isn't allowed, the last command is zero: the next ramp starts
  from zero. There is no latch (S1, pinned by three tests).

The caps are about 3.6 m/s² on CAN and about 2.4 m/s² on CAN FD (`FORD_BP_LIMITS`).

Without bit 5, the stock check runs: `ford_stock_curvature_checks`, comma's `steer_curvature_cmd_checks`
with its band. While lateral isn't allowed it restarts from zero, as the stock controller does. After a
refused frame it restarts from the measured curvature. The review's flag-only fix
(`inactive_curvature_is_zero`) tripled refusals on replay, so it was replaced (`clean-build.md` §14.1 #6).

### 3.3 How a row is made from firmware

The pipeline lives in the project's research notes (to be published):

1. **Files.** The module's VBF files (program, data file, calibration) for a strategy come from Ford's
   service-software archive, collected per VIN (`notes/sourcing/`; what's missing is in
   `notes/sourcing/wanted.md`). Raw files and decompiles are kept apart from the notes.
2. **Read the control law.** Decompile (Ghidra: RH850 on the 2021–23 F-150's module, PowerPC VLE on the
   2020–24 Explorer's) and write a per-strategy note: path law, preview T and d tables, integrator, slews, a_y limit,
   authority, latches (`firmware-core.md` for ML3V-BD; `rl14-firmware.md`, `explorer-firmware.md`,
   `rz1c-firmware.md`, `lj9c-firmware.md`, `lx6c-firmware.md` and the others). This gives a **curvature**
   row.
3. **Compare data files** (`cfg-diff.md`): which data files hold the same lane-centering numbers and can
   share a row.
4. **Check against a model** (`twin.md`). `twin/` is the module's lateral controller in
   Python, block for block. It was checked against the firmware itself in an emulator: 595,165 block
   calls, no mismatches after fixes (`twin-vs-firmware.md`). Against drives, its angle target matches the
   measured pinion angle to 0.41° RMS on BD trucks. §10 covers the older library. A row checked this way,
   or by a drive, is **full**. Its preview and rates come from the twin's calibration objects
   (`scripts/clean-build/row_numbers.py`), and its K2 is checked over speed and curvature
   (`k2_rows_check.py`, `k2_envelope.py`, which also has to stay under the panda's K2max).
5. **Row in `strategy-table.md`** with every value marked proven, supported or hypothesis. Then a
   `StrategyRow` in `L/core/strategy.py` that cites its note.
6. **Tests:** a `test_strategy.py` case for its pairs and what must not match it; the window test with
   the row (path angle on and off); `table_coverage.py` re-run.
7. **Path angle on:** only after `drive_check.py` passes on a drive of that family (2.10).

**Evidence a row needs, by level:** curvature needs the control law read from that strategy's software
and the exact pair. Full also needs a twin run or a drive. A preview in the row needs both distance
clamps at all eight speed points (RZ1C and PZ11 have four, so their path angle stays zero). The path
angle switch needs a confirming drive. Unverified numbers err toward a high K2 and a low c2 rate.

### 3.4 Logs and the drive check

- Every start logs the read (`carlog` "BluePilot steering module read"), the support record ("BluePilot
  steering support"), a part-number change ("BluePilot steering module changed", with old and new) and
  any plan mismatch ("BluePilot lane message plan").
- Every frame: `controllerStateBP.lateral` (request, command, both signals, measured curvature, limited,
  module state, limit, capability, health).
- `scripts/clean-build/drive_check.py <rlog or route dir>... [--json out.json]` (or
  `--selftest`), in the tree's venv. Per speed band (5–12, 12–20, 20–45 m/s), hands off, it fits
  `measured(t + lag) = a·curvature + b·(commandedCurvature − curvature) + offset` at the best lag
  (0–0.8 s). It passes where |b − 1| ≤ 0.30 with at least 400 frames of path angle at work, and it also
  fits small and large path angle apart. The self-test recovers 1.0, 0.8 and 0.55 and fails 0.55. No
  route has these fields yet.
- Replay: `scripts/replay/cleanreplay.py` (`run_clean.sh`, `analyze_clean.py`) runs these
  files with the compiled panda in the loop. `--pscm` gives a what-if row.

### 3.5 Tests

Python environment: `UV_PROJECT_ENVIRONMENT=~/tools/venvs/clean-build uv sync --frozen --extra testing
--extra tools`, plus `pyyaml`, `pytest` and `cantools`. Then `scons -j9` at the root. Whole suites are
heavy: run them in CI or on a large machine, not on a shared small one. From `opendbc_repo/` with
`PYTHONPATH=$PWD`:

```
.venv/bin/python -m unittest discover -s opendbc/bluepilot_lateral/tests -t .          # library: 108 OK
(cd opendbc/safety/tests && ../../../.venv/bin/python -m unittest test_ford test_bluepilot_lateral)  # 650 OK, 107 skipped
.venv/bin/python -m unittest opendbc.car.tests.test_car_interfaces opendbc.car.ford.tests.test_ford  # 284 OK
bash opendbc/bluepilot_lateral/safety/tests/bluepilot_coverage.sh                       # 100 % of both headers and modes/ford.h
```

Also `opendbc/sunnypilot/car/ford/tests` (75 OK), ruff and ty on the library, MISRA (`test_misra.sh`,
passes), mutation (`L/safety/tests/bluepilot_mutation.py`, 58 of 58 killed), and the bluepilot tree on a
built tree:

```
tools/test_runner.py bluepilot openpilot/sunnypilot/selfdrive/controls/tests openpilot/sunnypilot/sunnylink/tests \
  openpilot/selfdrive/ui/bp openpilot/selfdrive/controls openpilot/selfdrive/selfdrived openpilot/selfdrive/car
```

That gives 545 passed, 1 skipped and 1 error. The error is `test_ui_frames`' harness: a `params.cc` log
line lands on the child's result line. The counts are `clean-build.md` §14.4's, on a build machine.

What the main tests guard:

| Test | Guards against |
|---|---|
| `L/safety/tests/test_ford_bluepilot_window.py` | the real car interface and controller leaving the compiled panda's window: speed grid, engaging in a curve, hard acceleration and braking, random requests, module Unavailable/RampOut/Denied, panda withdrawing; CAN and CAN FD, rows, fallback, stock path. Zero refused frames |
| `L/safety/tests/test_ford_bluepilot.py` | the panda rule: `test_curvature_cannot_hide_behind_an_opposite_path_angle`, no latch, restart from zero, bit 6 off refuses path angle, upstream's `test_steer_allowed` sweep |
| `L/tests/test_strategy.py` | wrong matches: exact pairs, generations never sharing, BC, superseded reasons, read-only reads, retries, the switch off on every row |
| `L/tests/test_controller.py` | the request rule, the exact c2 rate, the first frame, never asking through Unavailable |
| `L/tests/test_recorded_turns.py` | path-angle wind-up on recorded low-speed turns (`tests/data/low_speed_turns.csv`) |
| `L/tests/test_health.py` | false "degraded" after a disengage; the gain window's rules |
| `L/tests/test_wire_clamp.py` | signals wrapping past their range (failed first against `f267286`) |
| `L/tests/test_lane_message_plan.py` | the plan against the panda's transmit table and the car controller |
| `L/tests/test_safety_param_bits.py` | Python's bits drifting from `ford_bp_init`'s |
| `L/tests/test_core_imports.py`, `test_bluepilot_upstream_copies.py` | absolute host imports; copies of upstream functions going stale |
| `B/bluepilot/selfdrive/car/tests/test_ford_lateral_support.py` | the support row, notices and `record_start` with the real Ford interface |
| `upstream test_ford.py` | passes unchanged against the stock check |

## 4. Gaps, extending, upstream

### 4.1 Known gaps (`clean-build.md` §6, §14.6)

- F10A and F124 on a real truck (no vehicle read yet). Without F10A every car is on the fallback.
- Coverage: 39 of 63 factory records and 25 of 67 logged vehicles have no exact row. Rows are wanted for
  PZ11-14D003-EA and the unread files; RZ1C and PZ11 need preview tables; calibrations aren't pinned yet.
- No route exercises a row's own numbers yet. Path angle has never been sent on a drive by this build.
- The stock path ("Disable BP Lateral Control") still has 143 refused frames over seven routes, inside
  comma's band (outside this build's goal).
- IQ.Pilot: its coverage step (needs clang-18 and mull) and the start-up read on that host. StarPilot:
  untested. Neither host reads the module, so every car there is on the fallback.
- Hook 6's action on a plan mismatch, once one is logged. The model replay for the delay learner's start.
  `tools/test_on_host.sh` end to end.
- Future improvements: camera and fault-code reads at start, the DE01 decode, K2max per row
  (`clean-build.md` §10).
- Not checked: a comma four install.

### 4.2 Extending

- **A row:** section 3.3. Keep exact pairs. Mark the level by its evidence. Leave `path_angle` False.
- **A new way a module takes its lane:** a new variant in `L/core/lane_message.py`, its own check in
  `L/safety/`, and rows that name it. The controller and the table don't change.
- **A new host:** an adapter against `BluePilotLateral(CP, safety_param, packer, CAN)`, `update`,
  `step_lateral`, `status` (`L/HOSTS.md`).
- **Every change** to the safety: the window test, coverage at 100 %, MISRA and mutation.

### 4.3 The path upstream

The aim is a build that can go upstream, even if that means larger changes to comma's Ford code where
the current implementation rests on faulty logic. What it needs:
- the library as plain files in `opendbc/car/ford` and `opendbc/safety`, without host bindings;
- the early commits squashed into safety / controller / tests / A′. A′'s panda change and controller
  change are already their own commits;
- the fallback half is nearest to upstream: comma's command without the clip. The band's removal is a
  relaxation that has to be argued with the acceleration-and-jerk bound;
- the path-angle half needs comma to accept a c1 rule (the two-bound check above, with a K2max curve
  derived from read modules), and confirming drives;
- upstream's `test_ford.py` already passes unchanged, and MISRA and mutation pass on the headers.
