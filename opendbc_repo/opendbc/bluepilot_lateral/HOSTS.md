# Hosting the BluePilot lateral

A host is an opendbc fork with this library at `<pkg>/bluepilot_lateral/` and the hook patch applied, plus the openpilot on top of it that fills the library's inputs each frame. `<pkg>` is the fork's opendbc package: `opendbc` on sunnypilot/opendbc (ford-op/opendbc, under bp-sunnypilot), `iqdbc` on IQ.Pilot, `opendbc` on StarPilot. This page is what a host provides and what it gets back.

## Mounting the library

A host mounts this repo at `<pkg>/bluepilot_lateral/`, directly inside its opendbc package, as a git submodule (ford-op/opendbc) or, in a host that takes no submodules (IQPilot, at `artifacts/package_sources/iqdbc/iqdbc/bluepilot_lateral/`), as a stamped copy of a release. Updating is a bump of the submodule's commit, or vendoring the new release (below). The hook patch below carries every file ford-op/opendbc adds or changes outside the library, so since it mounts the library it also carries the mount's `.gitmodules` entry.

The library never names its host's package. Python imports are relative: the library's own modules by `.`/`..`, the host's opendbc by one more level (`from ...car.ford.values import CarControllerParams` in `core/`), and `tests/test_core_imports.py` fails on any absolute `opendbc`/`iqdbc`/`openpilot` import. Safety headers include each other as `../safety/<file>.h` and the host's declarations as `../../safety/declarations.h`, relative to the including file. The host includes the library's safety as `"<pkg>/bluepilot_lateral/safety/ford.h"`.

### Without submodules: a stamped copy

From a clone of this repo, `tools/vendor.sh <release tag> <host>/<pkg>/bluepilot_lateral` makes the folder exactly the library at that tag (files it no longer has are removed), without `.github/` (`.gitattributes`), plus a `VERSION` file: the tag and the commit. Nothing inside the folder is ever edited: a change goes into this repo, gets released, and the host vendors the new tag. `tools/check_vendored.sh <folder>` lists anything in the folder that differs from the release in its `VERSION` and fails if there is something; a host can run it in its CI, and this repo's IQPilot checks run it on IQPilot's copy.

### Host bindings

The safety headers are written against upstream opendbc's names. A host whose names differ defines these before it includes `safety/ford.h` (`safety/lateral_declarations.h` has the defaults):

| macro | default (upstream, sunnypilot) | IQ.Pilot |
|---|---|---|
| `BP_HOST_CURVATURE_LIMITS` | `CurvatureSteeringLimits` | `NativeCurvatureSteeringLimits` |
| `BP_HOST_CURVATURE_STATE` | `curvature_state` | `native_curvature_state` |
| `BP_HOST_RT_CURVATURE_RATE_CHECK` | `rt_curvature_rate_limit_check` | `rt_native_curvature_rate_limit_check` |
| `BP_HOST_LAT_ALLOWED()` | `(controls_allowed \|\| controls_allowed_lateral)` | `is_lat_active()` |
| `BP_HOST_SPEED_MISMATCH_CHECK()` | `speed_mismatch_check(vehicle_speed_2 ...)` | `((void)0)`: IQ's `ford_rx_hook` runs it |

The limits struct needs only `max_curvature`, `curvature_to_can`, `frequency`, `max_curvature_error` and `curvature_error_min_speed`.

## The hook patch

`tools/make_hook_patch.sh` prints it: every file outside the library that ford-op/opendbc changes from `master`, minus its own `.github/` (its CI). It fails unless the edited files are exactly the non-`.github` entries of `.github/bluepilot-hooks.txt`.

| file | what the hook does |
|---|---|
| `opendbc/car/ford/carcontroller.py` | Builds one `BluePilotLateral(CP, CP_SP, packer, CAN)`, the sunnypilot adapter (`hosts/sunnypilot/lateral.py`, below). Each frame `bp.update(CC_SP, frame)` says whether BluePilot lateral is active; if it is, `bp.step_lateral` sends the steering message at `STEER_STEP` (and returns the curvature to pass back next time) and `bp.step_lka` sends Lane_Assist_Data1 at `LKA_STEP`. Otherwise the stock path runs. |
| `opendbc/car/structs.py` | Imports `LateralInputs` from `host_types.py` and adds `CarControlSP.lateralInputs` and `CarParamsSP.lateralInputsRequired`. |
| `opendbc/sunnypilot/car/interfaces.py` | Calls `init_ford_safety_param_sp(CP, CP_SP, params_dict)` (`hosts/sunnypilot/safety_param.py`) at car init: the startup safety bits below, and `lateralInputsRequired` for every Ford. |
| `opendbc/safety/modes/ford.h` | Includes `safety/ford.h` and makes one call per hook site: counter and quality checks for the pinion message, the yaw measurement skipped while the pinion source is on, `ford_bp_rx_hook` (pinion measurement), `ford_bp_lka_tx` (angle-mode state from Lane_Assist_Data1), `ford_bp_lmc_tx` in place of the stock steering check when `BP_LATERAL` is set (the stock check runs unchanged when it isn't), and the pinion RX check list from `ford_bp_init`. |
| `opendbc/safety/tests/test_bluepilot_lateral.py` | New: hands the library's `safety/tests/` to upstream's test discovery, so upstream's `test.sh` and `mutation.py` run the BluePilot safety tests. |

## Every frame: inputs

A host fills these and the settings below for a car whose `CarParamsSP.lateralInputsRequired` is set: every Ford, BluePilot lateral on or off (`init_ford_safety_param_sp`).

`CarControlSP.lateralInputs` (`host_types.LateralInputs`):

| fields | from |
|---|---|
| `pathYawRate`, `pathX`, `pathY` | `modelV2.orientationRate.z`, `modelV2.position.x`, `.y` (33 points) |
| `laneLineLeftX/Y`, `laneLineRightX/Y` | `modelV2.laneLines[1]`, `[2]` |
| `laneLineProbs`, `laneLineStds` | `modelV2.laneLineProbs`, `.laneLineStds` |
| `laneChangeState`, `laneChangeDirection` | `modelV2.meta` |
| `steerRatio`, `stiffnessFactor`, `roll`, `angleOffsetDeg` | the learned vehicle parameters (`vehicleParameters`, paramsd) |
| `lateralDelay` | `lateralDelay.lateralDelay` |
| `valid` | true only when both the model and the vehicle parameters have been seen and every field above is filled |

Each path array has `host_types.MODEL_PATH_POINTS` (33) entries at the times in `host_types.MODEL_T_IDXS` (openpilot's `ModelConstants.T_IDXS`): the core interpolates over that grid. On a frame that isn't valid, or whose arrays aren't the full shape or contain a non-finite value, the library keeps the previous inputs.

## Every frame: settings

Every key in `opendbc.bluepilot_lateral.core.values_ext.BP_LATERAL_PARAMS` (the per-frame settings in `params/manifest.py`; hosts import the list from there), every frame, as (key, value) with the value as bytes (sunnypilot: `CarControlSP.params` entries): bools `"1"`/`"0"`, numbers as text. A missing, empty or unparsable value gives the manifest default. A host publishes its own registry default for a key nobody has set, so a host can carry its own defaults (BP7 uses 3.0 for `LC_PID_gain_UI_curv`). `python -m <pkg>.bluepilot_lateral.params.manifest` prints the keys as `params_keys.h` lines (IQ.Pilot drops `BACKUP`, which its params don't have).

## At startup: the safety bits

`values_ext.ford_safety_param(CP, params_dict)` reads the startup settings (`disable_BP_lat_UI`, `FordPrefSteerAngleCurvature`) and returns the bits. The host sends them to the panda in its second safety param (sunnypilot: the adapter ORs them into `CP_SP.safetyParam`). StarPilot has no second param: its adapter shifts them into bits 9-14 of its Ford `safetyParam` (`hosts/starpilot/safety_param.py`, clear of the bits StarPilot's Ford safety reads), and its `ford_init` hands `(param >> 9) & 0x3F` to `ford_bp_init`:

| bit | |
|---|---|
| 5 `BP_LATERAL` | the panda runs the BluePilot steering checks; the car controller sends the 4-signal messages only when it's set |
| 0 `STEER_ANGLE_CURVATURE`, 1-4 geometry index | the pinion angle replaces the yaw rate as the curvature measurement, on platforms with a row in the pinion table |

The panda keeps what it was given at init: changing `disable_BP_lat_UI` while driving takes full effect at the next init.

## Reading status

`BluePilotLateral.status()` returns `host_types.LateralStatus`, this frame's values for a BluePilot UI, under the names BP7's `controllerStateBP` publisher reads off its car controller:

| field | |
|---|---|
| `lateralUncertainty` | from whichever strategy ran |
| `angleRateLimited` | angle mode: the path angle soft rate limit bit this frame |
| `curvatureRateLimited` | angle mode: the equivalent curvature would have been rate-limited |
| `curvatureDeviationLimited` | either mode: the current-curvature error clip constrained the command |
| `humanTurnLateralPaused` | angle mode: lateral sent inactive during a manual turn |
| `stallBlipActive` | angle mode: the brief mode-0 pulse after a post-override stall |
| `disable_BP_lat_UI` | BluePilot lateral off: the setting, or the panda bit not set at init |
| `primary_lateral_control` | 0 = curvature, 1 = angle |

The four angle-mode values read false outside angle mode. bp-sunnypilot doesn't read status; it has no BluePilot UI.

## What the library uses from the host's opendbc

Every host is an opendbc fork, so the library uses these directly:

- `<pkg>.car`: `structs`, `DT_CTRL`, `ACCELERATION_DUE_TO_GRAVITY`, `scale_rot_inertia_per_mass` and `scale_tire_stiffness_per_mass` (or, on an opendbc from before the per-mass vehicle model, `scale_rot_inertia` and `scale_tire_stiffness`, at unit mass), `common.pid.PIDController`, `vehicle_model.VehicleModel` (either signature), `lateral` (`AngleSteeringLimits`, `apply_std_steer_angle_limits`, `ISO_LATERAL_ACCEL`), `ford.fordcan.CanBus`, `ford.values` (`CarControllerParams`, `FordFlags`).
- `can/fordcan_ext.py` sets LateralMotionControl2's checksum itself: upstream's packer fills Ford checksums (and overwrites it with the same value), IQ.Pilot's doesn't.
- The safety headers use `modes/ford.h`'s Ford constants and upstream's safety helpers, through the bindings above. `safety/lateral.h` and `can/fordcan_ext.py` copy four upstream functions, pinned per host package by `tests/test_bluepilot_upstream_copies.py`, so a host where one changed fails that test (a host with no pins skips it).
- The safety tests use the host's test harness: `<pkg>.safety.tests.test_ford`, `common` and `libsafety.libsafety_py`. `tests/host.py` names the host's own structs, adapter and libsafety setters for the tests that touch them.

## The core's API, and the host adapters

The core takes only the library's types:

- `core.carcontroller_ext.BluePilotLateral(CP, safety_param, packer, CAN)`: `safety_param` is the startup bits (`ford_safety_param`).
- `update(inputs, params, frame)`: `inputs` is a `host_types.LateralInputs`, `params` a `params.param_store.ParamStore` built from the frame's (key, value) settings. Then `step_lateral`, `step_lka` and `status()` as above.

Nothing under `core/`, `can/`, `params/` or `safety/` imports a host (`tests/test_core_imports.py`). `hosts/sunnypilot/`:

- `lateral.py`: a `BluePilotLateral` subclass with sunnypilot's types, `(CP, CP_SP, packer, CAN)` and `update(CC_SP, frame)`. It passes `CP_SP.safetyParam`, `CC_SP.lateralInputs` and the `CC_SP.params` entries as (key, value) pairs.
- `safety_param.py`: `init_ford_safety_param_sp(CP, CP_SP, params_dict)` ORs `ford_safety_param` into `CP_SP.safetyParam`. Its own module, so car init imports only `values_ext`.

`hosts/iqpilot/`, the same for IQ.Pilot's types:

- `lateral.py`: `BluePilotLateral(CP, CP_IQ, packer, CAN)` and `update(CC_IQ, frame)`, passing `CP_IQ.iqSafetyFlags`, `CC_IQ.lateralInputs` and the `CC_IQ.params` entries.
- `safety_param.py`: `init_ford_safety_param_iq(CP, CP_IQ, params_dict)` ORs `ford_safety_param` into `CP_IQ.iqSafetyFlags` (the panda's `current_safety_param_iq`) and sets `CP_IQ.lateralInputsRequired`.

`hosts/starpilot/`, for StarPilot, which has neither a second safety param nor CarControl / CarParams extension structs:

- `lateral.py`: `BluePilotLateral(CP, packer, CAN)`, reading the startup bits back out of the Ford `safetyParam`, and `update(frame_inputs, frame)`. `FrameInputs` carries `lateralInputs` and the `params` entries; StarPilot's openpilot side fills one each frame and hands it to the Ford CarController, in place of a CarControl extension.
- `safety_param.py`: `init_ford_safety_param_starpilot(CP, params_dict)` ORs `ford_safety_param << 9` into `CP.safetyConfigs[-1].safetyParam`; `bp_safety_param(CP)` reads them back; `lateral_inputs_required(CP)` says whether to fill `FrameInputs`. StarPilot's Transit (`FordFlags.LKA_STEERING`, steering through Lane_Assist_Data1 alone) gets neither.

Another host writes its own adapter against the same API.

## Running the tests

- Python tests: `python -m unittest discover -s <pkg>/bluepilot_lateral/tests -t .`, from the directory that holds `<pkg>`.
- Safety tests: the host's `<pkg>/safety/tests/test.sh` runs them, through a loader (ford-op/opendbc's `test_bluepilot_lateral.py`) or by naming `../../bluepilot_lateral/safety/tests` (IQ.Pilot: its cross-mode TX sweep reads `TX_MSGS` off every `Test*` class in its directory, so a loader there breaks it). Coverage of the BluePilot headers and `modes/ford.h`: `safety/tests/bluepilot_coverage.sh`, which runs the library's tests itself where the host has no loader; mutation: `safety/tests/bluepilot_mutation.py` (needs the host's `mutation.py`, which IQ.Pilot doesn't have).
- On a clean sunnypilot/opendbc checkout: `tools/test_on_host.sh <checkout>`, from a ford-op/opendbc checkout. It copies the library as committed and applies the hook patch to a temporary clone, then runs both.
- On IQPilot: `hosts/iqpilot/host.sh tree <dir>` builds iqdbc from IQ.Lvbs's repo at the pinned commit with `hosts/iqpilot/hooks.patch` (IQPilot's hooks, kept here until IQPilot carries them) and the library, vendored; `host.sh test <dir>` runs the library tests, IQPilot's safety suite with the BluePilot ones (minus `known_failures.txt`, IQPilot's own) and coverage; `host.sh make <dir>` writes the hook patch back after editing the hooks in `<dir>/host`. CI's `iqpilot` job runs tree and test. Every night `iqpilot pin check` tries IQ.Lvbs's latest `beta`: green moves the pin in a PR it merges itself (dropping `hooks.patch` once IQPilot carries the hooks), red keeps one issue open saying which step failed.
