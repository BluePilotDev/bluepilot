# AGENTS.md

Rules for agents and people changing BluePilot. This file holds what you can't find by reading the tree: where a
change belongs, what CI enforces, and the mistakes that have already cost someone a PR.

BluePilot is a Ford-focused fork of sunnypilot (itself a fork of openpilot) that steers and controls the speed of real
cars. openpilot's priorities apply in this order: safety, stability, quality, features ([docs/SAFETY.md](docs/SAFETY.md)).

## Route the change to the repo that owns it

BluePilot is four repos. Route the change before writing code: a PR in the wrong repo can't be merged.

| The change touches | It belongs in | PR against |
|---|---|---|
| Ford steering: curvature and angle modes, the lateral CAN messages, lateral safety limits, lateral settings | [BluePilotDev/bluepilot-lateral](https://github.com/BluePilotDev/bluepilot-lateral) (the library) | `main` |
| Ford car code: longitudinal, cruise buttons, cluster and HUD messages, car state, fingerprints, Ford panda safety | [ford-op/opendbc](https://github.com/ford-op/opendbc), folder `opendbc/sunnypilot/car/ford/bluepilot/` | `bp-full` |
| CI workflow files shared with the sunnypilot + BluePilot-lateral build (`tests.yaml`, `bp-tests.yml`, `install-smoke.yml`) | [ford-op/bp-sunnypilot](https://github.com/ford-op/bp-sunnypilot) | `bp-sunnypilot` |
| Everything else: the BluePilot UI, the portal, settings, BluePilot processes, the openpilot side of a Ford feature | this repo | `bp-dev` |

How the pieces arrive here: `opendbc_repo` is a submodule pointing at ford-op/opendbc `bp-full`, which carries the
library as its own nested submodule (`opendbc/bluepilot_lateral`). A sync job moves the `opendbc_repo` pointer and
merges upstream sunnypilot; leave both to it. A fix for upstream's own behavior goes to sunnypilot or comma, and
reaches BluePilot through the sync.

Car code in opendbc never imports openpilot. A setting or value the car code needs is published to it from this repo
(`bluepilot/selfdrive/`), so a Ford feature is often two PRs: the car code in `bp-full`, then its openpilot half here.

## Branches

- `bp-dev`: the source of truth and the default branch. Every PR targets it.
- `bp-stable` and the version branches (`bp-7.1`, ...): written by the release job. A version branch takes only
  cherry-picks of fixes already on `bp-dev`.
- `bp-sunnypilot`: an automated mirror of ford-op/bp-sunnypilot. It is "upstream" for the hook check below.
- `bp-7.0` and older: frozen releases.

All of these are protected. Work on your own branch or fork and open a PR.

## Where code goes in this repo

BluePilot's own code lives in two places:

- `bluepilot/`: the portal (`backend/`, `web/`), settings (`params/`), processes and openpilot-side logic
  (`selfdrive/`, `system/`), vendored third-party Python (`vendor/`), images (`assets/`).
- `openpilot/selfdrive/ui/bp/`: the UI. `onroad/`, `layouts/` and `widgets/` are the comma 3X (upstream calls it
  tici); `mici/` is the comma four.

Everything else is an **upstream file**. Paths start with `openpilot/` (`openpilot/selfdrive/...`,
`openpilot/common/...`); older notes and branches that say `selfdrive/...` predate that move.

### Changing an upstream file: the hook rule

First try to put the code in a BluePilot file. When an upstream file must change, make the change a **hook**: a few
lines that call into BluePilot code, written like this:

```python
# BluePilot: with ICBM on a Ford the target follows the cluster set speed
from openpilot.common.bluepilot import is_bluepilot
if is_bluepilot():
  from openpilot.bluepilot.selfdrive.car.bp_cluster_set_speed import ClusterSetSpeedBP
# End BluePilot
```

- Open with `# BluePilot: <why>` and close with `# End BluePilot`.
- Gate behavior with `is_bluepilot()` so the upstream path runs unchanged when it is false.
- Add the file to `.github/bluepilot-hooks.txt` with a one-line reason. The `bluepilot-hooks` check fails any file
  that exists on `bp-sunnypilot`, differs from it, and is missing from that list.
- Keep upstream's own lines in place. A hook that rewrites or deletes them conflicts at the next sync.
- New processes are added with `procs +=` in `process_config.py`; upstream's stay.

UI classes subclass upstream's and carry the `BP` suffix (`HudRendererBP`); files carry `_bp`. Where upstream uses a
sunnypilot subclass (`...SP`), build on that one.

## The six required checks

A PR into `bp-dev` merges only with all six green. Run what you can before pushing.

| Check | Locally |
|---|---|
| `build release` | `scons` covers most of it |
| `static analysis` | `scripts/lint/lint.sh` (ruff, ty, codespell, shell and file checks) |
| `unit tests` | `tools/op.sh test`, or `python tools/test_runner.py <file or folder>` |
| `process replay` | `openpilot/selfdrive/test/process_replay/test_processes.py` |
| `install smoke` | CI only: installs the way a device does |
| `bluepilot-hooks` | `git fetch origin bp-sunnypilot && python3 .github/bluepilot_hooks.py origin/bp-sunnypilot` |

- A fork PR's checks start after a maintainer approves the run.
- `process replay` compares a Ford route against stored references. A difference means driving output changed: explain
  the cause in the PR, and a maintainer adds the `replay-accepted` label once it is understood.

## Tests

Tests are `unittest.TestCase` classes. The runner collects with `unittest.TestLoader`, so a pytest-style test (a bare
`test_*` function, or a class that is not a `TestCase`) is never collected, and `bluepilot/tests/test_test_files.py`
fails the check for a test file that contributes no tests. Put tests in a `tests/` folder beside the code.

UI code fails only when it is drawn, so two tests stand in for a device:

- `openpilot/selfdrive/ui/bp/tests/test_ui_classes.py` checks every BluePilot UI class against its upstream base.
- `openpilot/selfdrive/ui/bp/tests/test_ui_frames.py` draws frames for both devices. Add a new widget or page to it,
  in each state it can show.

Then look at it on a device, and say in the PR which device you saw it on. The comma 3X and the comma four run
different UI code.

## Things that have already gone wrong

- **Params.** A new param is declared in `openpilot/common/params_keys.h` (the BluePilot block) and in
  `bluepilot/params/params.json`. An undeclared key raises when it is read. The Ford lateral lines in
  `params_keys.h` are generated from the library's manifest: change them there.
- **Cereal.** BluePilot's messages take reserved slots in `custom.capnp` / `log.capnp` (`controllerStateBP` is
  `customReserved13`, `carStateBP` is `customReserved14`). sunnypilot claims slots from the low numbers upward, so
  take the highest free one and name it in the PR. Moving a message to another slot makes older logs unreadable for
  that message.
- **Submodules can be empty on a device.** comma's installer can leave a submodule folder empty. Code that imports
  from a submodule must run when the folder is empty. Third-party Python is carried as a stamped copy under
  `bluepilot/vendor/` for that reason.
- **Upstream moves under you.** Upstream renames cereal fields, removes params and changes base-class signatures. An
  override that copies upstream internals breaks at the next sync; call `super()` and add to it.
- **The portal's web bundle is committed.** After changing `bluepilot/web/src`, run `npm run build` in
  `bluepilot/web` and commit the result.
- **The UI must hold its frame rate.** Read params, files and USB state on a timer or on a state change, outside the
  per-frame draw.
- **Spelling.** American spellings; codespell runs in `static analysis`.

## Driving behavior

- A change to what the car does ships behind a setting that defaults to off, with the existing behavior unchanged
  when it is off, unless the PR is a fix to existing behavior.
- A driving change needs evidence from a real drive: the route, the car, the device, and what the logs show.
  Replays and unit tests support a drive; they do not replace one. State plainly what was and wasn't driven.
- Panda safety and its limits change only in the repos in the routing table, with their own tests.

## Pull requests

- One purpose per PR, against `bp-dev`. Split a feature from unrelated fixes; send a fix for upstream code upstream.
- The description says why, what changes for users who leave the feature off, and how it was tested.
- End the description with a `## Validation` checklist of the six checks plus any device or drive check. Tick only
  what has passed.
- Follow [docs/AI_POLICY.md](docs/AI_POLICY.md): understand and test what you submit, and disclose AI help with an
  `Assisted-by:` line in the commit message.
- Maintainers squash-merge, so the PR title and description become the commit.
