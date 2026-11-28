# How BluePilot is put together

A map of BluePilot's own code for someone opening the repo for the first time. The rules for changing it (which repo,
which branch, what CI checks) are in [AGENTS.md](../AGENTS.md); this page explains what is where and why.

## Three layers

BluePilot is openpilot with two layers on top:

```
openpilot    comma's driving software                              (github.com/commaai/openpilot)
sunnypilot   a fork that adds features (MADS, ICBM, speed limits)  (github.com/sunnypilot/sunnypilot)
BluePilot    sunnypilot plus Ford steering, Ford extras and its own UI
```

sunnypilot changes every day and BluePilot takes those changes automatically. That only works while BluePilot's code
stays out of upstream's files, so nearly all of it lives in its own folders, and upstream files carry only small,
marked edits that hand over to BluePilot code. Those edits are called **hooks**, and every upstream file that has one
is listed in `.github/bluepilot-hooks.txt` with its reason.

A hook looks like this, and runs only in a BluePilot build (`is_bluepilot()` in `openpilot/common/bluepilot.py` is
true when the `BPVERSION` file exists at the repo root):

```python
# BluePilot: after the sunnypilot imports, so BluePilot's home layout wins
if is_bluepilot():
  from bluepilot.ui.layouts.home_bp import HomeLayoutBP as HomeLayout
# End BluePilot
```

## Four repos

Not all of BluePilot is in this repo. Ford steering is a library, and Ford car code lives in a fork of opendbc; both
arrive here as git submodules under `opendbc_repo/`.

| What | Where |
|---|---|
| Ford steering (lateral control and its safety code) | [BluePilotDev/bluepilot-lateral](https://github.com/BluePilotDev/bluepilot-lateral) |
| Ford car code (CAN messages, speed control, cluster, hybrid data) | [ford-op/opendbc](https://github.com/ford-op/opendbc), branch `bp-full` |
| sunnypilot plus Ford steering only, no BluePilot UI | [ford-op/bp-sunnypilot](https://github.com/ford-op/bp-sunnypilot) |
| Everything else: UI, portal, settings, sounds, crash reporting | this repo, branch `bp-dev` |

## Where BluePilot's code lives in this repo

| Folder | What it holds |
|---|---|
| `openpilot/selfdrive/ui/bp/` | The driving-screen and settings UI. `onroad/`, `layouts/`, `widgets/` are the comma 3X; `mici/` is the comma four; `lib/` is shared (theme packs, sounds, icon styles). |
| `bluepilot/ui/` | The comma 3X home screen, sidebar and the onroad debug panels. |
| `bluepilot/backend/`, `bluepilot/web/` | The portal: a web server on the device and the web app it serves. |
| `bluepilot/selfdrive/` | BluePilot logic that runs inside openpilot's processes: the extra log messages, the cluster set speed, blinker pause. |
| `bluepilot/system/` | Crash reporting (`sentry.py`, to glitchtip.bluepilot.dev) and Wi-Fi helpers. |
| `bluepilot/backend_switch.py` | Where the device sends drives: comma connect, Konik, or nowhere (offline). |
| `bluepilot/vendor/` | Third-party Python shipped with BluePilot. |
| `bluepilot/assets/` | Images and icons (boot logo, spinner). Theme packs are in `openpilot/selfdrive/assets/bp_themes/`. |
| `bluepilot/tests/`, `openpilot/selfdrive/ui/bp/tests/` | Tests. |

Everything outside these folders is upstream's.

## The UI

The comma 3X and the comma four run different UI code. Upstream's names for them are **tici** (3X) and **mici**
(four), and you will see both in paths and class names.

Each piece of the screen is a class. sunnypilot extends openpilot's class, and BluePilot extends sunnypilot's (or
openpilot's, where sunnypilot has none). BluePilot's classes end in `BP`, sunnypilot's in `SP`:

```
HudRenderer (openpilot)  →  HudRendererSP (sunnypilot)  →  HudRendererBP (BluePilot)
AlertRenderer (openpilot)  →  AlertRendererSP (sunnypilot)  →  AlertRendererBP
AugmentedRoadView (openpilot)                           →  AugmentedRoadViewBP        the driving view
ModelRenderer (openpilot)                               →  ModelRendererBP            the path and lane lines
```

The swap happens in one place per device, where the screen is assembled: `openpilot/selfdrive/ui/layouts/main.py`
(3X) and `openpilot/selfdrive/ui/mici/layouts/main.py` (four). Each imports the BluePilot class under the stock
class's name, as in the hook example above, so the rest of the file doesn't change.

Two things to know before changing a `BP` class:

- **It depends on upstream's internals.** When upstream renames a method or changes a signature, the BluePilot
  override breaks, and only when that screen is drawn. `test_ui_classes.py` and `test_ui_frames.py` in
  `openpilot/selfdrive/ui/bp/tests/` catch much of this, but a real device is the only full check.
- **A change usually needs doing twice**, once for each device, and checking on each.

**Settings.** The comma 3X's BluePilot panel is one file, `openpilot/selfdrive/ui/bp/layouts/settings/bluepilot.py`.
The comma four's is split by section in `openpilot/selfdrive/ui/bp/mici/layouts/settings/` (`lateral_mici.py`,
`visuals_mici.py` and so on). A setting is stored as a **param**, declared in `openpilot/common/params_keys.h`.

## Log messages

openpilot's processes talk to each other with messages defined in `openpilot/cereal/`. BluePilot adds two, both
published by `card` (the process that talks to the car) through `bluepilot/selfdrive/car/bp_card_publisher.py`:

- `carStateBP`: Ford data the UI draws, such as hybrid power flow and battery.
- `controllerStateBP`: what the Ford steering controller did each frame, plus a copy of the BluePilot settings, so a
  drive can be analyzed afterward without the device.

They are defined in `openpilot/cereal/custom.capnp` and registered in `openpilot/cereal/services.py`.

## Processes

BluePilot adds processes at the end of `openpilot/system/manager/process_config.py` rather than replacing upstream's:
`bp_portal` and `bp_route_preprocessor`. The one exception is `soundd`, where BluePilot runs a subclass
(`openpilot/selfdrive/ui/bp/soundd_bp.py`) for custom sounds.

## The portal

The portal is a website the device serves on your Wi-Fi at `http://<device-ip>:8088` for browsing and exporting
drives, changing settings and reading logs. It is off until turned on in settings.

- **Server:** `bluepilot/backend/bp_portal.py`, plain Python, with the work split into folders beside it (`routes/`,
  `video/`, `params/`, `system/`, `realtime/`, `cache/`). Live updates go over a WebSocket on port 8089.
- **Web app:** `bluepilot/web/src/`, written in React and TypeScript and built with Vite. The built files in
  `bluepilot/web/public/` are committed, because the device has no Node to build them; a change to `src/` is rebuilt
  and committed with it.
- **While driving** the portal answers only a short list of read-only requests; everything else is refused.
- **Route preprocessor:** a second process that, while parked, reads new drives and prepares thumbnails, distances
  and place names for the portal.

## Where to start for common changes

| To change | Start in |
|---|---|
| Something on the driving screen, comma 3X | `openpilot/selfdrive/ui/bp/onroad/` (`hud_renderer_bp.py`, `alert_renderer_bp.py`, `model_renderer_bp.py`) |
| Something on the driving screen, comma four | `openpilot/selfdrive/ui/bp/mici/onroad/` |
| The hybrid gauges | `powerflow_gauge*.py`, `hybrid_battery_gauge*.py` in the two `onroad/` folders |
| A BluePilot setting | `params_keys.h`, then the settings files named under "Settings" above, for both devices |
| The comma 3X home screen or sidebar | `bluepilot/ui/layouts/home_bp.py`, `bluepilot/ui/widgets/sidebar.py` |
| Which class draws a part of the screen | the two `layouts/main.py` files |
| A portal page | `bluepilot/web/src/` |
| A portal API | `bluepilot/backend/bp_portal.py` and the folder for that area |
| A new log field | `openpilot/cereal/custom.capnp`, `bluepilot/selfdrive/car/bp_card_publisher.py` |
| A new process | `openpilot/system/manager/process_config.py` (BluePilot block at the end) |
| How the Ford steers | not here: [bluepilot-lateral](https://github.com/BluePilotDev/bluepilot-lateral) |
| Ford speed control, cruise buttons, cluster | not here: [ford-op/opendbc](https://github.com/ford-op/opendbc) `bp-full` |
