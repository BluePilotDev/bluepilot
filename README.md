![](selfdrive/assets/img_bluepilot_boot.jpg)

Table of Contents
=======================

- [Table of Contents](#table-of-contents)
  - [Updates on Branch Names and Links](#-updates-on-branch-names-and-links)
  - [Join our Discord](#-join-our-discord)
  - [What is bluepilot?](#-what-is-bluepilot)
  - [Prohibited Safety Modifications](#-prohibited-safety-modifications)
  - [Vehicle Compatibility](#-vehicle-compatibility)
  - [Installation](#-installation)
  - [BluePilot Specific Features - bp-7.0](#-bluepilot-specific-features---bp-70)
  - [BluePilot Settings Reference](#-bluepilot-settings-reference)
  - [sunnypilot Features and Settings](#-sunnypilot-features-and-settings)
  - [Recommended Settings](#-recommended-settings)
  - [Tuning Guide](#-tuning-guide)
  - [Troubleshooting & FAQ](#-troubleshooting--faq)
  - [How to Ask for Help](#-how-to-ask-for-help)
  - [Version History](#-version-history)
  - [Special Thanks](#-special-thanks)
  - [User Data](#-user-data)
  - [Licensing](#licensing)
  - [Support sunnypilot](#-support-sunnypilot)
  - [Technical Documentation](#-technical-documentation)

---

<details><summary><h3>💭 Updates on Branch Names and Links</h3></summary>

---

As of May 2025, we are updating the way branches are named and how links are generated. We had initially intended to use a branch naming system similar to openpilot and sunnypilot where there was a "stable" or "release" branch which included all fully vetted code, and then "staging" or "beta" branches with new code that would eventually move into the stable/release branches.  However as we evolved we found everyone liked being able to bounce between newer and older branches to compare features and control. Moving forwards all releases will simply be named bp-"feature release number" as an example "staging-1.1" which features the bluepilot 1.1 features (custom tuning) will become "bp-1.1".  We will not delete older branches so that anyone can go back and view older code for references.  Branches that no longer work properly will be denoted as -deprecated.

To install any version of bluepilot, use the following URL formula (URL is case sensitive)

installer.comma.ai/BluePilotDev/"branch name"

For example

installer.comma.ai/BluePilotDev/bp-7.0

will install the **bp-7.0** branch (current release, synced with sunnypilot master as of June 10, 2026).  Branches known to no longer work due to changes in the comma codebase will be appended with -deprecated so it will be obvious they will not install or work correctly.

</details>


---

<details><summary><h3>💭 Join our Discord</h3></summary>

---

Join the official #ford channel at the sunnypilot Discord server to stay up to date with all the latest features and be a part of shaping the future of bluepilot!
* [sunnypilot Discord server](https://discord.gg/sunnypilot)

Other places to get information:
* [BluePilot website — Release Notes and Announcements](https://bluepilot.dev/announcements)
* [BluePilot FAQ](https://bluepilot.dev/FAQ/)
* [GitHub Issues](https://github.com/BluePilotDev/bluepilot/issues)
* [sunnypilot community forum](https://community.sunnypilot.ai/)

</details>

<details><summary><h3>🌞 What is bluepilot?</h3></summary>

---

[bluepilot](https://github.com/bluepilotdev/bluepilot) is a fork of the hugely popular SunnyPilot project for the comma 3X and comma 4. The lineage is: **openpilot → sunnypilot → bluepilot**. The goal of BluePilot is to develop, test, and stage Ford specific enhancements, validating them before submission to the SunnyPilot team for inclusion in the parent project.  BluePilot is always based upon sunnypilot's master branch.

BluePilot is not a company — it is a handful of enthusiasts working out of home garages with no profit motive. Anything not documented by BluePilot should be looked up in the [sunnypilot documentation](https://docs.sunnypilot.ai/).

**BluePilot bp-7.0** is synced with **sunnypilot master (June 10, 2026, openpilot 0.11.2 base)** and includes all upstream sunnypilot features plus Ford-specific enhancements. This release runs on AGNOS 18.4 and supports both the comma 3X and the comma 4.

⚠️ **This is a Level 2 driver-assistance system, not self-driving.** Keep your eyes on the road and be ready to take over at all times. BluePilot tracks sunnypilot *master* — a development branch — so treat every release as developmental software.

⚠️ **Warranty note:** Ford does not support comma devices and dealers have been known to blame unrelated issues on them. Consider removing the device and harness before service visits that are not routine maintenance.

### Where features come from

BluePilot includes **all** features from the upstream SunnyPilot project, plus its own Ford-specific work. Throughout this README, features are labeled by origin:

| Origin | What it covers |
|---|---|
| **BluePilot** | Ford lateral control (Curvature and Angle schemes with per-scheme tuning), Ford longitudinal refinements (coasting, downhill compensation), Ford radar parsing and vision-only mode, BlueCruise cluster UI, hybrid/EV gauges, radar lead overlay, the BluePilot Portal web app, Connect Backend selection (Comma / Konik / Offline), custom engage sounds, extra visual themes, the entire **BluePilot** settings menu, and the Ford extension of ICBM |
| **sunnypilot** | MADS, NNLC, DEC, Speed Limit Assist, ICBM core, Smart Cruise Control (Map & Vision), Driving Model Manager, sunnylink, lane change customization, torque tuning suite, OSM offline maps, and all the standard settings panels (Device, Toggles, Steering, Cruise, Visuals, etc.) |

For complete upstream documentation see the [sunnypilot docs](https://docs.sunnypilot.ai/) and [README_SP.md](README_SP.md).

</details>

<details><summary><h3>⛔ Prohibited Safety Modifications</h3></summary>

---

All [official sunnypilot branches](https://github.com/sunnyhaibin/sunnypilot/branches) strictly adhere to [comma.ai's safety policy](https://github.com/commaai/openpilot/blob/master/docs/SAFETY.md). Any changes that go against this policy will result in your fork and your device being banned from both comma.ai and sunnypilot channels. This same stipulation applies to all bluepilot instances as well.

The following changes are a **VIOLATION** of this policy and **ARE NOT** included in any sunnypilot branches:
* Driver Monitoring:
    * ❌ "Nerfing" or reducing monitoring parameters.
* Panda safety:
    * ❌ No preventing disengaging of <ins>**LONGITUDINAL CONTROL**</ins> (acceleration/brake) on brake pedal press.
    * ❌ No auto re-engaging of <ins>**LONGITUDINAL CONTROL**</ins> (acceleration/brake) on brake pedal release.
    * ❌ No disengaging on ACC MAIN in OFF state.

</details>

<details><summary><h3>🚙 Vehicle Compatibility</h3></summary>

---

### "My Ford has lane keep and adaptive cruise — why isn't it supported?"

Ford has shipped two different lane-assist generations, and only one of them works:

* **Lane Keep Assist (LKA)** — older system that only nudges the car back toward the lane in short bursts when you drift near a line. Openpilot cannot use it with stock PSCM firmware: Ford's power steering module enforces an "LKA lockout" that stops accepting commands if too many arrive in a time window. Working under that limit was tested extensively and produced unusable swerving. (Experimental, unreleased PSCM-firmware modification work has demonstrated lockout removal on one vehicle — follow the [announcements](https://bluepilot.dev/announcements) for progress.)
* **Lane Centering Assist (LCA)** — newer system that steers continuously. This is what BluePilot requires.

Compatibility varies by year, model, and trim — even between similar vehicles. When in doubt, ask in the #ford Discord channel before buying hardware.

### TRON — Ford's encrypted CAN bus

**TRON** ("Trusted Realtime Operating Network") is Ford's encrypted/authenticated CAN bus, rolling out since the 2023 Super Duty as models get refreshed. It only affects CAN-FD (Q4 harness) vehicles. **Any vehicle with TRON will not work with comma/openpilot/sunnypilot/bluepilot.** Rule of thumb: if the vehicle has Ford's new "Phoenix" entertainment system (Android Automotive), it has TRON. Comma has a standing $10,000 bounty (as of 2025) on breaking TRON, but there is no guarantee a crack will ever arrive or persist.

Confirmed TRON status (last updated Aug 19, 2025 — check [bluepilot.dev](https://bluepilot.dev/announcements) for updates):

| Vehicle | No TRON (works) | TRON (will not work) |
|---|---|---|
| Ford Bronco Sport | —* | 2025+ |
| Ford Escape | 2023–2025 | — |
| Ford Explorer / Lincoln Aviator | —* | 2025+ |
| Ford Expedition / Lincoln Navigator | 2022–2024 | 2025+ |
| Ford F-150 | 2021–2023 | 2024+ |
| Ford F-150 Lightning | 2022–2025 | — |
| Ford Mustang Coupe | — | 2024+ |
| Ford Mustang Mach-E | 2021–2024 | 2025+ |
| Ford Maverick | —* | 2025+ |
| Ford Ranger | 2024–2025 | — |
| Ford Super Duty | — | 2023+ |
| Ford Transit | — | 2026+ |
| Lincoln Nautilus | — | 2024+ |

\* Earlier model years of these vehicles are CAN (Q3) platforms, which TRON does not affect — see the known-to-work list below; the confirmed-TRON post only lists their TRON side.

### Vehicles known to work

From the bp-5.0 model recommendations (Nov 2025) — ask in the #ford channel about newer model years:

* **CAN (Q3 harness):** Bronco Sport (2021–24), Edge (2022), Escape (2020–22), Explorer (2020–24), Focus (2018), Maverick (2022–24), Lincoln Aviator (2020–24)
* **CAN-FD (Q4 harness):** Escape (2023–24), Kuga (2020–24, CAN or CAN-FD depending on year), Mustang Mach-E (2021–24), Ranger (2024), Expedition (2022–24), F-150 (2021–23), F-150 Lightning (2022–23)

</details>


<details><summary><h3>⚒ Installation</h3></summary>

---

### 1. Hardware

* A **comma 4** ([comma.ai shop](https://comma.ai/shop)) — the comma 3X is no longer sold but remains supported; comma 3 support was dropped upstream in late 2025 (use bp-2.1).
* The correct **Ford harness** for your exact year/model (selected on the comma shop page). CAN Fords use the **Q3** harness; CAN-FD Fords use the **Q4** harness.
* **F-150 Lightning owners** also need the [BluePilot Lightning Coupler](https://BluePilot.dev/lightning-coupler-order-page/).
* Optional: [BluePilot quick-release sliding dock](https://bluepilot.dev/dock-order-page/) for the comma device.
* CAN-FD installs need a long, high-quality USB-C cable: only **USB-C 3.2 Gen2 (2x2)** cables work.

### 2. Physical install

The harness plugs in-line with the **IPMA** (Ford's camera module); it only fits one way. Mount the device high and centered on the windshield.

* **Q3 (CAN) vehicles:** the IPMA is at the rearview mirror — [comma's official video](https://www.youtube.com/watch?v=0Q3MndBEAiY) applies.
* **Q4 (CAN-FD) vehicles:** the IPMA is elsewhere in the vehicle, so a long USB-C cable run is needed. Watch the [general CAN-FD/IPMA port video](https://www.youtube.com/watch?v=uUGkH6C_EQU) first, then your vehicle:
  * [F-150 / Lightning / Expedition](https://www.youtube.com/watch?v=MewJc9LYp9M) — IPMA in the driver's footwell
  * [Mustang Mach-E](https://www.youtube.com/watch?v=AR4_eTF3b_A) — IPMA in the trunk area
  * [CAN-FD Escape](https://www.youtube.com/watch?v=M6uXf4b2SHM) — IPMA in the trunk area
  * [Ranger](https://www.youtube.com/watch?v=2oJlXCKYOy0) (and [part 2](https://youtu.be/c5ZmMHuJ1Os)) — IPMA behind the instrument cluster
* **Lightning:** with the Lightning Coupler the long USB-C cable becomes orientation-specific. If you get errors, rotate one end of the cable 180° and power-cycle; if that fails, rotate the other end. The Lightning also normally shows some startup errors that clear after ~30 seconds.

### 3. Software install

* bluepilot not installed
  1. [Factory reset/uninstall](https://github.com/commaai/openpilot/wiki/FAQ#how-can-i-reset-the-device) the previous software if you have another software/fork installed. On a brand-new device (or after flash.comma.ai) install stock openpilot once first, then reinstall with the fork URL.
  2. After factory reset/uninstall and upon reboot, select `Custom Software` when given the option. (If you connect to WiFi first, swipe down to get back to the installation menu.)
  3. Input the installation URL based on the desired branch. Example: ```installer.comma.ai/BluePilotDev/bp-7.0``` (note: `https://` is not required on the comma device; the URL is case sensitive)
  4. Complete the rest of the installation following the onscreen instructions. The download takes 10–20 minutes with several restarts.

* bluepilot already installed
  1. On the device, go to `Settings` ▶️ `Software`.
  2. At the `Download` option, press `CHECK`. This will fetch the list of latest branches.
  3. At the `Target Branch` option, press `SELECT` to open the Target Branch selector.
  4. Scroll to select the desired branch

⚠️ **Upgrading to bp-7.0:** bp-7.0 requires a newer AGNOS version. It is recommended to uninstall the existing branch or factory reset before upgrading.

### Which version should I install?

| Device | Version | Install URL |
|---|---|---|
| comma 4 | bp-7.0 | `installer.comma.ai/BluePilotDev/bp-7.0` (short: bit.ly/bp-70) |
| comma 3X | bp-7.0 | `installer.comma.ai/BluePilotDev/bp-7.0` |
| comma 3 | bp-2.1 | `installer.comma.ai/BluePilotDev/bp-2.1` |

### 4. First-time setup

1. **Pair sunnylink first** (`Settings` ▶️ `sunnylink`, scan the QR). This is the single most recommended setup step — settings, models, and tuning can then be managed from a phone or computer at [sunnylink.ai](https://sunnylink.ai). In bp-7.0 all BluePilot settings appear under sunnylink's **Vehicle** tab.
2. Select your exact Ford model if prompted (Settings ▶️ Vehicle).
3. Start with the default driving model and default tuning for your first drives.

Need help with the install? Join the [sunnypilot Discord server](https://discord.gg/sunnypilot) and ask in the `#ford` channel.

</details>

<details><summary><h3>🚗 BluePilot Specific Features - bp-7.0</h3></summary>

---

## What's New in bp-7.0

BluePilot 7.0 is a **major update** synced with sunnypilot master (June 10, 2026, openpilot 0.11.2 base) on AGNOS 18.4, headlined by a second, selectable Ford lateral control scheme with full panda safety enforcement.

### Angle-Primary Lateral Control (New)

Reverse-engineering the PSCM firmware revealed why Ford steering ping-pongs: the PSCM applies a speed-indexed low-pass filter to its internal curvature state, "remembering" the previous command for up to ~1 second at highway speed (longer at city speeds) — the model was reacting to PSCM-created oscillation, not causing it. Angle mode sidesteps the filter by driving **path_angle** directly. The result: no filter memory, no windup, better centering without offsets, cleaner lane changes, and far more tolerance for switching driving models. ([Full write-up](https://bluepilot.dev/announcements))

A second Ford steering strategy, selectable at any time from the settings menu (no reinstall or reboot):

* **Primary Control Variable selector** - Choose between Angle (path_angle as the primary actuator) and Curvature (the proven 4-signal strategy from previous releases)
* **Per-platform tuning built in** - Separate gain defaults for CAN vehicles (Escape, Bronco Sport, Maverick, Edge), CAN-FD trucks (F-150, Lightning, Expedition, Ranger), and CAN-FD SUVs (Mustang Mach-E, Escape MK4.5)
* **Variable lookahead** - Model lookahead time adapts to speed and curve depth for earlier, smoother curve entry without early unwind at the apex
* **PSCM saturation handling** - Detects when the steering module is at its authority limit and manages the command to avoid snap corrections on release
* **User feel adjustments** - Low Speed, High Speed, and High Speed Low Curve Adjustment Factors, an angle-specific Lane Change Factor, and angle-mode Lane Positioning (in-lane offset and centering strength)
* **Live lateral debug screen** - Watch the commanded vs. actual steering in real time while tuning (comma 3X and comma 4)

### Panda Safety for Angle Mode

Angle mode runs under full panda safety enforcement, not a bypass:

* The safety firmware corroborates that angle mode is genuinely engaged through a redundant CAN channel before accepting angle-mode frames
* A shadow curvature cross-check continuously validates the commanded steering intent against the measured vehicle motion
* path_angle rate-of-change limits are enforced independently on the panda, tuned to the actual control cadence
* Full regression test coverage in the safety test suite, validated against recorded real-world routes with a replay harness

### Per-Scheme Tuning Parameters

Curvature and angle control no longer share any tuning values:

* Every lateral tuning parameter now belongs to exactly one scheme (curvature or angle)
* Switching schemes never overwrites or reuses the other scheme's tuning; each keeps its own values
* Existing settings migrate automatically on first boot after updating; no retuning required

### Settings Menu Reorganization

* **Nested lateral tuning sections** - Lateral Tuning now contains collapsible Angle Tuning and Curvature Tuning sub-sections; the disable toggle and scheme selector sit at the top
* Controls for the unselected scheme stay visible but grey out, so both tuning sets are always discoverable (comma 3X)
* Cleaned up and reorganized comma 4 menus, with an on-device vehicle selector and integer/float value pickers on both platforms

### SunnyLink Remote Settings

* **Ford BluePilot settings under the "vehicle" panel in sunnylink** - Configure BluePilot from the sunnylink interface, grouped by System, Vehicle, Audio, Visuals, Longitudinal Tuning, and Lateral Tuning
* Scheme-aware: angle and curvature tuning entries appear only when their control scheme is selected, with the same dependency gating as the on-device menu

### Connect Backend

* **Connect Backend** - Choose Comma Connect (stock servers), Konik Stable (stable.konik.ai), or Offline Mode (unreachable hosts so uploads never succeed)
* Dongle ID switches automatically between Comma and Konik and is cached per backend, so switching is reversible in both directions with a reboot
* Pairing QR codes and instructions follow the selected backend

### Longitudinal (carried forward from 6.x)

* **BluePilot longitudinal tuning** - When using sunnypilot longitudinal control, BluePilot softens openpilot's binary gas/brake behavior with more coasting, mimicking Ford's own CAN-FD ACC feel (bypassable in settings)
* **Ford ACC remains the default** - in community polls, about 90% of CAN-FD drivers prefer stock Ford ACC; BluePilot fully supports either
* **Downhill compensation** - pitch-based gas/brake compensation, with a disable toggle if your Ford already handles hills and the two systems conflict
* **Ford radar parsing** - real radar data for lead tracking (~98% parse accuracy), with an optional vision-only mode

### Visuals

* **Rainbow Lane Lines** - Inner lane lines become rainbow colored when longitudinal control is active
* **Minimal Driving View** - Hide the camera feed and show only lane lines and the model path
* **Themes** - 8-Bit Racer retro game view or seasonal theme packs that recolor the road and wheel icon, with automatic holiday-week switching
* **Show Lateral Control Mode** - Steering wheel icon overlay showing the active lateral scheme (C = curvature, A = angle, OP = BP lateral disabled)
* Hybrid/EV battery and power-flow gauges, Ford ACC radar lead overlay, blindspot overlay, brake status, and more — see the full settings reference below
* Steering arc fix for the comma 3X, with reduced false left-steer bias from road roll

### Platform & Stability

* Synced with sunnypilot master (June 10, 2026): Chrysler CUSW, Rivian, Tesla, Honda, and Hyundai updates, lateralManeuverPlan support, updated panda/msgq/rednose submodules
* AGNOS 18.4 with the AGNOS thermal framework managing CPU frequency (replaces the static frequency cap)
* Display initialization fixes for git-clone installs (EGL/GLES driver re-pointing on boot)
* is_bluepilot() detection system; all stock file overrides are guarded and labeled for clean upstream merges
* Ford opendbc code refactored into a sunnypilot extension layer, with the safety test harness synced to upstream

### BluePilot Portal

The React-based Progressive Web App served directly from the device — home dashboard, routes with video playback and GPS maps, browser settings with live updates, and diagnostics. See [Technical Documentation](#-technical-documentation).

</details>

<details><summary><h3>📋 BluePilot Settings Reference</h3></summary>

---

Every selection in the **BluePilot menu** (`Settings` ▶️ `BluePilot`). All of these are BluePilot-specific features. Items marked **(C3X)** or **(C4)** only appear on that device; everything else is on both. In sunnylink, these same settings appear under the **Vehicle** tab.

🎥 Video walkthrough: [BluePilot 7.0 Menu Tour](https://www.youtube.com/watch?v=gS1NBPlugY0)

Many settings can only be changed while the vehicle is off (or in `Always Offroad` mode). The comma 4 menu uses shortened variants of some titles (e.g. "lat. tuning", "Show Hybrid Power Flow").

### System

| Setting | Type | Description |
|---|---|---|
| Connect Backend | selector: Comma Connect / Konik Stable / Offline Mode | Comma Connect uses stock servers. Konik Stable sends routes to stable.konik.ai (dongle ID switches automatically). Offline Mode points at unreachable hosts so uploads never succeed. Reboot to apply. |
| Restore Cached Dongle ID | button | If switching backends left the device unregistered, restores a previously registered ID cached on device. Only enabled when one is found; use when instructed by a dev. |
| Preferred WiFi Network | selector: saved networks | Automatically connect to this network when available — e.g. always prefer home WiFi over a hotspot. |
| Clear Crashed Model | button | Clears the model runner cache and reboots. Fixes "Communication Issue" when modeld fails to start. |
| UI Debug Logging | toggle | Log UI state transitions for diagnosing rendering issues. |
| Reset Menu Layout (C3X) | button | Collapse all sections to fix overlapping items if the menu looks broken. |

### Vehicle

| Setting | Type | Description |
|---|---|---|
| Show BlueCruise UI on Cluster | toggle | Display the BlueCruise hands-free UI on supported CAN-FD digital dashes (works on some non-BlueCruise vehicles, e.g. 2024–25 Ranger). |
| Use Pinion Yaw Sensor | toggle | Measure vehicle rotation from the steering pinion angle sensor instead of a faulty RCM yaw sensor (symptoms: "Turn Exceeds Steering Limit" warnings, weak curve tracking, "Service AdvanceTrac"). Applies on next car start; not available on the Edge. Only use if instructed or with a confirmed faulty sensor. |
| 12V Battery Limit | value: 11.0–14.0 V | The 12V battery charging pause limit protecting the car battery (default 11.8 V). |

### Audio

| Setting | Type | Description |
|---|---|---|
| Use Custom Engage/Disengage Sounds | toggle | Replace the engage/disengage sounds with the selected sound pack. Reboot required. |
| Engage/Disengage Sound | selector: Comma 4 / Comma 3x / Tesla | The sound pack to use (with custom sounds enabled). |

### Visuals

| Setting | Type | Description |
|---|---|---|
| Hide Onroad Border | toggle | Hide the colored status border around the driving view. |
| Hide Onroad Fade (C4) | toggle | Hide the onroad fade overlay. |
| Disable Lane Line Status Color (C3X) | toggle | Keep lane lines grey instead of green when engaged. |
| Minimal Driving View | toggle | Hide the camera feed; show only lane lines and the model path (high contrast). |
| Theme | selector: Off / 8-Bit Racer / seasonal packs | 8-Bit Racer retro game view, or a seasonal theme pack that recolors the road and wheel icon. |
| Auto Seasonal Theme | toggle | During a holiday week, switch to that seasonal theme pack automatically; otherwise the Theme selection applies. |
| Rainbow Lane Lines | toggle | Inner lane lines turn rainbow when longitudinal control is active. |
| Show Blindspot Overlay | toggle | Red screen-edge overlay when a vehicle is in your blind spot (the BluePilot alternative to sunnypilot's Show Blind Spot Warnings). |
| Show Brake Status | toggle | Show braking (including ACC-commanded braking) — red speed setpoint on C3X, red steering icon on C4. |
| Show Confidence Ball (C3X) | toggle | Model-confidence ball on the left of the driving view. |
| Animate Steering Wheel | toggle | Rotate the wheel icon with the actual steering angle. |
| Wheel Icon Style | selector: Comma 4 / Comma 3x | Steering wheel icon style. |
| DM Icon Style | selector: Comma 4 / Comma 3x | Driver monitoring icon style. |
| Show Radar Lead Overlay (Ford ACC) (C3X) | toggle | Chevron with lead distance/speed/follow time when using stock Ford ACC. |
| Radar Overlay Size (C3X) | selector: Small / Medium / Large | Size of the radar overlay chevron and info boxes (with the overlay enabled). |
| Show Hybrid/EV Battery Status (C3X) | toggle | Hybrid battery gauge with state of charge, voltage, and amps (CAN-FD hybrids/EVs). |
| Show Hybrid/EV Power Flow | toggle | Power-flow gauge showing throttle demand and regenerative braking, plus EV/Hybrid/ICE mode. (C4 label: "Show Hybrid Power Flow") |
| Hybrid/EV Gauge Size (C3X) | selector: Small / Large | Size of the battery and power-flow gauges (with Power Flow enabled). |
| Hybrid Gauge Style (C3X) | selector: Flat / Arched | Flat: horizontal bar. Arched: arch above the torque bar (with Power Flow enabled). |
| Hybrid/EV Power Flow Style (C4) | selector: Flat / Round | Gauge style on the comma 4. |
| Lower Right Display (C4) | selector: Off / Lead Car Speed / Speed / Lead Car Distance / Time to Lead Car | What is shown in the lower-right corner of the driving view. |

*(The comma 4 Visuals menu also exposes sunnypilot's Rainbow Mode path effect.)*

### Longitudinal Tuning

| Setting | Type | Description |
|---|---|---|
| Bypass BP Longitudinal Control | toggle | Use stock longitudinal logic instead of BluePilot's coasting/TTC tuning (no effect when using Ford ACC). |
| Disable Downhill Compensation | toggle | Disable pitch-based gas/brake compensation. Turn on if you get jerky braking on hills — most Fords already manage downhill speed and the two systems can fight. |
| Disable Ford Radar (Vision-Only Leads) | toggle | Ignore the vehicle radar and track leads exclusively from the vision model. Requires reboot. |

### Lateral Tuning

Top-level items (always available):

| Setting | Type | Description |
|---|---|---|
| Primary Control Variable | selector: Angle / Curvature | Angle drives path_angle directly, sidestepping the PSCM's internal filter; Curvature is the proven 4-signal strategy and the current default. Switchable at any time; each scheme keeps its own tuning. The unselected scheme's controls grey out on the comma 3X and are hidden on the comma 4 (where this selector sits on the BluePilot menu's top level). |
| Disable Lane Change Under Speed | toggle | Pause lateral control when the blinker is on below the minimum speed (prevents lane-changing into the median when signaling a turn). |
| Minimum Speed to Pause Lane Change | value: 5–50 | The speed threshold for the above. |
| Show Lateral Control Mode | toggle | Overlay on the wheel icon: **C** = curvature, **A** = angle, **OP** = BP lateral disabled. |
| Disable BP Lateral Control | toggle | Run stock openpilot lateral instead of BluePilot's Ford lateral control — useful to isolate whether an issue is the model or your tuning. |

**Angle Tuning** (active when Primary Control Variable = Angle):

| Setting | Type | Description |
|---|---|---|
| Low Speed Adjustment Factor | value: 0.5–1.5 | Scales low-speed (~city) steering response. Default 1.0. |
| High Speed Adjustment Factor | value: 0.5–1.5 | Scales high-speed (~highway) steering response. Default 1.0. |
| High Speed Low Curve Adjustment Factor | value: 0.25–1.25 | Straightaways at highway speed: reduce if oversteering, increase if understeering. Default 1.0. |
| Lane Change Factor High | value: 0.85–1.50 | Scales steering during lane changes. Some larger vehicles need ~1.05–1.10 to prevent occasional failed lane changes. |
| Enable Lane Positioning | toggle | Nudge the vehicle toward true lane-line center (plus optional bias) in angle mode; unlocks the two below. |
| In-Lane Offset | value: −0.5–0.5 | Bias within the lane; negative = left, positive = right. |
| Lane Centering Strength | value: 0.0–1.0 | How much authority the lane centering trim has vs. the model's own path. |

**Curvature Tuning** (active when Primary Control Variable = Curvature):

| Setting | Type | Description |
|---|---|---|
| Enable Human Turn Detection | toggle | Detect human-initiated turns so the wheel resets afterward instead of fighting you. Recommended on. |
| Lane Change Factor High | value: 0.5–1.0 | Scales steering during lane changes; best results 0.75–0.85 (too low prevents lane changes). |
| Enable Lane Positioning | toggle | Path-angle-based lane centering; required for In-Lane Offset and Lanefull Mode. |
| In-Lane Offset | value: −0.5–0.5 | Negative = left, positive = right. ±0.05–0.1 is the usable range; more causes twitch/ping-pong. |
| Enable Lanefull Mode | toggle | Bias toward centering between detected lane lines rather than whole-scene awareness. Good on well-marked highways. |
| Use Custom Tuning Profile | toggle | Unlocks the blend ratios and PID gain below. |
| Predicted Curvature Blend Ratio Low | value: 0.0–1.0 | Blend on straights. Default 0.4; most drivers like 0.4–0.6. Higher = smoother but lazier. |
| Predicted Curvature Blend Ratio High | value: 0.0–1.0 | Blend in curves. Default 0.4; most drivers like 0.4–0.6. Higher = smoother but may ping-pong in curves. |
| Centering PID Gain | value: 0.0–50.0 | Centering strength on straights (with Lane Positioning + Custom Profile). Default 3.0; adjust in small steps. |

### BP Portal panel (C3X)

| Setting | Type | Description |
|---|---|---|
| Enable Web Routes Server | toggle | Serves the BluePilot Portal on your WiFi at `http://<device-ip>:8088` — with a QR code, live route stats, and a Refresh button in the panel. |

On the comma 4 there is no separate BP Portal panel: the `web routes server` toggle and a `QR code` button (enabled while the server is on) sit directly in the BluePilot menu.

</details>

<details><summary><h3>🌻 sunnypilot Features and Settings</h3></summary>

---

Everything in this section is **upstream sunnypilot** (BluePilot includes it all). Authoritative docs: [docs.sunnypilot.ai](https://docs.sunnypilot.ai/). Highlights:

### Headline features

* **MADS — Modular Assistive Driving System.** Decouples steering from cruise: lane centering engages/disengages independently of longitudinal. Options: Toggle with Main Cruise, Unified Engagement Mode (engage both with cruise; lateral stays on until the MADS button or car-off), and Steering Mode on Brake Pedal (Remain Active / Pause / Disengage).
* **NNLC — Neural Network Lateral Control.** Neural network in place of the default torque controller (torque-steering cars only — on Fords, BluePilot's own lateral control is what steers).
* **DEC — Dynamic Experimental Control.** Lets the model decide when to use ACC-style longitudinal vs. end-to-end Experimental mode. Requires sunnypilot longitudinal.
* **SLA — Speed Limit Assist.** Off / Info / Warning / Assist modes, a source policy (Car Only / Map Only / Car First / Map First / Combined), and fixed or percentage offsets. Map data comes from the OSM database (download in the OSM panel).
* **ICBM — Intelligent Cruise Button Management (Alpha).** Emulates cruise-button presses for limited longitudinal control on platforms without full openpilot longitudinal. *The Ford extension of ICBM is a BluePilot contribution.*
* **SCC-V / SCC-M — Smart Cruise Control Vision & Map.** Slow down for curves ahead using vision path predictions and/or map data. Requires sunnypilot longitudinal or ICBM.
* **Driving Model Manager.** Download and switch among a large library of driving model bundles, with favorites and cache management. Model descriptions: [sunnylink.wiki/models](https://sunnylink.wiki/models).
* **sunnylink.** Secure remote access: settings management from your phone/computer, backup/restore, GitHub sponsor pairing. Pairing sunnylink is the recommended first setup step.

### Settings panels (all upstream sunnypilot)

* **Device** — Pair device, calibration reset, language, Wake Up Behavior (Default/Offroad), Max Time Offroad (Always On…30h), Quiet Mode, driver camera preview, Onroad Uploads, Always Offroad mode, Reset Settings, reboot/power off.
* **Network** — WiFi (with manual Scan), tethering, and the stock network options.
* **sunnylink** — Enable sunnylink, sponsor status, GitHub pairing, settings backup/restore.
* **Toggles** — The stock openpilot toggles: Enable sunnypilot, Experimental Mode, Disengage on Accelerator Pedal, driving personality, and friends.
* **Software** — Updater, target branch selector (tree view of all BluePilot branches), Disable Updates (visible with Developer ▶️ Show Advanced Controls; offroad only, requires reboot).
* **Models** — Current model selector with per-artifact download progress, Refresh Model List, Clear Model Cache, Use Lane Turn Desires (plan the correct turn direction under ~20 mph with blinker on), Live Learning Steer Delay (or a fixed software delay).
* **Steering** — MADS + Customize MADS; Customize Lane Change: Auto Lane Change by Blinker (Off / Nudge / Nudgeless / 0.5–3 s timers) and Delay with Blind Spot; Pause Lateral Control with Blinker (with minimum speed and post-blinker delay); Enforce Torque Lateral Control + Customize Torque Params (tune versions, Self-Tune with a relaxed beta mode, Enable Custom Tuning gating custom Lateral Acceleration Factor / Friction and Manual Real-Time Tuning); NNLC.
* **Cruise** — ICBM; Dynamic Experimental Control; Smart Cruise Control Vision/Map; Custom ACC Speed Increments (short/long press); Speed Limit sub-panel (mode, source policy, offset).
* **Visuals** — Show Blind Spot Warnings, Steering Arc, Rainbow Mode, Standstill Timer, Display Road Name (needs OSM download), Green Traffic Light Alert (Beta), Lead Departure Alert (Beta), true-speed display, hide speedometer, turn signals on HUD, Real-time Acceleration Bar, Metrics Below Chevron (Off / Distance / Speed / Time / All), Developer UI.
* **Display** — Onroad brightness (+delay), settings-UI interactivity timeout.
* **OSM** — Offline OpenStreetMap database downloads by country/US state (for road names and map-based speed limits).
* **Trips** — All-time and past-week drive statistics.
* **Vehicle** — Fingerprint/platform selector with search, plus brand-specific extras (Hyundai longitudinal tuning, Toyota/Subaru stop-and-go, Tesla cooperative steering).
* **Firehose** — comma's Firehose data program.
* **Developer** — Show Advanced Controls, copyparty file server, Quickboot Mode, GitHub runner, error log, and the stock developer toggles.

On BluePilot builds, two extra panels appear in this list: **BluePilot** and **BP Portal** (documented in the [BluePilot Settings Reference](#-bluepilot-settings-reference)).

</details>

<details><summary><h3>⭐ Recommended Settings</h3></summary>

---

**On bp-7.0, start by switching the Primary Control Variable to Angle** and following the [tuning video](https://www.youtube.com/watch?v=DFEHfBmmI3s) — angle mode ships tuned per-platform and mostly just works. The list below comes from the [most frequently recommended settings post](https://bluepilot.dev/2026/03/19/bluepilot-most-frequently-reccomended-settings/) (March 2026) and its lateral entries apply to **curvature mode**; "best" always depends on your driving preferences:

**Toggles:** Enable sunnypilot ✅ · Experimental Mode ❌

**Models — Angle mode:** angle control is very tolerant of driving models — the default model works well, and **Dark Souls v2**, **CD210**, and **PoP v2** are community favorites. Switching models needs no recalibration; give the steering-delay estimator a few days of highway driving after big changes.

**Models — Curvature mode:** legacy models steer best — **WD40** or **Notre Dame** (especially on larger CAN-FD Fords; the default model suits most mid-size/smaller Fords). Model descriptions: [sunnylink.wiki/models](https://sunnylink.wiki/models)

**Steering:** MADS ✅ · Customize Lane Change → Auto Lane Change by Blinker: **0.5s** · Delay with Blind Spot ✅

**Cruise:** sunnypilot longitudinal (Alpha Long) ❌ — use stock Ford ACC (Experimental Mode unavailable) · ICBM ❌

**sunnypilot Visuals:** Show Blind Spot Warnings ❌ (most prefer BluePilot's Show Blindspot Overlay instead) · Green Traffic Light Alert ✅ · Lead Departure Alert ✅

**BluePilot Visuals:** Show Blindspot Overlay ✅ · Show Brake Status ✅ · Show Confidence Ball ✅ (C3X) · Show Radar Lead Overlay ✅ (C3X) · Lower Right Display: Lead Speed (C4)

**BluePilot Lateral Tuning (Curvature) — F-150 / Lightning / Expedition / Navigator:**
* Use Custom Tuning Profile ❌ · Enable Lane Positioning ✅ · Enable Lanefull Mode ✅ · In-Lane Offset **−0.05**

**BluePilot Lateral Tuning (Curvature) — smaller vehicles (Mach-E, Escape, Maverick, …):**
* Model: legacy **WD40** · Use Custom Tuning Profile ✅ · Predicted Curvature Blend Ratio Low **0.30** · High **0.30** · Enable Lane Positioning ✅ · Enable Lanefull Mode ✅ · In-Lane Offset **−0.05** · Centering PID Gain **1.0**

</details>

<details><summary><h3>🎯 Tuning Guide</h3></summary>

---

🎥 Video walkthrough: [BluePilot 7.0 Lateral Tuning](https://www.youtube.com/watch?v=DFEHfBmmI3s)

### Why Ford steering needs tuning at all

On most brands openpilot commands the wheel directly. On Ford, the lateral planner lives inside the **PSCM** (the power steering module): openpilot must impersonate Ford's camera and send a described *path* (curvature, curvature rate, path offset, path angle, …), then hope the PSCM's proprietary logic steers as intended. Comma's driving models are also tuned for looser ADAS platforms, so their corrective commands can cause **ping-pong** on Ford's tight steering. BluePilot's answer in curvature mode is blending desired and predicted curvature (the Blend Ratio settings); in angle mode, driving path_angle directly avoids the PSCM's internal filter altogether. Background reading: [how Ford lateral controls work](https://bluepilot.dev/2025/07/13/how-do-ford-lateral-controls-work-and-why-are-they-such-a-challenge-for-openpilot/), [desired vs. predicted curvature](https://bluepilot.dev/2025/07/13/desired-curvature-versus-predicted-curvature-and-why-it-matters-for-ford/), [what is "the model"](https://bluepilot.dev/2025/07/13/what-isthe-model-why-does-it-change-so-often-and-why-cant-we-fix-problem-x-y-z-on-fords/).

### The method ("Goldilocking")

1. Keep **default tuning** for your first drives.
2. Change **one thing at a time**, in small steps (0.05–0.2), and test on the same quiet stretch of highway.
3. If you get lost, revert to defaults and start over.

### Angle mode (recommended starting point on bp-7.0)

Angle mode ships with per-platform base gains, so start with defaults and the [tuning video](https://www.youtube.com/watch?v=DFEHfBmmI3s). The adjustment factors are straight multipliers on your platform's baseline (1.10 = 10% more steering).

* **Feel:** tune the **Low Speed Adjustment Factor** (~city) and **High Speed Adjustment Factor** (~highway) in small steps; most drivers land between 0.9 and 1.1.
* **Straight roads:** the curve factors don't affect straightaways — use the **High Speed Low Curve Adjustment Factor**: raise it if the car wanders slowly, lower it if it ping-pongs quickly.
* **Centering:** **Enable Lane Positioning**, then set **Lane Centering Strength** and, if needed, a small **In-Lane Offset**.
* **Lane changes:** boost **Lane Change Factor High** to ~1.05–1.10 if changes occasionally fail on large vehicles.
* Between tuning rounds, give the live steering-delay estimator a few days of highway driving to converge. The live lateral debug screen shows commanded vs. actual steering while you tune.

### Curvature mode, in order

* **Phase 1 — smooth steering:** raise **Blend Ratio Low** until straight-road wandering stops (most land 0.40–0.60; 0.8+ = lazy steering that may not hold the lane). Then raise **Blend Ratio High** until in-curve ping-pong stops (lower it if the car hesitates in curves).
* **Phase 2 — centering (only if needed):** Enable Lane Positioning → tune **Centering PID Gain** (start 3.0, gentle 3.0–3.5) → Enable Lanefull Mode if still not centered → finally **In-Lane Offset** (±0.05, never past ±0.1, and only after the PID gain is settled).
* Other: Lane Change Factor High 0.85 for smooth/natural (0.75–0.80 if too abrupt); keep Human Turn Detection on; leave Downhill Compensation alone unless braking is jerky on hills.

### Longitudinal expectations

Stock Ford ACC coasts up to leads and brakes gently by design — that's why most drivers keep it. Openpilot longitudinal is more binary (brake-pump/gas-jab), which BluePilot softens but cannot fully fix. **Neither system replaces you at red lights or stopped traffic:** radar is poor at stationary objects, and vision braking is limited to regular (non-emergency) braking authority — a 59 mph test against a stopped car required driver intervention. These are Level 2 systems; stay ready. ([details](https://bluepilot.dev/2026/02/19/red-lights-traffic-jams-and-adas-limitations/))

</details>

<details><summary><h3>❓ Troubleshooting & FAQ</h3></summary>

---

Common questions, known quirks, and their fixes.

#### Install & first start

**Nothing happens on crank / canbus errors / dash alerts.** The device isn't seeing the ignition signal — almost always the long USB-C cable (CAN-FD installs). Connect the device directly to the harness box with the short cable: if errors persist, the harness is bad or in the wrong IPMA port (the Q4 harness goes on the **second** IPMA port — the first port throws a storm of cluster errors); if it works, re-add the cable/coupler/dock one item at a time — the last item added is the faulty one.

**Dash warning lights after install (Service AdvanceTrac, Check Headlamp, Pre-Collision Assist Not Available).** Usually a USB-C cable damaged during the install — unplug the USB-C at the harness box and see if the errors clear, then replace the cable.

**Fresh comma 4 loops/reboots at ~10% during install.** Install stock openpilot first, then reinstall with the BluePilot URL. Also note a comma 4 cannot run bp-5.0 or older — those branches predate the hardware.

**"Unable to identify your car" / device stuck in dashcam mode.** Auto-fingerprinting fails occasionally — a Ford OTA update changing module firmware versions is a common cause. Fix: manually select your vehicle (`Settings` ▶️ `Vehicle`, or via sunnylink), then restart. The device will not calibrate — and cruise won't engage — until it knows what vehicle it's in.

**How do I update between versions?** Major versions (6.0 → 7.0) are a **new branch**: full uninstall/reinstall with the new URL is required. Hotfixes are pushed to the same branch and arrive automatically through the on-device updater (whether you installed by URL or SSH clone). If the updater won't download: go offroad, then `Settings` ▶️ `Software` ▶️ `Check`, and explicitly press Download.

#### Settings

**A setting is greyed out / can't be modified.** Most settings can't change while the vehicle is running — turn the vehicle off or use `Settings` ▶️ `Device` ▶️ `Always Offroad` (which itself can't engage until the dash shows cruise fully OFF, not just canceled). Tuning settings also depend on the selected control scheme: angle-mode items are inactive in curvature mode and vice versa (Lanefull Mode is curvature-only).

**Experimental Mode is greyed out.** Experimental Mode requires openpilot longitudinal control instead of stock Ford ACC — enable sunnypilot Longitudinal Control (Alpha). Note most Ford drivers prefer stock ACC and skip Experimental Mode.

**MADS doesn't engage with the main cruise button anymore.** Check `Steering` ▶️ `Customize MADS` ▶️ `Toggle with Main Cruise` — updates have been known to flip it off; toggle it off and on if it seems stuck.

**The menu looks broken / items overlap (C3X).** `Settings` ▶️ `BluePilot` ▶️ `System` ▶️ `Reset Menu Layout`.

#### Driving behavior & tuning

**Ping-pong on the highway.** In angle mode: the curve adjustment factors do **not** affect straight roads — use the **High Speed Low Curve Adjustment Factor** (raise it for slow wander, lower it for fast ping-pong). In curvature mode: raise the Predicted Curvature Blend Ratios per the [Tuning Guide](#-tuning-guide) and [recommended settings](#-recommended-settings). Also check the physical basics: the device must be mounted centered on the windshield (a couple of inches off causes ping-pong), and a worn cable will not cause it.

**The car hugs one side of the lane.** Often model behavior — switching driving models is the first fix. Both schemes also offer Enable Lane Positioning with an In-Lane Offset (angle mode adds a Lane Centering Strength control).

**Which driving model should I use?** Angle mode is very tolerant of models — the default works, and Dark Souls v2 / CD210 / PoP v2 are community favorites; legacy WD40 remains a solid curvature-mode choice (see [sunnylink.wiki/models](https://sunnylink.wiki/models)). Switching models does **not** require recalibration, but switching control schemes (curvature ↔ angle) is the time to reset calibration. After tuning changes or recalibration, give the live steering-delay estimator a few days of highway driving (it only learns above ~50 mph on current upstream) before judging or re-tuning.

**Steering warnings on slight turns after 7.0.** Enable angle mode and tune the Low/High Speed Adjustment Factors per the [tuning video](https://www.youtube.com/watch?v=DFEHfBmmI3s). "Turn exceeds steering limit" is a speed-based authority gate, not a physical limit.

**Are the recommended settings still valid on bp-7.0?** The [recommended settings post](https://bluepilot.dev/2026/03/19/bluepilot-most-frequently-reccomended-settings/) applies to **curvature mode only**. For 7.0 the community recommendation is: switch to Angle, then follow the tuning video.

**Occasional radar errors on CAN-FD Fords.** Known cosmetic bug: BluePilot parses raw Ford radar data (~98% accuracy) and occasionally hits frames it can't interpret (believed to be roadside objects, never a lead vehicle). It clears itself within ~10 seconds. If you must restart the vehicle to restore ACC, that's a physical radar module problem, not BluePilot. The `Disable Ford Radar (Vision-Only Leads)` toggle avoids radar entirely.

**Hard brake slams in stop-and-go (openpilot longitudinal).** The radar loses a too-slow lead and AEB kicks in — one reason the recommended setup keeps stock Ford ACC (Alpha Long off). If you do run openpilot longitudinal: enable Dynamic Experimental Control, disable Downhill Compensation, and on hybrids avoid Eco mode (aggressive regen makes braking harsh).

**Can I keep stock Ford ACC but still slow for curves?** Yes — that's ICBM (emulated cruise-button presses); Speed Limit Assist works with ICBM too. Many Fords also have their own curve slowdown ("Predictive Speed Assist" / "Intelligent Cruise" in the Sync driver-assistance menu), though Ford couples curve and speed-limit slowdowns together.

**The dash flashes "Lane-Keeping System On" then "Off" on every engage/disengage.** A years-old cosmetic quirk of how openpilot engages on Fords. Ignore it.

#### Device & misc

**"Communication Issue" / modeld fails to start.** `Settings` ▶️ `BluePilot` ▶️ `System` ▶️ `Clear Crashed Model`. A "speed error: nan m/s" on engagement is usually fixed by re-downloading the driving model.

**Overheating.** The comma 3X runs hot on the new upstream base (the live tuning screen is the worst offender); bp-7.0's AGNOS thermal framework mitigates but doesn't eliminate it. Near the thermal limit the device stops working, including as a dashcam.

**Lightning shows errors at startup.** Some startup errors are normal on the Lightning and clear after ~30 seconds. Persistent errors: rotate the long USB-C cable, and check the coupler orientation — comma-symbol side toward the device cable, lightning-bolt side toward the truck. The coupler only powers from the car's 12V circuit, so a bench 5V USB supply won't light the device — that's normal.

**Do I need OBD power?** No — the harness powers the device. OBD power only keeps it alive with the car off (useful for WiFi uploads).

**Where is my dashcam footage?** On-device at `/data/media/0/realdata` (via SSH), or use comma connect / the BluePilot Portal. The Portal runs on port **8088**.

**One device across multiple vehicles?** No per-vehicle profiles yet — you'll be re-selecting the vehicle and re-tuning when you swap.

**Dealer blames the comma for an unrelated issue.** In the US, Magnuson-Moss prevents denying warranty claims over aftermarket parts unless the part caused the damage — but the practical advice stands: remove the device and harness before service visits.

More: [BluePilot FAQ](https://bluepilot.dev/FAQ/) · [Release Notes and Announcements](https://bluepilot.dev/announcements) · [GitHub Issues](https://github.com/BluePilotDev/bluepilot/issues)

</details>

<details><summary><h3>🆘 How to Ask for Help</h3></summary>

---

BluePilot is volunteer-run — help us help you. When reporting a problem in the Discord #ford channel:

1. **State which BluePilot branch you are on** (e.g. bp-7.0).
2. **Photograph your menus** (especially the BluePilot menus) or type out all of your values.
3. **Upload logs of the problem drive:**
   1. Make sure the device is on WiFi and the car is off (not in accessory mode).
   2. Log in to [connect.comma.ai](https://connect.comma.ai) and click on the drive with the problem.
   3. Click **Files**, then next to "All logs" click **upload xx logs**.
   4. Click **view upload queue** and wait for all logs to finish transferring.
   5. Click **More info**, toggle **Public access** on, then use **Share this route** and send the link to **ajzride** on Discord.

</details>

<details><summary><h3>📜 Version History</h3></summary>

---

| Release | Date | Highlights |
|---|---|---|
| bp-7.0 | Jul 2026 | Angle-primary lateral control with panda safety, per-scheme tuning, angle-mode lane positioning, sunnylink Vehicle-tab settings, Connect Backend, AGNOS 18.4, comma 3X + comma 4 |
| bp-6.x | Feb–Jun 2026 | Port to comma's new UI; comma 4 support; separate BP/SP blind-spot toggles; softer openpilot-long coasting; on-device vehicle selector; Edge MK2; ICBM fixes |
| bp-5.0 | Nov 2025 | BluePilot Portal PWA (dashboard, routes/video, settings, diagnostics); Models/Software panels; upstream MADS, NNLC, DEC, SLA, model manager, sunnylink; dropped comma 3 |
| bp-4.0 | Jul 2025 | Rewritten UI; bp-2.1 curvature limits restored; modular Advanced Lane Positioning; Ford-specific steering-limit logic; Ford ACC vs. sunnypilot longitudinal toggle |
| bp-3.0 / 3.1 | 2025 | First release on the post-opendbc-split sunnypilot base; raw Ford radar parsing; first radar/EV overlays |
| bp-2.0 / 2.1 | 2025 | Introduced path_angle/path_offset control; extensive lateral tunables; BlueCruise dash icons; tuning profiles |
| bp-1.0 / 1.1 | 2025 | Human-turn windup fix with MADS; tunable desired/predicted curvature blending; first Ford tuning menu |

Full details: [BP_CHANGES.json](BP_CHANGES.json) (covers bp-4.0 through 6.x) and the [version history post](https://bluepilot.dev/2025/07/13/bluepilot-version-history/); bp-7.0 details are in the What's New section above and the [release announcement](https://bluepilot.dev/announcements).

</details>

<details><summary><h3>🏆 Special Thanks</h3></summary>

---

* [twilsonco](https://github.com/twilsonco/openpilot)
* B177y and KingStraasha — authors of the Beginner's Guide
* Zorro and Cone_Guy — PSCM firmware research

</details>

<details><summary><h3>📊 User Data</h3></summary>

---

By default, sunnypilot/bluepilot uploads the driving data to comma servers. You can also access your data through [comma connect](https://connect.comma.ai/). The Connect Backend setting can instead send data to Konik (stable.konik.ai), or Offline Mode (unreachable hosts so uploads never succeed).

sunnypilot/bluepilot is open source software. The user is free to disable data collection if they wish to do so.

sunnypilot/bluepilot logs the road-facing camera, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver-facing camera and microphone are only logged if you explicitly opt-in in settings.

By using this software, you understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.

</details>

<details><summary><h3>Licensing</h3></summary>

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

For full license terms, please see the [`LICENSE`](LICENSE) file.

</details>

<details><summary><h3>💰 Support sunnypilot</h3></summary>

---

If you find any of the features useful, consider becoming a [sponsor on GitHub](https://github.com/sponsors/sunnyhaibin) to support future feature development and improvements.


By becoming a sponsor, you will gain access to exclusive content, early access to new features, and the opportunity to directly influence the project's development.

<h3>GitHub Sponsor</h3>

<a href="https://github.com/sponsors/sunnyhaibin">
  <img src="https://user-images.githubusercontent.com/47793918/244135584-9800acbd-69fd-4b2b-bec9-e5fa2d85c817.png" alt="Become a Sponsor" width="300" style="max-width: 100%; height: auto;">
</a>
<br>

<h3>PayPal</h3>

<a href="https://paypal.me/sunnyhaibin0850" target="_blank">
<img src="https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif" alt="PayPal this" title="PayPal - The safer, easier way to pay online!" border="0" />
</a>
<br>
</details>

<details><summary><h3>🔧 Technical Documentation</h3></summary>

---

### BluePilot Portal (technical)

The BluePilot Portal is a React-based Progressive Web App served directly from the device at `http://<device-ip>:8088` (WebSocket updates on port 8089):

* **Home Dashboard** - System status, drive stats, and disk space visualization
* **Routes** - Browse recorded routes with video playback, GPS maps, camera exports, and qlog/rlog downloads
* **Settings** - Configure all parameters from the browser with live WebSocket updates
* **Diagnostics** - Real-time TMUX streaming and parameter browser

Modification endpoints are blocked (HTTP 403) while the vehicle is driving.

### Ford lateral control docs

* Announcement deep-dives: [documenting Ford CAN-FD ADAS lateral control](https://bluepilot.dev/announcements) (message-level detail of Lane_Assist_Data, LateralMotionControl2, ACCDATA_3, and friends)

### Backend Architecture

```
bluepilot/backend/
├── bp_portal.py             # Main HTTP server (BluePilot Portal)
├── config.py                # Configuration and constants
├── handlers/                # HTTP endpoint handlers
├── params/                  # Parameter management and watching
├── routes/                  # Route discovery, GPS metrics, preprocessing
├── video/                   # FFmpeg export and HEVC to MP4 remuxing
├── realtime/                # WebSocket broadcasting
├── logs/                    # Cereal log parsing
├── cache/                   # Metrics/thumbnail/remux caching
├── storage/                 # Route preservation
├── network/                 # Network utilities
├── system/                  # CPU/memory/disk metrics
├── core/                    # Thread-safe state and lifecycle
└── utils/                   # Helper functions
```

### Development

**Backend testing:**
```bash
python3 bluepilot/backend/test_backend_import.py   # Test backend imports
python3 bluepilot/backend/test_modules_only.py     # Test modular components
python3 bluepilot/test_web_routes.py               # Run local server for testing
```

**Frontend development (React + Vite):**
```bash
cd bluepilot/web
npm install
npm run dev      # Development server
npm run build    # Production build
```

</details>


<span>-</span> BluePilotDev Team
