![](https://private-user-images.githubusercontent.com/56047663/329263894-14976646-2e10-4c51-946b-e5e0b91821ee.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MTUyNjIyMzksIm5iZiI6MTcxNTI2MTkzOSwicGF0aCI6Ii81NjA0NzY2My8zMjkyNjM4OTQtMTQ5NzY2NDYtMmUxMC00YzUxLTk0NmItZTVlMGI5MTgyMWVlLnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNDA1MDklMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjQwNTA5VDEzMzg1OVomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTMxMzQwZTE0NjQ0NjNmNTgxMzdmNGI2ZTc4ZGRmNWZmMGE4MmEwMTE2ODc1MDE5ODBjNzk0ZTlkNjc3ZjNmMjcmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0JmFjdG9yX2lkPTAma2V5X2lkPTAmcmVwb19pZD0wIn0.cqqRUk4IZHyoqETqO2fRK89kfSrYXxI1eovSVQ7X0I4)

Table of Contents
=======================

* [Updates on Branch Names and Links](#-updates-on-branch-names-and-links)
* [Join our Discord](#-join-our-discord)
* [What is bluepilot?](#-what-is-sunnypilot)
* [Running in a car](#-running-on-a-dedicated-device-in-a-car)
* [Read Before Installing](#-read-before-installing)
* [Prohibited Safety Modifications](#-prohibited-safety-modifications)
* [Installation](#-installation)
* [Bluepilot Specific Features](#-bluepilot-specific-features)
* [Special Thanks](#-special-thanks)
* [User Data](#-user-data)
* [Licensing](#licensing)

---

<details><summary><h3>💭 Updates on Branch Names and Links</h3></summary>

---

As of May 2025, we are updating the way branches are named and how links are generated. We had intially intended to use a branch naming system similar to openpilot and sunnypilot where there was a "stable" or "release" branch which included all fully vetted code, and then "staging" or "beta" branches with new code that would eventually move into the stable/release branches.  However as we evolved we found everyone liked being able to bounce between newer and older branches to compare features and control. Moving forwards all releases will simply be named bp-"feature release number" as an example "staging-1.1" which features the bluepilot 1.1 features (custom tuning) will become "bp-1.1".  We will not delete older branches so that anyone can go back and view older code for references.  Branches that no longer work properly will be denoted as -depricated.

To install any version of bleupilot, use the following URL formula (URL is case sensitive)

installer.comaa.ai/BluePilotDev/"branch name"

For example

installer.comma.ai/BluePilotDev/bp-2.1

will install the bp-2.1 branch.  Branches known to no longer work due to changes in the comma codebase will be apended with -deprecated so it will be obvious they will not install or work correctly.

</details>


---

<details><summary><h3>💭 Join our Discord</h3></summary>

---

Join the official #ford channel at the sunnypilot Discord server to stay up to date with all the latest features and be a part of shaping the future of bluepilot!
* [sunnypilot Discord server](https://discord.sunnypilot.com)

</details>

<details><summary><h3>🌞 What is bluepilot?</h3></summary>

---

[bluepilot](https://github.com/bluepilotdev/bluepilot) is a fork of the hugely popular SunnyPilot project for the Comma3 and Comma3X.  The goal of BluePilot is to develop, test, and stage Ford specific enhancements, validating them before submission to the SunnyPilot team for inclusion in the parent project.

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


<details><summary><h3>⚒ Installation</h3></summary>

* bluepilot not installed
  1. [Factory reset/uninstall](https://github.com/commaai/openpilot/wiki/FAQ#how-can-i-reset-the-device) the previous software if you have another software/fork installed.
  2. After factory reset/uninstall and upon reboot, select `Custom Software` when given the option.
  3. Input the installation URL based on the desired branch. Example: ```installter.comma.ai/BluePilotDev/staging-2.1``` [^4] (note: `https://` is not requirement on the comma three)
  4. Complete the rest of the installation following the onscreen instructions.

* bluepilot already installed and you installed a version after 0.8.17?
  1. On the comma three, go to `Settings` ▶️ `Software`.
  2. At the `Download` option, press `CHECK`. This will fetch the list of latest branches from sunnypilot.
  3. At the `Target Branch` option, press `SELECT` to open the Target Branch selector.
  4. Scroll to select the desired branch


Requires further assistance with software installation? Join the [sunnypilot Discord server](https://discord.sunnypilot.com) and message us in the `#ford` channel.

  </details>

<details><summary><h3>🚗 BluePilot Specific Features</h3></summary>

---
In addition to all sunnypilot features, bluepilot incorporates the following Ford specific enhacements.

- [**Improved Ford Longitudinal controls**] - logic to adjust stock OpenPilot single acceleration signal into seperate gas and brake signals for much smoother long control on ford vehicles.
- [**Anti-Windup in Turns**] - Logic to reset the EPAS back to zero when a human turn is detected.  This prevents the EPAS from winding up and fighting to keep turning after the car has straightened up.  Makes experimental mode and MADS safer to use.
- [**OEM Style Lateral Control Logic**] - This is a complete rewrite of the lateral control logic to match the behavior of the OEM EPAS system.  It includes a blend of desired curvature and predicted curvature to achieve smoother signals, incorporation of curvature_rate signal to help with getting in and out of curves more ontime and the path_offset and path_angle variables to increase the available steering torque and to allow for custom in lane positioning as well as enabling the legacy sunnypilot style lanefull mode.
- [**Improved Lane Change Logic**] - Even further improvements to the lange change logic to help fix issues with the older logic that caused the lane change to overshoot and then have to correct back.

The following settings and tuning are available in the BluePilot menu (after completing one drive in a Ford vehnicle)

- [**Show Animated Steer Angle Icon**] - This option will animate the steering icon in the upper right to match the approximate angle of the steering wheel
- [**Custom Model Path Color**] - Choose different colors for the path animation on the UI.  This does not work when experimental mode is engaged.
- [**Show Radar Lead Vehicle Overview**] - Animate the icon on the lead vehicle.  Blue equals radar lead, yellow means vision lead.
- [**Show Hybrid Electric Data Overlays**] - Add a panel at the bottom of the UI showing how the electrical battery and motor are being utilized
- [**Show Hybrid Battery Data Overlay**] - Add a panel at the bottom of the UI showing the hybrid or electic battery status
- [**Hybrid Drive Gauge Size**] - Adjust the size of the hybrid/electric data panel on the UI
- [**Show Standstill Timer**] - A sunnypilot feature not yet ported to dev-c3-new.  Still a work in progress.
- [**Show Blindspot Indicators**] - Another sunny feature not yet ported but available in BluePilot.  Get red inidcators on the UI of blindspot detection.
- [**Show Stop Indicator Overlay**] - Add a stop sign on the UI where OpenPilot has detect a stop or redlight.  Great for knowing what experimental mode detected.
- [**Show Hands Free UI**] - This setting enables the hands free Blue Cruise dash.  This will only work in vehicles with a digital dash that can display the icon.  Known to work in Mach E, Lightning, Ranger, and F150s with digital dash.
- [**Send Lane Depature Signals**] - This sends the lane depature alerts to the dash on vehicles with a supporting dash.
- [**Send Driver Monitor Signals**] For vehicles with the Blue Cruise dash, this will send the pay attention alerts to the dash.
- [**Enable Human Turn Detection**] - This setting enables the human turn detection logic.  This will help prevent the EPAS from winding up and fighting to keep turning after the car has straightened up.  Makes experimental mode and MADS safer to use.
- [**Lane Change Factor**] - This is a tunable reduction in lane change signal to enable less aggressive lane changes.  Lower numbers equal slower lane changes.  Too low and the lane chagne might fail.
- [**Use Custom Tuning Profile**] - There are 2 preset tuning profiles which are automatically selected based on the vehicle driven (CAN or CANFD). Toggling this parameter will enable custom tuning values for lateral control
- [**Predicted Curvature Blend Ratio Low**] - This setting allows for a blend of desired curvature and predicted curvature on straight aways to achieve smoother signals.  Lower numbers will make steering more aggressive.  To tune this value begin to increase the blend ratio until the car starts to "wander" or experience a slow ping pong effect at highway speeds.  If using advanced lateral control, leave it at a wander, it is not desired for this number to be low enough for complete control, because if the signal is too strong it will conflict with the other signals. If not using advanced lateral control, then lower the PC Blend Ratio back down until wadnering stops. Default Values are CANFD = 0.65, CAN = 0.4
- [**Predicted Curvature Blend Ratio High**] - This setting allows for a blend of desired curvature and predicted curvature in curves to achieve smoother signals.  Lower numbers will make steering more aggressive.  To tune this value begin to decrease the blend ratio until the car starts to ping pong inside of curves, then increase slightly until the behavior stops. Default Values are CANFD = 0.4, CAN = 0.2
- [**Enable Advanced Lateral Control**] - This settings enables the OEM style logic with path_offset and path_angle variables.  This allows for custom in lane positioning. Disabling this setting will revert to only using curvature and curvature_rate which was the previous behavior in bluepilot 1.0
- [**Enable Legacy Style Lanefull Mode**] - This setting enables the legacy style lanefull mode which will use lane lines to determine the ideal path for the vehicle rather than Commas model.  This setting is only applicable when linelines are strong and the lane width is not too wide.
- [**In lane offset**] - This setting allows for custom in lane positioning.  Negative numbers will move the car to the left, positive numbers will move the car to the right.
- [**Path Angle Amplitude**] - This setting controls how aggressive the path_angle signal is applied to correct for deviations from the ideal path.  To tune this number, first find the ideal value of PREDICTED CURVATURE BLEND RATIO, then begin to increase until the car starts to feel "twitchy" or experiences a very fast ping pong effect at highway speeds.  Then reduce the number until the car is smooth and stable. Default Values are CANFD = 1.8, CAN = 7.5
- [**Path Angle in-Curve Reduction**] - This setting controls how aggressive the path_angle signal is applied to correct for deviations from the ideal path while in curves. It is a fraction of the amount of path angle applied on straightaways.  To tune this number, first find the ideal value of PATH ANGLE AMPLITUDE, then begin to increase until the car starts to feel "twitchy" or experience a very fast ping pong effect in curves at highway speeds.  Then reduce the number until the car is smooth and stable in curves at highway speeds. Default Values are CANFD = 0.00, CAN = 0.20

</details>

<details><summary><h3>📗 How To's</h3></summary>

---

How-To instructions can be found in [HOW-TOS.md](https://github.com/sunnyhaibin/openpilot/blob/(!)README/HOW-TOS.md).

</details>

<details><summary><h3>🏆 Special Thanks</h3></summary>

---

* [twilsonco](https://github.com/twilsonco/openpilot)

</details>

<details><summary><h3>📊 User Data</h3></summary>

---

By default, sunnypilot/bluepilot uploads the driving data to comma servers. You can also access your data through [comma connect](https://connect.comma.ai/).

sunnypilot/bluepilot is open source software. The user is free to disable data collection if they wish to do so.

sunnypilot/bluepilot logs the road-facing camera, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver-facing camera is only logged if you explicitly opt-in in settings. The microphone is not recorded.

By using this software, you understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.

</details>

<details><summary><h3>Licensing</h3></summary>

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

</details>

<details><summary><h3>Support sunnypilot</h3></summary>
<h3>💰 Support sunnypilot</h3>

---

If you find any of the features useful, consider becoming a [patron on Patreon](https://www.patreon.com/sunnyhaibin) or a [sponsor on GitHub](https://github.com/sponsors/sunnyhaibin) to support future feature development and improvements.


By becoming a patron/sponsor, you will gain access to exclusive content, early access to new features, and the opportunity to directly influence the project's development.

<h3>Patreon</h3>

<a href="https://www.patreon.com/sunnyhaibin">
  <img src="https://user-images.githubusercontent.com/47793918/244128051-bc7e913e-a196-4455-926e-23aec9a4bd3b.png" alt="Become a Patron" width="300" style="max-width: 100%; height: auto;">
</a>
<br>

<h3>GitHub Sponsor</h3>

<a href="https://github.com/sponsors/sunnyhaibin">
  <img src="https://user-images.githubusercontent.com/47793918/244135584-9800acbd-69fd-4b2b-bec9-e5fa2d85c817.png" alt="Become a Sponsor" width="300" style="max-width: 100%; height: auto;">
</a>
<br>

<h3>PayPal</h3>

<a href="https://paypal.me/sunnyhaibin0850" target="_blank">
<img src="https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif" alt="PayPal this" title="PayPal - The safer, easier way to pay online!" border="0" />
</a>
<br></br>
</details>


<span>-</span> BluePilotDev Team
