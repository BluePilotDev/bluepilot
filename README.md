![](https://user-images.githubusercontent.com/47793918/233812617-beab2e71-57b9-479e-8bff-c3931347ca40.png)

Table of Contents
=======================

* [Join our Discord](#-join-our-discord)
* [What is bluepilot?](#-what-is-sunnypilot)
* [Running in a car](#-running-on-a-dedicated-device-in-a-car)
* [Read Before Installing](#-read-before-installing)
* [Prohibited Safety Modifications](#-prohibited-safety-modifications)
* [Installation](#-installation)
* [Highlight Features](#-highlight-features)
* [Branch Definitions](#-branch-definitions)
* [Recommended Branches](#-recommended-branches)
* [Special Thanks](#-special-thanks)
* [User Data](#-user-data)
* [Licensing](#licensing)

---

<details><summary><h3>💭 Join our Discord</h3></summary>

---

Join the official Ford channel at the sunnypilot Discord server to stay up to date with all the latest features and be a part of shaping the future of bluepilot!
* https://discord.com/channels/880416502577266699/1064822699085545522

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

Please refer to [Recommended Branches](#-recommended-branches) to find your preferred/supported branch. This guide will assume you want to install the latest `stable` branch.

* bluepilot not installed
  1. [Factory reset/uninstall](https://github.com/commaai/openpilot/wiki/FAQ#how-can-i-reset-the-device) the previous software if you have another software/fork installed.
  2. After factory reset/uninstall and upon reboot, select `Custom Software` when given the option.
  3. Input the installation URL per [Recommended Branches](#-recommended-branches). Example: ```https://bit.ly/bp-stable``` [^4] (note: `https://` is not requirement on the comma three)
  4. Complete the rest of the installation following the onscreen instructions.

* bluepilot already installed and you installed a version after 0.8.17?
  1. On the comma three, go to `Settings` ▶️ `Software`.
  2. At the `Download` option, press `CHECK`. This will fetch the list of latest branches from sunnypilot.
  3. At the `Target Branch` option, press `SELECT` to open the Target Branch selector.
  4. Scroll to select the desired branch per [Recommended Branches](#-recommended-branches). Example: `stable`

|    Branch    |         Installation URL         |
|:------------:|:--------------------------------:|
| `stable` | https://bit.ly/bp-stable |
| `experimental` | https://bit.ly/bp-experimental |

Requires further assistance with software installation? Join the [sunnypilot Discord server](https://discord.sunnypilot.com) and message us in the `#ford` channel.

  </details>

<details><summary><h3>🚗 Highlight Features</h3></summary>

---
In addition to all sunnypilot features, bluepilot incorporates the following Ford specific enhacements.

- [**Improved Ford Longitudinal controls**](#Imporoved-long) - logic to adjust stock OpenPilot single acceleration signal into seperate gas and brake signals for much smoother long control on ford vehicles.
- [**Anti-Windup in Turns**](#awp-turns) - Logic to reset the EPAS back to zero when a human turn is detected.  This prevents the EPAS from winding up and fighting to keep turning after the car has straightened up.  Makes experimental mode and MADS safer to use.
- [**Anti Ping Pong Logic**](#app) - Applies a blend of desired curvature and predicted curvature when no curves have been detected for at least the next 3 seconds.  This greatly reduce steering wheel wiggle and ping pong.
- [**Less Aggressive Lance Changes**](#lane-change) - utilitizes a blend of desired curvature and predicted curvature to achieve smoother, less aggressive lane changes.
   

</details>

<details><summary><h3>⚒ Branch Definitions</h3></summary>

---

|    Tag    | Definition           | Description                                                                                                                                                                                 |
|:---------:|----------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `stable` | stable branches     | Include features that are **verified** by trusted testers and the community. Ready to use. ✅                                                                                                |
| `experimental` | experimental branches     | Include new features that are in testing, this branch might fail to boot, crash, or have unpredictable behavior.  Please test with caution ⚠                                                                                        

Example:
* [`stable`](https://github.com/bluepilotdev/bluepilot/stable): Latest stable branch that is verified by trusted testers and the community. Ready to use.
* [`experimental`](https://github.com/bluepilotdev/bluepilot/experimental): Latest development branch  that include all sunnypilot and experimental bluepilot features  Testing required with extreme caution

</details>

<details><summary><h3>✅ Recommended Branches</h3></summary>

---

| Branch                                                                              | Definition                                              | Compatible Device |                                                                                |
|:------------------------------------------------------------------------------------|---------------------------------------------------------|-------------------|--------------------------------------------------------------------------------------------|
| [`stable`](https://github.com/bluepilotdev/bluepilot/stable)           | • Latest release/stable branch                          | comma three       | 
| [`experimental`](https://github.com/bluepilotdev/bluepilot/experimental)                   | • Latest development branch with experimental features  | comma three       | 

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



<span>-</span> BluePilotDev Team
