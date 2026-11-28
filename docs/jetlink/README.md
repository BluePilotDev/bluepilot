# JetLink: the big driving model on a computer beside the comma

JetLink runs openpilot's large driving models on a computer connected to the comma's USB-C port. The comma sends its
camera frames over the cable and drives on the big model's output. The comma's own small model drives until the big
model is ready, and takes back over whenever the link drops or falls behind.

JetLink is [zoompilot/jetlink](https://github.com/zoompilot/jetlink) by Zeph Leggett, MIT-licensed. BluePilot carries
it as the `jetlink_repo` submodule, pinned to v0.9.0. **It is experimental.** If the link drops or lags while
engaged, the comma says **TAKE CONTROL** and stays engaged on the small model. Be ready to take over.

- [For owners](#for-owners): what you need, setting it up, what the comma shows, what to do.
- [For developers](#for-developers): where the code is, logs, tests, and moving the pin.

## For owners

### What you need

- A comma four or a comma 3X. **JetLink is tested upstream on the comma four only.** The 3X shares its kernel and is
  expected to work, but nobody has tested it on BluePilot yet.
- A device that runs JetLink, and the link setting that goes with it:

  | Device | Link setting | Connection |
  |---|---|---|
  | Jetson Orin Nano Super, Apple silicon Mac, Linux PC with an NVIDIA GPU | **USB** | A USB 3 data cable |
  | Android phone, Android 12 or later with USB 3 (experimental) | **USB** | A powered USB 3 hub: the phone and a charger on the hub, the hub's USB-A port to the comma |
  | USB-C iPhone or iPad on iOS 26.1 or later, a USB 3 model (experimental) | **iOS** | A USB 3 USB-C data cable; keep the JetLink app open on screen |

- USB 3 end to end: the device, the cable and the port. JetLink is built for USB 3; a USB 2 link is not supported.
- **Separate power for the comma and the computer.** With the link on, the comma draws nothing from its USB-C port.
  It must run from the car harness.

Set up the device with JetLink's own guide: `jetlink_repo/README.md`, or
[JetLink's README](https://github.com/zoompilot/jetlink#quick-start). Its "Quick start" table links one guide per
device. Set up offroad, with both devices online. Where a guide says to set the comma's **Settings > Models >
Jetlink**, use BluePilot's **Big Model Link (JetLink)** setting instead.

### Turning it on

The settings are in `Settings` ▶ `BluePilot` ▶ **JetLink** on the comma 3X, and in the **jetlink** screen of the
BluePilot menu on the comma four. Both change **offroad only**.

| Setting | Type | Description |
|---|---|---|
| Big Model Link (JetLink) | selector: Off / USB / iOS | Run the big driving model on a device on the comma's USB-C port: **USB** for a Jetson, Mac, PC or Android phone, **iOS** for an iPhone or iPad. Reboot after changing it. |
| JetLink Status | read-only | What the link is doing (below). On the comma 3X, tap it for the details: progress, the USB port, the big model and the ADB note. On the comma four it shows the one line that matters, and "reboot to apply the new link setting" after you change the link until you reboot. |
| JetLink: Charge Phone | toggle | Let a phone running JetLink charge from the comma's USB-C port. |

### On the home screen

The **Driving Models** card shows the on-device model and, with the link on, JetLink's big model: its state, how it
is linked, the link's USB speed (USB3 green, USB2 yellow: JetLink wants USB 3), the build's progress, and which model
drives. The on-device model drives whenever the big one cannot. On the comma 3X the card is in the home screen's right
column on a paired device; on the comma four it is its own page, a swipe left from home, offroad. Tap it for the
Models settings.

With the link on, the driving-model picker lists JetLink's big models in place of the comma's own big models, as long
as no chestnut is fitted. The model that runs when you have not picked one is JetLink's default, shown in the picker.

### What the status means

| Status | Meaning |
|---|---|
| Off | The link is off. |
| Not available | This build has no usable JetLink, so the link stays off. |
| Connecting | Waiting for the device on the USB-C port. A count of drops means the link keeps dropping: check the cable, and on a phone that the JetLink app is open. |
| Preparing *n*% | The computer is downloading or building the big model. The first build of a model takes minutes. |
| Ready | The big model is ready. It takes over when you next engage. |
| Ready: re-engage to switch | The big model became ready while engaged. Disengage and re-engage to switch to it. |
| Big model driving | The big model drives. |
| Reconnecting | The link dropped. The small model drives while it reconnects. |
| Failed | The link failed. The small model drives. Check that JetLink is running on the device and that the link setting matches it (USB or iOS). The comma tries again on its own. |
| Unavailable | The big model cannot run on this device or link. The details say why, and so does an offroad alert, "JetLink unavailable: …". |

### Alerts while driving

| Alert | What happened | What to do |
|---|---|---|
| Big Model Ready / Re-engage to switch | The big model became ready while engaged. The small model still drives. | Nothing, or disengage and re-engage to switch. |
| TAKE CONTROL / Big model lost: small model driving | The link dropped or fell behind while the big model drove. The comma stays engaged on the small model. | Take control, and check the cable and the computer. |

### Good to know

- **ADB over USB is unavailable while JetLink holds the port.** The BluePilot portal over WiFi still works.
- With the link on, a JetLink script runs as root at boot. It cuts the comma's draw from the USB-C port and sets up the
  port for the computer. With the link off it never runs.
- When the comma powers off for good, it first asks the computer to shut down, and waits for it for at most 30
  seconds.
- Switching between the small and big models is a step change in what steers the car. Re-engaging is the only way
  the big model takes over. The hand-back to the small model happens on its own and comes with the TAKE CONTROL alert.

## For developers

### Where the code is

| What | Where |
|---|---|
| JetLink, pinned to v0.9.0 (API 4) | `jetlink_repo` (submodule), linked as `jetlink` by `launch_chffrplus.sh` |
| The adapter: JetLink's hooks, or every hook answering as the link off when JetLink is missing, too old or another API | `bluepilot/jetlink_adapter/__init__.py` |
| The big-model list in sunnypilot's model manager, and a stale pick carried over | `bluepilot/jetlink_adapter/catalog.py`, wired in `openpilot/sunnypilot/models/manager.py` |
| modelDataV2BP, what both modelds publish | `bluepilot/jetlink_adapter/telemetry.py` |
| jetlinkd, JetLink's owner process, restarted with backoff | `bluepilot/system/manager/restarting_process.py`, added with `procs +=` in `openpilot/system/manager/process_config.py` |
| The swap events and their alerts | `bluepilot/selfdrive/selfdrived/jetlink_events.py`, `events_bp.py` |
| The power-off wait and the offroad alert | `bluepilot/system/hardware/jetlink_shutdown.py`, wired in `hardwared.py` |
| The modeld seams | `openpilot/selfdrive/modeld/modeld.py`, `openpilot/sunnypilot/modeld_v2/modeld.py` |
| Settings and status | `openpilot/selfdrive/ui/bp/lib/jetlink_settings.py`, `jetlink_status.py`; the comma four's screen is `openpilot/selfdrive/ui/bp/mici/layouts/settings/jetlink_mici.py` |
| Licenses and credits | `bluepilot/jetlink_adapter/THIRD_PARTY_NOTICES.md` |

JetLink's porting contract is `jetlink_repo/docs/porting.md`. Every change to an openpilot or sunnypilot file is inside
`# BluePilot:` / `# End BluePilot` markers.

### Params

| Param | Holds |
|---|---|
| `JetlinkLink` | The link: 0 off, 1 USB (a Jetson, Mac, PC or Android phone), 2 iOS. |
| `JetlinkChargePhone` | The charge-phone toggle. |
| `JetlinkProgress` | JetLink's offroad progress: stage, fraction, message, drops. |
| `JetlinkSpec` | JetLink's record of the host and its models. |
| `Offroad_JetlinkUnavailable` | The offroad alert, with JetLink's reason. |

### Logs

| Message | Field | Holds |
|---|---|---|
| `modelDataV2BP` (20 Hz, both modelds) | `acceleratorState` | JetLink's big model: `none`, `joining`, `ready`, `running`, `retrying` or `unavailable`. `none` with the link off. |
| | `handovers` | Swaps and hand-backs since modeld started. Each step is a change of driving model. |
| `modelV2` | `big` | The big model drove this frame. |
| `onroadEventsBP` | `events` | `bigModelAvailable`, `bigModelLinkLost`. |

`modelDataV2BP` claims the `customReserved11` slot and `onroadEventsBP` claims `customReserved12`, both in
`openpilot/cereal/custom.capnp`. The `acceleratorState` ordinals are zoompilot's, so one reader serves both forks'
logs. To find the swaps in a route, look for each change in `handovers`, then at `modelV2.big` and the lateral
commands around it.

### Tests

```bash
python tools/test_runner.py bluepilot/jetlink_adapter/tests
```

These tests include JetLink's own fork kit (adapter, seams, fallback). Without the checkout, every hook answers as the
link off and the kit's tests skip. `.github/workflows/bp-jetlink.yml` runs both cases.

Not tested yet, and needed before JetLink is offered to users:

- **Ford lateral across swaps.** Replay a route with forced swaps through BluePilot's lateral stack. The commands must
  stay inside the panda's window, with no overshoot at the step.
- **The comma 3X.** The USB gadget, the USB power role swap, the boot draw-off, and running on harness power alone.
- **Power on every Ford harness**, including the TRON F-150 with CAN-only ignition, and the 30 s power-off wait against
  its offroad timing.

### Moving the pin

JetLink bumps its API between releases, with one minor release of overlap. To move the pin:

1. Read the CHANGELOG's **Forks** group.
2. Review the diff of `jetlink/comma/jetlink-root.sh`: it runs as root at boot.
3. Move `jetlink_repo` to the new tag, and the adapter's `API` with it.
4. Run the tests above, with and without the checkout.
5. Update the pin row in `THIRD_PARTY_NOTICES.md`.
