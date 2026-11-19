# How BluePilot steers your Ford: a guide for owners

This guide explains how the clean Ford steering build works and why it was built this way. It is for
owners who install software on a comma device but don't read code. The developer guide
([DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md)) covers the same build in technical depth. Installing,
updating and going back are in [INSTALL.md](INSTALL.md).

**This is a test build.** It lives in a private repository and has not been released. Nothing in it has
been driven yet with its new parts switched on.

## 1. How openpilot steers a Ford

openpilot does not turn your steering wheel with its own motor. It uses the same channel Ford's own
lane centering uses: it sends a message to the car's **power steering module** (Ford calls it the PSCM,
the power steering control module) and the module does the steering.

That message describes a path for the car to follow. It has four parts:

- **curvature:** how tightly to turn (a curve with a 500 m radius is a curvature of 1/500);
- **path angle:** which way the path points compared with where the car points now;
- **path offset:** how far the path is to the left or right of the car;
- **curvature rate:** how fast the curvature is changing.

openpilot's driving model plans the path and works out the curvature the car should drive. Stock
openpilot sends that curvature and leaves the other three parts at zero. The steering module then
decides, with its own software, how much torque to put on the wheel to get that curvature. The module
keeps control of its own safety limits: how much torque it gives, how quickly it lets go when you
steer, and so on.

## 2. Why different Fords respond differently

Owners have noticed for a long time that the same openpilot code feels different on different Fords.
The research behind this build found the main reason: **each steering module runs its own program
with its own set of numbers**, and those differ between models, model years and even software
updates.

Two part numbers in the module say which program and numbers it has:

- the **strategy**: the program itself (for example ML3V-14D003-BD on a 2021–23 F-150);
- the **data file**: the vehicle numbers that program uses (for example ML34-14D007-EDL).

Ford can change both over the air or at a dealer visit. The numbers decide things you can feel:

- **How fast the module lets the curvature change.** On the F-150s studied, the module moves its
  copy of the curvature request no faster than a fixed rate. At town speeds that rate is slow: a
  curve that tightens quickly takes the truck several seconds to follow, however quickly openpilot
  asks.
- **How far ahead the module looks** along the path when it uses the path angle. This differs between
  models, so the same path angle turns one truck more than another.
- **How much sideways acceleration the module allows** before it stops adding steering.
- **How quickly it reacts.** The delay from a request to the car turning differs between software
  levels. 33 F-150s on one level answered 0.15–0.3 s sooner than trucks on the next level.

Earlier BluePilot versions tried to make up for these differences with many tuning settings and a
second steering mode. Those settings were tuned on one truck and felt different on the next.

## 3. What this build changes

### One way of steering, for every Ford

The build has **one steering controller**. It sends openpilot's planned curvature to the module, and
nothing else touches it. Gone are: the second "angle" steering mode, the small pulses earlier versions
sent at hand-off, the resets when you turned the wheel, blending in predicted curvature, the lane
offset (driving off-centre in the lane), the lane-change adjustments, and 18 of the 19 lateral
settings.

**Why:** most of those settings patched problems that came from three causes: a panda safety shortcut,
a clip of the command to what the car was measured doing, and the angle mode, which each model's
module answers differently. With the causes fixed, the patches aren't needed. The lane offset will come
back later as part of openpilot's plan, not as a steering setting.

### The command is no longer held to what the car is doing

Earlier versions kept the curvature they sent close to the curvature the car's yaw sensor measured.
On a car whose yaw sensor reads a little off, that held openpilot's request back. On recorded drives
from an Explorer whose yaw sensor reads off, the old command sat noticeably away from what openpilot
asked for. In the same replays the new command stays within a hair of openpilot's request.

### The build reads your steering module's part numbers at every start

At every start the build asks the steering module for its part numbers: strategy, data file,
calibration, an over-the-air update counter, and a few configuration blocks that are only logged. It
only reads. It never writes anything to the module.

It reads fresh every time because Ford can change the module's software between drives, and a
setting tuned for one software level must not be used on another.

### A table of the steering software the build knows

The build has a table with one row per known combination of strategy and data file. A row holds that
module's own numbers: its sideways-acceleration limit, its reaction time, and, for some rows, how far
it looks ahead. A row is used only when the module answers **exactly** the part numbers in the row.
A newer or older level, or another data file, is not that row.

**Why exact:** the research compared data files. Files that look alike can hold different steering
numbers. One F-150 data file has other thresholds, and the Expedition's and Navigator's files run the
same program with other path numbers. Guessing from the family name would apply the wrong numbers.

### The fallback

If your module doesn't answer, or its part numbers aren't in the table, the build uses the
**fallback**: it steers by curvature alone with comma's own stock settings. This is the one command
with drives from many Fords behind it, and openpilot's panda safety accepts it.

### Path angle: built in, switched off for now

The module's speed limit on curvature changes is what makes Fords slow in town curves. The path angle
part of the message has its own, faster limit. The build can use it: it sends the curvature as usual,
and on rows where it knows how far the module looks ahead, it puts the part of the request the
curvature limit is still holding back onto the path angle. In a steady curve the path angle goes back
to zero and curvature carries the whole turn.

**Why this way:** the research compared many approaches (developer guide, section 2.1). This one
keeps openpilot's plan as the command and lets the module run its own tested steering. It only tells
the module where openpilot's path is, in the module's own terms. In a computer model of the F-150's
module, it made town-speed changes 2 to 3.5 times faster than curvature alone.

**Why it is off:** the numbers for each row come from reading the module's software and from that
computer model, not from a drive. **Path angle is switched off on every row in this build.** It will be
switched on for a family of cars only after a drive on one of its modules confirms that the module
really turns as much as the row says (section 5).

### A health check

While you drive, the build checks whether the car follows the steering command the way the row says
it should. The check only watches and reports. It never changes the steering (section 6).

### Limits

The build uses comma's limits for how fast the sideways acceleration may change (about 3.6 m/s³) and
how much sideways acceleration may be asked for (about 3.6 m/s² on older Fords, kept lower at about
2.4 m/s² on newer Fords with the faster CAN FD data bus). Where the row says the module itself allows
less, the build asks for no more than that. openpilot's planner is told the same limit, so a turn the
module can't take shows as openpilot's usual "turn exceeds steering limit" warning. It doesn't fail
quietly.

## 4. What this means for your car

### Support levels

Look in **Settings → BluePilot → Lateral Tuning → Steering Support** (on a comma four: the BluePilot
lateral page, **steering support**). It shows one of these:

| Shown | What it means for you |
|---|---|
| **Full** | Your module's exact software is in the table, and its row was checked against a computer model of the module or a drive. The build steers by curvature with your module's own numbers. This row is the one that can get path angle once a drive confirms it. Today it steers by curvature only. |
| **Curvature** | Your module's software is in the table. Its steering program was read, but it hasn't been checked against a model or a drive. Steers by curvature with your module's own numbers. |
| **Fallback** | Your module didn't answer, or its software isn't in the table. Steers by curvature with comma's stock settings. This is how stock openpilot steers, without the clip to the measured curvature. |
| **Not supported** | The software is known and can't be steered by this build. The car runs as dashcam only. No row says this today. |
| **Off** | You switched on "Disable BP Lateral Control". openpilot's stock Ford steering runs. |
| **Not read yet** | The car hasn't been started with this build yet. |

", degraded" after the level means the health check found a problem on the last drive (section 6).
The text under the row gives the reason, the part numbers your module answered, and how the car was
recognised.

A car that openpilot itself doesn't support is not supported by this build either.

### Which software has a row

The table at this build. A row applies only when your module answers exactly these part numbers. The
"Steering Support" text shows yours.

**Full:**

| Vehicle | Strategy | Data file(s) |
|---|---|---|
| F-150 2021–23 | ML3V-14D003-BD | ML34-14D007-EDL, -BDL |
| F-150 Lightning | RL38-14D003-AA | RL38-14D007-DD, -AD |
| Explorer 2020–24 | M1MC-14D003-AC; L1MC-14D003-AL | MB5C-14D007-BD; MB5C-14D007-BA |
| Ranger 2024–25 | RB3C-14D003-AA | RB3C-14D007-ECC |
| Maverick 2022–24 | NZ6C-14D003-AL | NZ6C-14D007-ABD |
| Corsair, Escape and Bronco Sport 2020–22 | LX6C-14D003-AL | LX6C-14D007-CAH |
| Bronco Sport 2025 | RZ1C-14D003-AC | RZ1C-14D007-AAF |
| Escape 2025 | PZ11-14D003-FC | PZ11-14D007-EAD |

**Curvature:**

| Vehicle | Strategy | Data file(s) |
|---|---|---|
| F-150 2021–23, level BC | ML3V-14D003-BC | any, but the module must answer it (below) |
| F-150 2024–26 | RL14-14D003-AC | SL34-14D007-ECD, -BCD, -CCD, -BAD, -EFD, -EAD; TL34-14D007-BCA |
| Explorer 2025 | V1MC-14D003-AA | V1MC-14D007-GAA |
| Mustang 2025 | SR3C-14D003-AA | SR3C-14D007-AA, -DA |
| Mustang Mach-E 2025 | SJ9C-14D003-AD | SJ9C-14D007-AAF, -ABG, -ADF |
| Mustang Mach-E 2021–22 | LJ9C-14D003-GA; LJ9C-14D003-HA | LJ9C-14D007-ANA, -AMA, -ARA; LJ9C-14D007-KNB |
| Mustang Mach-E 2023–24 | LJ9C-14D003-FA | LJ9C-14D007-VDA, -VBA |
| Mustang 2024 | PR3C-14D003-AD | PR3C-14D007-BF, -EF |
| Transit and E-Transit 2024 | KK21-14D003-AM | LK41-14D007-AH, NK41-14D007-BC |

**Known, and kept on the fallback:** Maverick 2025 (SZ1C-14D003-AE), Super Duty 2023 (PC3C-14D003-BG)
and 2025 (PC3C-14D003-BH).

**Two older software levels with their own message.** A 2024 F-150 on its first software level
(RL34-14D003-AD) stays on the fallback. If its calibration is in the family Ford's service message
covers, the support row says: "Ford has a service message for wheel oscillation with lane centering on
some 2024 F-150s (SSM 53105). Ask a Ford dealer to update the power steering module". A 2021–23 F-150
on level ML3V-14D003-BA is told "This steering software is older than Ford's current level".

**2021–23 F-150s on level BC.** Most logged 2021–23 F-150s run level BC (42 of 45 in the logs we
studied). No BC software file was available to read, so its row comes from how 33 BC trucks behaved
on public drives: they answer a curvature command the same way, and 0.15–0.3 s sooner than the BD
level. It steers by curvature only, with comma's stock settings and BC's own measured timing. **A BC
truck gets this row only when its module answers the data-file question at start.** The row accepts
any data file, but it still needs one answered.

**Not checked yet:** whether steering modules answer the data-file question at all during a normal
start. It has not been tried on a real truck. If they don't, every car runs on the fallback, whatever
the table says, and the support row says "The steering module didn't answer".

### Your first drive

- **Start the car and look at Steering Support** after the start. It tells you the level and the part
  numbers your module answered.
- **A notice after the drive.** If the car is on the fallback, or the module's software changed since
  the last drive, the comma shows a notice while parked. A fallback notice is shown on the first three
  starts. The support row keeps saying it for as long as it applies. If the software isn't in the table
  yet, the notice asks you to upload a route from that drive.
- **Engaging while already in a curve.** The build has to start every engagement from a straight-ahead
  command and build up. That is comma's safety rule and the build follows it. On every car in this
  build the steering builds into the curve at the module's own rate: **about 5 to 6 seconds at 30 km/h
  (about 20 mph) and about 2 seconds at 60 km/h (about 37 mph)**. On cars whose path angle has been
  confirmed (none yet) it will take one to three seconds. **Hold the wheel through the first seconds of
  an engagement in a curve.**
- **On a 2021–23 F-150 on level BC:** it steers by curvature only, with timing measured on BC trucks.
- **Changing "Disable BP Lateral Control"** takes effect at the next start, not while driving.

## 5. The release plan

Jacob decided on two steps, documented, with one release after his first drive:

1. **The base:** everything in section 3, with path angle switched off on every row. Every car steers
   by curvature.
2. **Path angle, one car family at a time:** a family's row is switched on after a drive on one of its
   modules shows the module turns as much as the row says, within 30 %, at the speeds where path angle
   helps (below about 70 km/h). A low-speed turn released at the exit also has to show no lingering
   steer.

Both steps are tested before anything ships. Jacob's own Explorer makes the first confirming drive.
After it, the rows confirmed by then are switched on, and the build goes out as one release. Rows not
confirmed by then ship with path angle off.

## 6. The health check and its messages

The health check compares what the car does with what it was asked to do. It never changes the
steering. If it finds a problem, the support row shows ", degraded" and one of these reasons:

| Message | What it means | What to do |
|---|---|---|
| "The car isn't following the steering command as expected" | Over three one-minute stretches of hands-off driving above about 32 km/h (20 mph) in a row, the car turned more than 15 % more or less than it was asked. Every Ford measured so far follows within about 5 %. | Report it with a route. A worn or misaligned car, a biased yaw sensor, or a wrong table row can cause it. |
| "The steering module is rejecting BluePilot's steering messages" | The module reported it was rejecting the frames it received. | Report it with a route. |
| "The panda blocked messages while BluePilot was steering" | The comma's safety chip refused messages while it allowed steering. By design that should not happen. | Report it with a route. |

The one-minute stretches carry over between drives of the same car with the same steering software,
because one drive seldom holds several.

## 7. Safety

- **You are always responsible for driving.** Keep your hands ready and your eyes on the road. This is
  driver assistance, as with any openpilot.
- **Your car's steering module stays in charge of its own limits.** The build steers only through
  Ford's lane-centering message, the same one Ford's own system uses. The module still decides how much
  torque to give and still gives way when you steer.
- **The comma's safety chip (the panda) checks every steering message on its own**, separately from the
  software that makes them. It refuses any message over comma's limits on sideways acceleration and on
  how fast it changes. It refuses steering while openpilot isn't engaged, makes every engagement start
  from straight, and refuses path angle on any car whose row doesn't have it switched on (all of them,
  in this build).
- **One difference from comma's own Ford check:** comma's check also refuses a curvature far from what
  the car is measured doing. This build drops that check on BluePilot's steering, because the path angle
  needs the curvature to differ from the measurement on purpose. The limits on acceleration and its rate
  of change still apply. Jacob chose this. "Disable BP Lateral Control" brings back comma's stock
  command and comma's stock check.
- **What ends steering:** what ends it in openpilot (pressing the brake, cancelling, openpilot
  disengaging). The panda then refuses steering at once. The steering module can also stop on its own:
  when it is unavailable, ramping out, or reports lane centering isn't available on this vehicle
  ("Denied"). The build then sends a "not steering" message and doesn't ask again until the module
  says it is available. You can always steer over it. **Not checked:** what the driving screen shows
  while the module isn't available.

## 8. Reporting and sending a drive log

What to report: the Steering Support text (level, reason, part numbers), the route, roughly when in
the drive it happened, and the speed and type of road (tight town curve, highway, engaging in a curve).

How to send a drive log: the build already logs what it needs. It records the part numbers at start,
and on every steering frame the request, the command, what the car did and the health check. To share
the full logs, open the drive in comma connect and upload all of its files (the full logs, not only the
preview), then send us the route name. If BluePilot's **Connect Backend** setting is on Offline Mode,
uploads never succeed. **Not checked:** the upload steps on BluePilot's other backend choices.

## 9. Questions

**Will my car feel different from stock BluePilot?** Probably yes. It follows openpilot's request more
closely, especially if your yaw sensor reads a little off. The old tuning settings are gone, so values
you had tuned no longer apply.

**Why does my car show Fallback when it is on the list?** Your module answered other part numbers (a
newer or older level, or another data file), or didn't answer. The support text shows what it answered.

**Is the fallback worse?** It is how stock openpilot steers a Ford, without the clip to the measured
curvature. It is the best-tested command there is.

**Why is path angle off if it's better?** Because its numbers come from reading the software and from a
model, not from a drive. If a row's number were wrong, the car would turn more or less than planned in
town curves until curvature caught up. A drive per family settles it first.

**Can I bring back the old settings or the angle mode?** Not in this build.

**My car turns in slowly when I engage in a curve.** That is expected (section 4). Hold the wheel
through it.

**I updated my truck at the dealer and now it says Fallback.** Ford changed the module's software. The
notice shows the old and new part numbers. Upload a route so the new level can be added.

**The Steering Support row says "Not read yet".** Start the car once with this build.

## 10. Troubleshooting

| What you see | Likely cause | What to try |
|---|---|---|
| "The steering module didn't answer" every start | The module doesn't answer the start-up read | Report it with a route. The car steers on the fallback. |
| "isn't in BluePilot's table yet" | Software without a row | Upload a route from the drive. |
| "is in BluePilot's table, with another data file or calibration" | Known program, unknown data file | Upload a route. |
| ", degraded" | The health check (section 6) | Report it with a route. |
| openpilot's "turn exceeds steering limit" warning in sharp curves | The request was over the module's own limit | Slow down for the curve. This is the module's ceiling. |
| The build doesn't start after install | See [INSTALL.md](INSTALL.md) | Go back to your previous install (INSTALL.md). |
