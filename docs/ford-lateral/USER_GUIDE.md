# How BluePilot steers your Ford

BluePilot is a version of openpilot, the driver-assistance software that runs on a comma device, with
extra work for Fords. This guide is for Ford owners who use it. It explains how the steering works, why
it was built this way, what the messages on screen mean, and what to do about them. You don't need to
read code to follow it.

- Installing, updating and going back to your old version: [INSTALL.md](INSTALL.md).
- The same build in technical depth: [DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md).

**This is a test build, for owners willing to test.** It has not been released. This version has path
angle switched on for seven groups of cars (section 3). Its own steering has not been driven yet. One thing could change everything below: whether your steering module answers the
build's questions at start has not been tried on a real car. The modules' own software says they
should. If one doesn't, the build steers that car the fallback way (section 4).

## 1. How openpilot steers a Ford

openpilot doesn't turn your wheel itself. Your Ford has a computer in its power steering, the
**steering module** (Ford calls it the PSCM, the power steering control module). Ford's own lane
centering works by sending that module a message, and openpilot sends the same kind of message. The
module then does the steering.

The message describes the path the car should follow. It has four parts:

| Part | What it says |
|---|---|
| **Curvature** | How sharply to turn. A gentle highway bend is a small curvature, a tight town corner a large one. |
| **Path angle** | Which way the path points compared with where the car points now. |
| **Path offset** | How far the path is to the left or right of the car. |
| **Curvature rate** | How fast the curvature is changing. |

openpilot plans the path and works out the curvature to drive. Stock openpilot sends only the curvature
and leaves the other three parts at zero. The module decides how much force to put on the wheel to
follow it. The module also keeps its own safety rules: how much force it will use, and how quickly it
lets go when you turn the wheel.

## 2. Why different Fords steer differently

Owners have long noticed that the same openpilot feels different on different Fords. The research
behind this build found the main reason. **Each steering module runs its own program with its own set
of numbers**, and these differ between models, model years and even software updates.

Part numbers stored in the module say which program and which numbers it has:

| Part number | What it is | Example (2021-23 F-150) |
|---|---|---|
| **Strategy** | The program. The last letters are its software version (BD here). | ML3V-14D003-BD |
| **Data file** | The vehicle numbers the program uses | ML34-14D007-EDL |
| **Calibration** | Further tuning values. The build reads it but doesn't use it to pick a row yet. | |

Ford can change both, over the air or at a dealer visit. The numbers decide things you can feel:

- **How quickly the module lets the curvature change.** The module keeps its own copy of the
  curvature it was asked for. On the F-150s studied, it moves that copy no faster than a fixed rate. At town speeds that rate is slow. When a curve tightens
  quickly, the truck takes several seconds to catch up, however fast openpilot asks.
- **How far ahead the module looks** when it follows the path angle. This differs between models, so
  the same path angle turns one truck more than another.
- **How hard the module will turn** before it stops adding steering.
- **How quickly it reacts.** On public drives, 33 F-150s on software version BC reacted 0.15 to 0.3
  seconds sooner than the two trucks measured on the newer version BD.

Earlier BluePilot versions tried to make up for these differences with many tuning settings and a
second steering mode. Those were tuned on one truck and felt different on the next.

## 3. Why curvature first, with path angle on top

In 2023 comma, the company behind openpilot, moved Ford steering to curvature. comma's change gave its
reason as "better control". The BluePilot developers' account is that it was the more universal way to
talk to the steering module. They have driven more than 50,000 miles steering by path angle alone, and
in their experience steering by angle, without the module's slow limit on curvature, follows the road
better. One of those drives has been measured so far: on it, steps below 60 km/h reached most of their
turn in 0.14 seconds, against 0.28 seconds on trucks steered by curvature (10 steps against 50).

The catch is that how much a path angle turns the car depends on how far ahead each module looks, and
that differs between models. A setting tuned for one truck turned another truck too much or too little.

So this build does both:

- **Curvature carries every turn, on every Ford.** That is comma's approach, and it is the one tested
  on the most cars.
- **Path angle helps only where curvature is too slow,** and only on modules whose numbers the build
  knows. It fills the gap while curvature catches up, then goes back to zero once the car is steady in
  the curve. In a computer model of the F-150's module, this made town-speed changes 2 to 3.5 times
  faster than curvature alone.

**Path angle is switched on in this version for:**

- F-150 2021-23 on software version BD;
- F-150 2021-23 on version BC, with BD's numbers (below);
- F-150 Lightning;
- Explorer 2020-24;
- Ranger 2024-25;
- Maverick 2022-24;
- Escape 2020-22 and Bronco Sport 2021-22 on LX6C-14D003-AL.

These are the rows whose module's software was read and holds what path angle needs: how far ahead the
module looks, and how fast it lets curvature and path angle change. Every other car, and the fallback,
steers by curvature alone.

**Why it is on.** The BluePilot developers' 50,000+ miles of steering by path angle alone show that it
steers Fords well. Each row's numbers are read from that module's software files by a script, not typed
in. For the 2021-23 F-150 on software BD the module's own steering code was also run in an emulator, and
the computer model matches it; the Lightning runs the same code, though its own copy hasn't been run yet. For the Ranger and the
2024 Mustang the main part of it was run, and for the 2020-24 Explorer its lane-centering code, from start-up (the data file
on some of these trucks, MB5C-14D007-BA, read since from a 2021 Explorer, holds the same numbers). For the 2022-24
Maverick, the 2020-22 Escape and Bronco Sport, the Mach-E 2021-24 and the 2025 cars on one shared program (Bronco
Sport, Escape, Mach-E) the lane-centering code was run too. For the other rows the code has been read, not run,
and BC's software has not been read at all.
On two rows they have also been measured on the road with earlier BluePilot versions. On
the 2020-24 Explorer the truck turned 0.95 to 0.97 times what the model predicts. On the 2021-23 F-150
with software BD it turned 0.93 to 1.01 times what the row predicts, which passes the release check
(section 6). What hasn't been driven yet is this build's own steering with path angle on.

If a number were wrong, the car would turn a little more or less than planned in town curves until the
curvature caught up. The drive checks in section 6 look for that.

**Tight slow turns.** On the fallback and on every car without path angle, the car steers by curvature
alone. Then it cannot make a turn tighter than about a 50 m radius, and it takes about 7 seconds to build a 50 m curve
at 30 km/h (20 mph). That is the limit of the curvature signal and of the module's own pace, the same as
with comma's stock code. Steer those turns yourself.

## 4. What this build changes

### One way of steering

The build has **one steering controller**. It sends openpilot's planned curvature to the module, and
nothing else changes it. These are gone:

- the second "angle" steering mode;
- the small pulses earlier versions sent when you let go of the wheel;
- the resets when you turned the wheel yourself;
- blending in a predicted curvature;
- the lane offset (driving off-centre in the lane);
- the lane-change adjustments;
- 18 of the 19 steering settings.

**Why:** most of those settings worked around problems with three causes. One was a shortcut in earlier
BluePilot's version of the safety checks on the comma. One was holding the command close to what the
car measured (next part). The third was the angle mode, which each model's module answers differently.
This build removes the first two causes and replaces the angle mode with the path angle in section 3,
so the workarounds have nothing left to fix. The lane offset will come back later as part of openpilot's plan, not as a
steering setting.

### The command follows openpilot, not the yaw sensor

Your car has a **yaw sensor** that measures how fast the car is turning. Stock openpilot and earlier
BluePilot keep the curvature they send close to what that sensor measures. If the sensor reads a little
off, that holds openpilot's request back. On recorded Explorer drives with such a sensor, the old
command was held well away from openpilot's request. In replays of six of those drives, this build's
command matched the request on average to within the smallest step the message can carry on four of
them. On the other two, the limits on how hard the car turns held back 2 to 2.5 % of the commands. On an
Escape with a good sensor, the old and new command are the same 98 % of the time.

### Your module's part numbers are read at every start

At every start the build asks the steering module for its part numbers (strategy, data file and
calibration) and a few other details that are only logged. It only reads. It never writes anything to
the module.

It reads fresh every time because Ford can update the module between drives, and numbers meant for one
software version must not be used on another.

### A table of known steering software

The build has a table with one row for each known combination of strategy and data file. A row holds
that module's own numbers, such as how hard it will turn and how quickly it reacts. A row is used only
when your module answers **exactly** the part numbers in it. A newer or older version, or another data
file, does not count.

On a row without path angle, the row can change up to three things compared with the fallback, where
the row has the number:

- the build never asks for more turning than your module allows;
- openpilot starts from the reaction time measured on logged drives of your module's software;
- on most rows, the build asks for the same ramp-out (how the module eases off its steering) that
  earlier BluePilot used. Not every module uses it.

On the rows with path angle, the row also sends path angle (section 3).

**Why exact:** the research compared Ford's data files, and files that look alike can hold different
steering numbers. Guessing from the model name would use the wrong numbers.

### The fallback

If your module doesn't answer, or its part numbers aren't in the table, the build uses the
**fallback**. It is stock openpilot's Ford steering, as it is: comma's own car controller, steering by
curvature alone, including the smoothing comma's code adds on the 2021-23 F-150 and the Bronco Sport, and
the panda's stock check for your car. So it holds the command close to what the yaw sensor measures (as
earlier BluePilot did, above), and on CAN FD cars it uses comma's limit on how hard the car turns, 3.6 m/s²
rather than BluePilot's 2.4. It is the approach tested on the most Fords.

### A health check

While you drive, the build checks whether the car turns the way it was asked to. The check only watches
and reports. It never changes the steering (section 7).

### Limits on how hard the car turns

The build uses comma's limits on how hard the car may be pushed sideways in a curve, and how quickly
that push may change, with one exception. On newer Fords with a faster internal network (called CAN
FD), such as the F-150, Mach-E and Ranger, BluePilot keeps a lower limit of its own: about 2.4 against
comma's 3.6 (in m/s², sideways acceleration). An old comment in comma's code gave the reason as their
steering having more force; nothing measured shows that. Switching on **Disable BP Lateral Control**
gives a CAN FD car comma's 3.6 again, and so does the fallback (stock openpilot's code steers it). Where your module allows less than the limit, the build asks for
no more than your module allows.

openpilot is told the same limit. The comma's safety chip, the **panda**, checks it too (section 8). So when a curve needs more than that, you see openpilot's usual
"turn exceeds steering limit" warning instead of the car quietly turning less.

## 5. What you'll see on your car

### Steering Support

"Lateral" means side-to-side steering, and "BP" means BluePilot. On a comma 3X, open **Settings →
BluePilot → Lateral Tuning → Steering Support**. On a comma four it is on the BluePilot lateral page as
**steering support**. The **Disable BP Lateral Control** switch is in the same place. Steering Support
shows one of these:

| Shown | What it means |
|---|---|
| **Full** | Your module's exact software is in the table, and its row was compared with a computer model of the module or a drive. For most modules that model is a reading of their code, not checked against the code running. Steers by curvature with the row's numbers. Seven of these rows also send path angle (section 3). |
| **Curvature** | Your module's software is in the table, but its row hasn't been compared with a model or a drive. Steers by curvature with the row's numbers. |
| **Lane centering denied by the steering module** | On the last drive your module refused lane centering. Either lane centering isn't set up on this truck, and the build can't steer a truck built without it, or the module stopped for a fault for that drive, which the next start clears. |
| **Fallback** | Your module didn't answer, or its software isn't in the table. Steers by curvature with comma's settings. |
| **Not supported** | The software is known and this build can't steer it, so the car only records, as a dashcam. The 2024 Transit's KK21-14D003-AM is marked this way: its module has no lane centering at all. |
| **Off** | You switched on **Disable BP Lateral Control**. Stock openpilot steering runs. |
| **Not read yet** | The car hasn't been started with this build yet. |

Full and Curvature rows without path angle steer the same way. Only Full rows can have path angle.

How sure each row's numbers are is a separate question. Every row's numbers are read from its
module's own files. For the 2021-23 F-150 on BD (and, through the same code, the Lightning and most of
the Ranger's) and for the 2020-24 Explorer the module's steering code was also run and matches. For the 2024 Mustang
its controller chain, and for the 2024-26 F-150 its steering library and limiter, were run too, and so was
the code of the 2022-24 Maverick, the 2020-22 Escape and Bronco Sport, the Mach-E 2021-24, the 2025 Bronco
Sport, Escape and Mach-E, the Super Duty and the Transit. For the 2025 Explorer and Mustang and the 2026
Explorer the code has been read but not run, and for the F-150 on BC no file has been read at all.

The text under it (on a comma four, tap **steering support**) gives the reason, the part numbers your
module answered, and how openpilot recognised your car. If it says **", degraded"** after the support level, the health check found a problem on
the latest drive (section 7).

### Which software the build knows

A row applies only when your module answers exactly these part numbers. Steering Support shows yours.

**Full:**

| Vehicle | Strategy | Data file |
|---|---|---|
| F-150 2021-23 | ML3V-14D003-BD | ML34-14D007-EDL or -BDL |
| F-150 2021-23, version BC | ML3V-14D003-BC | any (see below) |
| F-150 Lightning | RL38-14D003-AA | RL38-14D007-DD or -AD |
| Explorer 2020-24 | M1MC-14D003-AC | MB5C-14D007-BD |
| Explorer 2020-24 | L1MC-14D003-AL | MB5C-14D007-BA |
| Explorer 2023 | P1MC-14D003-AA | PB5C-14D007-NB |
| Ranger 2024-25 | RB3C-14D003-AA | RB3C-14D007-ECC |
| Maverick 2022-24 | NZ6C-14D003-AL | NZ6C-14D007-ABD |
| Escape 2020-22 and Bronco Sport 2021-22 | LX6C-14D003-AL | LX6C-14D007-CAH |

**Curvature:**

| Vehicle | Strategy | Data file |
|---|---|---|
| Mustang Mach-E 2021-22 | LJ9C-14D003-GA | LJ9C-14D007-ANA, -AMA or -ARA |
| Mustang Mach-E 2021-22 | LJ9C-14D003-HA | LJ9C-14D007-KNB |
| Mustang Mach-E 2023-24 | LJ9C-14D003-FA | LJ9C-14D007-VDA or -VBA |

**Cars this build doesn't steer:** F-150 2024-26, Expedition and Navigator 2025, Explorer 2025, Mustang
2024-25, Mustang Mach-E 2025, Maverick and Bronco Sport 2025, Super Duty 2023 on, and Nautilus 2025 on.
These are not supported by this build. openpilot runs them as a dashcam only, and so does this build. The
same goes for any car openpilot doesn't support.

The table also has rows for the 2026 Maverick (on the 2025 Bronco Sport's row), the 2026 Mach-E
(SJ9C-14D003-BA with -CAA, on the 2025 Mach-E's row) and the 2026 Explorer (R1MC-14D003-AH, curvature).
openpilot doesn't steer any of them yet.

**Known software, but openpilot can't recognise the car.** The table has rows for the 2025 Escape
(PZ11-14D003-FC) and the Lincoln Corsair (LX6C-14D003-AL). openpilot in this build has no Corsair, and its
Escape is the 2020-22 and 2023-24 car, not the 2025 one. So neither can be steered by this build yet. The
Transit and E-Transit 2024 (KK21-14D003-AM) is in the table as **Not supported**: its module takes no lane
centering request at all, so no build can steer it that way. **Not checked:** whether a 2025 Escape is
recognised anyway.

**2021-23 F-150s on version BC.** Most 2021-23 F-150s run software version BC (42 of the 45 in
the drive logs studied). No copy of the BC software was available to decode, so unlike the other rows,
its row comes from how 33 BC trucks behaved on public drives. They follow a curvature command just like
version BD, and react 0.15 to 0.3 seconds sooner. The row steers with BD's path-angle numbers, because
BC's own were never read, with comma's ramp-out and BC's own measured reaction time, and without the
smoothing comma's code adds on these trucks. **Not checked:** whether BC's numbers really are BD's. A
copy of a BC data file, or a BC drive with path angle, would show it. It accepts any data file, but **your module still has to answer the data-file question at
start**.

**An older F-150 version.** A 2021-23 F-150 on ML3V-14D003-BA stays on the fallback, and Steering
Support says "This steering software is older than Ford's current level".

**Not checked yet:** whether steering modules answer the data-file question during a normal start. The
software of every module with a row allows it, but it has not been tried on a real truck, and BC's
software has never been read. If a module doesn't answer, that car runs on the fallback whatever the
table says, and Steering Support says "The steering module didn't answer".

### Your first drives

- **After you start the car, look at Steering Support.** It shows the support level and the part numbers your
  module answered.
- **A notice when you park.** If the car is on the fallback, or the module's software changed since the
  last drive, the comma shows a notice when you're parked. The fallback notice appears the first three
  times only. Steering Support keeps showing the fallback for as long as it applies. If your software
  isn't in the table, the notice asks you to upload a route from that drive (section 9).
- **Engaging while already in a curve is slow.** Every engagement has to start from a "straight ahead"
  command and build up. That is comma's safety rule. So at first the car steers less than the curve
  needs, and the steering builds into the curve at the module's own pace. On the F-150, the Explorer and
the other modules with the same pace, that is **about 5 to 6 seconds at 30 km/h (20 mph), and about 2
seconds at 60 km/h (37 mph)**. The 2022-24 Maverick's module moves over three times faster. On the
  rows with path angle it is faster still. A computer model of the 2021-23 F-150's module puts it at
  **about 2.7 seconds at 30 km/h and about 1 second at 60 km/h**. The other rows have not been modelled
  for this. **Hold the wheel through
  the first seconds when you engage in a curve.**
- **Disable BP Lateral Control** takes effect the next time you start the car, not while driving.

## 6. The plan for releasing it

This version has path angle on for the rows in section 3. It is ready to release once testing is done.

What testing checks, for each group of cars, on a drive:

- the module turns as much as its row predicts, within 30 %, at the speeds where path angle helps
  (below about 70 km/h, 45 mph);
- after a slow turn, when the driver lets go at the exit, no steering is left over.

A drive with an earlier BluePilot on a 2021-23 F-150 with software BD already measured that row within
that range. A row that fails goes back to curvature only. That is one entry in the table's generator, with the
test lists that name the rows.

There are two other versions, and both stay as they are. The base has path angle off on every car. A
test version has it on for the 2020-24 Explorer only, for a first drive with this build.

## 7. The health check and its messages

The health check compares what the car does with what it was asked to do. It never changes the
steering. If it finds a problem, Steering Support shows ", degraded" and one of these reasons:

| Message | What it means | What to do |
|---|---|---|
| "The car isn't following the steering command as expected" | The check measures one-minute stretches of driving above about 32 km/h (20 mph) where openpilot steered and you didn't turn the wheel. In three such stretches in a row, the car turned more than 15 % more or less than it was asked. Most Fords measured stay within about 10 %, and a single stretch can read further off. Run over 84 logged Fords, each with its own measured delay, the check flagged none. | Report it with a route (section 9). A worn or misaligned car, a yaw sensor that reads off, or a wrong table row can cause it. |
| "The steering module reports its lane-centering inputs as invalid (our steering messages or a vehicle sensor)" | The module reported that an input it needs for lane centering failed its own check: BluePilot's steering messages, or a signal from another part of the car. Only CAN FD cars (such as the F-150, Lightning, Mach-E and Ranger) report this, so on an Explorer, a 2022-24 Maverick or a 2020-22 Escape it can't appear. | Report it with a route. |
| "The panda blocked messages while BluePilot was steering" | The panda (section 8) refused messages while it was allowing steering. That should never happen. | Report it with a route. |

A stretch only counts if it has enough steering in it, so a long straight road takes longer to fill
one. Stretches carry over between drives of the same car with the same steering software.

", degraded" shows what the check found on the latest drive. Each message clears in its own way:

- **"The car isn't following…"** stays, across drives, until a stretch reads within range again.
- **"The panda blocked messages…"** stays until the next start.
- **The module's message** shows only while the module reports the problem. It can come and go during a
  drive, and be gone by the time you park.

The check never changes the steering, so the car steers the same way while it shows.

## 8. Safety

- **You are always responsible for driving.** Keep your hands ready and your eyes on the road. This is
  driver assistance, as with any openpilot.
- **Your steering module stays in charge of its own limits.** The build only sends Ford's lane-centering
  message, the same one Ford's own system uses. The module still decides how much force to use and
  still gives way when you turn the wheel.
- **The panda checks every message.** The comma has a small separate safety computer, the **panda**,
  that checks each steering message before it reaches the car. It refuses any message over
  comma's sideways limits. It refuses steering while openpilot isn't engaged, makes every engagement
  start from straight ahead, and refuses path angle on any car whose row doesn't have it switched on.
  That is every car except the rows in section 3.
- **The panda counts how the module follows a message.** The module doesn't jump to each new
  message. It moves its own copy of each value at a set pace, so for a moment it can hold an old turn
  together with a new path angle. The panda keeps track of where those copies can be, and refuses a
  message that would let the module steer past the sideways limit that way. On these rows it also uses
  each module's own path-angle numbers, read from its software, not one figure for every Ford.
- **One difference from comma's own Ford check.** comma's check also refuses a curvature far from what
  the yaw sensor measures. This build drops that part, on every car. It ties the command to a sensor
  that may read off, which this build moves away from (section 4), and path angle needs the
  curvature to differ from the measurement on purpose. The sideways limits still apply. Switching on **Disable BP
  Lateral Control** brings back comma's own command and comma's own check.
- **What ends steering.** The usual openpilot ways: pressing the brake, cancelling, or openpilot
  disengaging. The panda then refuses steering at once. The steering module can also stop on its own,
  for example when it reports that lane centering isn't available right now. On CAN FD cars the build
  then sends "not steering" and doesn't ask again until the module says it is ready. Modules on other
  cars report no such state, so there the build keeps asking. You can always steer over it. **Not checked:** what the driving screen shows while the module isn't ready.

## 9. Reporting a problem

Tell us:

- what Steering Support says (level, reason, part numbers);
- the route name, and roughly when in the drive it happened;
- the speed and the kind of road (tight town curve, highway, engaging in a curve).

A **route** is the comma's recording of one drive. The build already logs what we need in it,
including the part numbers at start and every steering command. To send the full logs, open the drive
in comma connect (comma's app and website), upload all of its files (the full logs, not only the
smaller files sent by default), and send us the route name. **Not checked:** where to send reports. If BluePilot's **Connect Backend** setting is on
**Offline Mode**, uploads never succeed. **Not checked:** the upload steps with BluePilot's other
backend choices.

## 10. Questions and fixes

| What you see | Why | What to do |
|---|---|---|
| "Not read yet" | The car hasn't been started with this build. | Start the car once. |
| "The steering module didn't answer", every start | The module doesn't answer the start-up read. | The car steers on the fallback. Report it with a route. |
| "This steering software isn't in BluePilot's table yet" | Your module runs software with no row. | Upload a route from the drive. |
| "This steering software is in BluePilot's table, with another data file or calibration" | The program is known, but not with your data file. | Upload a route. |
| "This steering software is older than Ford's current level" | Your module runs a version Ford has replaced. | It steers on the fallback. **Not checked:** whether a dealer update moves it to a version with a row. |
| Fallback after a dealer visit | Ford updated the module's software. The notice shows the old and new part numbers. | Upload a route so the new version can be added. |
| ", degraded" | The health check (section 7). | Report it with a route. |
| "Turn exceeds steering limit" in sharp curves | The curve needs more than your module allows. | Slow down for the curve. |
| Slow turn-in when you engage in a curve | Expected (section 5). | Hold the wheel through it. |
| Steering stops on its own | The module stopped steering (section 8). | Steer yourself. Report it with a route if it keeps happening. |
| The build doesn't start after installing | See [INSTALL.md](INSTALL.md). | Go back to your old version (INSTALL.md). |

**Will my car feel different from earlier BluePilot?** Probably. It follows openpilot's request more
closely, especially if your yaw sensor reads a little off. The old tuning settings are gone, so values
you tuned no longer apply.

**My car is in the table, so why does it show Fallback?** Your module answered other part numbers (a
newer or older version, or another data file), or didn't answer. Steering Support shows what it answered.

**Is the fallback worse?** On a row without path angle, the row differs from the fallback only in the
things listed in section 4, under the table. The fallback is stock openpilot's Ford steering code and its own
panda check, as they are: it holds the command to the yaw sensor, and on CAN FD cars it uses comma's limit.

**Can I bring back the old settings or the angle mode?** Not in this build.
