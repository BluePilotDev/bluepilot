# Ford Angle-Mode Auto-Calibration — User Guide

BluePilot can tune your car's two angle-mode adjustment factors for you, while you
drive, using exactly the comparison you'd do by hand — and stop when it's done.

---

## What it does (and why you'd want it)

On Fords running **angle mode**, BluePilot sends the car a target steering angle and the
car's power-steering computer (the PSCM) turns the wheel. That conversion isn't perfectly
1:1, and it drifts from car to car with tires, alignment, and platform. Two menu values
correct for it:

- **Low Speed Adjustment Factor** (`FordLowSpeedFactor_ang`)
- **High Speed Adjustment Factor** (`FordHighSpeedFactor_ang`)

The manual tuning method is: drive, plot requested vs. actual turn, compare the tops and
bottoms of the two curves, tap +/- until the peaks line up, repeat. It works, but it's
per-car, slow, and easy to get subtly wrong.

**Auto-calibration automates that exact loop.** It watches requested vs. actual curvature
in real time, collects evidence only from clean cornering, and nudges the same two menu
values you would have tapped — in small steps, with statistical error bars instead of an
eyeball. When there's nothing left to adjust, it **locks** and stops touching anything.

## Requirements

- A Ford running BluePilot with **Lateral Control set to Angle** (the toggle is greyed
  out in curvature mode).
- Nothing else. It's **off by default** and changes nothing until you turn it on.

## Turning it on

**comma 3X:** Settings → BluePilot → Lateral Tuning → **Auto-Calibrate Adjustment Factors**

**comma four:** Lateral menu → **Auto-Calibrate Factors**

**Sunnylink:** [Lateral Tuning] Auto-Calibrate Adjustment Factors

Then just drive normally with lateral engaged.

## What you'll see

Open the Lateral Tuning menu during or after a drive: the low/high factor values **move on
their own**, at most 0.02 at a time. That's it working. There's no ceremony — the
calibrator uses the same values the +/- buttons use, so the menu is always the truth.

What it's doing underneath:

- Evidence comes from engaged curves — including **winding roads where the wheel never
  stops moving**: the comparison is made against the command from the car's own measured
  steering delay ago, so a continuously changing curve reads correctly instead of being
  discarded. Curve **apexes** (the "tops and bottoms of the graphs") count separately.
- Everything suspicious is thrown away: potholes and bump-flicks, rough washboard
  surfaces, hard braking/accelerating, tire-limit cornering, any moment your hands are on
  the wheel (plus a cooldown after), and crowned/banked roads that push all the evidence
  to one side.
- **Only calm data counts.** Evidence is taken solely while the steering loop is quietly
  tracking — the moments when the car is swinging wide or catching itself back are the
  loop's dynamics, not the car's gain, and they are refused outright. A step takes as
  many curve passes as calm data requires; a slower right answer beats a faster wrong one.
- **Every adjustment is checked before the next one.** After a step, the calibrator
  collects a fresh batch of clean curves *at the new value* and confirms the step
  actually brought the car **closer to doing exactly what's asked** (the measured
  response strictly nearer 100% of requested than before). Confirmed → it keeps going.
  Contradicted → it stops moving that factor and demands twice the evidence before
  trying again. Poll a couple turns, adjust, poll some more — enforced, not hoped.
- Evidence **survives ignition cycles** — progress is saved every 30 seconds and picked
  up on the next drive.

## Watching it live from your phone

The [phone graph page](lateral-phone-graph.md) (`http://192.168.43.1:8088/lateral` on the
device hotspot) shows a **calibration dashboard** whenever the calibrator is armed: one
card per speed band (low, under 30 mph / high, over 60 mph) with

- how much clean-curve evidence each band has collected (and how much it needs),
- what the car is measured doing right now — e.g. **"turns 93% of requested"**,
- the current factor and the step it wants to try next — **"factor 1.00 → try 1.08"**,
- live *checking…* progress while a fresh step is being verified, and whether the last
  step **confirmed ✓** or didn't.

A pill in the corner shows which band your current speed is feeding ("42 mph · blend
zone"). Between 30 and 60 mph evidence splits between both anchors.

## How long does it take?

Honest answer from real drives: **roughly an hour of mixed driving**, but it depends
heavily on the roads.

- **Highway curves calibrate the high factor quickly** — sweeping interstate curves are
  ideal evidence and pile up fast.
- **The low factor is slower on purpose.** City cornering is exactly where hands, bumps,
  and sharp maneuvers contaminate the data, so most of it is rejected. Gentle 25–45 mph
  curvy roads with hands off are what it wants.
- The first few minutes of every drive contribute nothing — the calibrator waits for the
  car's own sensor-calibration stack to warm up before trusting any measurement.

You don't have to do anything special. It gets there on normal driving; special trips
just get there sooner.

## Your +/- buttons still win

Tap +/- any time, calibrating or not. Your value is adopted immediately and the
calibrator treats it as a strong hint — it softens its accumulated evidence rather than
fighting you. It will only move the value again if fresh driving data genuinely disagrees.

## Locking

When both factors have solid evidence behind them and the applied values have sat within
0.03 of the statistical target for 5 minutes of driving, the calibration **locks**:

- The factors stop changing. Permanently, for this car.
- The toggle stays on but does nothing further.

**The lock is optional.** A **Calibration Lock** toggle sits next to the main switch
(default on). Turn it off and the calibrator never freezes — it keeps adapting for as
long as the main toggle is on. Turning the lock off on an *already locked* car resumes
calibration from its saved evidence, losing nothing; turning it back on re-enables
freezing once things are stable again.

**To recalibrate** (new tires, alignment work, seasonal tire swap, or you just want a
fresh pass): toggle it **off and back on**. That clears the evidence and starts a clean
collection *from the current factor values*.

## Erase Calibration Memory

Next to the toggle sits **Erase Calibration Memory** — the full do-over. One tap:

- wipes all collected evidence and any lock,
- clears the calibrator's error log,
- and puts **both factors back to 1.00** (stock).

Use it when a calibration run went somewhere you don't trust and you want to retry from
a clean slate rather than from wherever the factors ended up. It works offroad or
mid-drive (takes effect within a second while driving), and the phone dashboard shows
"memory erased" when it lands.

## What it will never do

- It never moves a factor more than **0.02 per step**, and never steps the same factor
  again until fresh driving data at the new value has confirmed the previous step.
  There is deliberately **no cap on total movement** — a car that is genuinely far off
  is allowed to walk all the way to its fit — because every step of that walk has to
  keep verifying against the road.
- It never acts on thin data: each factor needs sustained clean evidence and a tight
  error bar before its first nudge.
- It never runs in curvature mode, never runs while locked, and never runs before the
  measurement stack is warmed up.
- Turning the toggle off stops it instantly and clears its state.

## Troubleshooting

| Symptom | Likely reason |
|---|---|
| Factors never move | Normal for the first drives — evidence takes time, and city-heavy driving is mostly rejected by design. Check you're in angle mode and the toggle is on. Highway curves speed things up. |
| Factors moved, then stopped | It probably **locked** — that's success. Toggle off/on if you want a re-run. |
| Low factor barely changes while high converged | Expected — see "How long does it take?". Gentle mid-speed curves with hands off are the low anchor's food. |
| A value looks wrong after calibration | Tap +/- to your preferred value; the calibrator adopts it. If it drifts back, the data disagrees with you — try a re-run after checking tire pressures/alignment. |
| Whole run went somewhere you don't trust | **Erase Calibration Memory** — factors back to 1.00, evidence wiped, clean retry. |
| Steps keep showing "didn't verify" on the phone dashboard | The car's measured response is contradicting the model — usually bad data conditions (crosswind, rough roads, constant light grip). The calibrator is protecting you by refusing to walk further; give it cleaner roads. |
| Suspected fault | The calibrator writes any internal error to the `FordAngleAutoCalError` param (visible in logs) instead of failing silently — include it when reporting. |

## For the curious

The estimator is pure math shared byte-for-byte with an offline analyzer. If you upload
your drives, anyone can replay exactly what the car's calibrator saw — every accepted
sample, every rejection and its reason, and the nudge-by-nudge timeline:

```
python bp/angle_autocal_analyze.py <folder-with-rlogs> <route-id>
```

(from the [bp-tools](https://github.com/ghbarker/bp-tools) repo; writes a self-contained
HTML report.)

That analyzer is also how the feature was tuned and validated: thresholds were chosen on
logged reference drives, and every code change is checked by replaying a known drive and
confirming the calibrator's decisions are unchanged.
