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

- Evidence comes from **steady engaged curves** and from **curve apexes** (the "tops and
  bottoms of the graphs"), so winding roads count even when nothing is steady.
- Everything suspicious is thrown away: potholes and bump-flicks, rough washboard
  surfaces, hard braking/accelerating, tire-limit cornering, any moment your hands are on
  the wheel (plus a cooldown after), and crowned/banked roads that push all the evidence
  to one side.
- Evidence **survives ignition cycles** — progress is saved every 30 seconds and picked
  up on the next drive.

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

**To recalibrate** (new tires, alignment work, seasonal tire swap, or you just want a
fresh pass): toggle it **off and back on**. That clears everything and starts a clean
collection.

## What it will never do

- It never moves a factor more than **0.02 per step**, **0.10 per drive** for the high
  factor and **0.04 per drive** for the low factor — one drive can't transform how your
  car steers.
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
