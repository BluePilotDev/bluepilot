# Ford Angle-Mode Anti-Weave Smoothing — User Guide

BluePilot can damp the slow left-right "rhythmic centering" weave some Fords show in
angle mode on straight roads — with a strength dial you control, and a guarantee that
the neutral setting is exactly stock steering.

---

## What it does (and why you'd want it)

Some angle-mode Fords develop a gentle, rhythmic side-to-side motion on straights — the
car works the wheel every few seconds even though the lane is dead straight. Log analysis
traced it to a feedback loop: tiny curvature noise crosses internal thresholds, those
crossings modulate the steering command, and the car's power-steering computer integrates
the result into motion you can feel.

Smoothing breaks that loop at its sources: it filters the noise that feeds the thresholds
and holds the steering command steady when changes are smaller than what the car can even
represent on the wire. It does **not** slow the steering down — all shaping is on the
input side, and curve entry is deliberately left fast at every strength.

Measured on the same car, straight-road driving, with and without:

| | Without | With (menu 1.8) |
|---|---|---|
| Slow lane sway (the weave) | ±0.46 m | **±0.26 m (−43%)** |
| Fast wheel-working dither | ±0.033 m | ±0.022 m (−33%) |

Curve entry and exit were checked specifically: entries are unaffected by design, and
exits measured *cleaner* with smoothing on.

## The strength dial

**Settings → BluePilot → Lateral Tuning → Smooth Steering (Anti-Weave)** — a master
toggle plus a **Smoothing Strength** stepper. Also on the comma four lateral menu and in
Sunnylink.

The scale is deliberately simple:

- **1.0 — stock.** Not "a little smoothing": *bit-for-bit identical* to the feature not
  existing. This is verified by an automated test on every change.
- **1.1 – 1.9** — increasing damping.
- **2.0 — the tuned setting.** Chosen on logged drives; this is where the numbers in the
  table above come from (measured at 1.8, tuned default 2.0).
- **2.5 — maximum.** More damping, with a measurable cost (below). For cars that still
  weave at 2.0.

The toggle defaults ON with strength 1.0 — which means **stock behavior until you step
the strength up**. Damping is always your explicit choice.

## What it costs

Nothing is free in a control loop. The closed-loop simulator puts numbers on it:

| Setting | Lane-keeping tightness (std) |
|---|---|
| 1.0 / off | 0.039 m |
| 2.0 | 0.042 m (+3 mm) |
| 2.5 | 0.047 m (+8 mm) |

At the tuned setting you trade **three millimeters** of station-keeping for roughly half
the weave. At 2.5 the cost triples for diminishing extra damping — try 2.0 first.

## How to find your setting

1. Leave strength at 1.0 for a drive or two so you know your baseline.
2. If you feel the slow weave on straights, step to **2.0** and drive the same roads.
3. Still feel it? Step toward 2.5 one notch at a time.
4. If the car ever feels *lazier* than you like, step back down — every value between
   1.0 and your current setting is a valid operating point.

Changes take effect within a second (no reboot), and stepping strength mid-drive is safe:
the filters are built to pick up from the live steering state, never from stale values.

## What it will never do

- **1.0 is stock, provably.** The passthrough is tested for bit-identity, not "close".
- It never adds lag on top of the steering output — that specific design was tested in a
  closed-loop simulator, measured to *hurt* lane-keeping 2.5x, and rejected. Input-side
  shaping only.
- Curve entry speed is independent of strength: the entry filter is fixed-fast, and an
  automated test pins it.
- All of it disengages instantly with lateral control, and every filter resets across
  takeovers — no state survives a disengagement, a driver override, or a steering pause.

## Troubleshooting

| Symptom | Likely reason |
|---|---|
| No difference at 1.0 | Correct — 1.0 *is* stock. Step up to feel the feature. |
| Still weaving at 2.0 | Step toward 2.5. If maxed and still weaving, report it with a route ID — your car may need the underlying factors calibrated first (see the auto-calibration guide). |
| Feels slow into curves | Not a smoothing effect at any strength (entry is fixed-fast) — check your speed-factor calibration instead. |
| Wandering within the lane | Distinguish: the weave is *rhythmic* (a steady few-second cycle); random wander is usually crosswind, crown, or camera calibration. Smoothing targets the rhythm. |

## Relationship to auto-calibration

They're complementary and independent. Auto-calibration fixes the *average* correction
(how much turn you get per command); smoothing fixes the *oscillation* around it. A car
with badly-off factors can weave for that reason alone — calibrate first, then judge how
much smoothing you still want.

## For the curious

The smoothing math lives in one pure, unit-tested module
(`opendbc/sunnypilot/car/ford/angle_smoothing.py`) with five elements: a hysteresis on
the curve-entry decision, a low-pass on the model's predicted curvature, a slew on the
exit blend, an asymmetric filter on the gain schedule (the primary fix — fast attack,
strength-scaled release), and a one-LSB hold on the outgoing wire value. The measurement
tooling — spectral weave analysis of any logged drive, and the closed-loop simulator used
to bound the costs above — lives in [bp-tools](https://github.com/ghbarker/bp-tools)
(`bp/angle_weave_analyze.py`, `sim/closed_loop_weave.py`).
