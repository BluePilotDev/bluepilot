# Long control: why brake_actuate can be true with positive gas/accel

## Your scenario (summary)

- `disable_BP_long_UI` = False (BP long intended on)
- vego ≈ 34 m/s, dRel = 0 (no lead)
- gas = 0.25, accel = 0.2440 (both positive)
- orientationNED[1] = -0.03 (downhill)
- **brake_actuate = true** ← unexpected

## How brake_actuate is set

There are two sources of `brake_actuate` in the Ford car controller:

1. **Stock path:** `op_brake_actuate` (pitch‑compensated stock accel + hysteresis)
2. **BP path:** `bp_brake_actuate` (from `bp_accel` when `apply_bp_long` is True)

You only ever get **positive output accel and brake_actuate true** when the **stock path** is used. When the BP path is used, `brake_actuate = bp_brake_actuate` and `accel = bp_accel`; brake is only set when `bp_accel < brake_actuate_target (-0.14)`, so output accel would be negative in that same frame.

So in your log, the frame where brake_actuate is true is almost certainly using the **stock** path (`apply_bp_long` False), and the brake flag comes from **op_brake_actuate**.

---

## 1. Stock path: op_brake_actuate (pitch + hysteresis)

Relevant code (around 792–801):

```python
# Pitch compensation: PCM interprets accel in vehicle frame; we compensate for brake bits
accel_due_to_pitch = math.sin(CC.orientationNED[1]) * ACCELERATION_DUE_TO_GRAVITY  # NED[1] = pitch, rad
accel_pitch_compensated = op_accel + accel_due_to_pitch

op_brake_actuate = self.op_brake_actuate_last
if accel_pitch_compensated > self.brake_actuate_release:   # -0.06
  op_brake_actuate = False
elif accel_pitch_compensated < self.brake_actuate_target:   # -0.14
  op_brake_actuate = True
# else: keep previous (hysteresis between -0.14 and -0.06)
```

Constants:

- `brake_actuate_target` = -0.14  
- `brake_actuate_release` = -0.06  

So:

- Brake **on** when `accel_pitch_compensated < -0.14`
- Brake **off** when `accel_pitch_compensated > -0.06`
- In between, **keep** `op_brake_actuate_last` (hysteresis)

With your numbers:

- `accel_due_to_pitch = sin(-0.03) * 9.81 ≈ -0.294 m/s²`
- If **op_accel** (the value used in this block) were **0.244**:
  - `accel_pitch_compensated = 0.244 - 0.294 = -0.05` → **> -0.06** → brake would be **released** (False).

So with “accel = 0.244” and pitch -0.03, the **current frame** would not set brake **unless** the accel value used here is **not** 0.244. That leads to the next point: the accel that matters is **op_accel**, which can be **rate‑limited** below actuators.accel.

---

## 2. Rate limit on op_accel (most likely cause)

Stock accel is rate‑limited **only on the way down** (around 779–784):

```python
op_accel = actuators.accel
if CC.longActive:
  op_accel = apply_creep_compensation(op_accel, CS.out.vEgo)
  op_accel = max(op_accel, self.accel - (3.5 * CarControllerParams.ACC_CONTROL_STEP * DT_CTRL))
```

- `ACC_CONTROL_STEP = 2`, `DT_CTRL = 0.01` → max drop per ACC frame = **3.5 * 2 * 0.01 = 0.07 m/s²** (ACC runs at 50 Hz).

So if **last frame** `self.accel` was 0.24, this frame **op_accel** can be no lower than 0.24 - 0.07 = **0.17**. If controlsd requested 0.244, you can still have **op_accel = 0.17** in the controller.

Then:

- `accel_pitch_compensated = 0.17 - 0.294 = -0.124`
- -0.124 is **between -0.14 and -0.06** → hysteresis: **op_brake_actuate is unchanged**.
- If the previous frame had **op_brake_actuate = True** (e.g. from an earlier more negative accel or steeper pitch), it **stays True** this frame even though requested accel is positive.

Output in that case:

- `accel = op_accel = 0.17` (not 0.244)
- `brake_actuate = op_brake_actuate = True`
- Then gas is forced off: `if brake_actuate: gas = INACTIVE_GAS`

So the scenario that fits your log is:

1. **apply_bp_long** is False (so stock path is used; see below why).
2. **op_accel** is **rate‑limited** below actuators.accel (e.g. 0.17 instead of 0.244).
3. **accel_pitch_compensated** lands in the hysteresis band and **op_brake_actuate** stays True from the previous frame.
4. The “accel” and “gas” you’re logging may be **actuators.accel / actuators.gas** (0.244, 0.25) from **carControl**, not the **car controller output** (0.17 and then gas cleared). So you see positive gas/accel in the log while the car is actually sending brake and reduced accel.

---

## 3. When is the stock path used? (apply_bp_long False)

BP long is only used when **all** of these hold (around 937–938):

```python
apply_bp_long = (
  self.disable_BP_long_UI == False
  and self.bpSpeedAllow
  and gasPressed == False
  and brakePressed == False
  and (lead is None or v_lead_mph > 40.0)
)
```

So even with **disable_BP_long_UI = False**, the **stock** path is used if:

- **bpSpeedAllow** is False (e.g. vego &lt; 45 mph, or you’re in the 45–50 mph deadband and it hasn’t been set True yet),
- **gasPressed** or **brakePressed** is True,
- or there **is** a lead with **v_lead_mph ≤ 40**.

If the UI says “BP long on” but you’re in one of these cases, the controller will use stock logic and **op_brake_actuate**; the rate limit + pitch + hysteresis then explain brake staying on with “positive” requested accel.

---

## 4. disable_BP_long_UI: is it read correctly?

- **Where it’s read:** `_update_params()` (around 293):  
  `self.disable_BP_long_UI = self.params.get_bool("disable_BP_long_UI")`
- **When:** `_update_params()` is called from `update()` every control cycle (around 374).
- So the value is refreshed every frame from Params. If the UI writes the param when you toggle, the controller should see it shortly after. A one‑frame delay is possible; a persistent mismatch (UI shows False but code always behaves as True) would point to param name or write path, not “never read”.

---

## 5. Checks you can do in your logs

1. **Which accel/gas are you logging?**
   - **carControl.actuators.accel / .gas** = controlsd request (e.g. 0.244, 0.25).
   - **Car controller output** = what gets sent on CAN (and after `if brake_actuate: gas = INACTIVE_GAS`). If you log `self.accel` / `self.gas` from the controller, you’d see 0.17 and 0 in the brake frame.
2. **Confirm stock path in the brake frame:**  
   Log or infer **apply_bp_long** (or at least: **bpSpeedAllow**, **gasPressed**, **brakePressed**, and lead presence/speed). If apply_bp_long is False in that frame, brake is from **op_brake_actuate**.
3. **Pitch and rate limit:**  
   Log **op_accel** and **accel_pitch_compensated** (or add them for debugging). You should see op_accel pulled down by the 0.07 m/s² rate limit and accel_pitch_compensated in the -0.14 to -0.06 band when brake stays on with “positive” requested accel.
4. **Hysteresis:**  
   Log **op_brake_actuate_last** at the start of the frame; if it’s True and accel_pitch_compensated is in (-0.14, -0.06), brake will stay True.

---

## 6. Summary

- **brake_actuate** can be true with “positive” gas/accel only when the **stock** path is used.
- **op_brake_actuate** is driven by **accel_pitch_compensated** and **hysteresis** (-0.14 / -0.06).
- **Rate limiting** can make **op_accel** lower than actuators.accel (e.g. 0.17 vs 0.244); with your pitch that puts **accel_pitch_compensated** in the hysteresis band so **op_brake_actuate** can **stay** True from the previous frame.
- So the most likely explanation is: **apply_bp_long** False in that frame, **op_accel** rate‑limited, **accel_pitch_compensated** in (-0.14, -0.06), and **op_brake_actuate_last** True → brake stays on while the **requested** (actuators) accel/gas are positive. Verifying in the log which accel/gas you’re looking at and adding the suggested signals will confirm it.
