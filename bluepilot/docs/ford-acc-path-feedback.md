# Ford ACC Path Feedback

BluePilot → Visuals → **Ford ACC Path Feedback** (`BPFordACCPathFeedback`, default off).

Description: Model path changes shading and color for Ford ACC accel / brake signals

The selected path is drawn first, then receives a green/cyan tint for
acceleration or orange/red tint for deceleration/braking. Opacity scales with
request magnitude, reaching 49% for acceleration and 61% for braking at the near
end, with a depth fade toward the horizon. A 250 ms transition
avoids abrupt changes. Neutral cruise mode returns to the selected path colors.
Rainbow animation and all base path gradients are retained. Tesla's existing
geometry visibility also applies to the tint. Rad Racer's dashed road is excluded
because it has no filled model path to shade.

This is read-only telemetry from the existing Ford powertrain CAN parser:

- Camera bus `ACCDATA.AccPrpl_A_Rq`: positive propulsion demand.
- Camera bus `ACCDATA.AccBrkTot_A_Rq`: negative acceleration demand.
- `ACCDATA.AccBrkDecel_B_Rq`: braking request takes priority.

The inactive propulsion sentinel (-5) is ignored. A small deadband keeps
near-zero requests neutral. Feedback magnitude is bounded to [-1, 1].
Vehicle acceleration and model predictions are not substituted for Ford signals.
The feedback requires valid CAN, both messages received within 500 ms, stock
cruise enabled, and no driver pedal/cruise override. It is disabled whenever
openpilot longitudinal control is configured or `carControl.longActive` is true.
Missing or unhealthy UI messages immediately remove the tint.

No CAN subscriptions, control requests, driving-model code, or model catalogs are
changed. The new optional `carStateBP.fordAcc` fields default to unavailable in
old logs. C3X and C4 use the same telemetry and blending logic.

Validation: Ford DBC packing/decoding, capnp publication, stale/invalid/override
gating, smoothing, startup without CarParams, and existing path-color tests.
The native Params extension and generated C++ cereal schemas build successfully.
Full graphical validation was unavailable on the locked Mac. Live vehicle signal
behavior still needs device verification.
