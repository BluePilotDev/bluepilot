# Standard QCOM model catalog v26

The standard model manager now reads `driving_models_v26.json`, including
Cheddar Model (November 05, 2026), and accepts selector versions 19 and 21.
An older cached v22 list is refreshed on the next catalog request; an offline
device can still use its compatible cached list.

## Runtime isolation

Stock models, build-time compilation, and selector-19 models retain the original
`tinygrad_repo` revision. Selector-21 models use the separate submodule
`sunnypilot/modeld_v2/tinygrad_v26_repo`, pinned to the catalog's tinygrad revision
`5d186bdb43c2bcc6c56b6ececf427c34bada0de9`.

Only the custom modeld process selects this alternate Python import path, before
importing tinygrad. Normal device updates initialize the submodule through the
existing updater's recursive submodule update. Manual checkouts need:

```sh
git submodule update --init --recursive
```

Portable `SPTGPLN1` artifacts use the upstream-style data-only loader, adapted to
the pinned revision's typed-buffer APIs. Native models retain their recurrent
state in device memory and alias next-state outputs to the input state buffers.
Their two-camera NV12 preparation uses the warp implementation from that same
pinned runtime. Existing pickle and out-of-band artifact paths remain available.

The loader translates portable dtype aliases (including Cheddar's `f16` and
`i64`) to the long names exposed by the pinned runtime. Without this translation,
Cheddar exits during loading, so it publishes no camera odometry and calibration
remains at 0%. Calibration acceptance thresholds are unchanged.

If catalog migration changes a model's files or selector version, the existing
model validation can reset that cached selection to the stock default; select
and download the model again from the updated list.

## Validation

Run the legacy model-loader and new portable-runtime tests with:

```sh
python -m pytest -q -n 0 sunnypilot/models/tests/test_v22_catalog.py \
  sunnypilot/models/tests/test_v26_catalog.py \
  sunnypilot/modeld_v2/tests/test_combined_pkl_loader.py
```

The portable-runtime test runs in a fresh subprocess with the alternate runtime,
compiles a CPU fixture into the portable format, checks load/cache round trips,
and exercises NV12 preparation, changing inputs, prepare-only frames, and
recurrent state. The legacy tests continue to use the original tinygrad.

Desktop validation of the downloaded Cheddar artifact verifies its published
SHA-256 and sends its embedded compile-time reference output through the branch's
parser and model/pose message publishers. The C3X/C4 model selection widgets can
render the real v26 catalog. These checks do not execute the artifact's QCOM
kernels on the Mac or establish on-device frame rate; those require comma hardware.
