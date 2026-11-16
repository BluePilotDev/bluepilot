# Experimental DCam features

BluePilot → Visuals → **Enable Experimental DCam Features** controls the restored
driver-camera side-window overlays. The adjacent **DCam Activation Options** menu
is disabled until this master switch is enabled.

The submenu contains independent toggles:

- **When Turn Signal Active** — Show DCam view when turn signal active
- **When BSM Active** — Show DCam view when vehicle Blind-Spot Monitoring System active

Parameters: `BPCroppedDcam` (master, default off), `BPCroppedDcamTurnSignal`
(default on), and `BPCroppedDcamBSM` (default on). Each active side is the OR of
its enabled trigger sources. BSM uses the vehicle's native `leftBlindspot` and
`rightBlindspot` fields. There is no V-ASM/vision detection daemon.

The original final design is restored from commit `f2103dcd9c`: mirrored,
calibration-aware side-window crops, smoothly feathered wedge masks, low-light
shadow enhancement, fade-in/out and a one-second post-trigger hold. C4 uses its
compact wedge profile and preserves the 60-pixel side panel. Both cameras can
appear together, leaving a central corridor. The overlay is drawn above scene
geometry and below the HUD, blindspot warnings, and alerts.

The independent driver-stream client is allocated only on the first selected
trigger while enabled. Settings are cached for one second. Camera buffers are
released after the view finishes fading, on disable, or when the host view hides.
The shader is reused on later activations. Missing/stale camera frames are not
drawn; recovery starts a new fade. Stale vehicle state suspends the view. A
shader initialization failure disables the optional view until the master is
toggled off, rather than interrupting the forward view.

The forward normal/wide stream, its Automatic Camera View selection, driver
monitoring, and vehicle controls are unchanged. Cropped side-window visibility
depends on the device installation and camera field of view.

Validation: 31 targeted tests, native Params/schema build, both native road-view
constructors, and both Raylib crop shaders. C3x and C4 menus and crop layouts were
rendered and visually inspected using a labeled camera test pattern. The supplied
route had quick logs but no uploaded camera recordings, so no claim is made of
real-footage replay validation or on-device boot/performance validation.
