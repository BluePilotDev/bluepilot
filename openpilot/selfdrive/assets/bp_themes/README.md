# BluePilot seasonal theme packs

Asset-only theme packs loaded by `selfdrive/ui/bp/lib/theme_pack.py` and selected with
the `BPThemePack` param (Settings → Visuals → Theme, on both the comma 3X and comma
four UIs). The same selector also carries the built-in 8-Bit Racer code theme, so one
param controls all theming.

```
<pack_name>/
  colors/colors.json        # RGBA: Path, PathEdge, LaneLines, LeadMarker, RoadEdges, Accent, Background
  scene.json                # optional: animated + static scene layers (see below)
  sounds/<name>.wav         # optional: overrides stock alert sounds by filename (mono 16-bit 48kHz)
  steering_wheel/wheel.png  # optional: steering wheel icon override
```

`Path`/`PathEdge` color the path ribbon, `LaneLines` the lane lines, `RoadEdges` the
road boundary lines, `LeadMarker` the vision-lead chevron and its info box, `Accent`
the torque-bar fill and set-speed value, and `Background` paints a sky gradient behind
the road when Minimal Driving View hides the camera. Every key is optional; missing
keys keep the stock color.

The bundled packs cover the seasonal holidays: New Year's, Valentine's, St. Patrick's,
April Fools, Easter, Cinco de Mayo, Fourth of July, Halloween, Thanksgiving, and
Christmas. Palettes are original; steering-wheel icons are from
[OpenMoji](https://openmoji.org) (CC BY-SA 4.0).

User packs can be dropped into `/data/bp_themes/` on the device — same layout, no code
or reinstall needed; same-name user packs shadow bundled ones. Malformed or missing
pieces degrade gracefully (bad wavs fall back to stock sounds, missing entries leave
stock visuals in place).

## scene.json — animated and static scene layers

A pack may ship a `scene.json` describing what the theme draws around the road: falling
snow, rising hearts, firework bursts, string lights, garlands. One generic engine
(`selfdrive/ui/bp/lib/theme_scene.py`) renders every pack; sprites are procedurally
generated (no images needed). Scenes render at full strength in Minimal Driving View
and subtly over the live camera; alerts always render on top. A pack without
`scene.json` is colors-only, exactly as before — and a user pack in `/data/bp_themes`
shadowing a bundled name without its scene.json disables that pack's animation.

```jsonc
{
  "version": 1,
  "sky": {"camera_tint_alpha": 40},          // 0-120: Background-color tint over live camera
  "layers": [
    { "type": "particles",
      "sprite": "snowflake",                 // snowflake | heart | leaf | petal | clover |
                                             // bat | firefly | spark | bulb | pennant |
                                             // pumpkin | garland
      "mode": "fall",                        // fall | rise | wander | flutter
      "count": 44,                           // minimal-view count (aggregate cap 60)
      "camera_count_scale": 0.4,             // over-camera count multiplier (cap 24)
      "size_px": [10, 24],                   // at 1080p; scaled by screen height
      "fall_speed": [0.05, 0.11],            // screen fractions per second
      "sway_amp": [0.004, 0.018], "sway_freq_hz": [0.15, 0.5],
      "spin_dps": [-30, 30],
      "tints": ["#FFFFFF", [221, 232, 255], "colors:Accent"],
      "minimal_alpha": 235, "camera_alpha": 110,
      "speed_scale": 0.5,                    // extra fall/drift with vehicle speed
      "depth_split": 0.6,                    // fraction drawn BEHIND the road (smaller/dimmer)
      "blend": "alpha" },                    // or "additive" (glows, sparks)
    { "type": "burst",                       // fireworks: periodic radial explosions
      "period_s": [1.5, 3.0], "particles_per_burst": 24,
      "radial_speed": [0.10, 0.22], "gravity": 0.12, "fade_s": [1.0, 1.6],
      "region": [0.1, 0.05, 0.8, 0.35], "tints": ["#FFD700", "#FFFFFF"] },
    { "type": "decor",                       // static decoration, optional gentle idle
      "sprite": "bulb", "anchor": "top_edge",// top_edge | top_left | top_right |
                                             // bottom_left | bottom_right
      "size_frac": 0.030, "spacing_frac": 0.045, "droop_frac": 0.030,
      "tints": ["#FF4040", "#FFD24A", "#39C25C", "#3E7BFF"],
      "idle": "twinkle",                     // none | twinkle | pulse | sway
      "alpha_minimal": 220, "alpha_camera": 90 }
  ],
  "foreground": {"particles_over_hud": true, "corner_accents": true, "corner_alpha": 55}
}
```

All values are validated and clamped; invalid layers are dropped and a malformed file
falls back to colors-only, so experimentation on-device is safe.
