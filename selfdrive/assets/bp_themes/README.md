# BluePilot seasonal theme packs

Asset-only theme packs loaded by `selfdrive/ui/bp/lib/theme_pack.py` and selected with
the `BPThemePack` param (Settings → Visuals → Theme, on both the comma 3X and comma
four UIs). The same selector also carries the built-in 8-Bit Racer code theme, so one
param controls all theming.

```
<pack_name>/
  colors/colors.json        # RGBA: Path, PathEdge, LaneLines, LeadMarker, RoadEdges, Accent, Background
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
