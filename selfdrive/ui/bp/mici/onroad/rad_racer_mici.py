"""
BluePilot: Rad Racer 8-bit theme, comma four (MICI) variant.

Same scene as the TICI theme (sky, stars, skyline, roadside signs, car sprites) rescaled
for the 536x240 MICI display. The gauge cluster is deliberately not rendered on MICI --
at 240 px tall it would cover the entire screen, and the MICI HUD/complication already
shows speed and lead info. The host (MiciAugmentedRoadViewBP) therefore only calls
render_background() / render_foreground(), never render_cluster().
"""
from openpilot.selfdrive.ui.bp.onroad.rad_racer_theme import RadRacerTheme


class RadRacerThemeMici(RadRacerTheme):
  # ~0.25x of the TICI 2160x1080 values (see RadRacerTheme class attributes)
  SKYLINE_PX_SCALE = 1.2       # 56 px skyline tex -> ~67 px band (~28% of 240 px height)
  SKYLINE_MAX_OFFSET = 100.0
  STAR_COUNT = 14
  EGO_CAR_PX = 2.5             # 24 px ego tex -> 60 px (~11% of 536 px width)
  EGO_CLUSTER_PAD = 2
  LEAD_EGO_GAP = 6
  LEAD_SPRITE_PX = (3.0, 1.5, 1.0)  # 16 px lead tex at dRel 5/40/100 m
