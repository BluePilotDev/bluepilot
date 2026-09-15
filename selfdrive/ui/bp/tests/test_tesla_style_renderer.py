import struct
from importlib.resources import as_file, files
from types import SimpleNamespace

import numpy as np
import pytest
import pyray as rl

from openpilot.selfdrive.ui.bp.onroad.tesla_style_renderer_bp import (
  LEAD_HEIGHT_TO_WIDTH,
  LEAD_MAX_WIDTH_PX,
  LEAD_SHADOW_CENTER_Y_FRACTION,
  LEAD_SHADOW_RADIUS_Y_FRACTION,
  LEAD_SPRITE_ASSET,
  LeadFadeState,
  ROAD_CHAIN_POINT_COUNT,
  ROAD_DROPOUT_FADE_SECONDS,
  ROAD_DROPOUT_HOLD_SECONDS,
  RoadGeometryState,
  TeslaStyleRendererBP,
  color_with_opacity,
  extend_road_edges_to_bottom,
  lead_actor_base_y,
  lead_actor_height,
  lead_actor_width,
  normalized_road_polygon,
  project_car_space_unclipped,
  road_geometry_reliable,
  valid_primary_lead_values,
)
from openpilot.selfdrive.ui.bp.onroad.model_renderer_bp import lead_values_indicate_handoff


def test_road_edges_extend_past_viewport_bottom_without_crossing() -> None:
  rect = rl.Rectangle(100, 50, 1000, 800)
  left = [(400.0, 500.0), (500.0, 400.0), (540.0, 300.0)]
  right = [(700.0, 500.0), (600.0, 400.0), (560.0, 300.0)]

  extended_left, extended_right = extend_road_edges_to_bottom(left, right, rect)

  assert len(extended_left) == len(extended_right) == 4
  assert extended_left[0][1] == extended_right[0][1] == pytest.approx(851.0)
  assert rect.x <= extended_left[0][0] < extended_right[0][0] <= rect.x + rect.width
  assert extended_left[0][0] <= left[0][0]
  assert extended_right[0][0] >= right[0][0]


def test_road_extension_uses_offset_fallback_for_degenerate_projection() -> None:
  rect = rl.Rectangle(200, 100, 600, 300)
  left = [(400.0, 250.0), (390.0, 250.0), (410.0, 200.0)]
  right = [(600.0, 250.0), (610.0, 250.0), (590.0, 200.0)]

  extended_left, extended_right = extend_road_edges_to_bottom(left, right, rect)

  assert extended_left[0] == pytest.approx((230.0, 401.0))
  assert extended_right[0] == pytest.approx((770.0, 401.0))


def test_road_extension_is_not_added_when_closure_is_already_offscreen() -> None:
  rect = rl.Rectangle(0, 0, 1000, 500)
  left = [(100.0, 510.0), (300.0, 400.0)]
  right = [(900.0, 510.0), (700.0, 400.0)]

  assert extend_road_edges_to_bottom(left, right, rect) == (left, right)


def test_normalized_road_polygon_has_fixed_paired_chains() -> None:
  rect = rl.Rectangle(0, 0, 1000, 500)
  left = [(250.0, 450.0), (380.0, 350.0), (450.0, 250.0)]
  right = [(750.0, 450.0), (620.0, 350.0), (550.0, 250.0)]

  road = normalized_road_polygon(left, right, rect)

  assert road is not None
  assert road.shape == (ROAD_CHAIN_POINT_COUNT * 2, 2)
  road_left = road[:ROAD_CHAIN_POINT_COUNT]
  road_right = road[ROAD_CHAIN_POINT_COUNT:][::-1]
  assert np.all(road_left[:, 0] < road_right[:, 0])
  assert road_left[0, 1] == road_right[0, 1] == pytest.approx(501.0)


def test_unreliable_startup_has_no_synthetic_road_polygon() -> None:
  state = RoadGeometryState()

  points, opacity = state.update(None, False, now=0.0)

  assert points is None
  assert opacity == 0.0


def test_background_road_requires_live_finite_xyz_model_geometry() -> None:
  path = np.asarray([[0.0, 0.0, 0.0], [45.0, 1.0, 0.0]])
  class SubMaster:
    def __init__(self):
      self.valid = {"carState": True, "modelV2": True}
      self.alive = {"carState": True, "modelV2": True}

    def __getitem__(self, key):
      return SimpleNamespace(vEgo=4.0)

  submaster = SubMaster()
  assert road_geometry_reliable(path, submaster)
  submaster.valid["modelV2"] = False
  assert not road_geometry_reliable(path, submaster)
  submaster.valid["modelV2"] = True
  assert not road_geometry_reliable(np.asarray([[0.0, 0.0, 0.0], [45.0, np.nan, 0.0]]), submaster)


def test_road_dropout_holds_real_geometry_then_fades_to_ground() -> None:
  state = RoadGeometryState()
  initial = np.zeros((ROAD_CHAIN_POINT_COUNT * 2, 2), dtype=np.float32)

  points, opacity = state.update(initial, True, now=0.0)
  assert np.array_equal(points, initial)
  assert opacity == 1.0
  state.update(None, False, now=0.0)

  held, held_opacity = state.update(None, False, now=ROAD_DROPOUT_HOLD_SECONDS)
  assert np.array_equal(held, initial)
  assert held_opacity == 1.0

  fading, fading_opacity = state.update(
    None, False, now=ROAD_DROPOUT_HOLD_SECONDS + ROAD_DROPOUT_FADE_SECONDS / 2.0,
  )
  assert np.array_equal(fading, initial)
  assert fading_opacity == pytest.approx(0.5)

  gone, gone_opacity = state.update(
    None, False, now=ROAD_DROPOUT_HOLD_SECONDS + ROAD_DROPOUT_FADE_SECONDS,
  )
  assert gone is None
  assert gone_opacity == 0.0


def test_road_recovery_tracks_without_morphing_through_fake_geometry() -> None:
  state = RoadGeometryState()
  initial = np.zeros((ROAD_CHAIN_POINT_COUNT * 2, 2), dtype=np.float32)
  target = np.full_like(initial, 100.0)
  state.update(initial, True, now=0.0)
  state.update(None, False, now=0.1)

  recovered, opacity = state.update(target, True, now=0.2)

  assert recovered is not None
  assert np.all(recovered > initial)
  assert np.all(recovered < target)
  assert opacity == 1.0


def test_standstill_road_freezes_last_real_geometry() -> None:
  state = RoadGeometryState()
  initial = np.zeros((ROAD_CHAIN_POINT_COUNT * 2, 2), dtype=np.float32)
  jitter = np.full_like(initial, 500.0)
  state.update(initial, True, now=0.0)

  frozen, opacity = state.update(jitter, False, freeze=True, now=3.0)

  assert np.array_equal(frozen, initial)
  assert opacity == 1.0


@pytest.mark.parametrize(("lead", "expected"), [
  (SimpleNamespace(status=True, dRel=35.0, yRel=0.2, vRel=-1.0), (35.0, 0.2, -1.0)),
  (SimpleNamespace(status=False, dRel=35.0, yRel=0.2, vRel=-1.0), None),
  (SimpleNamespace(status=True, dRel=0.0, yRel=0.2, vRel=-1.0), None),
  (SimpleNamespace(status=True, dRel=141.0, yRel=0.2, vRel=-1.0), None),
  (SimpleNamespace(status=True, dRel=float("nan"), yRel=0.2, vRel=-1.0), None),
])
def test_primary_lead_validation(lead: SimpleNamespace,
                                 expected: tuple[float, float, float] | None) -> None:
  assert valid_primary_lead_values(lead) == expected


def test_route67_lane_change_is_detected_as_lead_handoff() -> None:
  # Segment 11 changes from radar track 29 to a materially different
  # vision-fused target without dropping leadOne.status.
  previous = (51.10, -1.50, 0.0)
  replacement = (61.56, -0.91, -3.09)

  assert lead_values_indicate_handoff(previous, replacement, 29, -1)


def test_lead_handoff_ignores_small_source_change_but_rejects_impossible_depth_jump() -> None:
  assert not lead_values_indicate_handoff((52.4, 0.10, 0.0), (54.34, 0.14, -0.72), 27, -1)
  assert not lead_values_indicate_handoff((52.4, 0.10, 0.0), (53.0, 0.14, -0.2), -1, -1)
  assert lead_values_indicate_handoff((85.31, -0.30, -1.0), (109.05, -0.20, -1.2), -1, -1)
  assert lead_values_indicate_handoff((30.0, 0.0, 0.0), (30.2, 0.1, 0.0), 7, 8)


def test_lead_fades_old_vehicle_out_before_replacement_fades_in() -> None:
  fade = LeadFadeState()
  old_lead = (30.0, 0.0, -1.0)
  new_lead = (48.0, 0.2, 0.0)

  assert fade.update(old_lead, 1, 0.5) == (old_lead, 0.5)
  assert fade.update(old_lead, 1, 0.5) == (old_lead, 1.0)
  assert fade.update(new_lead, 2, 0.5) == (old_lead, 0.5)
  assert fade.update(new_lead, 2, 0.5) == (old_lead, 0.0)
  assert fade.update(new_lead, 2, 0.5) == (new_lead, 0.5)


def test_lead_fades_out_when_detection_disappears_and_back_in_when_restored() -> None:
  fade = LeadFadeState()
  lead = (25.0, 0.0, -0.5)

  fade.update(lead, 3, 1.0)
  assert fade.update(None, None, 0.5) == (lead, 0.5)
  assert fade.update(None, None, 0.5) == (lead, 0.0)
  assert fade.phase == "hidden"
  assert fade.update(lead, 4, 0.5) == (lead, 0.5)


def test_lead_fade_scales_existing_actor_alpha() -> None:
  faded = color_with_opacity(rl.Color(10, 20, 30, 200), 0.25)

  assert (faded.r, faded.g, faded.b, faded.a) == (10, 20, 30, 50)


def test_lead_actor_shrinks_progressively_with_distance() -> None:
  near = lead_actor_width(100.0, 1080.0, 5.0)
  middle = lead_actor_width(100.0, 1080.0, 30.0)
  far = lead_actor_width(100.0, 1080.0, 55.0)

  assert near == pytest.approx(55.0)
  assert near > middle > far
  assert middle == pytest.approx(47.276596, abs=1e-6)
  assert far == pytest.approx(38.5)


def test_lead_actor_obeys_smaller_display_scaled_clamps() -> None:
  assert lead_actor_width(1.0, 1080.0, 100.0) == pytest.approx(30.0)
  assert lead_actor_width(1000.0, 1080.0, 5.0) == pytest.approx(LEAD_MAX_WIDTH_PX)
  assert lead_actor_width(1000.0, 1080.0, 55.0) == pytest.approx(105.0)
  assert lead_actor_width(1.0, 540.0, 100.0) == pytest.approx(15.0)


def test_lead_contact_shadow_stays_tucked_under_tires() -> None:
  shadow_top = LEAD_SHADOW_CENTER_Y_FRACTION - LEAD_SHADOW_RADIUS_Y_FRACTION
  shadow_bottom = LEAD_SHADOW_CENTER_Y_FRACTION + LEAD_SHADOW_RADIUS_Y_FRACTION

  assert -0.05 < shadow_top < -0.02
  assert -0.01 < shadow_bottom <= 0.0


def test_close_lead_base_is_clamped_inside_viewport() -> None:
  rect = rl.Rectangle(100.0, 50.0, 1000.0, 540.0)
  width = 90.0

  assert lead_actor_base_y(400.0, width, rect) == pytest.approx(400.0)
  expected_base = rect.y + rect.height - max(24.0 * 0.5, width * LEAD_HEIGHT_TO_WIDTH * 0.14)
  assert lead_actor_base_y(700.0, width, rect) == pytest.approx(expected_base)


def test_sedan_proportions_keep_far_actor_at_road_horizon() -> None:
  far_width = lead_actor_width(1000.0, 540.0, 20.0)
  far_top = 335.0 - lead_actor_height(far_width)

  assert LEAD_HEIGHT_TO_WIDTH == pytest.approx(456.0 / 512.0)
  assert far_top >= 258.0


def test_road_level_sedan_sprite_is_rgba_and_matches_layout_aspect() -> None:
  sprite = files("openpilot.selfdrive").joinpath("assets", LEAD_SPRITE_ASSET)
  with as_file(sprite) as sprite_path:
    data = sprite_path.read_bytes()[:26]

  assert data[:8] == b"\x89PNG\r\n\x1a\n"
  width, height, bit_depth, color_type = struct.unpack(">IIBB", data[16:26])
  assert (width, height) == (512, 456)
  assert (bit_depth, color_type) == (8, 6)
  assert height / width == pytest.approx(LEAD_HEIGHT_TO_WIDTH)


def test_road_level_sedan_proportions_are_distance_independent() -> None:
  assert lead_actor_height(100.0) == pytest.approx(100.0 * LEAD_HEIGHT_TO_WIDTH)


def test_traffic_projection_retains_points_outside_model_clip_region() -> None:
  transform = np.asarray([
    [2.0, 0.0, 0.0],
    [0.0, 3.0, 0.0],
    [0.0, 0.0, 0.5],
  ])

  assert project_car_space_unclipped(transform, 1000.0, 500.0, 2.0) == pytest.approx((2000.0, 1500.0))
  assert project_car_space_unclipped(np.zeros((3, 3)), 1.0, 2.0, 3.0) is None


def test_compact_device_returns_before_accessing_tici_lead_geometry() -> None:
  renderer = TeslaStyleRendererBP(relative_projection=True, show_lead_vehicle=False)

  # None sentinels ensure this early return cannot accidentally touch UI state,
  # the model renderer's TICI-only smoothing getter, or custom actor drawing.
  assert renderer.render_traffic(None, None) is None
