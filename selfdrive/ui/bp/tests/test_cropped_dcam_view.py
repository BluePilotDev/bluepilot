from types import SimpleNamespace

from openpilot.selfdrive.ui.bp.onroad.cropped_dcam_geometry import (
  POST_TRIGGER_HOLD_SECONDS,
  compact_panel_region,
  PostTriggerHold,
  Region,
  active_dcam_sides,
  adaptive_window_center_y,
  ease_visibility_alpha,
  low_light_enhancement,
  source_crop,
  wedge_canvas_region,
  wedge_edge_x,
  wedge_insets,
  wedge_local_insets,
)


def test_post_trigger_hold_delays_fade_for_one_second() -> None:
  hold = PostTriggerHold()

  assert hold.update(True, 10.0)
  assert hold.update(False, 10.1)
  assert hold.update(False, 10.1 + POST_TRIGGER_HOLD_SECONDS - 0.001)
  assert not hold.update(False, 10.1 + POST_TRIGGER_HOLD_SECONDS)


def test_post_trigger_hold_restarts_after_reactivation() -> None:
  hold = PostTriggerHold()

  assert hold.update(True, 20.0)
  assert hold.update(False, 20.1)
  assert hold.update(True, 20.8)
  assert hold.update(False, 21.0)
  assert hold.update(False, 21.9)
  assert not hold.update(False, 22.0)


def test_active_dcam_sides_are_independent() -> None:
  state = SimpleNamespace(leftBlinker=True, rightBlinker=False, leftBlindspot=False, rightBlindspot=True)
  assert active_dcam_sides(state) == (True, True)

  state = SimpleNamespace(leftBlinker=False, rightBlinker=False, leftBlindspot=True, rightBlindspot=False)
  assert active_dcam_sides(state) == (True, False)


def test_single_camera_follows_the_reference_side_wedge() -> None:
  content = Region(30, 30, 2100, 1020)
  top, bottom = wedge_insets(content, 0.0)

  assert top == 0.31
  assert bottom == 0.80
  assert wedge_edge_x(content, 0.0, 0.0) == top
  assert wedge_edge_x(content, 0.5, 0.0) == (top + bottom) / 2
  assert wedge_edge_x(content, 1.0, 0.0) == bottom


def test_wedge_canvas_maps_the_complete_crop_across_its_top_edge() -> None:
  content = Region(30, 30, 2100, 1020)
  left = wedge_canvas_region(content, "left", 0.0)
  right = wedge_canvas_region(content, "right", 0.0)
  local_top, local_bottom = wedge_local_insets(content, 0.0)

  assert left.x == content.x
  assert right.x + right.width == content.x + content.width
  assert left.width == right.width == content.width * 0.69
  assert left.height == right.height == content.height
  assert local_top == 0.0
  assert 0.70 < local_bottom < 0.72


def test_comma_four_boxes_keep_center_hud_and_side_panel_clear() -> None:
  content = Region(0, 0, 476, 240)
  left = compact_panel_region(content, "left")
  right = compact_panel_region(content, "right")
  assert left.width == right.width
  assert left.height == right.height
  assert left.y == right.y
  assert left.width / left.height == 1.6
  assert left.x > content.x
  assert left.x + left.width < right.x
  assert right.x + right.width < 476  # C4 side-control strip starts here.
  assert left.y >= 60  # Upper driver-monitoring icon remains clear.
  assert left.y + left.height < 175  # Lower steering wheel and torque UI remain clear.


def test_comma_four_boxes_follow_content_origin_and_scale() -> None:
  content = Region(20, 10, 476, 240)
  left = compact_panel_region(content, "left")
  original = compact_panel_region(Region(0, 0, 476, 240), "left")
  assert left.x == original.x + 20
  assert left.y == original.y + 10
  assert left.width == original.width


def test_companion_fade_continuously_contracts_each_wedge() -> None:
  content = Region(30, 30, 2100, 1020)
  single = wedge_insets(content, 0.0)
  transitioning = wedge_insets(content, 0.5)
  dual = wedge_insets(content, 1.0)

  assert single[0] < transitioning[0] < dual[0]
  assert single[1] < transitioning[1] < dual[1]


def test_visibility_easing_is_clamped_and_smooth() -> None:
  assert ease_visibility_alpha(-1.0) == 0.0
  assert ease_visibility_alpha(0.0) == 0.0
  assert ease_visibility_alpha(0.5) == 0.5
  assert ease_visibility_alpha(1.0) == 1.0
  assert ease_visibility_alpha(2.0) == 1.0


def test_physical_sides_select_opposite_raw_edges_for_mirroring() -> None:
  left = source_crop(1928, 1208, 540, 390, "left", (0.0, 0.0, 0.0))
  right = source_crop(1928, 1208, 540, 390, "right", (0.0, 0.0, 0.0))

  assert left.x > 1928 / 2
  assert right.x < 1928 / 2
  assert left.width == right.width
  assert left.height == right.height


def test_calibration_compensates_mount_pitch_yaw_and_roll() -> None:
  base_left = source_crop(1928, 1208, 540, 390, "left", (0.0, 0.0, 0.0))
  adjusted_left = source_crop(1928, 1208, 540, 390, "left", (0.05, 0.05, -0.05))
  base_right = source_crop(1928, 1208, 540, 390, "right", (0.0, 0.0, 0.0))
  rolled_right = source_crop(1928, 1208, 540, 390, "right", (0.05, 0.0, 0.0))

  assert adjusted_left.x < base_left.x  # negative yaw moves the vehicle-relative raw crop left
  assert adjusted_left.y != base_left.y
  assert rolled_right.y < base_right.y  # roll moves opposite sides in opposite directions


def test_raw_face_landmark_is_not_pitch_corrected_twice() -> None:
  fallback = source_crop(1928, 1208, 1449, 1020, "left", (0.0, 0.07, 0.0))
  neutral_fallback = source_crop(1928, 1208, 1449, 1020, "left", (0.0, 0.0, 0.0))
  detected = source_crop(1928, 1208, 1449, 1020, "left", (0.0, 0.07, 0.0), 0.50)
  neutral_detected = source_crop(1928, 1208, 1449, 1020, "left", (0.0, 0.0, 0.0), 0.50)

  assert fallback.y < neutral_fallback.y
  assert detected.y == neutral_detected.y


def test_route_installation_prioritizes_the_outer_window() -> None:
  # Route 0000000e--ddbe55853b: stable face and liveCalibration samples from segment 24.
  window_center_y = adaptive_window_center_y((0.198, 0.047), 0.96)
  content = Region(30, 30, 2100, 1020)
  destination = wedge_canvas_region(content, "left", 0.0)
  left = source_crop(1928, 1208, destination.width, destination.height,
                     "left", (0.0001, 0.0691, 0.0038), window_center_y)

  assert window_center_y is not None
  assert 0.49 < window_center_y < 0.51
  assert abs(left.width - 1928 * 0.34) < 0.01
  assert abs(left.x + left.width - 1928) < 0.01
  assert 360 < left.y < 380
  assert 820 < left.y + left.height < 840


def test_face_landmark_adapts_and_clamps_window_height() -> None:
  assert adaptive_window_center_y(None, 0.0) is None
  assert adaptive_window_center_y((0.0, -10.0), 1.0) == 0.45
  assert adaptive_window_center_y((0.0, 10.0), 1.0) == 0.63


def test_low_light_enhancement_is_adaptive_and_safe() -> None:
  assert low_light_enhancement(-1.0) == 0.0
  assert low_light_enhancement(float("nan")) == 0.0
  assert low_light_enhancement(100.0) == 0.0
  assert low_light_enhancement(70.0) == 0.0
  assert 0.0 < low_light_enhancement(45.0) < 1.0
  assert low_light_enhancement(20.0) == 1.0
  assert low_light_enhancement(0.0) == 1.0


def test_dcam_triggers_can_be_selected_independently():
  state = SimpleNamespace(leftBlinker=True, rightBlinker=False, leftBlindspot=False, rightBlindspot=True)
  assert active_dcam_sides(state, turn_signal=False, bsm=False) == (False, False)
  assert active_dcam_sides(state, turn_signal=True, bsm=False) == (True, False)
  assert active_dcam_sides(state, turn_signal=False, bsm=True) == (False, True)
  assert active_dcam_sides(state, turn_signal=True, bsm=True) == (True, True)
