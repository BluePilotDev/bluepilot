import unittest

import numpy as np

from openpilot.selfdrive.ui.bp.onroad.rendered_world import (
  CAM_HEIGHT, FOG_MAX, NEAR_X, XS, ChaseCamera, RenderedWorld, SceneModel, SceneSmoother, dashed_lines,
  edge_bounds, fog_t, repeating, road_bounds,
)


def straight(offset: float, n: int = 33, length: float = 192.0) -> np.ndarray:
  x = np.linspace(0.0, length, n, dtype=np.float32)
  return np.stack([x, np.full(n, offset, dtype=np.float32)], axis=-1)


def three_lane_model() -> SceneModel:
  """You in the middle lane of three 3.6 m lanes; the road edges 0.6 m past the outer lines."""
  return SceneModel(lane_lines=[straight(-5.4), straight(-1.8), straight(1.8), straight(5.4)],
                    lane_line_probs=np.array([0.9, 0.95, 0.95, 0.9], dtype=np.float32),
                    road_edges=[straight(-6.0), straight(6.0)])


class TestChaseCamera(unittest.TestCase):
  def setUp(self):
    self.cam = ChaseCamera(0.0, 0.0, 2160.0, 1080.0)

  def test_center_of_the_road_is_the_center_of_the_view(self):
    for x in (0.0, 20.0, 150.0):
      self.assertAlmostEqual(float(self.cam.project(x, 0.0)[0]), 1080.0, places=3)

  def test_the_road_rises_toward_the_horizon_with_distance(self):
    ys = self.cam.project(np.array([0.0, 10.0, 50.0, 200.0]), 0.0)[:, 1]
    self.assertTrue(np.all(np.diff(ys) < 0))
    self.assertTrue(np.all(ys > self.cam.horizon_y))

  def test_a_point_at_camera_height_sits_on_the_horizon(self):
    self.assertAlmostEqual(float(self.cam.project(40.0, 0.0, CAM_HEIGHT)[1]), self.cam.horizon_y, places=3)

  def test_right_is_right_on_screen(self):
    self.assertGreater(float(self.cam.project(20.0, 2.0)[0]), float(self.cam.project(20.0, -2.0)[0]))

  def test_the_near_road_reaches_the_bottom_of_the_view(self):
    self.assertGreater(self.cam.y_fraction(NEAR_X), 1.0)


class TestLeadAnchor(unittest.TestCase):
  def test_anchor_is_where_the_lead_meets_the_road(self):
    world, rect = RenderedWorld(), type("R", (), {"x": 0.0, "y": 0.0, "width": 2160.0, "height": 1080.0})()
    cam = world.camera(rect)
    ax, ay = world.lead_anchor(rect, 30.0, 0.5)
    px, py = cam.project(30.0, 0.5, 0.0)
    self.assertAlmostEqual(ax, float(px), places=3)
    self.assertAlmostEqual(ay, float(py), places=3)

  def test_a_closer_lead_hangs_lower(self):
    world, rect = RenderedWorld(), type("R", (), {"x": 0.0, "y": 0.0, "width": 2160.0, "height": 1080.0})()
    self.assertGreater(world.lead_anchor(rect, 15.0, 0.0)[1], world.lead_anchor(rect, 60.0, 0.0)[1])


class TestRoad(unittest.TestCase):
  def test_bounds_follow_the_road_edges(self):
    bounds = road_bounds(three_lane_model(), np.array([0.0, 50.0], dtype=np.float32))
    assert bounds is not None
    left, right = bounds
    np.testing.assert_allclose(left, [-6.0, -6.0])
    np.testing.assert_allclose(right, [6.0, 6.0])

  def test_bounds_widen_to_hold_a_line_past_the_edge(self):
    model = three_lane_model()
    model.road_edges = [straight(-3.0), straight(6.0)]
    bounds = road_bounds(model, np.array([10.0], dtype=np.float32))
    assert bounds is not None
    left, _ = bounds
    self.assertLessEqual(float(left[0]), -5.4 - 0.3 + 1e-5)

  def test_no_road_without_edges(self):
    model = three_lane_model()
    model.road_edges = []
    self.assertIsNone(road_bounds(model, np.array([10.0], dtype=np.float32)))

  def test_lines_between_lanes_are_dashed_and_the_outer_lines_solid(self):
    self.assertEqual(dashed_lines(three_lane_model()), [False, True, True, False])

  def test_a_single_lane_road_has_no_dashes(self):
    model = SceneModel(lane_lines=[straight(-5.4), straight(-1.8), straight(1.8), straight(5.4)],
                       lane_line_probs=np.array([0.1, 0.95, 0.95, 0.1], dtype=np.float32),
                       road_edges=[straight(-2.4), straight(2.4)])
    self.assertEqual(dashed_lines(model)[1:3], [False, False])

  def test_fog_is_clear_near_and_capped_far(self):
    self.assertEqual(float(fog_t(5.0)), 0.0)
    self.assertAlmostEqual(float(fog_t(1000.0)), FOG_MAX, places=5)


def shifted(model: SceneModel, dy: float, right_edge: float | None = None, probs=None) -> SceneModel:
  """The same road moved dy to your right (you moved left), optionally with the right edge at another offset."""
  def move(ln: np.ndarray) -> np.ndarray:
    return ln + np.array([0.0, dy], dtype=np.float32)
  edges = [move(e) for e in model.road_edges]
  if right_edge is not None:
    edges[1] = straight(right_edge + dy)
  return SceneModel(lane_lines=[move(ln) for ln in model.lane_lines],
                    lane_line_probs=model.lane_line_probs if probs is None else np.array(probs, dtype=np.float32),
                    road_edges=edges)


def step(smoother: SceneSmoother, model: SceneModel) -> SceneModel:
  return smoother.smooth(model, 0.05)  # one 20 Hz frame


def edges_of(model: SceneModel) -> tuple[np.ndarray, np.ndarray]:
  bounds = edge_bounds(model, XS)
  assert bounds is not None
  return bounds


def frame_moves(values: list[float]) -> float:
  return max(abs(b - a) for a, b in zip(values[10:], values[11:], strict=False))


class TestSceneSmoother(unittest.TestCase):
  def test_a_width_change_eases_in(self):
    smoother, model = SceneSmoother(), three_lane_model()
    step(smoother, model)
    _, right = edges_of(step(smoother, shifted(model, 0.0, right_edge=8.0)))  # the right edge jumps 2 m out
    self.assertLess(float(right[5]), 6.2)
    for _ in range(120):  # 6 s
      _, right = edges_of(step(smoother, shifted(model, 0.0, right_edge=8.0)))
    self.assertAlmostEqual(float(right[5]), 8.0, places=2)

  def test_an_edge_that_flickers_barely_moves_the_road(self):
    smoother, model = SceneSmoother(), three_lane_model()
    rights = [float(edges_of(step(smoother, shifted(model, 0.0, right_edge=8.0 if i % 2 else 6.0)))[1][5])
              for i in range(40)]
    self.assertLess(frame_moves(rights), 0.15)

  def test_an_outer_line_that_comes_and_goes_barely_moves_the_road(self):
    # the right edge sits inside the outer right line; the road widens to hold that line only while it is seen
    smoother, model = SceneSmoother(), three_lane_model()
    rights = []
    for i in range(40):
      probs = [0.9, 0.95, 0.95, 0.9 if i % 2 else 0.1]
      rights.append(float(edges_of(step(smoother, shifted(model, 0.0, right_edge=3.0, probs=probs)))[1][5]))
    self.assertLess(frame_moves(rights), 0.25)

  def test_a_line_that_wobbles_is_steadied(self):
    smoother, model = SceneSmoother(), three_lane_model()
    ys = []
    for i in range(40):
      wobbly = shifted(model, 0.0)
      wobbly.lane_lines[2] = straight(1.8 + (0.2 if i % 2 else -0.2))
      ys.append(float(step(smoother, wobbly).lane_lines[2][5, 1]))
    self.assertLess(frame_moves(ys), 0.1)

  def test_a_line_fades_out_instead_of_popping(self):
    smoother, model = SceneSmoother(), three_lane_model()
    step(smoother, model)
    out = step(smoother, shifted(model, 0.0, probs=[0.9, 0.95, 0.95, 0.0]))  # the outer right line is lost
    self.assertGreater(float(out.lane_line_probs[3]), 0.75)
    for _ in range(60):  # 3 s
      out = step(smoother, shifted(model, 0.0, probs=[0.9, 0.95, 0.95, 0.0]))
    self.assertLess(float(out.lane_line_probs[3]), 0.05)

  def test_moving_across_the_road_does_not_lag(self):
    smoother, model = SceneSmoother(), three_lane_model()
    step(smoother, model)
    for i in range(1, 41):  # drift 2 m left over 2 s, as in a lane change
      out = step(smoother, shifted(model, 0.05 * i))
    left, right = edges_of(out)
    self.assertAlmostEqual(float(left[5]), -6.0 + 2.0, delta=0.2)
    self.assertAlmostEqual(float(right[5]), 6.0 + 2.0, delta=0.2)
    self.assertAlmostEqual(float(out.lane_lines[1][5, 1]), -1.8 + 2.0, delta=0.2)

  def test_a_lane_handover_starts_over(self):
    smoother, model = SceneSmoother(), three_lane_model()
    step(smoother, model)
    # the model now calls the lane to your left yours: lines move 3.6 m right, and the right edge is new
    _, right = edges_of(step(smoother, shifted(model, 3.6, right_edge=6.0)))
    self.assertAlmostEqual(float(right[5]), 6.0 + 3.6, places=4)

  def test_the_path_is_steadied_with_the_lines(self):
    smoother, model = SceneSmoother(), three_lane_model()
    ys = []
    for i in range(40):
      model.path = np.column_stack([straight(0.15 if i % 2 else -0.15), np.zeros(33, dtype=np.float32)])
      ys.append(float(step(smoother, model).path[5, 1]))
    self.assertLess(frame_moves(ys), 0.1)

  def test_no_edges_no_road(self):
    model = three_lane_model()
    model.road_edges = []
    self.assertIsNone(edge_bounds(SceneSmoother().smooth(model, 0.05), XS))


class TestPath(unittest.TestCase):
  def setUp(self):
    self.cam = ChaseCamera(0.0, 0.0, 2160.0, 1080.0)
    self.model = three_lane_model()
    self.model.path = np.column_stack([straight(0.0), np.zeros(33, dtype=np.float32)])
    self.model.path_max_x = 100.0

  def test_the_path_runs_from_the_bottom_of_the_view_and_keeps_the_model_points(self):
    ribbon = RenderedWorld.path_ribbon(self.cam, self.model, 150.0, None, 0.9)
    assert ribbon is not None
    left = ribbon[:len(ribbon) // 2]
    self.assertAlmostEqual(float(left[0][1]), float(self.cam.project(NEAR_X, -0.9)[1]), places=3)
    self.assertAlmostEqual(float(left[-1][1]), float(self.cam.project(100.0, -0.9)[1]), places=3)
    inside = int(np.sum((self.model.path[:, 0] > NEAR_X) & (self.model.path[:, 0] < 100.0)))
    self.assertEqual(len(left), inside + 2)

  def test_the_path_stops_short_of_the_lead_car(self):
    ribbon = RenderedWorld.path_ribbon(self.cam, self.model, 150.0, 30.0, 0.9)
    assert ribbon is not None
    self.assertAlmostEqual(float(ribbon[len(ribbon) // 2 - 1][1]), float(self.cam.project(29.0, -0.9)[1]), places=3)

  def test_no_path_under_a_lead_at_your_bumper(self):
    self.assertIsNone(RenderedWorld.path_ribbon(self.cam, self.model, 150.0, 2.0, 0.9))


class TestMotion(unittest.TestCase):
  def test_marks_slide_toward_you_as_you_drive(self):
    before = repeating(0.0, 15.0, 3.0, -50.0, 150.0)  # far enough behind that no mark leaves between the two
    after = repeating(2.0, 15.0, 3.0, -50.0, 150.0)
    np.testing.assert_allclose(after[:len(before)] - before[:len(after)], -2.0, atol=1e-4)

  def test_marks_cover_the_view_at_any_phase(self):
    for phase in np.linspace(0.0, 60.0, 37):
      xs = repeating(float(phase), 15.0, 3.0, NEAR_X, 150.0)
      self.assertGreaterEqual(float(xs[0]), NEAR_X)
      self.assertLess(float(xs[0]), NEAR_X + 15.0)
      self.assertTrue(np.allclose(np.diff(xs), 15.0))

  def test_road_motion_off_holds_the_marks_still(self):
    world = RenderedWorld()
    world.advance(30.0, 0.05, motion=False)
    self.assertEqual(world._phase, 0.0)
    world.advance(30.0, 0.05, motion=True)
    self.assertAlmostEqual(world._phase, 1.5)

  def test_reversing_does_not_run_the_road_backward(self):
    world = RenderedWorld()
    world.advance(-3.0, 0.05, motion=True)
    self.assertEqual(world._phase, 0.0)


if __name__ == "__main__":
  unittest.main()
