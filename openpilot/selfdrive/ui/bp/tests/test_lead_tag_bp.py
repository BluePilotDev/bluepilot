import unittest

import pyray as rl

from openpilot.selfdrive.ui.bp.onroad.lead_tag_bp import GAP, MARGIN, TOP_CLEAR, LeadBox, place_tag, tag_lines

VIEW = rl.Rectangle(0, 0, 2160, 1080)
W, H = 300, 120


def car_at(cx: float, cy: float = 600, half_w: float = 60, half_h: float = 40) -> LeadBox:
  return LeadBox(cx - half_w, cy - half_h, cx + half_w, cy + half_h)


class TestPlaceTag(unittest.TestCase):
  def test_a_centered_car_gets_the_tag_on_its_right(self):
    box = car_at(1080)
    x, _, on_right = place_tag(box, W, H, VIEW)
    self.assertTrue(on_right)
    self.assertEqual(x, box.right + GAP)

  def test_a_lead_in_your_lane_just_left_of_center_keeps_it_on_the_right(self):
    _, _, on_right = place_tag(car_at(1060), W, H, VIEW)
    self.assertTrue(on_right)

  def test_a_car_clearly_left_of_center_gets_the_tag_on_its_left(self):
    box = car_at(700)
    x, _, on_right = place_tag(box, W, H, VIEW)
    self.assertFalse(on_right)
    self.assertEqual(x + W, box.left - GAP)

  def test_no_room_on_the_outer_side_moves_it_across(self):
    box = car_at(VIEW.width - 150)
    x, _, on_right = place_tag(box, W, H, VIEW)
    self.assertFalse(on_right)
    self.assertLessEqual(x + W, box.left)
    box = car_at(150)
    x, _, on_right = place_tag(box, W, H, VIEW)
    self.assertTrue(on_right)
    self.assertGreaterEqual(x, box.right)

  def test_it_never_leaves_the_view(self):
    box = LeadBox(MARGIN, 500, VIEW.width - MARGIN, 700)  # a car filling the width: no side has room
    x, _, _ = place_tag(box, W, H, VIEW)
    self.assertGreaterEqual(x, MARGIN)
    self.assertLessEqual(x + W, VIEW.width - MARGIN)

  def test_it_is_centered_on_the_car_and_kept_below_the_speed(self):
    _, y, _ = place_tag(car_at(1080, cy=700), W, H, VIEW)
    self.assertEqual(y, 700 - H / 2)
    _, y, _ = place_tag(car_at(1080, cy=320), W, H, VIEW)  # a far car near the horizon
    self.assertEqual(y, TOP_CLEAR)
    _, y, _ = place_tag(car_at(1080, cy=1070), W, H, VIEW)
    self.assertLessEqual(y + H, VIEW.height - MARGIN)


class TestTagLines(unittest.TestCase):
  def test_the_distance_is_large_and_the_rest_share_a_line(self):
    big, small = tag_lines(["163 ft", "59 mph", "1.9 s"])
    self.assertEqual(big, "163 ft")
    self.assertEqual(small, ["59 mph", "1.9 s"])

  def test_one_value_has_no_small_line(self):
    self.assertEqual(tag_lines(["163 ft"]), ("163 ft", []))
    self.assertEqual(tag_lines([]), ("", []))


if __name__ == "__main__":
  unittest.main()
