"""BluePilot: the park sensors overlay (onroad/park_sensors_bp.py)."""
import math
import unittest

import pyray as rl

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.ui.bp.onroad.park_sensors_bp import (HOLD_S, ParkSensorsOverlay, ParkZones, band, layout,
                                                               zone_arcs, zones_from_park_aid)


class FakeParams:
  def __init__(self, on: bool):
    self.on = on

  def get_bool(self, key):
    assert key == "BPParkSensors"
    return self.on


class FakeSM:
  def __init__(self, front=(15,) * 4, rear=(15,) * 4, status=1, available=True):
    self.msg = messaging.new_message("carStateBP")
    aid = self.msg.carStateBP.parkAid
    aid.dataAvailable = available
    aid.frontStatus = aid.rearStatus = status
    aid.frontZones, aid.rearZones = list(front), list(rear)
    self.recv_frame = {"carStateBP": 10}

  def __getitem__(self, key):
    return getattr(self.msg, key)


class TestBands(unittest.TestCase):
  def test_steps_to_bands(self):
    self.assertEqual([band(z) for z in range(16)], [0, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0])

  def test_something_near_only_from_sensors_that_are_on(self):
    self.assertTrue(ParkZones(rear=(15, 15, 9, 15), rear_ok=True).something_near)
    self.assertFalse(ParkZones(rear=(15, 15, 9, 15), rear_ok=False).something_near)
    self.assertFalse(ParkZones((15,) * 4, (15,) * 4, True, True).something_near)
    self.assertTrue(ParkZones((15,) * 4, (15,) * 4, True, True).active)


class TestZonesFromCarState(unittest.TestCase):
  def test_zones_and_status(self):
    zones = zones_from_park_aid(FakeSM((15, 15, 3, 7), (12, 15, 15, 15))["carStateBP"].parkAid)
    self.assertEqual((zones.front, zones.rear, zones.front_ok, zones.rear_ok), ((15, 15, 3, 7), (12, 15, 15, 15), True, True))

  def test_faulted_or_missing_shows_nothing(self):
    self.assertFalse(zones_from_park_aid(FakeSM((3,) * 4, (3,) * 4, status=3)["carStateBP"].parkAid).something_near)
    self.assertEqual(zones_from_park_aid(FakeSM((3,) * 4, (3,) * 4, available=False)["carStateBP"].parkAid), ParkZones())


class TestZonePlacement(unittest.TestCase):
  def test_each_zone_drawn_on_its_side(self):
    # front up: the driver's left zones draw left of the truck's center line, the right ones right of it,
    # the front ones above the truck's middle and the rear ones below it
    lo = layout(rl.Rectangle(0, 0, 2160, 1080))
    middle = lo.truck.y + lo.truck.height / 2
    arcs = zone_arcs(lo, ParkZones((1, 2, 3, 4), (5, 6, 7, 8), True, True))
    self.assertEqual([z for _, _, z, _ in arcs], [1, 2, 3, 4, 5, 6, 7, 8])
    for i, (center, (a0, a1), _, _) in enumerate(arcs):
      mid = math.radians((a0 + a1) / 2)
      x, y = center.x + 120 * math.cos(mid), center.y + 120 * math.sin(mid)
      with self.subTest(zone=i):
        self.assertEqual(x < lo.cx, i % 4 < 2)
        self.assertEqual(y < middle, i < 4)

  def test_inside_the_view(self):
    rect = rl.Rectangle(30, 30, 2100, 1020)
    lo = layout(rect)
    self.assertGreaterEqual(lo.x, rect.x)
    self.assertGreaterEqual(lo.y, rect.y)
    self.assertLessEqual(lo.x + lo.w, rect.x + rect.width)
    self.assertLessEqual(lo.y + lo.h, rect.y + rect.height)


class TestOverlay(unittest.TestCase):
  NEAR = {"front": (15, 15, 3, 15)}

  def _run(self, overlay, sm, t0, seconds, dt=1 / 20):
    t = t0
    while t < t0 + seconds - 1e-9:
      t += dt
      overlay.update(sm, 0, now=t)
    return t

  def test_off_by_default_setting_never_shows(self):
    overlay = ParkSensorsOverlay(FakeParams(False))
    self._run(overlay, FakeSM(**self.NEAR), 0.0, 2.0)
    self.assertFalse(overlay.visible)

  def test_shows_while_near_and_holds_then_fades(self):
    overlay = ParkSensorsOverlay(FakeParams(True))
    t = self._run(overlay, FakeSM(), 0.0, 1.0)
    self.assertFalse(overlay.visible)  # the sensors on, nothing near
    t = self._run(overlay, FakeSM(**self.NEAR), t, 0.5)
    self.assertEqual(overlay._alpha, 1.0)
    t = self._run(overlay, FakeSM(), t, HOLD_S - 0.2)
    self.assertEqual(overlay._alpha, 1.0)  # held: a zone at a band's edge flickers between near and clear
    self._run(overlay, FakeSM(), t, 1.0)
    self.assertFalse(overlay.visible)

  def test_nothing_before_this_drive_s_first_car_state(self):
    overlay = ParkSensorsOverlay(FakeParams(True))
    sm = FakeSM(**self.NEAR)
    t = 0.0
    for _ in range(20):
      t += 0.05
      overlay.update(sm, started_frame=10, now=t)  # recv_frame 10 is not after the start
    self.assertFalse(overlay.visible)


class TestSetting(unittest.TestCase):
  def test_off_by_default(self):
    # a fresh install: the setting reads off, and the overlay stays hidden with something near
    with OpenpilotPrefix():
      params = Params()
      self.assertIs(params.get_default_value("BPParkSensors"), False)
      self.assertFalse(params.get_bool("BPParkSensors"))
      overlay = ParkSensorsOverlay(params)
      sm, t = FakeSM(front=(15, 15, 2, 15)), 0.0
      for _ in range(20):
        t += 0.05
        overlay.update(sm, 0, now=t)
      self.assertFalse(overlay.visible)


if __name__ == "__main__":
  unittest.main()
