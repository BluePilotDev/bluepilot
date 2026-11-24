import unittest

from . import host
from ..core.carcontroller_ext import InputTracker
from ..core.values_ext import MODEL_T_IDXS


def _cc(valid=True):
  cc = host.CarControlX()
  li = cc.lateralInputs
  li.valid = valid
  li.pathYawRate = [0.01 * i for i in range(33)]
  li.pathX = [float(i) for i in range(33)]
  li.pathY = [0.1 * i for i in range(33)]
  li.laneLineLeftX = li.laneLineRightX = [float(i) for i in range(33)]
  li.laneLineLeftY = [-1.8] * 33
  li.laneLineRightY = [1.8] * 33
  li.laneLineProbs = [0.1, 0.9, 0.8, 0.1]
  li.laneLineStds = [1.0, 0.1, 0.1, 1.0]
  li.laneChangeState = 2
  li.laneChangeDirection = 1
  li.steerRatio = 15.5
  li.stiffnessFactor = 0.9
  li.roll = 0.02
  li.angleOffsetDeg = 0.3
  li.lateralDelay = 0.12
  return cc


class _FakeInputs:
  """Bare object carrying only the attributes InputTracker.update touches."""
  def __init__(self):
    self.model = None
    self.lp = None
    self.lateral_delay = 0.0
    self.VM = self
    self.updated = None

  def update_params(self, x, sr):
    self.updated = (x, sr)


class TestInputTracker(unittest.TestCase):
  def test_model_view_has_modelv2_shape(self):
    ext = _FakeInputs()
    InputTracker.update(ext, _cc().lateralInputs)
    m = ext.model
    self.assertEqual(len(m.orientationRate.z), 33)
    self.assertEqual(m.position.y[10], 1.0)
    self.assertEqual(m.laneLines[1].y[0], -1.8)
    self.assertEqual(m.laneLines[2].y[0], 1.8)
    self.assertEqual(m.laneLineProbs[2], 0.8)
    self.assertEqual(m.laneLineStds[1], 0.1)
    self.assertEqual(m.meta.laneChangeState, 2)
    self.assertEqual(m.meta.laneChangeDirection, 1)

  def test_vehicle_params_and_delay(self):
    ext = _FakeInputs()
    InputTracker.update(ext, _cc().lateralInputs)
    self.assertEqual(ext.lp.angleOffsetDeg, 0.3)
    self.assertEqual(ext.lp.roll, 0.02)
    self.assertEqual(ext.lateral_delay, 0.12)
    self.assertEqual(ext.updated, (0.9, 15.5))

  def test_invalid_inputs_leave_previous_values(self):
    ext = _FakeInputs()
    InputTracker.update(ext, _cc().lateralInputs)
    InputTracker.update(ext, _cc(valid=False).lateralInputs)
    self.assertEqual(ext.lateral_delay, 0.12)
    self.assertIsNotNone(ext.model)

  def test_default_cc_is_not_valid(self):
    self.assertFalse(host.CarControlX().lateralInputs.valid)

  def test_t_idxs_matches_modeld(self):
    # openpilot ModelConstants: index_function(i, max_val=10.0) = 10 * (i/32)^2, 33 points
    self.assertEqual(len(MODEL_T_IDXS), 33)
    self.assertEqual(MODEL_T_IDXS[0], 0.0)
    self.assertAlmostEqual(MODEL_T_IDXS[16], 2.5)
    self.assertEqual(MODEL_T_IDXS[32], 10.0)

  def test_short_arrays_are_ignored_even_if_valid(self):
    # fuzzed / partial host CarControl: valid=True but arrays not the modelV2 shape -> frame ignored
    ext = _FakeInputs()
    InputTracker.update(ext, _cc().lateralInputs)
    bad = _cc()
    bad.lateralInputs.laneLineLeftY = []
    InputTracker.update(ext, bad.lateralInputs)
    self.assertEqual(ext.model.laneLines[1].y[0], -1.8)  # previous inputs kept
    empty = _cc()
    empty.lateralInputs.pathYawRate = [0.0] * 10
    InputTracker.update(_FakeInputs(), empty.lateralInputs)  # must not raise
    fresh = _FakeInputs()
    InputTracker.update(fresh, empty.lateralInputs)
    self.assertIsNone(fresh.model)

  def test_non_finite_inputs_are_ignored(self):
    # a NaN or inf anywhere in the frame -> frame ignored, previous inputs kept
    for field, index in (("laneLineProbs", 1), ("laneLineStds", 2), ("pathYawRate", 5), ("pathY", 0), ("laneLineRightY", 32)):
      for bad_value in (float("nan"), float("inf")):
        ext = _FakeInputs()
        InputTracker.update(ext, _cc().lateralInputs)
        model, lp = ext.model, ext.lp
        bad = _cc()
        values = list(getattr(bad.lateralInputs, field))
        values[index] = bad_value
        setattr(bad.lateralInputs, field, values)
        InputTracker.update(ext, bad.lateralInputs)
        self.assertIs(ext.model, model, field)
        self.assertIs(ext.lp, lp, field)
    for field in ("steerRatio", "lateralDelay"):
      ext = _FakeInputs()
      InputTracker.update(ext, _cc().lateralInputs)
      bad = _cc()
      setattr(bad.lateralInputs, field, float("nan"))
      InputTracker.update(ext, bad.lateralInputs)
      self.assertEqual(ext.lp.steerRatio, 15.5, field)
      self.assertEqual(ext.lateral_delay, 0.12, field)

if __name__ == "__main__":
  unittest.main()
