"""Tests for the theme scene system (GL-free: parsing, simulation, dispatch)."""
import json
import math
import os
import unittest
from unittest import mock

from openpilot.common.basedir import BASEDIR
from openpilot.selfdrive.ui.bp.lib import theme_pack, theme_scene
from openpilot.selfdrive.ui.bp.lib.theme_scene import (
  MAX_BURST_POOL, MAX_PARTICLES_MINIMAL, PackScene, RadRacerScene, SceneSpec,
  _resolve_tint, load_scene_spec,
)

PACKS_DIR = os.path.join(BASEDIR, "openpilot", "selfdrive", "assets", "bp_themes")

GOOD = {
  "version": 1,
  "sky": {"camera_tint_alpha": 40},
  "layers": [
    {"type": "particles", "sprite": "snowflake", "mode": "fall", "count": 20,
     "size_px": [10, 24], "fall_speed": [0.05, 0.11], "tints": ["#FFFFFF", "colors:Accent"]},
    {"type": "burst", "tints": ["#FFD700"]},
    {"type": "decor", "sprite": "bulb", "anchor": "top_edge", "idle": "twinkle"},
  ],
  "foreground": {"particles_over_hud": True, "corner_accents": True, "corner_alpha": 55},
}


def _write(tmpdir, data) -> str:
  p = os.path.join(tmpdir, "scene.json")
  with open(p, "w") as f:
    if isinstance(data, str):
      f.write(data)
    else:
      json.dump(data, f)
  return p


class _FakePack:
  name = "test_pack"
  root = "/nonexistent"
  colors = {"Accent": (10, 20, 30, 255)}

  def rl_colors(self):
    return {}


class TestSceneSpecParsing(unittest.TestCase):
  def setUp(self):
    import tempfile
    self.tmp = tempfile.mkdtemp()

  def test_happy_path(self):
    spec = load_scene_spec(_write(self.tmp, GOOD), {"Accent": (10, 20, 30, 255)})
    self.assertIsNotNone(spec)
    self.assertEqual(len(spec.particles), 1)
    self.assertEqual(len(spec.bursts), 1)
    self.assertEqual(len(spec.decor), 1)
    self.assertEqual(spec.particles[0].count, 20)
    self.assertEqual(spec.particles[0].tints[1], (10, 20, 30))  # colors:Accent resolved
    self.assertTrue(spec.corner_accents)

  def test_missing_file(self):
    self.assertIsNone(load_scene_spec(os.path.join(self.tmp, "nope.json")))

  def test_invalid_json(self):
    self.assertIsNone(load_scene_spec(_write(self.tmp, "{not json")))

  def test_wrong_version(self):
    self.assertIsNone(load_scene_spec(_write(self.tmp, {"version": 2, "layers": []})))

  def test_unknown_sprite_layer_dropped_others_kept(self):
    d = dict(GOOD)
    d["layers"] = [{"type": "particles", "sprite": "unicorn"}] + GOOD["layers"]
    spec = load_scene_spec(_write(self.tmp, d), {})
    self.assertEqual(len(spec.particles), 1)  # the snowflake layer survived

  def test_count_clamped_to_budget(self):
    d = {"version": 1, "layers": [
      {"type": "particles", "sprite": "snowflake", "count": 500},
      {"type": "particles", "sprite": "heart", "count": 500},
    ]}
    spec = load_scene_spec(_write(self.tmp, d), {})
    total = sum(pl.count for pl in spec.particles)
    self.assertLessEqual(total, MAX_PARTICLES_MINIMAL)

  def test_tint_resolution(self):
    colors = {"Accent": (1, 2, 3, 255)}
    self.assertEqual(_resolve_tint("#FF8000", colors), (255, 128, 0))
    self.assertEqual(_resolve_tint([4, 5, 6], colors), (4, 5, 6))
    self.assertEqual(_resolve_tint("colors:Accent", colors), (1, 2, 3))
    self.assertEqual(_resolve_tint("colors:Nope", colors), (255, 255, 255))
    self.assertEqual(_resolve_tint("garbage", colors), (255, 255, 255))


class TestSimulation(unittest.TestCase):
  def _scene(self, name="p1"):
    spec = load_scene_spec.__wrapped__ if hasattr(load_scene_spec, "__wrapped__") else None
    import tempfile
    tmp = tempfile.mkdtemp()
    p = _write(tmp, GOOD)
    s = load_scene_spec(p, {})
    pack = _FakePack()
    pack.name = name
    return PackScene(pack, s)

  def test_seed_determinism(self):
    a = self._scene("same")
    b = self._scene("same")
    c = self._scene("different")
    self.assertEqual(a.spec.particles[0].particles, b.spec.particles[0].particles)
    self.assertNotEqual(a.spec.particles[0].particles, c.spec.particles[0].particles)

  def test_long_sim_stays_bounded_no_alloc(self):
    scene = self._scene()
    layer = scene.spec.particles[0]
    burst = scene.spec.bursts[0]
    n0 = len(layer.particles)
    dt = 1.0 / 60.0
    for _ in range(10_000):
      layer.step(dt, 1.5)
      burst.step(dt, scene._rng, True)
      self.assertLessEqual(len(burst.pool), MAX_BURST_POOL)
    self.assertEqual(len(layer.particles), n0)
    for p in layer.particles:
      self.assertGreaterEqual(p[1], -0.2)
      self.assertLessEqual(p[1], 1.2)
      self.assertTrue(math.isfinite(p[0]) and math.isfinite(p[1]))

  def test_burst_pool_expires(self):
    scene = self._scene()
    burst = scene.spec.bursts[0]
    dt = 1.0 / 60.0
    for _ in range(2000):
      burst.step(dt, scene._rng, True)
    # after enough time with expiry, pool stays comfortably under the cap
    self.assertLess(len(burst.pool), MAX_BURST_POOL)


class TestBundledScenes(unittest.TestCase):
  def test_all_bundled_scene_files_parse(self):
    found = 0
    for pack in sorted(os.listdir(PACKS_DIR)):
      p = os.path.join(PACKS_DIR, pack, "scene.json")
      if not os.path.isfile(p):
        continue
      found += 1
      colors_p = os.path.join(PACKS_DIR, pack, "colors", "colors.json")
      colors = {}
      if os.path.isfile(colors_p):
        with open(colors_p) as f:
          colors = {k: (v["red"], v["green"], v["blue"], v["alpha"]) for k, v in json.load(f).items()}
      spec = load_scene_spec(p, colors)
      self.assertIsNotNone(spec, f"{pack}/scene.json failed to parse")
      self.assertGreater(len(spec.particles) + len(spec.bursts) + len(spec.decor), 0, pack)
    pack_dirs = [d for d in os.listdir(PACKS_DIR) if os.path.isdir(os.path.join(PACKS_DIR, d))]
    self.assertEqual(found, len(pack_dirs), "every bundled pack must ship a scene.json")

  def test_every_bundled_pack_has_a_hero(self):
    """The mascot layer is part of the pack contract now — parse must keep it."""
    for pack in sorted(os.listdir(PACKS_DIR)):
      p = os.path.join(PACKS_DIR, pack, "scene.json")
      if not os.path.isfile(p):
        continue
      spec = load_scene_spec(p, {})
      self.assertEqual(len(spec.heroes), 1, f"{pack} lost its hero layer in parsing")
      self.assertLessEqual(spec.heroes[0].y_frac, 0.80, f"{pack} hero would sit under the torque bar")

  def test_invalid_hero_sprite_dropped(self):
    import json as _json
    import tempfile
    d = {"version": 1, "layers": [{"type": "hero", "sprite": "no_such_sprite"},
                                  {"type": "particles", "sprite": "heart", "count": 4}]}
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
      _json.dump(d, f)
      path = f.name
    try:
      spec = load_scene_spec(path, {})
      self.assertEqual(len(spec.heroes), 0)
      self.assertEqual(len(spec.particles), 1)   # bad layer dropped, rest kept
    finally:
      os.unlink(path)

  def test_bundled_sprites_render(self):
    """Every sprite referenced by a bundled scene must render non-empty (GL-free)."""
    from openpilot.selfdrive.ui.bp.lib import theme_scene_assets as tsa
    names = set()
    for pack in os.listdir(PACKS_DIR):
      p = os.path.join(PACKS_DIR, pack, "scene.json")
      if os.path.isfile(p):
        with open(p) as f:
          for layer in json.load(f).get("layers", []):
            if "sprite" in layer:
              names.add(layer["sprite"])
    names |= {"spark", "bat2"}  # engine-internal uses
    for n in sorted(names):
      rgb, a = tsa.render_sprite(n, 64)
      self.assertEqual(a.shape, (64, 64), n)
      self.assertGreater(float(a.sum()), 10.0, f"sprite {n} rendered empty")


class TestDispatcher(unittest.TestCase):
  def _reset(self):
    theme_scene._scene_cache.update({"key": None, "scene": None, "checked_at": 0.0})

  def test_rad_racer_capabilities(self):
    self._reset()
    with mock.patch.object(theme_pack, "rad_racer_active", return_value=True):
      s = theme_scene.active_scene()
    self.assertIsInstance(s, RadRacerScene)
    self.assertTrue(s.replaces_hud())
    self.assertTrue(s.replaces_road())
    self._reset()

  def test_no_selection_returns_none(self):
    self._reset()
    with mock.patch.object(theme_pack, "rad_racer_active", return_value=False), \
         mock.patch.object(theme_pack, "get_active_pack", return_value=None):
      self.assertIsNone(theme_scene.active_scene())
    self._reset()

  def test_colors_only_pack_gets_default_scene(self):
    self._reset()
    pack = _FakePack()
    with mock.patch.object(theme_pack, "rad_racer_active", return_value=False), \
         mock.patch.object(theme_pack, "get_active_pack", return_value=pack):
      s = theme_scene.active_scene()
    self.assertIsInstance(s, PackScene)
    self.assertFalse(s.replaces_hud())
    self.assertEqual(len(s.spec.particles), 0)   # sky-only default
    self.assertEqual(s.spec.sky_camera_tint_alpha, 0)
    self._reset()


if __name__ == "__main__":
  unittest.main()
