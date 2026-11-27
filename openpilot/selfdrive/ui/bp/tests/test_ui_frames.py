"""BluePilot's UI drawn for real, offscreen, on both screens: the onroad view once per BluePilot display option
(synthetic messages, fake camera frames), then every settings page. A stale override only fails when its
code runs, so this draws frames instead of checking imports or signatures.

Each screen family runs in its own process (ui_frames_child.py): raylib has one window per process and the
3X layout (BIG) is chosen at import. On the Mac it draws into a hidden window. On Linux with no display raylib
picks its headless EGL backend by itself (needs libegl1 and libgles2, as upstream's UI report job installs).
Skips when no window can be created.
"""
import concurrent.futures
import json
import os
import subprocess
import sys
import unittest

CHILD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui_frames_child.py")
NO_WINDOW = 77


def run_family(family: str) -> tuple[int, str]:
  proc = subprocess.run([sys.executable, CHILD, family], capture_output=True, text=True, timeout=600)
  return proc.returncode, proc.stdout + proc.stderr


class TestUiFrames(unittest.TestCase):
  results: dict = {}
  output: dict = {}
  no_window: dict = {}

  @classmethod
  def setUpClass(cls):
    with concurrent.futures.ThreadPoolExecutor(2) as pool:
      runs = dict(zip(("tici", "mici"), pool.map(run_family, ("tici", "mici")), strict=True))
    for family, (code, out) in runs.items():
      cls.output[family] = out
      reason = next((ln for ln in out.splitlines() if ln.startswith("NOWINDOW ")), None)
      if code == NO_WINDOW and reason:  # this family's own tests skip; the other family still runs and can fail
        cls.no_window[family] = reason
      line = next((ln for ln in out.splitlines() if ln.startswith("RESULTS ")), None)
      cls.results[family] = json.loads(line[len("RESULTS "):]) if line else None

  def _results(self, family: str) -> dict:
    if family in self.no_window:
      self.skipTest(f"{family}: {self.no_window[family]}")
    results = self.results[family]
    self.assertIsNotNone(results, f"{family} child crashed:\n{self.output[family][-6000:]}")
    return results

  def _check(self, family: str, prefix: str, minimum: int):
    results = self._results(family)
    cases = {name: r for name, r in results.items() if name.startswith(prefix)}
    self.assertGreaterEqual(len(cases), minimum)
    for name, r in cases.items():
      with self.subTest(family=family, case=name):
        self.assertIsNone(r["error"], f"{family} {name}:\n{r['error']}")

  def test_tici_onroad(self):
    self._check("tici", "onroad/", 25)

  def test_tici_settings(self):
    self._check("tici", "settings/", 15)

  def test_mici_onroad(self):
    self._check("mici", "onroad/", 25)

  def test_mici_settings(self):
    self._check("mici", "settings/", 50)

  def test_tici_bluepilot_settings_page_drawn_with_sections_open(self):
    calls = self._results("tici")["settings/BLUEPILOT"]["calls"]
    for drawn in ("sections expanded", "CollapsibleSectionHeader._render", "FloatControlAction._render"):
      with self.subTest(drawn=drawn):
        self.assertGreater(calls.get(drawn, 0), 0)

  def test_lanefull_locked_drawn(self):
    # #285: the lateral settings drawn with Blendful running, in both lateral modes
    for family in ("tici", "mici"):
      with self.subTest(family=family):
        result = self._results(family)["settings/lanefull locked"]
        self.assertIsNone(result["error"], result["error"])
        self.assertEqual(result["calls"].get("lanefull locked pages drawn"), 2)

  def test_tici_controls_debug_panel_drawn(self):
    self.assertGreater(self._results("tici")["onroad/controls debug panel"]["calls"].get("DebugDataCard._render", 0), 0)

  def test_mici_model_source_icon_drawn(self):
    calls = self._results("mici")["onroad/model source icon"]["calls"]
    self.assertGreater(calls.get("model source icon drawn", 0), 0)

  def test_lane_center_line_drawn(self):
    # sunnypilot's lane-center (blendful) line, engaged with lateral active: the 3X override used to skip it
    for family in ("tici", "mici"):
      with self.subTest(family=family):
        self.assertGreater(self._results(family)["onroad/defaults"]["calls"].get("lane-center line polygon", 0), 0)


if __name__ == "__main__":
  unittest.main()
