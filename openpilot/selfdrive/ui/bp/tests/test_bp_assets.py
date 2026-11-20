"""BluePilot's images live under bluepilot/assets/, and every image path BluePilot code names exists."""
import os
import re
import subprocess
import unittest

from openpilot.common.basedir import BASEDIR

# gui_app.texture() resolves paths against this folder
ASSETS_DIR = os.path.join(BASEDIR, "openpilot", "selfdrive", "assets")
DEVICE_ROOT = "/data/openpilot/"

BP_CODE = ["bluepilot", "openpilot/selfdrive/ui/bp", "scripts/boot_logo.sh",
           "openpilot/selfdrive/ui/sunnypilot/mici/layouts/settings.py"]  # last one: BluePilot hook (settings button)
PATH_LITERAL = re.compile(r"""["']([\w.-]*/[\w./-]+\.(?:png|jpg|jpeg|gif|svg))["']""")
LIST_ICON = re.compile(r"""\bicon=["']([\w.-]+\.png)["']""")  # list_view items load these from icons/

# moved out of upstream asset folders (BluePilotDev/bluepilot#223)
MOVED = {
  "icons_mici/settings/car_icon.png": "bluepilot/assets/icons_mici/settings/car_icon.png",
  "images/button_fan.png": "bluepilot/assets/images/button_fan.png",
  "offroad/icon_debug.png": "bluepilot/assets/offroad/icon_debug.png",
  "offroad/icon_flag.png": "bluepilot/assets/offroad/icon_flag.png",
  "offroad/icon_settings.png": "bluepilot/assets/offroad/icon_settings.png",
}


def _tracked(*paths: str, ext=(".py", ".sh")) -> list[str]:
  out = subprocess.check_output(["git", "ls-files", "--", *paths], cwd=BASEDIR, text=True)
  return [p for p in out.splitlines() if not p.startswith("bluepilot/vendor/") and p.endswith(ext)]


def _image_refs() -> list[tuple[str, str]]:
  refs = []
  for rel in _tracked(*BP_CODE):
    if "/tests/" in rel:
      continue
    with open(os.path.join(BASEDIR, rel), encoding="utf-8") as f:
      text = f.read()
    refs += [(rel, m) for m in PATH_LITERAL.findall(text) if not m.startswith("/") or m.startswith(DEVICE_ROOT)]
    refs += [(rel, "icons/" + m) for m in LIST_ICON.findall(text)]
  return refs


def _resolve(ref: str) -> str:
  if ref.startswith(DEVICE_ROOT):
    return os.path.join(BASEDIR, ref[len(DEVICE_ROOT):])
  return os.path.normpath(os.path.join(ASSETS_DIR, ref))


class TestBluePilotAssets(unittest.TestCase):
  def test_every_referenced_image_exists(self):
    refs = _image_refs()
    self.assertGreater(len(refs), 5)
    missing = [f"{src}: {ref}" for src, ref in refs if not os.path.isfile(_resolve(ref))]
    self.assertEqual(missing, [])

  def test_moved_images_are_referenced_at_their_new_path(self):
    resolved = {os.path.relpath(_resolve(ref), BASEDIR) for _, ref in _image_refs()}
    for new in MOVED.values():
      self.assertTrue(os.path.isfile(os.path.join(BASEDIR, new)), new)
      self.assertIn(new, resolved)

  def test_no_reference_to_an_old_path_remains(self):
    stale = []
    for rel in _tracked(".", ext=(".py", ".sh", ".md", ".json", ".toml", ".txt", "SConscript", "SConstruct")):
      if rel.endswith("test_bp_assets.py") or not os.path.isfile(os.path.join(BASEDIR, rel)):
        continue
      with open(os.path.join(BASEDIR, rel), encoding="utf-8", errors="ignore") as f:
        text = f.read()
      stale += [f"{rel}: {old}" for old in MOVED if f'"{old}"' in text or f"/selfdrive/assets/{old}" in text]
    self.assertEqual(stale, [])

  def test_no_bluepilot_image_left_in_upstream_asset_folders(self):
    with open(os.path.join(BASEDIR, ".gitattributes"), encoding="utf-8") as f:
      per_file = [line.split()[0] for line in f if line.startswith("openpilot/selfdrive/assets/") and "*" not in line]
    # spinner_bluepilot.png moves in BluePilotDev/bluepilot#258
    self.assertEqual([p for p in per_file if not p.endswith("/spinner_bluepilot.png")], [])


if __name__ == "__main__":
  unittest.main()
