"""BluePilot code that follows upstream renames inside functions, where an import test can't see it."""
import unittest
from unittest import mock

import pyray as rl

from openpilot.cereal.visionipc import VisionStreamType
from openpilot.selfdrive.ui.ui_state import ui_state


class TestUpstreamRenames(unittest.TestCase):
  def test_hide_camera_view_never_hides_the_cabin_camera(self):
    from openpilot.selfdrive.ui.bp.onroad.cameraview_bp import CameraViewBP
    from openpilot.selfdrive.ui.bp.mici.onroad.cameraview_bp import MiciCameraViewBP
    for cls in (CameraViewBP, MiciCameraViewBP):
      view = cls.__new__(cls)  # no window: skip the GL setup in __init__
      view.close = lambda: None  # and the GL teardown in __del__
      view._bp_hide_camera_view = True
      view._bp_rad_racer_theme = False
      with self.subTest(view=cls.__name__), mock.patch.object(ui_state, "is_onroad", return_value=True):
        view._stream_type = VisionStreamType.VISION_STREAM_CABIN
        self.assertFalse(view._should_hide_camera_view())
        view._stream_type = VisionStreamType.VISION_STREAM_WIDE_ROAD
        self.assertTrue(view._should_hide_camera_view())

  def test_portal_panel_draws_its_qr_code_with_upstreams_helper(self):
    from openpilot.selfdrive.ui.bp.layouts.settings.bp_web_panel import _QRCodeSection
    section = _QRCodeSection(mock.Mock())
    texture = rl.Texture()
    url = "http://192.168.1.2:8088"
    with mock.patch.object(section, "get_server_url", return_value=url), \
         mock.patch("openpilot.common.qrcode.make_texture", return_value=texture) as make_texture:
      section._generate_qr()
    make_texture.assert_called_once_with(url)
    self.assertIs(section._qr_texture, texture)


if __name__ == "__main__":
  unittest.main()
