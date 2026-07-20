import pyray as rl

from msgq.visionipc import VisionStreamType
from openpilot.common.params import Params
from openpilot.selfdrive.ui.mici.onroad.cameraview import CameraView as MiciCameraView
from openpilot.selfdrive.ui.ui_state import ui_state
# BluePilot: unified theme selector (BPThemePack param)
from openpilot.selfdrive.ui.bp.lib import theme_pack


class MiciCameraViewBP(MiciCameraView):
  """BluePilot MICI camera view overrides."""

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._bp_camera_params = Params()
    self._bp_hide_camera_view = self._bp_camera_params.get_bool("BPHideCameraView")
    # BluePilot: Rad Racer theme draws its own scene over a black sky (mirrors TICI)
    self._bp_rad_racer_theme = theme_pack.rad_racer_active(self._bp_camera_params)
    self._bp_camera_param_counter = 0

  def _update_state(self):
    super()._update_state()
    self._bp_camera_param_counter += 1
    if self._bp_camera_param_counter >= 60:
      self._bp_camera_param_counter = 0
      self._bp_hide_camera_view = self._bp_camera_params.get_bool("BPHideCameraView")
      self._bp_rad_racer_theme = theme_pack.rad_racer_active(self._bp_camera_params)

  def _should_hide_camera_view(self) -> bool:
    return (
      (self._bp_hide_camera_view or self._bp_rad_racer_theme) and
      ui_state.is_onroad() and
      self._stream_type != VisionStreamType.VISION_STREAM_DRIVER
    )

  def _render(self, rect: rl.Rectangle):
    if self._should_hide_camera_view():
      self._calc_frame_matrix(rect)
      rl.draw_rectangle_rec(rect, rl.BLACK)

    MiciCameraView._render(self, rect)

  def _draw_placeholder(self, rect: rl.Rectangle):
    if self._should_hide_camera_view():
      rl.draw_rectangle_rec(rect, rl.BLACK)
    else:
      super()._draw_placeholder(rect)

  def _render_egl(self, src_rect: rl.Rectangle, dst_rect: rl.Rectangle) -> None:
    if self._should_hide_camera_view():
      return

    MiciCameraView._render_egl(self, src_rect, dst_rect)

  def _render_textures(self, src_rect: rl.Rectangle, dst_rect: rl.Rectangle) -> None:
    if self._should_hide_camera_view():
      return

    MiciCameraView._render_textures(self, src_rect, dst_rect)
