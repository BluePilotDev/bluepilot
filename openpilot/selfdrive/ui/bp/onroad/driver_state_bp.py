"""BluePilot-selectable comma 4 / comma 3X driver-monitoring icon renderer."""

import numpy as np
import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui import UI_BORDER_SIZE
from openpilot.selfdrive.ui.bp.lib.dm_icon_style import (
  DMIconStyle,
  ensure_dm_icon_style_initialized,
  get_dm_icon_style,
)
from openpilot.selfdrive.ui.mici.onroad.driver_state import DriverStateRenderer as DriverStateRendererMici
from openpilot.selfdrive.ui.onroad.driver_state import (
  ARC_ANGLES,
  ARC_LENGTH,
  ARC_THICKNESS_DEFAULT,
  ARC_THICKNESS_EXTEND,
  BTN_SIZE,
  IMG_SIZE,
  ArcData,
)
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui import get_bottom_dev_ui_offset
from openpilot.selfdrive.ui.sunnypilot.onroad.driver_state import DriverStateRendererSP
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget


PARAM_REFRESH_FRAMES = 60
COMMA_4_ICON_SIZE = DriverStateRendererMici.BASE_SIZE


class _CompactComma3XDriverStateRendererBP(DriverStateRendererSP):
  """Scale the comma 3X DMoji into the comma 4 icon footprint."""

  def __init__(self, icon_size: int):
    super().__init__()
    self._scale = icon_size / BTN_SIZE
    face_size = max(1, round(IMG_SIZE * self._scale))
    self.dm_img = gui_app.texture("icons/driver_face.png", face_size, face_size)

  def _render(self, _rect: rl.Rectangle) -> None:
    opacity = 0.65 if self.is_active else 0.2
    button_size = BTN_SIZE * self._scale

    rl.draw_circle(int(self.position_x), int(self.position_y), round(button_size / 2), rl.Color(0, 0, 0, 70))

    icon_pos = rl.Vector2(self.position_x - self.dm_img.width / 2, self.position_y - self.dm_img.height / 2)
    rl.draw_texture_v(self.dm_img, icon_pos, rl.Color(255, 255, 255, int(255 * opacity)))

    self.white_color.a = int(255 * opacity)
    rl.draw_spline_linear(self.face_lines, len(self.face_lines), max(1.0, 5.2 * self._scale), self.white_color)

    self.arc_color = self.engaged_color if ui_state.engaged else self.disengaged_color
    self.arc_color.a = int(0.4 * 255 * (1.0 - self.dm_fade_state))
    if self.h_arc_data:
      rl.draw_spline_linear(self.h_arc_lines, len(self.h_arc_lines), self.h_arc_data.thickness, self.arc_color)
    if self.v_arc_data:
      rl.draw_spline_linear(self.v_arc_lines, len(self.v_arc_lines), self.v_arc_data.thickness, self.arc_color)

  def _pre_calculate_drawing_elements(self) -> None:
    self.position_x = self._rect.x + self._rect.width / 2
    self.position_y = self._rect.y + self._rect.height / 2

    positioned_keypoints = self.face_keypoints_transformed * self._scale + np.array([self.position_x, self.position_y])
    for i, point in enumerate(positioned_keypoints):
      self.face_lines[i].x = point[0]
      self.face_lines[i].y = point[1]

    arc_length = ARC_LENGTH * self._scale
    delta_x = -self.driver_pose_sins[1] * arc_length / 2.0
    delta_y = -self.driver_pose_sins[0] * arc_length / 2.0
    self.h_arc_data = self._calculate_scaled_arc_data(
      delta_x,
      abs(delta_x),
      self.position_x,
      self.position_y - arc_length / 2,
      self.driver_pose_sins[1],
      self.driver_pose_diff[1],
      is_horizontal=True,
    )
    self.v_arc_data = self._calculate_scaled_arc_data(
      delta_y,
      abs(delta_y),
      self.position_x - arc_length / 2,
      self.position_y,
      self.driver_pose_sins[0],
      self.driver_pose_diff[0],
      is_horizontal=False,
    )

  def _calculate_scaled_arc_data(
    self,
    delta: float,
    size: float,
    x: float,
    y: float,
    sin_val: float,
    diff_val: float,
    is_horizontal: bool,
  ) -> ArcData | None:
    if size <= 0:
      return None

    arc_length = ARC_LENGTH * self._scale
    thickness = (ARC_THICKNESS_DEFAULT + ARC_THICKNESS_EXTEND * min(1.0, diff_val * 5.0)) * self._scale
    start_angle = (90 if sin_val > 0 else -90) if is_horizontal else (0 if sin_val > 0 else 180)
    x = min(x + delta, x) if is_horizontal else x
    y = y if is_horizontal else min(y + delta, y)
    arc_data = ArcData(
      x=x,
      y=y,
      width=size if is_horizontal else arc_length,
      height=arc_length if is_horizontal else size,
      thickness=max(1.0, thickness),
    )

    angles = ARC_ANGLES + np.deg2rad(start_angle)
    center_x = x + arc_data.width / 2
    center_y = y + arc_data.height / 2
    x_coords = center_x + np.cos(angles) * arc_data.width / 2
    y_coords = center_y - np.sin(angles) * arc_data.height / 2
    arc_lines = self.h_arc_lines if is_horizontal else self.v_arc_lines
    for i, (x_coord, y_coord) in enumerate(zip(x_coords, y_coords, strict=True)):
      arc_lines[i].x = x_coord
      arc_lines[i].y = y_coord

    return arc_data


class DriverStateRendererBP(Widget):
  """Render either native DMoji style while keeping each device's normal footprint."""

  def __init__(self, device_default: DMIconStyle):
    super().__init__()
    self._params = Params()
    self._device_default = device_default
    self._style = ensure_dm_icon_style_initialized(self._params, device_default)
    self._param_frame_counter = 0
    self._should_draw = True
    self._is_mici = device_default == DMIconStyle.COMMA_4

    self._comma4_renderer = self._child(DriverStateRendererMici())
    if self._is_mici:
      self.set_rect(rl.Rectangle(0, 0, COMMA_4_ICON_SIZE, COMMA_4_ICON_SIZE))
      self._comma3x_renderer = self._child(_CompactComma3XDriverStateRendererBP(COMMA_4_ICON_SIZE))
    else:
      self._comma4_renderer.set_rect(rl.Rectangle(0, 0, BTN_SIZE, BTN_SIZE))
      self._comma4_renderer.load_icons()
      self._comma4_renderer.set_should_draw(True)
      self._comma3x_renderer = self._child(DriverStateRendererSP())

  def set_should_draw(self, should_draw: bool) -> None:
    self._should_draw = should_draw

  @property
  def is_rhd(self) -> bool:
    renderer = self._comma4_renderer if self._style == DMIconStyle.COMMA_4 else self._comma3x_renderer
    return bool(renderer.is_rhd)

  def _update_state(self) -> None:
    self._param_frame_counter += 1
    if self._param_frame_counter >= PARAM_REFRESH_FRAMES:
      self._param_frame_counter = 0
      self._style = get_dm_icon_style(self._params, self._device_default)

  def _render(self, rect: rl.Rectangle) -> None:
    if self._is_mici:
      self._render_mici(rect)
    else:
      self._render_tici(rect)

  def _render_mici(self, rect: rl.Rectangle) -> None:
    if self._style == DMIconStyle.COMMA_4:
      if self._comma4_renderer.rect.width != rect.width or self._comma4_renderer.rect.height != rect.height:
        self._comma4_renderer.set_rect(rl.Rectangle(rect.x, rect.y, rect.width, rect.height))
        self._comma4_renderer.load_icons()
      else:
        self._comma4_renderer.set_position(rect.x, rect.y)
      self._comma4_renderer.set_should_draw(self._should_draw)
      self._comma4_renderer.render()
    elif self._should_draw:
      self._comma3x_renderer.render(rect)

  def _render_tici(self, rect: rl.Rectangle) -> None:
    if self._style == DMIconStyle.COMMA_3X:
      self._comma3x_renderer.render(rect)
      return

    is_rhd = ui_state.sm["driverMonitoringState"].isRHD
    x = rect.x + rect.width - UI_BORDER_SIZE - BTN_SIZE if is_rhd else rect.x + UI_BORDER_SIZE
    y = rect.y + rect.height - UI_BORDER_SIZE - BTN_SIZE - get_bottom_dev_ui_offset()
    self._comma4_renderer.set_position(x, y)
    self._comma4_renderer.render()
