"""Responsive model-path color picker for comma 3X (TICI) and comma 4 (MICI)."""

from __future__ import annotations

import colorsys
import math
from collections.abc import Callable

import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui.bp.lib.model_path_color import (
  PARAM_KEY,
  PRESET_COLORS,
  model_path_color_hex,
  model_path_color_is_visible,
  normalize_model_path_color,
)
from openpilot.system.ui.lib.application import FontWeight, MouseEvent, MousePos, TextAlignment, gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import Button, ButtonStyle
from openpilot.system.ui.widgets.label import gui_label


class ModelPathColorDialog(Widget):
  """Preset selector with an HSV wheel and dedicated hexadecimal keypad."""

  MIN_VALUE = 0.20
  WHEEL_TEXTURE_SIZE_BIG = 256
  WHEEL_TEXTURE_SIZE_SMALL = 192

  def __init__(self, on_saved: Callable[[str], None] | None = None):
    super().__init__()
    self._params = Params()
    self._on_saved = on_saved
    self._is_big = gui_app.big_ui()
    self._mode = "presets"
    self._drag_target: str | None = None
    self._error = ""

    self._initial_value = normalize_model_path_color(self._params.get(PARAM_KEY, return_default=True))
    initial_hex = model_path_color_hex(self._initial_value) or PRESET_COLORS["Green"] or "#0DF87A"
    self._set_hex(initial_hex)
    self._hex_digits = self._hex[1:]

    self._wheel_rect = rl.Rectangle(0, 0, 0, 0)
    self._value_rect = rl.Rectangle(0, 0, 0, 0)
    self._wheel_texture = self._create_wheel_texture(
      self.WHEEL_TEXTURE_SIZE_BIG if self._is_big else self.WHEEL_TEXTURE_SIZE_SMALL
    )

    preset_font_size = 58 if self._is_big else 20
    preset_text_padding = 20 if self._is_big else 4
    self._preset_buttons: dict[str, Button] = {}
    for name in PRESET_COLORS:
      button = Button(name, click_callback=lambda preset=name: self._choose_preset(preset),
                      button_style=ButtonStyle.PRIMARY if name == self._initial_value else ButtonStyle.NORMAL,
                      font_size=preset_font_size, text_padding=preset_text_padding, elide_right=not self._is_big)
      self._preset_buttons[name] = button
    custom = Button(lambda: tr("Custom"), click_callback=self._show_picker,
                    button_style=ButtonStyle.PRIMARY if self._initial_value.startswith("#") else ButtonStyle.NORMAL,
                    font_size=preset_font_size, text_padding=preset_text_padding, elide_right=not self._is_big)
    self._preset_buttons["Custom"] = custom

    font_size = 54 if self._is_big else 22
    self._presets_button = Button(lambda: tr("Presets"), self._show_presets, font_size=font_size)
    self._hex_button = Button("HEX", self._show_hex, font_size=font_size)
    self._back_button = Button(lambda: tr("Back"), self._go_back, font_size=font_size,
                               button_style=ButtonStyle.TRANSPARENT_WHITE_TEXT)
    self._cancel_button = Button(lambda: tr("Cancel"), self._cancel, font_size=font_size)
    self._save_button = Button(lambda: tr("Save"), self._save, button_style=ButtonStyle.PRIMARY, font_size=font_size)
    self._backspace_button = Button("⌫", self._backspace, font_size=font_size)

    self._hex_buttons: dict[str, Button] = {}
    for char in "123A456B789CD0EF":
      self._hex_buttons[char] = Button(char, click_callback=lambda value=char: self._append_hex(value),
                                       button_style=ButtonStyle.KEYBOARD, font_size=64 if self._is_big else 25,
                                       multi_touch=True)

  @staticmethod
  def _create_wheel_texture(size: int) -> rl.Texture:
    image = rl.gen_image_color(size, size, rl.BLANK)
    center = (size - 1) / 2
    radius = size / 2
    for y in range(size):
      for x in range(size):
        dx, dy = x - center, y - center
        saturation = math.hypot(dx, dy) / radius
        if saturation > 1.0:
          continue
        hue = (math.atan2(dy, dx) / (2 * math.pi)) % 1.0
        r, g, b = colorsys.hsv_to_rgb(hue, saturation, 1.0)
        rl.image_draw_pixel(image, x, y, rl.Color(round(r * 255), round(g * 255), round(b * 255), 255))
    texture = rl.load_texture_from_image(image)
    rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
    rl.unload_image(image)
    return texture

  def hide_event(self):
    super().hide_event()
    if self._wheel_texture.id:
      rl.unload_texture(self._wheel_texture)
      self._wheel_texture.id = 0

  def _set_hex(self, color_hex: str):
    normalized = normalize_model_path_color(color_hex)
    if not normalized.startswith("#"):
      normalized = PRESET_COLORS["Green"] or "#0DF87A"
    self._hex = normalized
    r, g, b = (int(normalized[i:i + 2], 16) / 255 for i in (1, 3, 5))
    self._hue, self._saturation, self._value = colorsys.rgb_to_hsv(r, g, b)

  def _sync_hex_from_hsv(self):
    r, g, b = colorsys.hsv_to_rgb(self._hue, self._saturation, self._value)
    self._hex = f"#{round(r * 255):02X}{round(g * 255):02X}{round(b * 255):02X}"
    self._hex_digits = self._hex[1:]
    self._error = ""

  def _show_presets(self):
    self._mode = "presets"
    self._error = ""

  def _show_picker(self):
    self._mode = "picker"
    self._error = ""

  def _show_hex(self):
    self._mode = "hex"
    self._hex_digits = self._hex[1:]
    self._error = ""

  def _go_back(self):
    if self._mode == "hex":
      self._show_picker()
    else:
      self._show_presets()

  def _choose_preset(self, preset: str):
    if preset == "Custom":
      self._show_picker()
      return
    self._params.put(PARAM_KEY, preset, block=False)
    gui_app.pop_widget()
    if self._on_saved:
      self._on_saved(preset)

  def _cancel(self):
    gui_app.pop_widget()

  def _save(self):
    if self._mode == "hex":
      if len(self._hex_digits) != 6:
        self._error = tr("Enter all 6 hexadecimal digits")
        return
      self._set_hex(f"#{self._hex_digits}")

    if not model_path_color_is_visible(self._hex):
      self._error = tr("Choose a brighter color for visibility")
      return

    value = normalize_model_path_color(self._hex)
    self._params.put(PARAM_KEY, value, block=False)
    gui_app.pop_widget()
    if self._on_saved:
      self._on_saved(value)

  def _append_hex(self, char: str):
    if len(self._hex_digits) >= 6:
      self._hex_digits = ""
    self._hex_digits += char
    if len(self._hex_digits) == 6:
      self._set_hex(f"#{self._hex_digits}")
    self._error = ""

  def _backspace(self):
    self._hex_digits = self._hex_digits[:-1]
    self._error = ""

  def _render(self, rect: rl.Rectangle):
    rl.draw_rectangle_rec(rect, rl.Color(12, 16, 20, 255))
    if self._mode == "presets":
      self._render_presets(rect)
    elif self._mode == "hex":
      self._render_hex(rect)
    else:
      self._render_picker(rect)

  def _render_header(self, rect: rl.Rectangle, title: str, right: str = "") -> float:
    height = 110 if self._is_big else 38
    header = rl.Rectangle(rect.x, rect.y, rect.width, height)
    rl.draw_rectangle_rec(header, rl.Color(21, 26, 32, 255))
    rl.draw_line(int(rect.x), int(rect.y + height), int(rect.x + rect.width), int(rect.y + height), rl.Color(61, 70, 81, 255))
    gui_label(header, title, font_size=62 if self._is_big else 24, font_weight=FontWeight.BOLD,
              color=rl.WHITE, alignment=TextAlignment.CENTER)
    if self._mode != "presets":
      back_width = 360 if self._is_big else 100
      self._back_button.render(rl.Rectangle(rect.x + (35 if self._is_big else 5), rect.y,
                                            back_width, height))
    if right:
      right_rect = rl.Rectangle(rect.x + rect.width * 0.72, rect.y, rect.width * 0.24, height)
      gui_label(right_rect, right, font_size=38 if self._is_big else 17, font_weight=FontWeight.NORMAL,
                color=rl.Color(170, 179, 189, 255), alignment=TextAlignment.RIGHT)
    return height

  def _render_presets(self, rect: rl.Rectangle):
    header_h = self._render_header(rect, tr("Model Path Color"), tr("Select a color"))
    margin = 85 if self._is_big else 10
    gap = 28 if self._is_big else 6
    bottom_h = 125 if self._is_big else 44
    grid_top = rect.y + header_h + gap
    grid_bottom = rect.y + rect.height - bottom_h - gap * 2
    columns = 5
    rows = 2
    button_w = (rect.width - margin * 2 - gap * (columns - 1)) / columns
    button_h = (grid_bottom - grid_top - gap * (rows - 1)) / rows
    names = list(self._preset_buttons)
    for i, name in enumerate(names):
      row, col = divmod(i, columns)
      button_rect = rl.Rectangle(rect.x + margin + col * (button_w + gap), grid_top + row * (button_h + gap), button_w, button_h)
      self._preset_buttons[name].render(button_rect)
      color_hex = model_path_color_hex(name)
      if color_hex is not None:
        rgb = tuple(int(color_hex[j:j + 2], 16) for j in (1, 3, 5))
        dot_r = 18 if self._is_big else 5
        rl.draw_circle(int(button_rect.x + button_rect.width - dot_r * 2), int(button_rect.y + dot_r * 2), dot_r, rl.Color(*rgb, 255))
    cancel_w = 520 if self._is_big else 150
    self._cancel_button.render(rl.Rectangle(rect.x + (rect.width - cancel_w) / 2, rect.y + rect.height - bottom_h - gap, cancel_w, bottom_h))

  def _picker_layout(self, rect: rl.Rectangle) -> tuple[rl.Rectangle, rl.Rectangle, rl.Rectangle, float]:
    header_h = 110 if self._is_big else 38
    if self._is_big:
      wheel_size = 650
      wheel = rl.Rectangle(rect.x + 100, rect.y + header_h + (rect.height - header_h - wheel_size) / 2, wheel_size, wheel_size)
      value = rl.Rectangle(wheel.x + wheel.width + 50, wheel.y, 72, wheel.height)
      actions = rl.Rectangle(value.x + value.width + 90, rect.y + header_h + 55,
                             rect.width - (value.x + value.width + 90) - 90, rect.height - header_h - 110)
    else:
      wheel_size = 158
      wheel = rl.Rectangle(rect.x + 14, rect.y + header_h + 20, wheel_size, wheel_size)
      value = rl.Rectangle(wheel.x + wheel.width + 10, wheel.y, 17, wheel.height)
      actions = rl.Rectangle(value.x + value.width + 18, rect.y + header_h + 8,
                             rect.width - (value.x + value.width + 18) - 10, rect.height - header_h - 14)
    return wheel, value, actions, header_h

  def _render_picker(self, rect: rl.Rectangle):
    self._render_header(rect, tr("Model Path Color"), tr("Custom"))
    self._wheel_rect, self._value_rect, actions, _ = self._picker_layout(rect)
    source = rl.Rectangle(0, 0, self._wheel_texture.width, self._wheel_texture.height)
    rl.draw_texture_pro(self._wheel_texture, source, self._wheel_rect, rl.Vector2(0, 0), 0, rl.WHITE)

    angle = self._hue * 2 * math.pi
    radius = self._saturation * self._wheel_rect.width / 2
    cursor = rl.Vector2(self._wheel_rect.x + self._wheel_rect.width / 2 + math.cos(angle) * radius,
                       self._wheel_rect.y + self._wheel_rect.height / 2 + math.sin(angle) * radius)
    cursor_r = 18 if self._is_big else 7
    rl.draw_circle_lines(int(cursor.x), int(cursor.y), cursor_r + 3, rl.BLACK)
    rl.draw_circle_lines(int(cursor.x), int(cursor.y), cursor_r, rl.WHITE)

    full_rgb = colorsys.hsv_to_rgb(self._hue, self._saturation, 1.0)
    top = rl.Color(*(round(c * 255) for c in full_rgb), 255)
    rl.draw_rectangle_gradient_v(int(self._value_rect.x), int(self._value_rect.y), int(self._value_rect.width), int(self._value_rect.height), top, rl.BLACK)
    marker_y = self._value_rect.y + (1.0 - self._value) * self._value_rect.height
    marker_h = 12 if self._is_big else 5
    rl.draw_rectangle_lines_ex(rl.Rectangle(self._value_rect.x - marker_h / 2, marker_y - marker_h / 2,
                                             self._value_rect.width + marker_h, marker_h), 3 if self._is_big else 1, rl.WHITE)

    rgb = tuple(int(self._hex[i:i + 2], 16) for i in (1, 3, 5))
    swatch_size = min(actions.width * 0.28, actions.height * 0.32)
    swatch = rl.Rectangle(actions.x, actions.y, swatch_size, swatch_size)
    rl.draw_rectangle_rounded(swatch, 0.08, 10, rl.Color(*rgb, 255))
    rl.draw_rectangle_rounded_lines_ex(swatch, 0.08, 10, 3 if self._is_big else 1, rl.Color(255, 255, 255, 100))
    label_rect = rl.Rectangle(swatch.x + swatch.width + (35 if self._is_big else 10), swatch.y,
                              actions.width - swatch.width - (35 if self._is_big else 10), swatch.height)
    gui_label(label_rect, self._hex, font_size=62 if self._is_big else 24, font_weight=FontWeight.BOLD,
              color=rl.WHITE, alignment=TextAlignment.LEFT)

    gap = 28 if self._is_big else 7
    buttons_y = swatch.y + swatch.height + gap
    button_h = (actions.y + actions.height - buttons_y - gap) / 2
    button_w = (actions.width - gap) / 2
    buttons = (self._presets_button, self._hex_button, self._cancel_button, self._save_button)
    for i, button in enumerate(buttons):
      row, col = divmod(i, 2)
      button.render(rl.Rectangle(actions.x + col * (button_w + gap), buttons_y + row * (button_h + gap), button_w, button_h))
    self._draw_error(actions)

  def _render_hex(self, rect: rl.Rectangle):
    header_h = self._render_header(rect, tr("Enter Custom Color"), tr("6 digits"))
    margin = 70 if self._is_big else 12
    gap = 36 if self._is_big else 8
    body = rl.Rectangle(rect.x + margin, rect.y + header_h + gap, rect.width - margin * 2, rect.height - header_h - gap * 2)
    left_w = body.width * (0.36 if self._is_big else 0.37)
    left = rl.Rectangle(body.x, body.y, left_w, body.height)
    keypad = rl.Rectangle(left.x + left.width + gap, body.y, body.width - left.width - gap, body.height)

    input_h = body.height * 0.25
    input_rect = rl.Rectangle(left.x, left.y, left.width, input_h)
    rl.draw_rectangle_rounded(input_rect, 0.08, 10, rl.Color(7, 10, 13, 255))
    rl.draw_rectangle_rounded_lines_ex(input_rect, 0.08, 10, 4 if self._is_big else 2, rl.Color(47, 128, 237, 255))
    display = f"#{self._hex_digits:<6}".replace(" ", "·")
    gui_label(input_rect, display, font_size=70 if self._is_big else 26, font_weight=FontWeight.BOLD,
              color=rl.WHITE, alignment=TextAlignment.CENTER)

    preview_y = input_rect.y + input_rect.height + gap
    swatch_h = body.height * 0.20
    rgb = tuple(int(self._hex[i:i + 2], 16) for i in (1, 3, 5))
    rl.draw_rectangle_rounded(rl.Rectangle(left.x, preview_y, swatch_h, swatch_h), 0.08, 8, rl.Color(*rgb, 255))
    gui_label(rl.Rectangle(left.x + swatch_h + gap, preview_y, left.width - swatch_h - gap, swatch_h),
              tr("Color preview"), font_size=40 if self._is_big else 16, color=rl.Color(170, 179, 189, 255),
              alignment=TextAlignment.LEFT)

    action_y = left.y + left.height - input_h
    action_w = (left.width - gap) / 2
    self._backspace_button.render(rl.Rectangle(left.x, action_y, action_w, input_h))
    self._save_button.render(rl.Rectangle(left.x + action_w + gap, action_y, action_w, input_h))

    key_gap = 22 if self._is_big else 5
    key_w = (keypad.width - key_gap * 3) / 4
    key_h = (keypad.height - key_gap * 3) / 4
    for i, char in enumerate("123A456B789CD0EF"):
      row, col = divmod(i, 4)
      self._hex_buttons[char].render(rl.Rectangle(keypad.x + col * (key_w + key_gap), keypad.y + row * (key_h + key_gap), key_w, key_h))
    self._draw_error(left, bottom_padding=input_h + (8 if self._is_big else 3))

  def _draw_error(self, rect: rl.Rectangle, bottom_padding: float = 0):
    if not self._error:
      return
    error_h = 55 if self._is_big else 18
    error_rect = rl.Rectangle(rect.x, rect.y + rect.height - error_h - bottom_padding, rect.width, error_h)
    gui_label(error_rect, self._error, font_size=34 if self._is_big else 13, font_weight=FontWeight.NORMAL,
              color=rl.Color(255, 105, 105, 255), alignment=TextAlignment.CENTER)

  def _set_from_wheel(self, pos: MousePos):
    cx = self._wheel_rect.x + self._wheel_rect.width / 2
    cy = self._wheel_rect.y + self._wheel_rect.height / 2
    dx, dy = pos.x - cx, pos.y - cy
    radius = self._wheel_rect.width / 2
    self._saturation = min(1.0, math.hypot(dx, dy) / radius)
    self._hue = (math.atan2(dy, dx) / (2 * math.pi)) % 1.0
    self._sync_hex_from_hsv()

  def _set_from_value(self, pos: MousePos):
    fraction = min(1.0, max(0.0, (pos.y - self._value_rect.y) / self._value_rect.height))
    self._value = max(self.MIN_VALUE, 1.0 - fraction)
    self._sync_hex_from_hsv()

  def _handle_mouse_press(self, mouse_pos: MousePos):
    if self._mode != "picker":
      return
    if rl.check_collision_point_circle(mouse_pos, rl.Vector2(self._wheel_rect.x + self._wheel_rect.width / 2,
                                                              self._wheel_rect.y + self._wheel_rect.height / 2),
                                       self._wheel_rect.width / 2):
      self._drag_target = "wheel"
      self._set_from_wheel(mouse_pos)
    elif rl.check_collision_point_rec(mouse_pos, self._value_rect):
      self._drag_target = "value"
      self._set_from_value(mouse_pos)

  def _handle_mouse_event(self, mouse_event: MouseEvent):
    if not mouse_event.left_down or self._mode != "picker":
      if mouse_event.left_released:
        self._drag_target = None
      return
    if self._drag_target == "wheel":
      self._set_from_wheel(mouse_event.pos)
    elif self._drag_target == "value":
      self._set_from_value(mouse_event.pos)
