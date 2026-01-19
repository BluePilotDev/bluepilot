import pyray as rl
import qrcode
import numpy as np
import subprocess
from typing import Callable

from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.ui.widgets.button import Button, ButtonStyle
from openpilot.system.ui.widgets.list_view import toggle_item


class WebServerQRDialogTici(Widget):
  """Dialog showing QR code for webserver access and toggle to disable (TICI version)."""

  def __init__(self):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
    self._params = Params()
    self._qr_texture: rl.Texture | None = None
    self._last_url = ""
    
    # Toggle to disable server
    self._disable_toggle = toggle_item(
      lambda: "Web Routes Server",
      lambda: "Enable/disable the web routes server.",
      initial_state=self._params.get_bool("BPPortalEnabled"),
      callback=self._handle_toggle,
    )
    
    # Close button
    self._close_button = Button(
      lambda: "Close",
      click_callback=lambda: self._set_result(DialogResult.CANCEL),
      button_style=ButtonStyle.PRIMARY
    )
    
    self._result: DialogResult = DialogResult.NO_ACTION
    self._font_medium = gui_app.font(FontWeight.MEDIUM)
    self._font_bold = gui_app.font(FontWeight.BOLD)

  def _get_wifi_ip(self) -> str:
    """Get WiFi interface IP address."""
    try:
      # Try using ip command (works on Linux/AGNOS)
      result = subprocess.run(['ip', 'addr', 'show', 'wlan0'],
                              capture_output=True, text=True, timeout=2)
      for line in result.stdout.split('\n'):
        if 'inet ' in line:
          ip = line.strip().split()[1].split('/')[0]
          if ip and not ip.startswith('127.'):
            return ip
    except Exception as e:
      cloudlog.warning(f"Failed to get WiFi IP: {e}")
    
    # Fallback: try other wlan interfaces
    try:
      for iface in ['wlan1', 'wlan2']:
        result = subprocess.run(['ip', 'addr', 'show', iface],
                                capture_output=True, text=True, timeout=2)
        for line in result.stdout.split('\n'):
          if 'inet ' in line:
            ip = line.strip().split()[1].split('/')[0]
            if ip and not ip.startswith('127.'):
              return ip
    except:
      pass
    
    return ""

  def _get_server_url(self) -> str:
    """Get the full server URL for QR code."""
    wifi_ip = self._get_wifi_ip()
    if not wifi_ip:
      return ""
    
    port = self._params.get("BPPortalPort") or "8088"
    return f"http://{wifi_ip}:{port}"

  def _generate_qr_code(self) -> None:
    """Generate QR code texture from server URL."""
    url = self._get_server_url()
    if not url:
      self._qr_texture = None
      return
    
    # Only regenerate if URL changed
    if url == self._last_url and self._qr_texture:
      return
    
    self._last_url = url
    
    try:
      qr = qrcode.QRCode(version=1, error_correction=qrcode.constants.ERROR_CORRECT_L, box_size=10, border=0)
      qr.add_data(url)
      qr.make(fit=True)

      pil_img = qr.make_image(fill_color="white", back_color="black").convert('RGBA')
      img_array = np.array(pil_img, dtype=np.uint8)

      if self._qr_texture and self._qr_texture.id != 0:
        rl.unload_texture(self._qr_texture)

      rl_image = rl.Image()
      rl_image.data = rl.ffi.cast("void *", img_array.ctypes.data)
      rl_image.width = pil_img.width
      rl_image.height = pil_img.height
      rl_image.mipmaps = 1
      rl_image.format = rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8

      self._qr_texture = rl.load_texture_from_image(rl_image)
    except Exception as e:
      cloudlog.warning(f"QR code generation failed: {e}")
      self._qr_texture = None

  def _handle_toggle(self, checked: bool):
    """Handle toggle click - if disabled, close dialog."""
    self._params.put_bool("BPPortalEnabled", checked)
    if not checked:
      # Server was disabled, close the dialog
      self._set_result(DialogResult.CANCEL)

  def _set_result(self, result: DialogResult):
    self._result = result

  def _render(self, rect: rl.Rectangle) -> int:
    self._generate_qr_code()
    
    # Dialog background
    margin = 100
    dialog_rect = rl.Rectangle(
      rect.x + margin,
      rect.y + margin,
      rect.width - 2 * margin,
      rect.height - 2 * margin
    )
    rl.draw_rectangle_rounded(dialog_rect, 0.02, 20, rl.Color(30, 30, 30, 255))
    
    # Content area
    content_margin = 50
    content_rect = rl.Rectangle(
      dialog_rect.x + content_margin,
      dialog_rect.y + content_margin,
      dialog_rect.width - 2 * content_margin,
      dialog_rect.height - 2 * content_margin
    )
    
    # Title
    title_y = content_rect.y
    gui_label(
      rl.Rectangle(content_rect.x, title_y, content_rect.width, 80),
      "Web Routes Server",
      font_size=70,
      font_weight=FontWeight.BOLD,
      color=rl.WHITE
    )
    
    # Layout: QR code on left, controls on right
    qr_size = min(600, (content_rect.width - content_margin) // 2)
    qr_x = content_rect.x
    qr_y = title_y + 100
    
    right_x = qr_x + qr_size + content_margin
    right_width = content_rect.width - qr_size - content_margin
    
    # Render QR code on left
    self._render_qr_code(rl.Rectangle(qr_x, qr_y, qr_size, qr_size))
    
    # Render URL label below QR code
    url = self._get_server_url()
    if url:
      gui_label(
        rl.Rectangle(qr_x, qr_y + qr_size + 20, qr_size, 50),
        url,
        font_size=40,
        font_weight=FontWeight.MEDIUM,
        color=rl.Color(200, 200, 200, 255)
      )
      
      gui_label(
        rl.Rectangle(qr_x, qr_y + qr_size + 70, qr_size, 40),
        "Scan to connect",
        font_size=35,
        font_weight=FontWeight.MEDIUM,
        color=rl.Color(150, 150, 150, 255)
      )
    else:
      # Show error if no IP
      gui_label(
        rl.Rectangle(qr_x, qr_y + qr_size // 2, qr_size, 50),
        "No WiFi connection",
        font_size=40,
        font_weight=FontWeight.MEDIUM,
        color=rl.RED
      )
    
    # Render toggle on right
    toggle_y = qr_y
    toggle_rect = rl.Rectangle(right_x, toggle_y, right_width, 170)
    self._disable_toggle.set_rect(toggle_rect)
    self._disable_toggle.render()
    
    # Close button at bottom
    button_height = 100
    button_y = content_rect.y + content_rect.height - button_height
    close_rect = rl.Rectangle(
      content_rect.x,
      button_y,
      content_rect.width,
      button_height
    )
    self._close_button.render(close_rect)
    
    return self._result

  def _render_qr_code(self, rect: rl.Rectangle) -> None:
    """Render QR code texture."""
    if not self._qr_texture:
      gui_label(
        rl.Rectangle(rect.x, rect.y + rect.height // 2 - 25, rect.width, 50),
        "QR Code Error",
        font_size=40,
        font_weight=FontWeight.BOLD,
        color=rl.RED
      )
      return

    scale = rect.height / self._qr_texture.height
    pos = rl.Vector2(rect.x, rect.y)
    rl.draw_texture_ex(self._qr_texture, pos, 0.0, scale, rl.WHITE)

  def _handle_mouse_release(self, mouse_pos):
    """Handle mouse clicks."""
    # Let the toggle handle its own clicks
    toggle_rect = self._disable_toggle._rect
    if toggle_rect:
      if (toggle_rect.x <= mouse_pos.x <= toggle_rect.x + toggle_rect.width and
          toggle_rect.y <= mouse_pos.y <= toggle_rect.y + toggle_rect.height):
        self._disable_toggle._handle_mouse_release(mouse_pos)
        return
    
    # Let close button handle its own clicks
    close_rect = self._close_button._rect
    if close_rect:
      if (close_rect.x <= mouse_pos.x <= close_rect.x + close_rect.width and
          close_rect.y <= mouse_pos.y <= close_rect.y + close_rect.height):
        self._close_button._handle_mouse_release(mouse_pos)
        return
    
    super()._handle_mouse_release(mouse_pos)

  def __del__(self):
    if self._qr_texture and self._qr_texture.id != 0:
      rl.unload_texture(self._qr_texture)
