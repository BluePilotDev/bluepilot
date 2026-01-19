import pyray as rl
import qrcode
import numpy as np
import subprocess
from typing import Callable

from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.system.ui.widgets import NavWidget
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.widgets.label import MiciLabel
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl


class WebServerQRDialog(NavWidget):
  """Dialog showing QR code for webserver access and toggle to disable."""

  def __init__(self, back_callback: Callable):
    super().__init__()
    self.set_back_callback(back_callback)
    self.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
    self._params = Params()
    self._qr_texture: rl.Texture | None = None
    self._last_url = ""
    
    # Toggle to disable server (initially enabled since dialog shows when server is on)
    self._disable_toggle = BigParamControl("web routes server", "BPPortalEnabled",
                                           toggle_callback=self._handle_toggle)
    # Ensure toggle reflects current state
    self._disable_toggle.refresh()
    
    # Labels
    self._title_label = MiciLabel("web routes server", 56, font_weight=FontWeight.BOLD,
                                  color=rl.Color(255, 255, 255, int(255 * 0.9)), line_height=50)
    self._url_label = MiciLabel("", 36, font_weight=FontWeight.MEDIUM,
                                color=rl.Color(200, 200, 200, int(255 * 0.8)), line_height=35)
    self._scan_label = MiciLabel("scan to connect", 32, font_weight=FontWeight.MEDIUM,
                                 color=rl.Color(150, 150, 150, int(255 * 0.7)), line_height=30)

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
    if not checked:
      # Server was disabled, close the dialog
      if self._back_callback:
        self._back_callback()

  def _render(self, rect: rl.Rectangle) -> int:
    self._generate_qr_code()
    
    # Layout: QR code on left, controls on right
    padding = 24
    qr_size = min(self._rect.height - (padding * 2), (self._rect.width - padding * 3) // 2)
    qr_x = self._rect.x + padding
    qr_y = self._rect.y + padding
    
    right_x = qr_x + qr_size + padding
    right_width = self._rect.width - right_x - padding
    
    # Render QR code on left
    self._render_qr_code(rl.Rectangle(qr_x, qr_y, qr_size, qr_size))
    
    # Render URL label below QR code
    url = self._get_server_url()
    if url:
      self._url_label.set_text(url)
      self._url_label.set_width(int(qr_size))
      self._url_label.set_position(qr_x, qr_y + qr_size + 16)
      self._url_label.render()
      
      # Scan label
      self._scan_label.set_width(int(qr_size))
      self._scan_label.set_position(qr_x, qr_y + qr_size + 16 + 40)
      self._scan_label.render()
    else:
      # Show error if no IP
      error_font = gui_app.font(FontWeight.MEDIUM)
      rl.draw_text_ex(
        error_font, "No WiFi connection", 
        rl.Vector2(qr_x + 20, qr_y + qr_size // 2 - 15), 
        32, 0.0, rl.RED
      )
    
    # Render title and toggle on right
    title_y = self._rect.y + padding
    self._title_label.set_width(int(right_width))
    self._title_label.set_position(right_x, title_y)
    self._title_label.render()
    
    # Toggle below title
    toggle_y = title_y + 80
    toggle_rect = rl.Rectangle(right_x, toggle_y, right_width, 60)
    self._disable_toggle.set_rect(toggle_rect)
    self._disable_toggle.render()
    
    return -1

  def _render_qr_code(self, rect: rl.Rectangle) -> None:
    """Render QR code texture."""
    if not self._qr_texture:
      error_font = gui_app.font(FontWeight.BOLD)
      rl.draw_text_ex(
        error_font, "QR Code Error", 
        rl.Vector2(rect.x + 20, rect.y + rect.height // 2 - 15), 
        30, 0.0, rl.RED
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
        # Refresh toggle state after click
        self._disable_toggle.refresh()
        return
    
    super()._handle_mouse_release(mouse_pos)

  def __del__(self):
    if self._qr_texture and self._qr_texture.id != 0:
      rl.unload_texture(self._qr_texture)
