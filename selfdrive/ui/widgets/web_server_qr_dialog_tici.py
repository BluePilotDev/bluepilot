import pyray as rl
import qrcode
import numpy as np
import subprocess

from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.ui.widgets.button import IconButton
from openpilot.system.ui.widgets.toggle import Toggle


class WebServerQRDialogTici(Widget):
  """Dialog showing QR code for webserver access and toggle to disable (TICI version)."""

  def __init__(self):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
    self._params = Params()
    self._qr_texture: rl.Texture | None = None
    self._last_url = ""
    
    # Toggle to disable server
    self._disable_toggle = Toggle(
      initial_state=self._params.get_bool("BPPortalEnabled"),
      callback=self._handle_toggle
    )
    
    # Close button
    self._close_btn = IconButton(gui_app.texture("icons/close.png", 80, 80))
    self._close_btn.set_click_callback(lambda: gui_app.set_modal_overlay(None))

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
      qr = qrcode.QRCode(version=1, error_correction=qrcode.constants.ERROR_CORRECT_L, box_size=10, border=4)
      qr.add_data(url)
      qr.make(fit=True)

      pil_img = qr.make_image(fill_color="black", back_color="white").convert('RGBA')
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
      gui_app.set_modal_overlay(None)

  def _render(self, rect: rl.Rectangle) -> int:
    rl.clear_background(rl.Color(224, 224, 224, 255))
    
    self._generate_qr_code()
    
    margin = 70
    content_rect = rl.Rectangle(rect.x + margin, rect.y + margin, rect.width - 2 * margin, rect.height - 2 * margin)
    y = content_rect.y

    # Close button
    close_size = 80
    pad = 20
    close_rect = rl.Rectangle(content_rect.x - pad, y - pad, close_size + pad * 2, close_size + pad * 2)
    self._close_btn.render(close_rect)

    y += close_size + 40

    # Title
    title = "Web Routes Server"
    title_font = gui_app.font(FontWeight.NORMAL)
    left_width = int(content_rect.width * 0.5 - 15)
    
    rl.draw_text_ex(title_font, title, rl.Vector2(content_rect.x, y), 75, 0.0, rl.BLACK)
    y += 100

    # Two columns: QR code and controls
    remaining_height = content_rect.height - (y - content_rect.y)
    right_width = content_rect.width // 2 - 20

    # QR code on right
    qr_size = min(right_width, remaining_height) - 40
    qr_x = content_rect.x + left_width + 40 + (right_width - qr_size) // 2
    qr_y = y
    self._render_qr_code(rl.Rectangle(qr_x, qr_y, qr_size, qr_size))
    
    # URL below QR code
    url = self._get_server_url()
    if url:
      url_y = qr_y + qr_size + 20
      gui_label(
        rl.Rectangle(qr_x, url_y, qr_size, 50),
        url,
        font_size=40,
        font_weight=FontWeight.MEDIUM,
        color=rl.BLACK
      )
      
      gui_label(
        rl.Rectangle(qr_x, url_y + 50, qr_size, 40),
        "Scan to connect",
        font_size=35,
        font_weight=FontWeight.MEDIUM,
        color=rl.Color(100, 100, 100, 255)
      )

    # Toggle on left
    toggle_y = y + 50
    toggle_rect = rl.Rectangle(content_rect.x, toggle_y, left_width, 80)
    self._disable_toggle.set_rect(toggle_rect)
    self._disable_toggle.render()
    
    # Toggle label
    gui_label(
      rl.Rectangle(content_rect.x, toggle_y + 90, left_width, 50),
      "Enable Web Routes Server",
      font_size=45,
      font_weight=FontWeight.MEDIUM,
      color=rl.BLACK
    )

    return -1

  def _render_qr_code(self, rect: rl.Rectangle) -> None:
    """Render QR code texture."""
    if not self._qr_texture:
      rl.draw_rectangle_rounded(rect, 0.1, 20, rl.Color(240, 240, 240, 255))
      error_font = gui_app.font(FontWeight.BOLD)
      rl.draw_text_ex(
        error_font, "QR Code Error", 
        rl.Vector2(rect.x + 20, rect.y + rect.height // 2 - 15), 
        30, 0.0, rl.RED
      )
      return

    source = rl.Rectangle(0, 0, self._qr_texture.width, self._qr_texture.height)
    rl.draw_texture_pro(self._qr_texture, source, rect, rl.Vector2(0, 0), 0, rl.WHITE)

  def __del__(self):
    if self._qr_texture and self._qr_texture.id != 0:
      rl.unload_texture(self._qr_texture)
