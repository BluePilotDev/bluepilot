"""
BluePilot Other Debug Panel
Displays vehicle state, radar, tuning, firmware, and device information
in card-based sub-tab layout. Port of Qt OtherDebugPanel.
"""

import time
import pyray as rl
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.ui_state import ui_state
from bluepilot.ui.lib.colors import BPColors
from bluepilot.ui.widgets.debug.debug_colors import DebugColors


# --- DebugDataCard ---

class DebugDataCard(Widget):
  """A card displaying a titled group of label-value rows."""

  HEADER_HEIGHT = 58
  ROW_HEIGHT = 48
  PADDING = 14
  CARD_RADIUS = 0.06

  def __init__(self, title: str, accent_color: rl.Color):
    super().__init__()
    self._title = title
    self._accent_color = accent_color
    self._rows: list[tuple[str, str]] = []
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font_semi = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_normal = gui_app.font(FontWeight.NORMAL)

  def set_rows(self, rows: list[tuple[str, str]]):
    self._rows = rows

  def get_height(self) -> float:
    return self.HEADER_HEIGHT + len(self._rows) * self.ROW_HEIGHT + self.PADDING * 2

  def _render(self, rect: rl.Rectangle):
    # Shadow
    shadow = rl.Rectangle(rect.x + 2, rect.y + 2, rect.width, rect.height)
    rl.draw_rectangle_rounded(shadow, self.CARD_RADIUS, 8, BPColors.SHADOW)

    # Card background
    rl.draw_rectangle_rounded(rect, self.CARD_RADIUS, 8, DebugColors.CARD_BG)

    # Accent bar on left (clipped)
    rl.begin_scissor_mode(int(rect.x), int(rect.y), 5, int(rect.height))
    rl.draw_rectangle_rounded(rect, self.CARD_RADIUS, 8, self._accent_color)
    rl.end_scissor_mode()

    # Header title
    rl.draw_text_ex(self._font_bold, self._title,
                    rl.Vector2(rect.x + self.PADDING, rect.y + 10),
                    48, 0, self._accent_color)

    # Separator line under header
    sep_y = rect.y + self.HEADER_HEIGHT
    rl.draw_line(int(rect.x + self.PADDING), int(sep_y),
                 int(rect.x + rect.width - self.PADDING), int(sep_y),
                 DebugColors.SEPARATOR)

    # Rows
    y = sep_y + 6
    for label, value in self._rows:
      # Label on left
      rl.draw_text_ex(self._font_normal, label,
                      rl.Vector2(rect.x + self.PADDING, y),
                      40, 0, DebugColors.LEGEND_LABEL)
      # Value on right
      val_size = measure_text_cached(self._font_semi, value, 40)
      rl.draw_text_ex(self._font_semi, value,
                      rl.Vector2(rect.x + rect.width - self.PADDING - val_size.x, y),
                      40, 0, DebugColors.LEGEND_TEXT)
      y += self.ROW_HEIGHT


# --- Sub-tab definitions ---

SUB_TAB_LABELS = ["Main", "Radar", "Tuning", "Firmware", "Device"]

# Update rates per sub-tab (seconds)
UPDATE_RATES = {
  0: 0.05,     # Main: 20Hz
  1: 0.05,     # Radar: 20Hz
  2: 0.2,      # Tuning: 5Hz
  3: 30.0,     # Firmware: every 30s
  4: 0.5,      # Device: 2Hz
}


def _fmt_bool(val: bool, true_text: str = "Yes", false_text: str = "No") -> str:
  return true_text if val else false_text


def _fmt_gear(gear) -> str:
  names = {"unknown": "Unknown", "park": "Park", "drive": "Drive", "neutral": "Neutral",
           "reverse": "Reverse", "sport": "Sport", "low": "Low", "brake": "Brake",
           "eco": "Eco", "manumatic": "Manual"}
  return names.get(str(gear), str(gear))


def _fmt_long_state(state) -> str:
  names = {"off": "Off", "pid": "PID", "stopping": "Stopping", "starting": "Starting"}
  return names.get(str(state), str(state))


def _fmt_safety(model) -> str:
  names = {"silent": "Silent", "hondaNidec": "Honda Nidec", "toyota": "Toyota",
           "elm327": "ELM327", "gm": "GM", "hondaBoschGiraffe": "Honda Bosch Giraffe",
           "ford": "Ford", "hyundai": "Hyundai", "chrysler": "Chrysler", "tesla": "Tesla",
           "subaru": "Subaru", "mazda": "Mazda", "nissan": "Nissan",
           "volkswagenMqb": "Volkswagen MQB", "allOutput": "ALL OUTPUT",
           "gmAscm": "GM ASCM", "noOutput": "No OUTPUT", "hondaBosch": "Honda Bosch",
           "volkswagenPq": "Volkswagen PQ", "subaruPreglobal": "Subaru Preglobal",
           "hyundaiLegacy": "Hyundai Legacy", "hyundaiCommunity": "Hyundai Community",
           "stellantis": "Stellantis", "faw": "FAW", "body": "Body",
           "hyundaiCanfd": "Hyundai CAN-FD", "rivian": "Rivian"}
  return names.get(str(model), str(model))


def _fmt_thermal(status) -> str:
  names = {"green": "Green", "yellow": "Yellow", "red": "Red", "danger": "Danger"}
  return names.get(str(status), str(status))


def _fmt_net_type(net_type) -> str:
  names = {"none": "None", "wifi": "WiFi", "cell2G": "2G", "cell3G": "3G",
           "cell4G": "4G/LTE", "cell5G": "5G", "ethernet": "Ethernet"}
  return names.get(str(net_type), str(net_type))


def _fmt_ecu(ecu) -> str:
  names = {"eps": "EPS", "abs": "ABS", "fwdRadar": "Fwd Radar", "fwdCamera": "Fwd Camera",
           "engine": "Engine", "unknown": "Unknown", "dsu": "DSU", "parkingAdas": "Parking ADAS",
           "transmission": "Transmission", "srs": "SRS", "gateway": "Gateway", "hud": "HUD",
           "combinationMeter": "Combination Meter", "vsa": "VSA", "programmedFuelInjection": "PFI",
           "electricBrakeBooster": "Elec Brake Booster", "shiftByWire": "Shift By Wire",
           "debug": "Debug", "hybrid": "Hybrid", "adas": "ADAS", "hvac": "HVAC",
           "cornerRadar": "Corner Radar", "epb": "EPB", "telematics": "Telematics", "body": "Body"}
  return names.get(str(ecu), str(ecu))


def _decode_fw_version(fw_version) -> str:
  """Decode firmware version bytes into a printable string (part number)."""
  try:
    if hasattr(fw_version, 'hex'):
      raw = bytes(fw_version)
      # Try to decode as ASCII/UTF-8, filtering non-printable chars
      decoded = ''.join(c for c in raw.decode('ascii', errors='replace') if c.isprintable())
      decoded = decoded.strip()
      if decoded:
        return decoded
      # Fall back to hex if no printable content
      return raw.hex()
    return str(fw_version)
  except Exception:
    return str(fw_version)



# --- OtherDebugPanel ---

class OtherDebugPanel(Widget):
  """Vehicle/device data panel with 5 sub-tabs rendered as scrollable card lists."""

  SUB_TAB_HEIGHT = 66

  def __init__(self):
    super().__init__()
    self._current_sub_tab = 0
    self._last_update_times: dict[int, float] = {i: 0.0 for i in range(len(SUB_TAB_LABELS))}
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font_semi = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_normal = gui_app.font(FontWeight.NORMAL)

    # Scroll state per tab
    self._scroll_offsets: dict[int, float] = {i: 0.0 for i in range(len(SUB_TAB_LABELS))}
    self._scroll_velocity: float = 0.0
    self._is_dragging: bool = False
    self._drag_start_y: float = 0.0
    self._drag_start_offset: float = 0.0

    # Cards per sub-tab
    self._tab_cards: dict[int, list[DebugDataCard]] = {}
    self._setup_cards()

  def _setup_cards(self):
    """Create DebugDataCard instances for each sub-tab."""
    # Tab 0: Main (Car State)
    self._tab_cards[0] = [
      DebugDataCard("Dynamics", DebugColors.ACCENT_DYNAMICS),
      DebugDataCard("Steering", DebugColors.ACCENT_STEERING),
      DebugDataCard("Pedals", DebugColors.ACCENT_PEDALS),
      DebugDataCard("Vehicle Systems", DebugColors.ACCENT_SYSTEMS),
      DebugDataCard("Cruise Control", DebugColors.ACCENT_CRUISE),
      DebugDataCard("Safety", DebugColors.ACCENT_SAFETY),
      DebugDataCard("Actuator Output", DebugColors.ACCENT_DYNAMICS),
    ]
    # Tab 1: Radar
    self._tab_cards[1] = [
      DebugDataCard("Lead 1", DebugColors.ACCENT_RADAR),
      DebugDataCard("Lead 2", DebugColors.ACCENT_RADAR),
      DebugDataCard("Radar Errors", DebugColors.ACCENT_SAFETY),
    ]
    # Tab 2: Tuning
    self._tab_cards[2] = [
      DebugDataCard("Car Parameters", DebugColors.ACCENT_TUNING),
      DebugDataCard("Lateral Tuning", DebugColors.ACCENT_STEERING),
      DebugDataCard("Longitudinal Tuning", DebugColors.ACCENT_DYNAMICS),
    ]
    # Tab 3: Firmware
    self._tab_cards[3] = [
      DebugDataCard("Identification", DebugColors.ACCENT_FIRMWARE),
      DebugDataCard("Firmware Versions", DebugColors.ACCENT_FIRMWARE),
    ]
    # Tab 4: Device
    self._tab_cards[4] = [
      DebugDataCard("System Utilization", DebugColors.ACCENT_DEVICE),
      DebugDataCard("Power", DebugColors.ACCENT_SYSTEMS),
      DebugDataCard("Temperatures", DebugColors.ACCENT_SAFETY),
      DebugDataCard("Network", DebugColors.ACCENT_RADAR),
    ]

  def _update_state(self):
    now = time.monotonic()
    tab = self._current_sub_tab
    rate = UPDATE_RATES.get(tab, 0.2)
    if now - self._last_update_times.get(tab, 0.0) >= rate:
      self._last_update_times[tab] = now
      self._update_tab_data(tab)

  def _update_tab_data(self, tab: int):
    sm = ui_state.sm
    if sm is None:
      return

    try:
      if tab == 0:
        self._update_main(sm)
      elif tab == 1:
        self._update_radar(sm)
      elif tab == 2:
        self._update_tuning(sm)
      elif tab == 3:
        self._update_firmware(sm)
      elif tab == 4:
        self._update_device(sm)
    except (KeyError, AttributeError, ValueError, IndexError):
      pass

  def _update_main(self, sm):
    cards = self._tab_cards[0]

    if sm.valid.get('carState', False):
      cs = sm['carState']
      cards[0].set_rows([
        ("Speed", f"{cs.vEgo:.1f} m/s ({cs.vEgo * 2.237:.1f} mph)"),
        ("Raw Speed", f"{cs.vEgoRaw:.2f} m/s"),
        ("Acceleration", f"{cs.aEgo:.2f} m/s\u00b2"),
        ("Yaw Rate", f"{cs.yawRate:.4f} rad/s"),
        ("Standstill", _fmt_bool(cs.standstill)),
      ])
      cards[1].set_rows([
        ("Angle", f"{cs.steeringAngleDeg:.1f}\u00b0"),
        ("Rate", f"{cs.steeringRateDeg:.1f}\u00b0/s"),
        ("Torque", f"{cs.steeringTorque:.2f}"),
        ("EPS Torque", f"{cs.steeringTorqueEps:.2f}"),
        ("Driver Steering", _fmt_bool(cs.steeringPressed)),
        ("Fault (Temp)", _fmt_bool(cs.steerFaultTemporary)),
        ("Fault (Perm)", _fmt_bool(cs.steerFaultPermanent)),
      ])
      cards[2].set_rows([
        ("Brake Pressed", _fmt_bool(cs.brakePressed)),
        ("Gas Pressed", _fmt_bool(cs.gasPressed)),
        ("Regen Braking", _fmt_bool(cs.regenBraking)),
        ("Parking Brake", _fmt_bool(cs.parkingBrake)),
        ("Brake Hold", _fmt_bool(cs.brakeHoldActive)),
      ])
      cards[3].set_rows([
        ("ESP Disabled", _fmt_bool(cs.espDisabled)),
        ("ESP Active", _fmt_bool(cs.espActive)),
        ("Left Blinker", _fmt_bool(cs.leftBlinker)),
        ("Right Blinker", _fmt_bool(cs.rightBlinker)),
        ("Gear", _fmt_gear(cs.gearShifter)),
        ("Fuel Gauge", f"{cs.fuelGauge:.1%}"),
        ("Charging", _fmt_bool(cs.charging)),
      ])
      cruise = cs.cruiseState
      cards[4].set_rows([
        ("Enabled", _fmt_bool(cruise.enabled)),
        ("Speed", f"{cruise.speed:.1f} m/s"),
        ("Available", _fmt_bool(cruise.available)),
        ("Standstill", _fmt_bool(cruise.standstill)),
        ("Non-Adaptive", _fmt_bool(cruise.nonAdaptive)),
        ("vCruise", f"{cs.vCruise:.1f} m/s"),
      ])
      cards[5].set_rows([
        ("Stock AEB", _fmt_bool(cs.stockAeb)),
        ("Stock FCW", _fmt_bool(cs.stockFcw)),
        ("Invalid LKAS", _fmt_bool(cs.invalidLkasSetting)),
        ("Door Open", _fmt_bool(cs.doorOpen)),
        ("Seatbelt", _fmt_bool(cs.seatbeltUnlatched, "Unlatched", "Latched")),
        ("Sensors Invalid", _fmt_bool(cs.vehicleSensorsInvalid)),
      ])

    if sm.valid.get('carOutput', False):
      try:
        co = sm['carOutput']
        ao = co.actuatorsOutput
        cards[6].set_rows([
          ("Accel", f"{ao.accel:.3f}"),
          ("Gas", f"{ao.gas:.3f}"),
          ("Brake", f"{ao.brake:.3f}"),
          ("Steer Angle", f"{ao.steeringAngleDeg:.1f}\u00b0"),
          ("Torque", f"{ao.torque:.3f}"),
          ("Curvature", f"{ao.curvature:.5f}"),
          ("Long State", _fmt_long_state(ao.longControlState)),
        ])
      except (AttributeError, ValueError):
        cards[6].set_rows([("Status", "No actuator output")])

  def _update_radar(self, sm):
    cards = self._tab_cards[1]

    if sm.valid.get('radarState', False):
      rs = sm['radarState']

      def lead_rows(lead):
        return [
          ("Distance", f"{lead.dRel:.1f} m"),
          ("Y Position", f"{lead.yRel:.2f} m"),
          ("Rel. Speed", f"{lead.vRel:.2f} m/s"),
          ("Lead Speed", f"{lead.vLead:.1f} m/s"),
          ("Status", _fmt_bool(lead.present, "Tracking", "No Lead")),
          ("Radar", _fmt_bool(lead.radar, "Yes", "Vision")),
        ]

      cards[0].set_rows(lead_rows(rs.leadOne))
      cards[1].set_rows(lead_rows(rs.leadTwo))

      errors = rs.radarErrors
      cards[2].set_rows([
        ("CAN Error", _fmt_bool(errors.canError)),
        ("Radar Fault", _fmt_bool(errors.radarFault)),
        ("Wrong Config", _fmt_bool(errors.wrongConfig)),
        ("Temp Unavail", _fmt_bool(errors.radarUnavailableTemporary)),
      ])

  def _update_tuning(self, sm):
    cards = self._tab_cards[2]

    if sm.valid.get('carParams', False):
      cp = sm['carParams']
      cards[0].set_rows([
        ("Wheelbase", f"{cp.wheelbase:.3f} m"),
        ("Steer Ratio", f"{cp.steerRatio:.2f}"),
        ("Steer Delay", f"{cp.steerActuatorDelay:.3f} s"),
        ("Long Delay", f"{cp.longitudinalActuatorDelay:.3f} s"),
        ("Tire Stiffness", f"{cp.tireStiffnessFactor:.3f}"),
        ("Steer Limit Timer", f"{cp.steerLimitTimer:.2f} s"),
        ("Radar Unavail", _fmt_bool(cp.radarUnavailable)),
      ])

      # Lateral tuning
      lat_rows = []
      try:
        lat_tuning = cp.lateralTuning
        which = lat_tuning.which()
        if which == 'torque':
          t = lat_tuning.torque
          lat_rows = [
            ("Type", "Torque"),
            ("Friction", f"{t.friction:.4f}"),
            ("Lat Accel Factor", f"{t.latAccelFactor:.4f}"),
            ("Lat Accel Offset", f"{t.latAccelOffset:.4f}"),
          ]
        elif which == 'pid':
          p = lat_tuning.pid
          kp_v = list(p.kpV) if hasattr(p, 'kpV') else []
          ki_v = list(p.kiV) if hasattr(p, 'kiV') else []
          lat_rows = [
            ("Type", "PID"),
            ("Kp", f"{kp_v[0]:.4f}" if kp_v else "N/A"),
            ("Ki", f"{ki_v[0]:.4f}" if ki_v else "N/A"),
            ("Kf", f"{p.kf:.4f}"),
          ]
        else:
          lat_rows = [("Type", str(which))]
      except (AttributeError, IndexError):
        lat_rows = [("Status", "N/A")]
      cards[1].set_rows(lat_rows)

      # Longitudinal tuning
      long_rows = []
      try:
        lt = cp.longitudinalTuning
        ki_bp = [f"{v:.1f}" for v in list(lt.kiBP)]
        ki_v = [f"{v:.4f}" for v in list(lt.kiV)]
        long_rows = [
          ("Ki BP", ", ".join(ki_bp) if ki_bp else "N/A"),
          ("Ki V", ", ".join(ki_v) if ki_v else "N/A"),
        ]
      except (AttributeError, IndexError):
        long_rows = [("Status", "N/A")]
      cards[2].set_rows(long_rows)

  def _update_firmware(self, sm):
    cards = self._tab_cards[3]

    if sm.valid.get('carParams', False):
      cp = sm['carParams']
      safety_model = "Unknown"
      safety_param = 0
      try:
        sc = list(cp.safetyConfigs)
        if sc:
          safety_model = str(sc[0].safetyModel)
          safety_param = sc[0].safetyParam
      except (AttributeError, IndexError):
        pass

      cards[0].set_rows([
        ("Fingerprint", str(cp.carFingerprint) if cp.carFingerprint else "Unknown"),
        ("VIN", str(cp.carVin) if cp.carVin else "Unknown"),
        ("Brand", str(cp.brand) if cp.brand else "Unknown"),
        ("Safety Model", str(safety_model)),
        ("Safety Param", str(safety_param)),
        ("Fuzzy FP", _fmt_bool(cp.fuzzyFingerprint)),
      ])

      # Firmware versions
      ecu_names = {"eps": "EPS", "abs": "ABS", "fwdRadar": "Fwd Radar", "fwdCamera": "Fwd Camera",
                   "engine": "Engine", "unknown": "Unknown", "dsu": "DSU", "parkingAdas": "Parking ADAS",
                   "transmission": "Transmission", "srs": "SRS", "gateway": "Gateway", "hud": "HUD",
                   "combinationMeter": "Combo Meter", "vsa": "VSA", "programmedFuelInjection": "PFI",
                   "electricBrakeBooster": "Elec Brake Boost", "shiftByWire": "Shift By Wire",
                   "debug": "Debug", "hybrid": "Hybrid", "adas": "ADAS", "hvac": "HVAC",
                   "cornerRadar": "Corner Radar", "epb": "EPB", "telematics": "Telematics", "body": "Body"}
      fw_rows = []
      seen = set()
      try:
        for fw in list(cp.carFw):
          raw_ecu = str(fw.ecu)
          addr = fw.address

          # Deduplicate by (ecu, address)
          key = (raw_ecu, addr)
          if key in seen:
            continue
          seen.add(key)

          # Decode firmware version bytes as ASCII part number
          try:
            raw_bytes = bytes(fw.fwVersion)
            version_str = ''.join(c for c in raw_bytes.decode('ascii', errors='replace') if c.isprintable()).strip()
          except Exception:
            version_str = ""

          # Skip entries that aren't real part numbers (must contain a dash)
          if not version_str or '-' not in version_str:
            continue

          ecu_name = ecu_names.get(raw_ecu, raw_ecu)
          if len(version_str) > 32:
            version_str = version_str[:29] + "..."

          addr_str = f"0x{addr:X}" if addr else ""
          bus_str = f"bus {fw.bus}" if hasattr(fw, 'bus') else ""
          detail = " | ".join(p for p in [version_str, addr_str, bus_str] if p)
          fw_rows.append((ecu_name, detail))
      except (AttributeError, IndexError):
        fw_rows = [("Status", "N/A")]
      cards[1].set_rows(fw_rows if fw_rows else [("Status", "No firmware data")])

  def _update_device(self, sm):
    cards = self._tab_cards[4]

    if sm.valid.get('deviceState', False):
      ds = sm['deviceState']

      # System utilization
      cpu_usage = list(ds.cpuUsagePercent) if hasattr(ds, 'cpuUsagePercent') else []
      avg_cpu = sum(cpu_usage) / len(cpu_usage) if cpu_usage else 0
      cards[0].set_rows([
        ("CPU Usage", f"{avg_cpu:.0f}%"),
        ("GPU Usage", f"{ds.gpuUsagePercent}%"),
        ("Memory Usage", f"{ds.memoryUsagePercent}%"),
        ("Free Space", f"{ds.freeSpacePercent:.1f}%"),
        ("Fan Speed", f"{ds.fanSpeedPercentDesired}%"),
        ("Brightness", f"{ds.screenBrightnessPercent}%"),
      ])

      # Power
      cards[1].set_rows([
        ("Power Draw", f"{ds.powerDrawW:.2f} W"),
        ("SoM Power", f"{ds.somPowerDrawW:.2f} W"),
        ("Car Battery", f"{ds.carBatteryCapacityUwh / 1000000:.1f} Wh"),
        ("Offroad Power", f"{ds.offroadPowerUsageUwh / 1000:.0f} mWh"),
      ])

      # Temperatures
      cpu_temps = list(ds.cpuTempC) if hasattr(ds, 'cpuTempC') else []
      gpu_temps = list(ds.gpuTempC) if hasattr(ds, 'gpuTempC') else []
      avg_cpu_temp = sum(cpu_temps) / len(cpu_temps) if cpu_temps else 0
      avg_gpu_temp = sum(gpu_temps) / len(gpu_temps) if gpu_temps else 0
      temp_rows = [
        ("CPU Temp", f"{avg_cpu_temp:.1f}\u00b0C"),
        ("GPU Temp", f"{avg_gpu_temp:.1f}\u00b0C"),
        ("Memory Temp", f"{ds.memoryTempC:.1f}\u00b0C"),
        ("Max Temp", f"{ds.maxTempC:.1f}\u00b0C"),
        ("Thermal Status", _fmt_thermal(ds.thermalStatus)),
      ]
      try:
        temp_rows.append(("Intake Temp", f"{ds.intakeTempC:.1f}\u00b0C"))
        temp_rows.append(("Exhaust Temp", f"{ds.exhaustTempC:.1f}\u00b0C"))
      except AttributeError:
        pass
      cards[2].set_rows(temp_rows)

      # Network
      net_rows = [
        ("Type", _fmt_net_type(ds.networkType)),
        ("Strength", f"{ds.networkStrength}/4"),
        ("Metered", _fmt_bool(ds.networkMetered)),
      ]
      try:
        ni = ds.networkInfo
        if hasattr(ni, 'technology') and ni.technology:
          net_rows.append(("Technology", str(ni.technology)))
        if hasattr(ni, 'operator') and ni.operator:
          net_rows.append(("Operator", str(ni.operator)))
        if hasattr(ni, 'band') and ni.band:
          net_rows.append(("Band", str(ni.band)))
        if hasattr(ni, 'state') and ni.state:
          net_rows.append(("State", str(ni.state)))
      except AttributeError:
        pass
      cards[3].set_rows(net_rows)

  def _render(self, rect: rl.Rectangle):
    # Sub-tab bar at top
    self._render_sub_tabs(rect)

    # Content area below sub-tab bar
    content_rect = rl.Rectangle(
      rect.x, rect.y + self.SUB_TAB_HEIGHT,
      rect.width, rect.height - self.SUB_TAB_HEIGHT
    )

    # Handle scrolling
    self._handle_scroll(content_rect)

    # Render scrollable card list
    rl.begin_scissor_mode(int(content_rect.x), int(content_rect.y),
                          int(content_rect.width), int(content_rect.height))
    self._render_cards(content_rect)
    rl.end_scissor_mode()

  def _render_sub_tabs(self, rect: rl.Rectangle):
    """Render pill-style sub-tab buttons at the top."""
    tab_count = len(SUB_TAB_LABELS)
    tab_spacing = 8
    total_spacing = (tab_count - 1) * tab_spacing
    available_w = rect.width - 20  # 10px padding on each side
    tab_w = (available_w - total_spacing) / tab_count
    tab_h = 52
    tab_y = rect.y + (self.SUB_TAB_HEIGHT - tab_h) / 2

    for i, label in enumerate(SUB_TAB_LABELS):
      tab_x = rect.x + 10 + i * (tab_w + tab_spacing)
      tab_rect = rl.Rectangle(tab_x, tab_y, tab_w, tab_h)

      is_active = (i == self._current_sub_tab)
      bg_color = DebugColors.TAB_ACTIVE if is_active else DebugColors.TAB_INACTIVE
      text_color = DebugColors.TAB_TEXT if is_active else DebugColors.TAB_TEXT_DIM

      rl.draw_rectangle_rounded(tab_rect, 0.4, 8, bg_color)
      if not is_active:
        rl.draw_rectangle_rounded_lines_ex(tab_rect, 0.4, 8, 1.0, DebugColors.TAB_BORDER)

      # Center text
      text_size = measure_text_cached(self._font_semi, label, 40)
      text_x = tab_x + (tab_w - text_size.x) / 2
      text_y = tab_y + (tab_h - text_size.y) / 2
      rl.draw_text_ex(self._font_semi, label,
                      rl.Vector2(text_x, text_y), 40, 0, text_color)

      # Click detection via mouse events
      for mouse_event in gui_app.mouse_events:
        if mouse_event.left_released:
          if rl.check_collision_point_rec(mouse_event.pos, tab_rect):
            if self._current_sub_tab != i:
              self._current_sub_tab = i
              self._scroll_offsets[i] = 0.0
              # Force immediate update for new tab
              self._last_update_times[i] = 0.0

  def _handle_scroll(self, content_rect: rl.Rectangle):
    """Handle touch/mouse scrolling of card list."""
    for mouse_event in gui_app.mouse_events:
      in_rect = rl.check_collision_point_rec(mouse_event.pos, content_rect)

      if mouse_event.left_pressed and in_rect:
        self._is_dragging = True
        self._drag_start_y = mouse_event.pos.y
        self._drag_start_offset = self._scroll_offsets.get(self._current_sub_tab, 0.0)
        self._scroll_velocity = 0.0
      elif mouse_event.left_down and self._is_dragging:
        delta = mouse_event.pos.y - self._drag_start_y
        self._scroll_offsets[self._current_sub_tab] = self._drag_start_offset + delta
        self._scroll_velocity = delta
      elif mouse_event.left_released:
        self._is_dragging = False

    # Apply velocity-based momentum when not dragging
    if not self._is_dragging and abs(self._scroll_velocity) > 0.5:
      self._scroll_offsets[self._current_sub_tab] = \
        self._scroll_offsets.get(self._current_sub_tab, 0.0) + self._scroll_velocity * 0.3
      self._scroll_velocity *= 0.92  # Friction
    elif not self._is_dragging:
      self._scroll_velocity = 0.0

    # Clamp scroll offset
    tab = self._current_sub_tab
    cards = self._tab_cards.get(tab, [])
    if cards:
      total_h = sum(c.get_height() + 10 for c in cards) + 10
      max_scroll = 0.0
      min_scroll = min(0.0, content_rect.height - total_h)
      self._scroll_offsets[tab] = max(min_scroll, min(max_scroll, self._scroll_offsets.get(tab, 0.0)))

  def _render_cards(self, content_rect: rl.Rectangle):
    """Render the cards for the current sub-tab with scroll offset."""
    tab = self._current_sub_tab
    cards = self._tab_cards.get(tab, [])
    if not cards:
      return

    scroll_y = self._scroll_offsets.get(tab, 0.0)
    card_spacing = 10
    card_margin = 10
    y = content_rect.y + card_margin + scroll_y

    for card in cards:
      card_h = card.get_height()
      card_rect = rl.Rectangle(
        content_rect.x + card_margin,
        y,
        content_rect.width - 2 * card_margin,
        card_h
      )

      # Only render if visible
      if card_rect.y + card_rect.height > content_rect.y and card_rect.y < content_rect.y + content_rect.height:
        card.render(card_rect)

      y += card_h + card_spacing
