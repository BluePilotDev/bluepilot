"""Draws BluePilot's UI offscreen for one screen family; run by test_ui_frames.py in its own process.

usage: ui_frames_child.py tici|mici
Prints one line `RESULTS <json>`: {case: {"error": traceback or None, "calls": {"Class.method": count}}}.
Exits NO_WINDOW (77) only when pyray can't load or raylib has no window after init; then the one
`NOWINDOW <reason>` line says why. Anything else that fails, init_window's own UI code included, is a crash.

raylib has one global window and BIG (tici) is read at import, so each family needs its own process.
Messages are synthetic (no route), and fake camera frames go through a local VisionIPC server so the
camera view sets the model transform and the model overlays really project and draw.
"""
import collections
import importlib
import json
import os
import sys
import time
import traceback

FAMILY = sys.argv[1]
if FAMILY == "tici":
  os.environ["BIG"] = "1"
os.environ["OFFSCREEN"] = "1"

import numpy as np

from openpilot.selfdrive.ui.bp.tests.test_ui_frames import NO_WINDOW

try:
  import pyray as rl
except (ImportError, OSError) as e:  # e.g. Linux with no display and no EGL for the headless backend
  print(f"NOWINDOW pyray can't load: {type(e).__name__}: {e}")
  sys.exit(NO_WINDOW)

from openpilot.common.prefix import OpenpilotPrefix

FRAMES = 8
CAMERA_W, CAMERA_H = 1928, 1208

# Draw methods counted per case (module, class, method). A case passes only if the ones it names were called.
COUNTED = {
  "tici": [
    ("openpilot.selfdrive.ui.bp.onroad.model_renderer_bp", "ModelRendererBP", "_draw_lane_lines"),
    ("openpilot.selfdrive.ui.bp.onroad.model_renderer_bp", "ModelRendererBP", "_draw_path"),
    ("openpilot.selfdrive.ui.bp.onroad.model_renderer_bp", "ModelRendererBP", "_update_leads"),
    ("openpilot.selfdrive.ui.bp.onroad.model_renderer_bp", "ModelRendererBP", "_draw_lead_indicator"),
    ("openpilot.selfdrive.ui.bp.onroad.chevron_metrics_bp", "ChevronMetricsBP", "draw_lead_status"),
    ("openpilot.selfdrive.ui.sunnypilot.onroad.blendful", "BlendfulRenderer", "draw"),
    ("openpilot.system.ui.lib.shader_polygon", None, "draw_polygon"),
    ("openpilot.selfdrive.ui.bp.onroad.hud_renderer_bp", "HudRendererBP", "_render"),
    ("openpilot.selfdrive.ui.bp.onroad.steering_wheel_bp", "SteeringWheelBP", "_render"),
    ("openpilot.selfdrive.ui.bp.onroad.rad_racer_theme", "RadRacerTheme", "render_background"),
    ("openpilot.selfdrive.ui.bp.onroad.rad_racer_theme", "RadRacerTheme", "render_foreground"),
    ("openpilot.selfdrive.ui.bp.onroad.blindspot_renderer", "BlindspotRendererMixin", "_draw_blindspot_screen_edges"),
    ("openpilot.selfdrive.ui.bp.widgets.section_header", "CollapsibleSectionHeader", "_render"),
    ("openpilot.selfdrive.ui.bp.widgets.float_control_item", "FloatControlAction", "_render"),
  ],
  "mici": [
    ("openpilot.selfdrive.ui.bp.mici.onroad.model_renderer_bp", "ModelRendererBP", "_draw_lane_lines"),
    ("openpilot.selfdrive.ui.bp.mici.onroad.model_renderer_bp", "ModelRendererBP", "_draw_path"),
    ("openpilot.selfdrive.ui.mici.onroad.model_renderer", "ModelRenderer", "_draw_lead_indicator"),
    ("openpilot.selfdrive.ui.sunnypilot.onroad.blendful", "BlendfulRenderer", "draw"),
    ("openpilot.system.ui.lib.shader_polygon", None, "draw_polygon"),
    ("openpilot.selfdrive.ui.bp.mici.onroad.hud_renderer_bp", "MiciHudRendererBP", "_draw_steering_wheel"),
    ("openpilot.selfdrive.ui.mici.onroad.hud_renderer", "HudRenderer", "_draw_model_source"),
    ("openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators", "BlindSpotIndicators", "render"),
    ("openpilot.selfdrive.ui.mici.onroad.long_indicator", "LongIndicator", "_render"),
    ("openpilot.selfdrive.ui.mici.onroad.navigation", "NavigationPanel", "_render"),
    ("openpilot.selfdrive.ui.bp.mici.onroad.complication", "MiciComplication", "_render_lead_speed"),
    ("openpilot.selfdrive.ui.bp.mici.onroad.complication", "MiciComplication", "_render_current_speed"),
    ("openpilot.selfdrive.ui.bp.mici.onroad.complication", "MiciComplication", "_render_lead_dist"),
    ("openpilot.selfdrive.ui.bp.mici.onroad.complication", "MiciComplication", "_render_lead_time"),
    ("openpilot.selfdrive.ui.bp.onroad.rad_racer_theme", "RadRacerTheme", "render_background"),
    ("openpilot.selfdrive.ui.bp.onroad.rad_racer_theme", "RadRacerTheme", "render_foreground"),
    ("openpilot.selfdrive.ui.bp.onroad.blindspot_renderer", "BlindspotRendererMixin", "_draw_blindspot_screen_edges"),
  ],
}

# The display params the cases touch; reset to their defaults before every case.
DISPLAY_PARAMS = ("FordPrefShowRadarLeadOverlay", "BPThemePack", "BPThemeAutoSeasonal", "mici_complication",
                  "BPSteeringWheelIconStyle", "BPAnimateSteeringWheel", "BpShowLateralControl", "BlindSpot",
                  "ShowBlindspotOverlay", "ShowBrakeStatus", "BPRainbowLines", "BPHideCameraView", "BPHideOnroadBorder",
                  "FordPrefHybridBatteryStatus", "FordPrefHybridPowerFlow", "FordPrefGaugeStyle", "RainbowMode",
                  "NavigationEnabled", "NavDestination")


def onroad_cases(family: str, theme_packs: list[str]) -> list[tuple[str, dict, dict, list[str]]]:
  """(name, params, message options, draw methods that must run; "!name": must not run)."""
  model = ["ModelRendererBP._draw_lane_lines", "ModelRendererBP._draw_path", "BlendfulRenderer.draw", "draw_polygon"]
  if family == "tici":
    hud = ["HudRendererBP._render", "SteeringWheelBP._render"]
    lead = ["ModelRendererBP._update_leads", "ChevronMetricsBP.draw_lead_status"]
    cases = [
      ("defaults", {}, {}, model + hud + lead),
      ("lead overlay off, openpilot long", {"FordPrefShowRadarLeadOverlay": False}, {"op_long": True},
       model + hud + ["ModelRendererBP._update_leads", "ModelRendererBP._draw_lead_indicator", "ChevronMetricsBP.draw_lead_status"]),
      ("lead overlay on, openpilot long", {}, {"op_long": True}, model + hud + lead),
      ("hybrid gauges flat", {"FordPrefHybridBatteryStatus": True, "FordPrefHybridPowerFlow": True}, {"hybrid": True}, model + hud),
      ("hybrid gauges arched", {"FordPrefHybridBatteryStatus": True, "FordPrefHybridPowerFlow": True, "FordPrefGaugeStyle": 1},
       {"hybrid": True}, model + hud),
    ]
  else:
    hud = ["MiciHudRendererBP._draw_steering_wheel", "HudRenderer._draw_model_source", "LongIndicator._render"]
    cases = [
      ("defaults", {}, {}, model + hud),
      ("openpilot long", {}, {"op_long": True}, model + hud + ["ModelRenderer._draw_lead_indicator"]),
    ]
    cases += [
      ("navigation panel", {"NavigationEnabled": True, "NavDestination": {"latitude": 40.0, "longitude": -83.0}}, {},
       model + hud + ["NavigationPanel._render"]),
      ("navigation panel, right blind spot",
       {"NavigationEnabled": True, "NavDestination": {"latitude": 40.0, "longitude": -83.0}, "ShowBlindspotOverlay": True},
       {"right_blind_spot": True, "red_right_edge": True}, model + hud + ["NavigationPanel._render"]),
      ("not started", {}, {"started": False}, ["!ModelRendererBP._draw_lane_lines", "!MiciHudRendererBP._draw_steering_wheel"]),
      ("model source icon", {}, {"chestnut": True}, model + hud + ["model source icon drawn"]),
    ]
    renders = {1: "_render_lead_speed", 2: "_render_current_speed", 3: "_render_lead_dist", 4: "_render_lead_time"}
    for value, method in renders.items():
      cases.append((f"complication {value}", {"mici_complication": value}, {}, model + hud + [f"MiciComplication.{method}"]))
  cases += [
    ("rad racer", {"BPThemePack": "rad_racer"}, {}, ["RadRacerTheme.render_background", "RadRacerTheme.render_foreground"]),
    ("rad racer, openpilot long", {"BPThemePack": "rad_racer"}, {"op_long": True},
     ["RadRacerTheme.render_background", "RadRacerTheme.render_foreground"] +
     (["!ModelRenderer._draw_lead_indicator"] if family == "mici" else [])),
    ("wheel comma four, still", {"BPSteeringWheelIconStyle": 0, "BPAnimateSteeringWheel": False}, {}, model + hud),
    ("wheel comma 3X, animated", {"BPSteeringWheelIconStyle": 1, "BPAnimateSteeringWheel": True}, {}, model + hud),
    ("wheel comma 3X, still", {"BPSteeringWheelIconStyle": 1, "BPAnimateSteeringWheel": False}, {}, model + hud),
    ("lateral mode letter", {"BpShowLateralControl": True}, {}, model + hud),
    ("disengaged, not engageable", {}, {"engaged": False}, hud),
    ("blind spot detected", {"BlindSpot": True, "ShowBlindspotOverlay": True}, {"blind_spot": True},
     model + hud + ["BlindspotRendererMixin._draw_blindspot_screen_edges"] +
     (["BlindSpotIndicators.render"] if family == "mici" else [])),
    ("brake status", {"ShowBrakeStatus": True}, {"brakes": True}, model + hud),
    ("rainbow lines and path", {"BPRainbowLines": True, "RainbowMode": True}, {}, model + hud),
    ("minimal view, no border", {"BPHideCameraView": True, "BPHideOnroadBorder": True}, {}, model + hud),
  ]
  cases += [(f"theme pack {name}", {"BPThemePack": name}, {}, model + hud) for name in theme_packs]
  return cases


def build_messages(opts: dict) -> dict:
  from openpilot.cereal import log, messaging
  engaged = opts.get("engaged", True)
  msgs = {}

  def msg(service, size=None):
    m = messaging.new_message(service, size) if size is not None else messaging.new_message(service)
    msgs[service] = m
    return getattr(m, service)

  ds = msg("deviceState")
  ds.started = opts.get("started", True)
  ds.chestnutPresent = bool(opts.get("chestnut"))
  ps = msg("pandaStates", 1)
  ps[0].ignitionLine = ds.started
  ps[0].pandaType = log.PandaState.PandaType.redPanda

  ss = msg("selfdriveState")
  ss.enabled = engaged
  ss.active = engaged
  ss.engageable = engaged
  ss.state = log.SelfdriveState.OpenpilotState.enabled if engaged else log.SelfdriveState.OpenpilotState.disabled

  cs = msg("carState")
  cs.vEgo = cs.vEgoCluster = 25.0
  cs.aEgo = -0.3
  cs.steeringAngleDeg = 20.0
  cs.vCruise = cs.vCruiseCluster = 110.0
  cs.cruiseState.enabled = cs.cruiseState.available = True
  cs.cruiseState.speed = 30.0
  cs.leftBlindspot = bool(opts.get("blind_spot"))
  cs.rightBlindspot = bool(opts.get("blind_spot") or opts.get("right_blind_spot"))
  cs.brakePressed = bool(opts.get("brakes"))

  cc = msg("carControl")
  cc.latActive = engaged
  cc.longActive = engaged and bool(opts.get("op_long"))
  co = msg("carOutput")
  co.actuatorsOutput.torque = 0.3
  ctl = msg("controlsState")
  ctl.curvature = 0.002
  ctl.desiredCurvature = 0.0025
  ctl.lateralControlState.init("angleState")

  cp = msg("carParams")
  cp.openpilotLongitudinalControl = bool(opts.get("op_long"))
  cp.brand = "ford"

  n = 33
  x = np.linspace(0.0, 100.0, n)
  md = msg("modelV2")
  md.position.x, md.position.y, md.position.z = x.tolist(), (0.0005 * x ** 2).tolist(), [0.0] * n
  md.acceleration.x = np.linspace(0.5, -0.5, n).tolist()
  lanes = md.init("laneLines", 4)
  for lane, offset in zip(lanes, (-5.4, -1.8, 1.8, 5.4), strict=True):
    lane.x, lane.y, lane.z = x.tolist(), (offset + 0.0005 * x ** 2).tolist(), [0.0] * n
  md.laneLineProbs = [0.9, 0.95, 0.95, 0.9]
  md.laneLineStds = [0.2, 0.1, 0.1, 0.2]
  edges = md.init("roadEdges", 2)
  for edge, offset in zip(edges, (-7.0, 7.0), strict=True):
    edge.x, edge.y, edge.z = x.tolist(), (offset + 0.0005 * x ** 2).tolist(), [0.0] * n
  md.roadEdgeStds = [0.2, 0.2]
  leads = md.init("leadsV3", 2)
  for lead, d in zip(leads, (30.0, 60.0), strict=True):
    lead.prob = 0.9
    lead.x, lead.y, lead.v = [d] * 6, [0.0] * 6, [22.0] * 6

  bf = msg("modelDataV2SP")
  bf.blendfulTrust = 0.8
  bf.blendfulLeftProb = bf.blendfulRightProb = 0.9
  bf.blendfulLaneWidth = 3.6

  cal = msg("extrinsicsCalibration")
  cal.calStatus = log.ExtrinsicsCalibration.Status.calibrated
  cal.rpyCalib = [0.0, 0.0, 0.0]
  cal.height = [1.22]

  rs = msg("radarState")
  for lead, d in ((rs.leadOne, 30.0), (rs.leadTwo, 60.0)):
    lead.present = True
    lead.radar = True
    lead.dRel, lead.yRel, lead.vRel, lead.vLead = d, 0.5, -2.0, 23.0

  plan = msg("longitudinalPlan")
  plan.hasLead = True

  bp = msg("controllerStateBP")
  bp.activeLateralMode = "angle"
  csbp = msg("carStateBP")
  csbp.brakeLightStatus.dataAvailable = True
  csbp.brakeLightStatus.brakeLightsOn = bool(opts.get("brakes"))
  if opts.get("hybrid"):
    hb, hd = csbp.hybridBattery, csbp.hybridDrive
    hb.dataAvailable = True
    hb.socActual, hb.socMinPerc, hb.socMaxPerc = 55.0, 20.0, 80.0
    hb.voltActual, hb.voltLowLimit, hb.voltHighLimit, hb.ampsActual = 250.0, 200.0, 300.0, -20.0
    hd.dataAvailable = True
    hd.throttleDemandPercent, hd.throttleThresholdPercent = 30.0, 60.0
  return msgs


def feed(msgs: dict) -> None:
  """Every frame, every service gets its message again (as patch_submaster does with a route)."""
  from openpilot.selfdrive.ui.ui_state import ui_state
  sm = ui_state.sm

  def update(timeout=None):
    sm.updated = dict.fromkeys(sm.services, False)
    now = time.monotonic()
    for service, m in msgs.items():
      if service in sm.data:
        sm.data[service] = getattr(m, service)
        sm.seen[service] = sm.updated[service] = sm.valid[service] = sm.alive[service] = True
        sm.logMonoTime[service] = m.logMonoTime
        sm.recv_time[service] = now
        sm.recv_frame[service] = sm.frame
    sm.frame += 1
  sm.update = update  # ty: ignore[invalid-assignment]  # synthetic messages instead of sockets


def reset_theme_caches() -> None:
  from openpilot.selfdrive.ui.bp.lib import theme_pack, theme_scene
  theme_pack._cache.update(checked_at=0.0, name=None, pack=None)
  theme_scene._scene_cache.update(key=None, scene=None, checked_at=0.0)


def main() -> None:
  results: dict = {}
  calls: collections.Counter = collections.Counter()

  with OpenpilotPrefix():
    from openpilot.selfdrive.ui.tests.diff.replay import setup_state
    setup_state()
    from openpilot.common.params import Params
    params = Params()
    for k in params.all_keys():  # manager_init's defaults
      d = params.get_default_value(k)
      if d is not None and params.get(k) is None:
        params.put(k, d, block=True)

    rl.set_config_flags(rl.ConfigFlags.FLAG_WINDOW_HIDDEN)
    from openpilot.system.ui.lib.application import gui_app
    gui_app.init_window("bp ui frames", fps=60)  # it also runs UI code (fonts, patches): an exception is a failure
    if not rl.is_window_ready():
      print("NOWINDOW raylib has no window after init")
      sys.exit(NO_WINDOW)

    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.selfdrive.ui.bp.lib import theme_pack

    def wrap(modname: str, clsname: str | None, meth: str) -> None:
      try:
        mod = importlib.import_module(modname)
      except ModuleNotFoundError:
        return  # a case that needs it then fails with "never called"
      owner = getattr(mod, clsname) if clsname else mod
      f = getattr(owner, meth)
      key = f"{clsname}.{meth}" if clsname else meth

      def counted(*a, **k):
        calls[key] += 1
        return f(*a, **k)
      setattr(owner, meth, counted)
      if not clsname:  # module function: also patch the names it was imported under
        for m in list(sys.modules.values()):
          if getattr(m, meth, None) is f and getattr(m, "__name__", "").startswith("openpilot."):
            setattr(m, meth, counted)

    def attempt(name: str, fn) -> None:
      calls.clear()
      try:
        fn()
        results[name] = {"error": None, "calls": dict(calls)}
      except Exception:
        results[name] = {"error": traceback.format_exc(), "calls": dict(calls)}

    def draw(widget, n: int, before=None, last_frame=None) -> None:
      rect = rl.Rectangle(0, 0, gui_app.width, gui_app.height)
      for i in range(n):
        if before:
          before()
        ui_state.update()
        rl.begin_drawing()
        rl.clear_background(rl.BLACK)
        try:
          widget.render(rect)
          if last_frame and i == n - 1:
            last_frame()
        finally:
          rl.end_drawing()

    def pixel(x: float, y: float) -> tuple[int, int, int]:
      """The drawn color at screen point (x, y), read back from the framebuffer before it's shown."""
      rl.rl_draw_render_batch_active()  # flush the draws raylib still holds in its batch
      img = rl.load_image_from_screen()
      scale = img.width / rl.get_screen_width()  # the framebuffer can be larger (HiDPI)
      c = rl.get_image_color(img, int(x * scale), int(y * scale))
      rl.unload_image(img)
      return c.r, c.g, c.b

    def put(key: str, value) -> None:
      params.put(key, value, block=True)

    # Main layout: the onroad view it builds must be BluePilot's
    if FAMILY == "tici":
      from openpilot.selfdrive.ui.layouts.main import MainLayout, MainState
      layout = MainLayout()
      onroad_view = layout._layouts[MainState.ONROAD]
    else:
      from openpilot.selfdrive.ui.mici.layouts.main import MiciMainLayout
      layout = MiciMainLayout()
      onroad_view = layout._car_onroad_layout
    gui_app.pop_widgets_to(layout, instant=True)  # skip onboarding
    view_cls = type(onroad_view)
    print("onroad view", view_cls.__module__, view_cls.__name__)

    # COUNTED wraps go on after the main layout exists, so module-level draw_polygon imports are all loaded
    for spec in COUNTED[FAMILY]:
      wrap(*spec)
    # polygons drawn by sunnypilot's lane-center line itself (BlendfulRenderer.draw returns early when lateral is off)
    from openpilot.selfdrive.ui.sunnypilot.onroad import blendful
    blendful_polygon = blendful.draw_polygon

    def lane_center_polygon(*a, **k):
      calls["lane-center line polygon"] += 1
      return blendful_polygon(*a, **k)
    blendful.draw_polygon = lane_center_polygon  # ty: ignore[invalid-assignment]  # counts the calls
    if FAMILY == "mici":  # textures the model-source (chestnut) icon really draws; the method returns early when there's none
      from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
      draw_model_source = HudRenderer._draw_model_source

      def model_source(self, rect):
        draw_texture_ex = rl.draw_texture_ex

        def counted_texture(*a, **k):
          calls["model source icon drawn"] += 1
          return draw_texture_ex(*a, **k)
        rl.draw_texture_ex = counted_texture
        try:
          return draw_model_source(self, rect)
        finally:
          rl.draw_texture_ex = draw_texture_ex
      HudRenderer._draw_model_source = model_source

    # Fake camera frames
    from msgq.visionipc import VisionIpcServer
    from openpilot.cereal.visionipc import VisionStreamType
    streams = (VisionStreamType.VISION_STREAM_NARROW_ROAD, VisionStreamType.VISION_STREAM_WIDE_ROAD)
    vipc = VisionIpcServer("camerad")
    for st in streams:
      vipc.create_buffers(st, 5, CAMERA_W, CAMERA_H)
    vipc.start_listener()
    yuv = np.full(CAMERA_W * CAMERA_H * 3 // 2, 90, dtype=np.uint8).tobytes()
    frame_id = [0]

    def send_frame():
      frame_id[0] += 1
      for st in streams:
        vipc.send(st, yuv, frame_id[0], frame_id[0] * 1000, frame_id[0] * 1000)

    packs = sorted(theme_pack.list_packs())
    defaults = {k: params.get(k) for k in DISPLAY_PARAMS}
    from opendbc.car.structs import car
    for name, case_params, opts, required in onroad_cases(FAMILY, packs):
      def run(case_params=case_params, opts=opts, required=required):
        for k, v in defaults.items():
          if v is None:
            params.remove(k)
          else:
            put(k, v)
        for k, v in case_params.items():
          put(k, v)
        cp = car.CarParams.new_message(openpilotLongitudinalControl=bool(opts.get("op_long")), brand="ford")
        put("CarParams", cp.to_bytes())
        put("CarParamsPersistent", cp.to_bytes())
        reset_theme_caches()
        if opts.get("chestnut"):  # the chestnut is picked up when the car starts: pass through offroad first
          feed(build_messages({**opts, "started": False}))
          ui_state.update()
        feed(build_messages(opts))
        ui_state.update_params()
        view = view_cls()
        view.show_event()
        probed: list = []

        def probe_right_edge():  # 3 px inside the road view's right edge (left of the navigation panel when shown)
          content = view._content_rect
          probed.append(pixel(content.x + content.width - 3, content.y + content.height / 2))
        if opts.get("red_right_edge"):  # the edge pulses on a 3 s cycle: start it at its brightest (0.75 s in)
          view._blindspot_pulse_start_time = time.monotonic() - 0.75
        draw(view, FRAMES, send_frame, probe_right_edge if opts.get("red_right_edge") else None)
        view.hide_event()
        for r, g, b in probed:
          calls[f"right edge rgb {r},{g},{b}"] += 1
          if not (r > 30 and r > g + 20 and r > b + 20):
            raise AssertionError(f"no red blind-spot edge at the road view's right edge: rgb {(r, g, b)}")
        missing = [m for m in required if not m.startswith("!") and not calls[m]]
        unexpected = [m[1:] for m in required if m.startswith("!") and calls[m[1:]]]
        if missing or unexpected:
          raise AssertionError(f"draw methods never called: {missing}; called but shouldn't be: {unexpected}")
      attempt(f"onroad/{name}", run)

    # The main layout itself, onroad
    def main_layout_onroad():
      feed(build_messages({}))
      if FAMILY == "mici":  # the mici scroller reaches the onroad page only through a timed scroll: show it directly
        gui_app.push_widget(onroad_view)
      draw(gui_app._nav_stack[-1], FRAMES, send_frame)
      gui_app.pop_widgets_to(layout, instant=True)
      if not calls["BlendfulRenderer.draw"]:
        raise AssertionError("the main layout never drew the onroad view")
    attempt("onroad/main layout", main_layout_onroad)

    if FAMILY == "tici":  # the 3X controls debug panel (slides over the onroad view): every tab and Other's sub-tabs
      def debug_panel():
        from bluepilot.ui.widgets.debug import ControlsDebugPanel
        panel = ControlsDebugPanel()
        panel.toggle_visibility()
        for tab in range(len(panel.TAB_LABELS)):
          panel._current_tab = tab
          for sub_tab in (range(5) if tab == 2 else [0]):
            panel._other_panel._current_sub_tab = sub_tab
            panel._other_panel._last_update_times = dict.fromkeys(range(5), 0.0)
            draw(panel, 30, send_frame)  # 30 frames: the panel slides in over ~0.3 s
        if not calls["DebugDataCard._render"]:
          raise AssertionError("no debug data card drawn")
        calls["debug panel tabs drawn"] = len(panel.TAB_LABELS)
      wrap("bluepilot.ui.widgets.debug.other_debug_panel", "DebugDataCard", "_render")
      attempt("onroad/controls debug panel", debug_panel)

    # Settings pages (offroad)
    feed(build_messages({"started": False}))
    for k, v in defaults.items():
      if v is None:
        params.remove(k)
      else:
        put(k, v)
    reset_theme_caches()
    draw(layout, 5)
    if FAMILY == "tici":
      draw_tici_settings(layout, draw, attempt, calls)
    else:
      draw_mici_settings(layout, draw, attempt)

  # flush: a line longer than the pipe buffer is written straight through while its newline stays buffered,
  # and raylib's close_window log (flushed C stdout) would land on the same line
  print("RESULTS " + json.dumps(results), flush=True)
  rl.close_window()


def draw_tici_settings(layout, draw, attempt, calls) -> None:
  from openpilot.selfdrive.ui.layouts.main import MainState
  settings = layout._layouts[MainState.SETTINGS]
  for panel_type, info in list(settings._panels.items()):
    def run(panel_type=panel_type, info=info):
      layout.open_settings(panel_type)
      draw(layout, 3)
      inst = info.instance
      scroller = getattr(inst, "_scroller", None)
      panel = getattr(scroller, "scroll_panel", None)

      def scroll_through():
        draw(layout, 2)
        if panel is not None:
          for y in range(0, 12000, 600):
            draw(layout, 1, lambda y=y: panel.set_offset(-y))

      # open each collapsible section in turn (BluePilot's sections are one-open-at-a-time) and draw it
      sections = [item for item in (getattr(scroller, "_items", None) or getattr(scroller, "items", None) or [])
                  if getattr(item, "_expanded", True) is False and hasattr(item, "_toggle")]
      for item in sections:
        if not item._expanded:
          item._toggle()
          calls["sections expanded"] += 1
        scroll_through()
      if not sections:
        scroll_through()
    attempt(f"settings/{getattr(panel_type, 'name', panel_type)}", run)


def draw_mici_settings(layout, draw, attempt) -> None:
  from openpilot.system.ui.lib.application import gui_app
  from openpilot.selfdrive.ui.sunnypilot.mici.layouts.settings import SettingsLayoutSP
  from openpilot.selfdrive.ui.bp.mici.layouts.settings.bluepilot import BluePilotLayoutMici
  real_push = gui_app.push_widget
  seen: set[int] = set()

  def scroll_items(widget):
    scroller = getattr(widget, "_scroller", None)
    return list(getattr(scroller, "items", None) or getattr(scroller, "_items", None) or [])

  def press_and_collect(item) -> list:
    pushed: list = []
    gui_app.push_widget = pushed.append  # ty: ignore[invalid-assignment]  # collect the pages a press opens
    try:
      if hasattr(item, "plus_clicked"):
        item.plus_clicked()
      else:
        item._handle_mouse_release(rl.Vector2(0, 0))
    finally:
      gui_app.push_widget = real_push  # ty: ignore[invalid-assignment]
    return pushed

  def visit(widget, path: str, press_all: bool, depth: int = 0) -> None:
    if id(widget) in seen:
      return
    seen.add(id(widget))
    path = f"{path}/{type(widget).__name__}"

    def run():
      real_push(widget)
      draw(widget, 3)
      panel = getattr(getattr(widget, "_scroller", None), "scroll_panel", None)
      if panel is not None:
        for x in range(0, 6000, 400):
          draw(widget, 1, lambda x=x: panel.set_offset(-x))
    attempt(f"settings{path}", run)
    for item in scroll_items(widget):
      # the settings root: follow the buttons that open a page; BluePilot's pages: press everything;
      # stock pages are drawn, not pressed
      if not press_all and (depth > 0 or type(item).__name__ not in ("SettingsBigButton", "BigButton", "BigButtonBP")):
        continue
      pages: list = []
      attempt(f"settings{path}/press {getattr(item, 'text', type(item).__name__)}",
              lambda item=item, pages=pages: pages.extend(press_and_collect(item)))
      for page in pages:
        visit(page, path, press_all or isinstance(page, BluePilotLayoutMici), depth + 1)
    gui_app.pop_widgets_to(layout, instant=True)

  visit(SettingsLayoutSP(), "", False)


if __name__ == "__main__":
  main()
