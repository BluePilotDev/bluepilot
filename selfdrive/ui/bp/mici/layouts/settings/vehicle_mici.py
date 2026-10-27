"""BluePilot MICI: Vehicle settings panel — cluster UI, 12V battery limit."""

from collections.abc import Callable

from openpilot.common.params import Params
from openpilot.bluepilot.vasm.config import normalize_annotation_config
from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigButtonBP, BigParamControlBP
from openpilot.selfdrive.ui.bp.mici.widgets.floatbutton import BigParamFloatControl
from openpilot.selfdrive.ui.bp.mici.widgets.web_server_qr_dialog import WebServerQRDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller


class VehicleLayoutMici(NavScroller):
  def __init__(self, back_callback: Callable[[], None] | None = None):
    super().__init__()
    if back_callback is not None:
      self.set_back_callback(back_callback)
    self._params = Params()

    self.show_hands_free_ui = BigParamControlBP("Show BlueCruise UI on Cluster", "send_hands_free_cluster_msg")
    # Init-time param (read once at car init, mirrored into panda safety); takes effect after restart.
    # FORD_EDGE_MK2's pinion sensor only reports a relative angle -- the safety/control
    # layers already no-op this toggle there, so grey it out too (see values_ext.py
    # FORD_PINION_GEOMETRY_INDEX).
    self.steer_angle_curvature = BigParamControlBP("Use Pinion Yaw Sensor", "FordPrefSteerAngleCurvature")
    self.steer_angle_curvature.set_enabled(self._pinion_yaw_sensor_supported)
    self.vbatt_pause_charging = BigParamFloatControl("12V Battery Limit", "vbatt_pause_charging", min=11.0, max=14.0, step=0.1)

    # BluePilot: StarPilot-derived driver-camera adjacent-spot monitoring.
    self.vasm_enabled = BigParamControlBP("Vision-Adjacent Spot Monitoring", "VASMEnabled")
    self.vasm_configure = BigButtonBP("V-ASM Camera Regions", "CONFIGURE")
    self.vasm_configure.set_click_callback(self._configure_vasm)
    self.vasm_confidence = BigParamFloatControl(
      "V-ASM Confidence Threshold", "VASMConfidenceThreshold", min=0.25, max=1.0, step=0.05,
    )
    self.vasm_smoothing = BigParamFloatControl(
      "V-ASM Smoothing (seconds)", "VASMSmoothSeconds", min=0.1, max=0.5, step=0.1,
    )
    # End BluePilot

    self._scroller.add_widgets([
      self.show_hands_free_ui,
      self.steer_angle_curvature,
      self.vasm_enabled,
      self.vasm_configure,
      self.vasm_confidence,
      self.vasm_smoothing,
      self.vbatt_pause_charging,
    ])

    self._refresh_toggles = (
      ("send_hands_free_cluster_msg", self.show_hands_free_ui),
      ("FordPrefSteerAngleCurvature", self.steer_angle_curvature),
      ("VASMEnabled", self.vasm_enabled),
    )

    ui_state.add_offroad_transition_callback(self._update_toggles)

  @staticmethod
  def _pinion_yaw_sensor_supported() -> bool:
    return ui_state.CP is None or ui_state.CP.carFingerprint != "FORD_EDGE_MK2"

  def show_event(self):
    super().show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))
    configured = self._has_vasm_config()
    enabled = ui_state.params.get_bool("VASMEnabled")
    self.vasm_enabled.set_enabled(configured)
    self.vasm_confidence.set_enabled(enabled)
    self.vasm_smoothing.set_enabled(enabled)

  def _has_vasm_config(self) -> bool:
    try:
      normalize_annotation_config(self._params.get("VASMAnnotationConfig", return_default=True) or {})
      return True
    except (TypeError, ValueError):
      return False

  def _configure_vasm(self) -> None:
    self._params.put_bool("EnableWebRoutesServer", True)
    gui_app.push_widget(WebServerQRDialog(
      back_callback=gui_app.pop_widget,
      path="/vasm",
      title=tr("Configure V-ASM"),
    ))
