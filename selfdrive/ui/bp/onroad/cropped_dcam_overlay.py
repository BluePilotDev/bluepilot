"""Optional DCam overlay, independent of the forward camera and driving controls."""

from time import monotonic

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.bp.onroad.cropped_dcam_geometry import active_dcam_sides, adaptive_window_center_y


class CroppedDcamOverlayMixin:
  def _init_cropped_dcam(self, compact: bool = False) -> None:
    self._cropped_dcam = None
    self._dcam_compact = compact
    self._dcam_settings_time = -float('inf')
    self._dcam_enabled = False
    self._dcam_turn_signal = True
    self._dcam_bsm = True
    self._dcam_suspended = True
    self._dcam_failed = False

  def _suspend_cropped_dcam(self) -> None:
    if self._cropped_dcam is not None and not self._dcam_suspended:
      self._cropped_dcam.suspend()
    self._dcam_suspended = True

  def _render_cropped_dcam(self, content_rect) -> None:
    from openpilot.selfdrive.ui.ui_state import ui_state

    now = monotonic()
    if now - self._dcam_settings_time >= 1.0:
      self._dcam_enabled = self._bp_params.get_bool('BPCroppedDcam')
      self._dcam_turn_signal = self._bp_params.get_bool('BPCroppedDcamTurnSignal')
      self._dcam_bsm = self._bp_params.get_bool('BPCroppedDcamBSM')
      self._dcam_settings_time = now
    sm = ui_state.sm
    if not self._dcam_enabled:
      self._dcam_failed = False
    if (not ui_state.started or not self._dcam_enabled or not sm.valid.get('carState', False) or
        not sm.seen.get('carState', False) or not 0 <= now - sm.recv_time.get('carState', 0) <= 0.5):
      self._suspend_cropped_dcam()
      return
    if self._dcam_failed:
      return
    left, right = active_dcam_sides(sm['carState'], self._dcam_turn_signal, self._dcam_bsm)
    if not left and not right and (self._cropped_dcam is None or not self._cropped_dcam.is_visible()):
      self._suspend_cropped_dcam()
      return
    if self._cropped_dcam is None:
      # No camera client, textures or extra shaders are allocated at UI startup.
      try:
        from openpilot.selfdrive.ui.bp.onroad.cropped_dcam_view import CroppedDcamViewBP, MiciCroppedDcamViewBP
        self._cropped_dcam = self._child((MiciCroppedDcamViewBP if self._dcam_compact else CroppedDcamViewBP)())
      except (RuntimeError, AssertionError):
        cloudlog.exception("Optional DCam view unavailable; retaining the forward view")
        self._dcam_failed = True
        return
    self._dcam_suspended = False
    calibration = (0.0, 0.0, 0.0)
    if sm.valid.get('liveCalibration', False) and len(sm['liveCalibration'].rpyCalib) == 3:
      calibration = tuple(sm['liveCalibration'].rpyCalib)
    window_y = None
    if sm.valid.get('driverStateV2', False):
      driver = sm['driverStateV2']
      data = driver.rightDriverData if driver.wheelOnRightProb > 0.5 else driver.leftDriverData
      window_y = adaptive_window_center_y(data.facePosition, data.faceProb)
    focal_length = self.device_camera.dcam.focal_length if self.device_camera is not None else 567.0
    self._cropped_dcam.render_crops(content_rect, left, right, calibration, window_y, focal_length,
                                    light_sensor=ui_state.light_sensor)

  def hide_event(self):
    self._suspend_cropped_dcam()
    super().hide_event()
