from cereal import car
from openpilot.selfdrive.car.chrysler.values import RAM_CARS

GearShifter = car.CarState.GearShifter
VisualAlert = car.CarControl.HUDControl.VisualAlert

def create_lkas_hud(packer, CP, lkas_active, mads_enabled, hud_alert, hud_count, car_model, auto_high_beam):
  # LKAS_HUD - Controls what lane-keeping icon is displayed

  # == Color ==
  # 0 hidden?
  # 1 white
  # 2 green
  # 3 ldw

  # == Lines ==
  # 03 white Lines
  # 04 grey lines
  # 09 left lane close
  # 0A right lane close
  # 0B left Lane very close
  # 0C right Lane very close
  # 0D left cross cross
  # 0E right lane cross

  # == Alerts ==
  # 7 Normal
  # 6 lane departure place hands on wheel

  color = 2 if lkas_active else 1 if mads_enabled and not lkas_active else 0
  lines = 3 if lkas_active else 0
  alerts = 7 if lkas_active else 0

  if hud_count < (1 * 4):  # first 3 seconds, 4Hz
    alerts = 1

  if hud_alert in (VisualAlert.ldw, VisualAlert.steerRequired):
    color = 4
    lines = 0
    alerts = 6

  values = {
    "LKAS_ICON_COLOR": color,
    "CAR_MODEL": car_model,
    "LKAS_LANE_LINES": lines,
    "LKAS_ALERTS": alerts,
  }

  if CP.carFingerprint in RAM_CARS:
    values['AUTO_HIGH_BEAM_ON'] = auto_high_beam
    values['LKAS_DISABLED'] = 0 if mads_enabled else 1

  return packer.make_can_msg("DAS_6", 0, values)


def create_lkas_command(packer, CP, apply_steer, lkas_control_bit):
  # LKAS_COMMAND Lane-keeping signal to turn the wheel
  enabled_val = 2 if CP.carFingerprint in RAM_CARS else 1
  values = {
    "STEERING_TORQUE": apply_steer,
    "LKAS_CONTROL_BIT": enabled_val if lkas_control_bit else 0,
  }
  return packer.make_can_msg("LKAS_COMMAND", 0, values)


def create_cruise_buttons(packer, frame, bus, CP, cruise_buttons_msg=None, buttons=0, cancel=False, resume=False):

  acc_accel = 1 if buttons == 1 else 0
  acc_decel = 1 if buttons == 2 else 0

  values = {
    "ACC_Cancel": cancel,
    "ACC_Resume": resume,
    "ACC_Accel": acc_accel,
    "ACC_Decel": acc_decel,
    "COUNTER": frame % 0x10,
  }

  if buttons == 0 and not (cancel or resume) and CP.carFingerprint in RAM_CARS:
    values = cruise_buttons_msg.copy()
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)


def create_lkas_heartbit(packer, lkas_disabled, lkas_heartbit):
  # LKAS_HEARTBIT (697) LKAS heartbeat
  values = lkas_heartbit.copy()  # forward what we parsed
  values["LKAS_DISABLED"] = 1 if lkas_disabled else 0
  return packer.make_can_msg("LKAS_HEARTBIT", 0, values)
