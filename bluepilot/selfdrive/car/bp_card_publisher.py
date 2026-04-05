"""
BluePilot: Publishes BP-specific messages from card.py.

Centralizes controllerStateBP and carStateBP message publishing so that
card.py only needs a single function call instead of inline BP blocks.
"""

import cereal.messaging as messaging
from opendbc.car import structs
from openpilot.selfdrive.car.helpers import convert_to_capnp


def publish_controller_state_bp(CI, pm):
  """Publish controllerStateBP if the car controller reports lateralUncertainty."""
  if hasattr(CI.CC, "lateralUncertainty"):
    cs_bp = structs.ControllerStateBP()
    cs_bp.lateralUncertainty = CI.CC.lateralUncertainty
    cs_bp_capnp = convert_to_capnp(cs_bp)
    cs_bp_send = messaging.new_message('controllerStateBP')
    cs_bp_send.valid = True
    cs_bp_send.controllerStateBP = cs_bp_capnp
    pm.send('controllerStateBP', cs_bp_send)


def publish_car_state_bp(CI, pm, can_valid):
  """Publish carStateBP (hybrid drive gauge data) if available from car state."""
  if hasattr(CI.CS, 'car_state_bp_msg') and CI.CS.car_state_bp_msg is not None:
    cs_bp_send = CI.CS.car_state_bp_msg
    cs_bp_send.valid = can_valid
    pm.send('carStateBP', cs_bp_send)
