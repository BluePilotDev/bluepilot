"""
modelDataV2BP: JetLink's big model as modeld saw it, one message per model frame, for logs and route review (D2 b).

Both modelds send it beside modelDataV2SP, with the link on or off. It holds what no other message has: the big
model's state (jetlink.openpilot STATES) and the count of swaps and hand-backs, each a step change in which model
drives. Which model drove the frame is modelV2.big. Offroad progress stays in the JetlinkProgress param.
"""
from openpilot.cereal import custom
import openpilot.cereal.messaging as messaging

SERVICE = "modelDataV2BP"
# the schema's states; JetLink promises one of its STATES, and a name outside the enum would raise on modeld's frame
# thread, so anything else is logged as none
STATES = frozenset(custom.ModelDataV2BP.AcceleratorState.schema.enumerants)
UINT32_MAX = 2 ** 32 - 1


def accelerator_state(hooks, model) -> str:
  try:
    state = hooks.state(model)
  except Exception:
    return 'none'
  return state if state in STATES else 'none'


def handovers(model) -> int:
  """The joining model's swaps and hand-backs (JoinedModel.handovers); 0 for any other model."""
  count = getattr(model, 'handovers', 0)
  return min(max(count, 0), UINT32_MAX) if isinstance(count, int) and not isinstance(count, bool) else 0


def message(hooks, model, valid: bool = True):
  msg = messaging.new_message(SERVICE)
  msg.valid = valid
  msg.modelDataV2BP.acceleratorState = accelerator_state(hooks, model)
  msg.modelDataV2BP.handovers = handovers(model)
  return msg
