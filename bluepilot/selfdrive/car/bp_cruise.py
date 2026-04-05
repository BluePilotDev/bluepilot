"""
BluePilot: ICBM (Intelligent Cruise Button Management) helpers for cruise control.

Extracts BP-specific cruise logic from selfdrive/car/cruise.py so that the
stock file only needs thin hook calls instead of inline BP blocks.
"""

from openpilot.common.constants import CV


def should_update_button_timers(CP, CP_SP):
  """Return True if button timers should be updated this cycle.

  For stock: only non-PCM cars update timers.
  For BluePilot: PCM cars with ICBM also need timer management because
  ICBM sends cruise buttons over CAN directly.
  """
  return not CP.pcmCruise or not CP_SP.pcmCruiseSpeed or CP_SP.intelligentCruiseButtonManagementAvailable


def clear_button_timers_if_disabled(button_timers, enabled):
  """Clear all button timers when cruise is disabled to prevent stale presses.

  Returns True if timers were cleared (caller should return early).
  """
  if not enabled:
    for k in button_timers:
      button_timers[k] = 0
    return True
  return False


def should_skip_pcm_init(CP, CP_SP):
  """Return True if v_cruise initialization should be skipped.

  For PCM cars WITHOUT ICBM, skip init (stock handles it).
  For PCM cars WITH ICBM, allow init because ICBM needs a valid
  initial value to prevent planner from using V_CRUISE_UNSET.
  """
  return CP.pcmCruise and not CP_SP.intelligentCruiseButtonManagementAvailable


def get_icbm_initial_cruise_speed(CP, CP_SP, CS):
  """For PCM cars with ICBM, return cluster speed if available.

  Returns the speed in kph, or None if ICBM initial speed doesn't apply.
  """
  if CP.pcmCruise and CP_SP.intelligentCruiseButtonManagementAvailable and CS.cruiseState.speedCluster > 0:
    return CS.cruiseState.speedCluster * CV.MS_TO_KPH
  return None
