#!/usr/bin/env python3
"""
Quick test to verify the ICBM/SLA circular dependency fix
"""

import sys

# Add the sunnypilot path to import the module
sys.path.append('sunnypilot')
sys.path.append('openpilot')

from cereal import custom, car
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist

def test_icbm_sla_fix():
    """Test that SLA provides target to ICBM vehicles in preActive state"""

    # Create mock CarParams for ICBM vehicle (non-PCM)
    CP = car.CarParams.new_message()
    CP.openpilotLongitudinalControl = False  # ICBM vehicle
    CP.pcmCruise = False

    CP_SP = custom.CarParamsSP.new_message()

    # Create SLA instance
    sla = SpeedLimitAssist(CP, CP_SP)

    # Set up test scenario: ICBM vehicle with speed limit in preActive state
    sla.long_enabled = True
    sla.enabled = True
    sla.is_enabled = True
    sla.is_active = False  # Not active yet
    sla.state = custom.LongitudinalPlanSP.SpeedLimit.AssistState.preActive
    sla._has_speed_limit = True
    sla._speed_limit_final_last = 35.0  # 35 m/s speed limit

    # Test the fix
    v_target = sla.get_v_target_from_control()

    print("Test Results:")
    print(f"  PCM Op Long: {sla.pcm_op_long}")
    print(f"  Is Enabled: {sla.is_enabled}")
    print(f"  Is Active: {sla.is_active}")
    print(f"  State: {sla.state}")
    print(f"  Has Speed Limit: {sla._has_speed_limit}")
    print(f"  Speed Limit Final Last: {sla._speed_limit_final_last}")
    print(f"  V Target Returned: {v_target}")

    # Verify the fix works
    if v_target == 35.0:
        print("✅ SUCCESS: SLA provides target to ICBM in preActive state")
        return True
    else:
        print(f"❌ FAILED: Expected 35.0, got {v_target}")
        return False

def test_pcm_vehicle_unchanged():
    """Test that PCM vehicles (non-ICBM) behavior is unchanged"""

    # Create mock CarParams for PCM vehicle
    CP = car.CarParams.new_message()
    CP.openpilotLongitudinalControl = True  # PCM vehicle
    CP.pcmCruise = True

    CP_SP = custom.CarParamsSP.new_message()

    # Create SLA instance
    sla = SpeedLimitAssist(CP, CP_SP)

    # Set up test scenario: PCM vehicle with speed limit in preActive state
    sla.long_enabled = True
    sla.enabled = True
    sla.is_enabled = True
    sla.is_active = False  # Not active yet
    sla.state = custom.LongitudinalPlanSP.SpeedLimit.AssistState.preActive
    sla._has_speed_limit = True
    sla._speed_limit_final_last = 35.0

    # Test that PCM behavior is unchanged
    v_target = sla.get_v_target_from_control()

    print("\nPCM Vehicle Test:")
    print(f"  PCM Op Long: {sla.pcm_op_long}")
    print(f"  Is Enabled: {sla.is_enabled}")
    print(f"  Is Active: {sla.is_active}")
    print(f"  State: {sla.state}")
    print(f"  V Target Returned: {v_target}")

    # PCM vehicles should still get target when enabled (even in preActive)
    if v_target == 35.0:
        print("✅ SUCCESS: PCM vehicle behavior unchanged")
        return True
    else:
        print("❌ FAILED: PCM vehicle should get target when enabled")
        return False

if __name__ == "__main__":
    print("Testing ICBM/SLA Circular Dependency Fix\n")

    try:
        test1_pass = test_icbm_sla_fix()
        test2_pass = test_pcm_vehicle_unchanged()

        if test1_pass and test2_pass:
            print("\n🎉 All tests passed! The fix should work.")
        else:
            print("\n❌ Some tests failed. Check the implementation.")

    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        print("This might be due to missing dependencies or import issues.")
        print("Try running the integration test instead.")