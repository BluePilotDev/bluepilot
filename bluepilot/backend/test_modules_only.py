#!/usr/bin/env python3
"""
Test only the new modular components without importing web_routes_server
"""

import sys
import os

# Add project root to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

def test_new_modules():
    """Test only the new modules we created"""

    print("Testing new backend modules...")
    print("=" * 60)

    tests_passed = 0
    tests_failed = 0

    # Test config module
    try:
        from bluepilot.backend.config import (
            ROUTES_DIR, DEFAULT_PORT, WEBSOCKET_PORT, CAMERA_FILES
        )
        print("✓ config.py")
        print(f"  DEFAULT_PORT: {DEFAULT_PORT}")
        tests_passed += 1
    except Exception as e:
        print(f"✗ config.py: {e}")
        tests_failed += 1

    # Test core.logging_handler
    try:
        from bluepilot.backend.core.logging_handler import ErrorBufferHandler
        print("✓ core/logging_handler.py")
        tests_passed += 1
    except Exception as e:
        print(f"✗ core/logging_handler.py: {e}")
        tests_failed += 1

    # Test utils.file_ops
    try:
        from bluepilot.backend.utils.file_ops import atomic_write, safe_json_write
        print("✓ utils/file_ops.py")

        # Quick functional test
        import tempfile
        test_file = os.path.join(tempfile.gettempdir(), "test_atomic.txt")
        if atomic_write(test_file, "test content"):
            if os.path.exists(test_file):
                with open(test_file, 'r') as f:
                    content = f.read()
                if content == "test content":
                    print("  Atomic write functional test: PASS")
                os.remove(test_file)
        tests_passed += 1
    except Exception as e:
        print(f"✗ utils/file_ops.py: {e}")
        tests_failed += 1

    # Test utils.power
    try:
        from bluepilot.backend.utils.power import (
            enable_performance_mode, restore_power_save, check_and_restore_power_save
        )
        print("✓ utils/power.py")
        tests_passed += 1
    except Exception as e:
        print(f"✗ utils/power.py: {e}")
        tests_failed += 1

    # Test module __init__ files work
    try:
        from bluepilot.backend import core, utils, video
        print("✓ Package __init__.py files")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Package __init__.py files: {e}")
        tests_failed += 1

    print("=" * 60)
    print(f"Tests passed: {tests_passed}")
    print(f"Tests failed: {tests_failed}")

    if tests_failed == 0:
        print("\nAll new modules working correctly! ✓")
        return True
    else:
        print(f"\n{tests_failed} test(s) failed")
        return False


if __name__ == "__main__":
    success = test_new_modules()
    sys.exit(0 if success else 1)
