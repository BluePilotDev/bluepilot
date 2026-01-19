#!/usr/bin/env python3
"""
Quick sanity check for bp_portal module structure
Tests that all imports work correctly
"""

import sys
import os

# Add project root to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

def test_imports():
    """Test all backend module imports"""

    print("Testing backend module imports...")
    print("=" * 60)

    # Test config module
    try:
        from bluepilot.backend.config import (
            ROUTES_DIR, WEBAPP_DIR, DEFAULT_PORT, WEBSOCKET_PORT,
            CAMERA_FILES, HEVC_CAMERAS
        )
        print("✓ config.py imports successful")
        print(f"  - ROUTES_DIR: {ROUTES_DIR}")
        print(f"  - DEFAULT_PORT: {DEFAULT_PORT}")
        print(f"  - WEBSOCKET_PORT: {WEBSOCKET_PORT}")
    except Exception as e:
        print(f"✗ config.py import failed: {e}")
        return False

    # Test core modules
    try:
        from bluepilot.backend.core.logging_handler import ErrorBufferHandler
        print("✓ core.logging_handler imports successful")
    except Exception as e:
        print(f"✗ core.logging_handler import failed: {e}")
        return False

    # Test utils modules
    try:
        from bluepilot.backend.utils.file_ops import atomic_write, safe_json_write
        print("✓ utils.file_ops imports successful")
    except Exception as e:
        print(f"✗ utils.file_ops import failed: {e}")
        return False

    try:
        from bluepilot.backend.utils.power import (
            enable_performance_mode, restore_power_save
        )
        print("✓ utils.power imports successful")
    except Exception as e:
        print(f"✗ utils.power import failed: {e}")
        return False

    # Test main portal module
    try:
        from bluepilot.backend import bp_portal
        print("✓ bp_portal module imports successful")
        print(f"  - Has main function: {hasattr(bp_portal, 'main')}")
    except Exception as e:
        print(f"✗ bp_portal import failed: {e}")
        return False

    # Test routes modules
    try:
        from bluepilot.backend.routes import haversine_distance
        print("✓ routes.processing imports successful")
    except Exception as e:
        print(f"✗ routes.processing import failed: {e}")
        return False

    # Test realtime modules
    try:
        from bluepilot.backend.realtime import WebSocketBroadcaster
        print("✓ realtime.websocket imports successful")
    except Exception as e:
        print(f"✗ realtime.websocket import failed: {e}")
        return False

    try:
        from bluepilot.backend.handlers.log_downloads import handle_qlog_download, handle_rlog_download
        print("✓ handlers.log_downloads imports successful")
    except Exception as e:
        print(f"✗ handlers.log_downloads import failed: {e}")
        return False

    print("=" * 60)
    print("All imports successful! ✓")
    return True


if __name__ == "__main__":
    success = test_imports()
    sys.exit(0 if success else 1)
