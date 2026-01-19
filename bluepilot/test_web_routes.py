#!/usr/bin/env python3
"""
Simple test script for BluePilot Portal
Run this locally to test the server without building the entire project

Usage:
    python3 bluepilot/test_web_routes.py

Then open http://localhost:8088 in your browser
"""

import sys
import os

# Add project root to path
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

# Mock Params if not available
try:
    from common.params import Params
except ImportError:
    print("Warning: Could not import Params, using mock")

    class MockParams:
        """Mock Params class for local testing"""
        def __init__(self):
            self._params = {
                "IsOnRoad": b"0",
                "BPPortalPort": b"8088"
            }

        def get_bool(self, key):
            value = self._params.get(key, b"0")
            if isinstance(value, bytes):
                return value == b"1"
            return bool(value)

        def get(self, key, encoding='utf-8'):
            value = self._params.get(key, b"")
            if encoding:
                return value.decode(encoding) if isinstance(value, bytes) else str(value)
            return value

        def put(self, key, value):
            if isinstance(value, str):
                value = value.encode()
            self._params[key] = value

        def put_bool(self, key, value):
            self._params[key] = b"1" if value else b"0"

    # Replace in sys.modules
    import types
    common = types.ModuleType('common')
    common.params = types.ModuleType('params')
    common.params.Params = MockParams
    sys.modules['common'] = common
    sys.modules['common.params'] = common.params

# Now import the server
if __name__ == '__main__':
    print("=" * 60)
    print("BluePilot Web Routes Server - Test Mode")
    print("=" * 60)
    print()
    print("This script runs the web routes server locally for testing.")
    print()
    print("Server URL: http://localhost:8088")
    print("API Status: http://localhost:8088/api/status")
    print()
    print("Press Ctrl+C to stop the server")
    print("=" * 60)
    print()

    # Import and run the server
    from bluepilot.backend.bp_portal import main

    try:
        main()
    except KeyboardInterrupt:
        print("\n\nServer stopped by user")
        sys.exit(0)
