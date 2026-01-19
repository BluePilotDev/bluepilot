#!/usr/bin/env python3
"""
BluePilot Web Routes Panel - Setup Verification Script

This script verifies that all necessary components are in place.
Requires: websockets (for real-time updates, graceful fallback if not available)
"""

import os
import sys
import subprocess
import importlib.util

def check_file(path, description):
    """Check if a file exists"""
    if os.path.exists(path):
        print(f"✓ {description}")
        return True
    else:
        print(f"✗ {description} - NOT FOUND: {path}")
        return False


def check_websockets_dependency():
    """Check if websockets library is available"""
    print("Checking websockets dependency...")

    # Check if websockets module can be imported
    if importlib.util.find_spec("websockets") is not None:
        try:
            import websockets
            print(f"✓ websockets library found (version {websockets.__version__})")
            return True
        except ImportError as e:
            print(f"✗ Failed to import websockets: {e}")
            return False

    print("✗ websockets library not found")
    print("  Note: The web server will attempt to install it automatically when started")
    print("  If running in a read-only environment, install manually first:")
    print("    uv sync --extra dev")
    print("    # OR: pip install websockets")
    return False

def main():
    print("=" * 60)
    print("BluePilot Web Routes Panel - Setup Verification")
    print("=" * 60)
    print()

    # Check if running from correct directory
    if not os.path.exists("bluepilot/backend/web_routes_server.py"):
        print("Error: Please run this script from the openpilot root directory")
        sys.exit(1)

    # Check and install dependencies first
    print("Checking dependencies...")
    print()
    deps_ok = check_websockets_dependency()
    print()

    print("Checking required files...")
    print()

    all_ok = deps_ok

    # Backend
    print("Backend:")
    all_ok &= check_file("bluepilot/backend/web_routes_server.py", "  Web server (Python stdlib)")
    all_ok &= check_file("bluepilot/backend/__init__.py", "  Backend module init")
    print()

    # Frontend
    print("Frontend (deployed):")
    all_ok &= check_file("bluepilot/web/public/index.html", "  index.html")
    all_ok &= check_file("bluepilot/web/public/styles.css", "  styles.css")
    all_ok &= check_file("bluepilot/web/public/app.js", "  app.js")
    print()

    # Check params system (optional)
    print("System Integration:")
    try:
        from common.params import Params
        params = Params()
        print("✓ Params system available")

        # Check for web server params
        try:
            port = params.get("BPPortalPort")
            if port:
                print(f"✓ BPPortalPort = {port}")
            else:
                print("  BPPortalPort not set (will default to 8088)")
        except:
            print("  BPPortalPort not set (will default to 8088)")

    except ImportError:
        print("⚠ Params system not available (normal on dev machines)")
        print("  Server will use default port 8088")
    print()

    # Summary
    print("=" * 60)
    if all_ok:
        print("✓ All required files and dependencies found!")
        print()
        print("DEPENDENCIES:")
        print("• websockets (Python package for real-time updates)")
        print("• Python standard library (http.server for HTTP API)")
        print()
        print("Note: websockets will be installed automatically when the web server starts")
        print()
        print("Next steps:")
        print("1. Build the project: scons -j$(nproc)")
        print("2. Enable web server in BluePilot UI (Routes panel)")
        print("3. Access at http://<device-ip>:8088")
        print("4. WebSocket server runs on port 8089 (auto-started)")
        print()
        print("For local testing:")
        print("  python3 bluepilot/test_web_routes.py")
    else:
        print("✗ Setup incomplete - some files or dependencies are missing")
        print()
        if not deps_ok:
            print("Dependencies missing:")
            print("  The websockets library is required for real-time updates.")
            print("  Run this script again or install manually: pip install websockets")
            print()
        print("Make sure you're on the correct branch:")
        print("  git checkout web-routes-panel")
        print()
        print("Or rebuild the web app:")
        print("  cd bluepilot/web && ./build.sh")

    print("=" * 60)

if __name__ == '__main__':
    main()
