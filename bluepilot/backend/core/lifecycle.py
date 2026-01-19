#!/usr/bin/env python3
"""
BluePilot Backend Lifecycle Management
Server lifecycle functions including crash tracking and dependency management
"""

import os
import sys
import time
import logging
import subprocess
import shutil
import threading

logger = logging.getLogger(__name__)

# Global flags to track restart state
_restart_pending = False      # True when dependencies installed and restart needed
_restart_triggered = False    # True when restart has been initiated (prevents double-trigger)
_restart_lock = threading.Lock()


def record_crash(params):
    """
    Record a server crash for monitoring (but don't disable server)

    Args:
        params: Params instance for storing crash tracking data
    """
    try:
        current_time = int(time.monotonic())

        # Get crash count - handle both real and mock params
        try:
            crash_count_str = params.get("BPPortalCrashCount")
            crash_count = int(crash_count_str) if crash_count_str else 0
        except (AttributeError, TypeError):
            # Mock params or other error
            crash_count = 0

        # Get last crash time - handle both real and mock params
        try:
            last_crash_str = params.get("BPPortalLastCrash")
            last_crash = int(last_crash_str) if last_crash_str else 0
        except (AttributeError, TypeError):
            # Mock params or other error
            last_crash = 0

        # Check if this is a recent consecutive crash
        if current_time - last_crash <= 30:  # Within 30 seconds
            crash_count += 1
        else:
            # Reset crash count if more than 30 seconds have passed
            crash_count = 1

        # Update crash tracking parameters for monitoring only
        try:
            params.put("BPPortalCrashCount", min(crash_count, 10))  # Cap at 10
            params.put("BPPortalLastCrash", current_time)
        except AttributeError:
            # Mock params object doesn't have put methods, skip
            pass

        logger.warning(f"Server error occurred ({crash_count} recent errors) - continuing operation")

    except Exception as e:
        logger.error(f"Error recording crash: {e}")


def check_and_handle_crashes(params):
    """
    Check server status but never disable it automatically

    Args:
        params: Params instance for checking crash tracking data

    Returns:
        bool: Always returns True (server always allowed to start)
    """
    try:
        # Get crash count for monitoring only - don't disable server
        try:
            crash_count_str = params.get("BPPortalCrashCount")
            crash_count = int(crash_count_str) if crash_count_str else 0
        except (AttributeError, TypeError):
            crash_count = 0

        # Get last crash time for monitoring only
        try:
            last_crash_str = params.get("BPPortalLastCrash")
            last_crash = int(last_crash_str) if last_crash_str else 0
        except (AttributeError, TypeError):
            last_crash = 0

        current_time = int(time.monotonic())

        # Log crash statistics for monitoring but don't disable server
        if crash_count > 0:
            logger.info(f"Server has experienced {crash_count} errors recently - continuing operation")

        # Reset crash count if server has been running stably for 10 minutes
        if crash_count > 0 and current_time - last_crash > 600:  # 10 minutes
            logger.info("Server running stably for 10 minutes, resetting error count")
            try:
                params.put("BPPortalCrashCount", 0)
                params.put("BPPortalLastCrash", 0)
            except AttributeError:
                # Mock params object doesn't have put methods, skip
                pass

        return True  # Always allow server to start

    except Exception as e:
        logger.error(f"Error checking crash status: {e}")
        return True  # Default to allowing server start


def check_dependencies():
    """
    Check if all required dependencies are available (without installing).

    Returns:
        bool: True if all dependencies are available, False if some are missing
    """
    try:
        import websockets  # noqa: F401
        return True
    except ImportError:
        return False
    except Exception as e:
        # Handle any other import errors (corrupted package, etc.)
        logger.warning(f"Unexpected error checking websockets dependency: {e}")
        return False


def is_restart_pending():
    """Check if a restart is pending due to dependency installation."""
    global _restart_pending
    with _restart_lock:
        return _restart_pending


def trigger_restart():
    """
    Trigger a server restart by re-executing the current process.

    This function should only be called when is_restart_pending() returns True.
    It includes protection against being called multiple times.
    """
    global _restart_pending, _restart_triggered

    with _restart_lock:
        # Check if restart has already been triggered (prevents double-trigger)
        if _restart_triggered:
            logger.info("Restart already triggered, skipping duplicate call")
            return

        # Check if restart is actually needed
        if not _restart_pending:
            logger.warning("trigger_restart called but no restart is pending")
            return

        # Mark restart as triggered to prevent concurrent calls
        _restart_triggered = True

    logger.info("Triggering server restart to load newly installed packages...")
    logger.info("Waiting 2 seconds before restart...")
    time.sleep(2)

    # Re-execute the current process
    logger.info("Restarting server process...")
    try:
        os.execv(sys.executable, [sys.executable] + sys.argv)
    except Exception as e:
        # If execv fails, reset the triggered flag so it can be retried
        logger.error(f"Failed to restart server: {e}")
        with _restart_lock:
            _restart_triggered = False
        raise


def install_dependencies_background(on_complete_callback=None):
    """
    Install missing dependencies in a background thread.
    The server continues running during installation.

    Args:
        on_complete_callback: Optional callback to call when installation completes.
                             Called with True if restart needed, False otherwise.
    """
    global _restart_pending

    def _install_thread():
        global _restart_pending
        try:
            # Check if websockets is available
            try:
                import websockets
                logger.info("websockets library is already available")
                if on_complete_callback:
                    on_complete_callback(False)
                return
            except ImportError:
                pass

            logger.info("=" * 60)
            logger.info("BACKGROUND DEPENDENCY INSTALLATION")
            logger.info("websockets library not available - installing in background")
            logger.info("HTTP API will continue working during installation")
            logger.info("=" * 60)

            # Try to install websockets package
            if not shutil.which("uv"):
                logger.warning("uv not available - cannot install websockets automatically")
                logger.info("To enable WebSocket support, run: uv pip install websockets websocket-client")
                if on_complete_callback:
                    on_complete_callback(False)
                return

            logger.info("Installing websockets using uv pip...")
            result = subprocess.run(
                ["uv", "pip", "install", "websockets", "websocket-client"],
                capture_output=True,
                text=True,
                timeout=120  # Allow more time since this is background
            )

            if result.returncode == 0:
                logger.info("=" * 60)
                logger.info("DEPENDENCY INSTALLATION COMPLETE")
                logger.info("websockets installed successfully")
                logger.info("Server will restart to enable WebSocket features")
                logger.info("=" * 60)

                with _restart_lock:
                    _restart_pending = True

                if on_complete_callback:
                    on_complete_callback(True)
            else:
                logger.error(f"Failed to install websockets: {result.stderr}")
                logger.info("WebSocket features will remain disabled (HTTP polling available)")
                if on_complete_callback:
                    on_complete_callback(False)

        except subprocess.TimeoutExpired:
            logger.error("Package installation timed out (120s)")
            logger.info("Network may not be available. WebSocket features will remain disabled")
            if on_complete_callback:
                on_complete_callback(False)
        except Exception as e:
            logger.error(f"Error during background package installation: {e}")
            if on_complete_callback:
                on_complete_callback(False)

    # Start installation in background thread
    install_thread = threading.Thread(
        target=_install_thread,
        daemon=True,
        name="DependencyInstaller"
    )
    install_thread.start()
    logger.info("Background dependency installation thread started")
    return install_thread


def ensure_dependencies():
    """
    Check if all required dependencies are available, install if missing, and restart server.
    DEPRECATED: Use install_dependencies_background() for non-blocking installation.

    Returns:
        bool: True if restart is needed (dependencies were installed), False otherwise
    """
    try:
        # Check if websockets is available (direct import)
        try:
            import websockets
            logger.info("websockets library is available")
            return False  # No restart needed
        except ImportError:
            pass

        logger.warning("websockets library not available - attempting installation")
        logger.info("HTTP API will still work during installation")

        # Try to install websockets package
        try:
            # Use uv pip install to avoid modifying pyproject.toml
            if shutil.which("uv"):
                logger.info("Installing websockets using uv pip (user site-packages)...")
                result = subprocess.run(
                    ["uv", "pip", "install", "websockets", "websocket-client"],
                    capture_output=True,
                    text=True,
                    timeout=60
                )

                if result.returncode == 0:
                    logger.info("websockets installed successfully")
                    logger.info("Server will restart in 3 seconds to enable WebSocket features")
                    return True  # Restart needed
                else:
                    logger.error(f"Failed to install websockets: {result.stderr}")
                    logger.info("WebSocket features will remain disabled")
                    return False
            else:
                logger.warning("uv not available - cannot install websockets automatically")
                logger.info("To enable WebSocket support, run: uv pip install websockets websocket-client")
                return False

        except subprocess.TimeoutExpired:
            logger.error("Package installation timed out (60s)")
            logger.info("Network may not be available yet. WebSocket features will remain disabled")
            return False
        except Exception as e:
            logger.error(f"Error during package installation: {e}")
            return False

    except Exception as e:
        logger.warning(f"Error during dependency check: {e}")
        return False
